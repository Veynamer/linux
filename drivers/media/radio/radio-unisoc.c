// SPDX-License-Identifier: GPL-2.0-only
/* Marlin3 FM receiver on the shared Unisoc WCN SDIO transport. */

#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/misc/unisoc-wcn.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>

#define UNISOC_FM_OPCODE		0xfc8c
#define UNISOC_FM_MIN		8750
#define UNISOC_FM_MAX		10800
#define UNISOC_FM_FREQ_SCALE	160 /* 10 kHz firmware units to 62.5 Hz V4L2 units */
#define UNISOC_FM_CONFIG_WORDS	64
#define UNISOC_FM_CMD_TIMEOUT	4000
#define UNISOC_FM_SEEK_TIMEOUT	20000

enum unisoc_fm_command {
	FM_POWERUP = 0x00,
	FM_TUNE = 0x01,
	FM_MUTE = 0x02,
	FM_SEEK = 0x04,
	FM_SEARCH_ABORT = 0x05,
	FM_AUDIO_MODE = 0x08,
	FM_REGION = 0x0a,
	FM_SCAN_STEP = 0x0b,
	FM_DEEMPHASIS = 0x0c,
	FM_GET_AUDIO_MODE = 0x0e,
	FM_VOLUME = 0x10,
	FM_POWERDOWN = 0x12,
	FM_RSSI = 0x13,
};

struct unisoc_fm {
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock; /* V4L2 operations, users and power state */
	struct mutex rx_lock; /* fragmented HCI event parser */
	spinlock_t response_lock; /* pending command/seek and response data */
	struct wcn_core *core;
	struct wcn_chan *tx;
	struct wcn_chan *rx;
	struct completion command_done;
	struct completion seek_done;
	bool command_pending;
	bool seek_pending;
	int command_result;
	int seek_result;
	u16 seek_freq;
	u8 response[251];
	size_t response_len;
	u8 rx_buf[258];
	size_t rx_used;
	bool powered;
	bool disconnected;
	unsigned int users;
	u16 frequency;
	u8 audio_mode;
	__le16 config[UNISOC_FM_CONFIG_WORDS];
};

static void unisoc_fm_event(struct unisoc_fm *fm, const u8 *buf, size_t len)
{
	unsigned long flags;

	spin_lock_irqsave(&fm->response_lock, flags);
	if (buf[1] == 0x0e && len >= 7 && get_unaligned_le16(buf + 4) == UNISOC_FM_OPCODE) {
		if (fm->command_pending) {
			fm->command_result = buf[6] ? -EREMOTEIO : 0;
			fm->response_len = len - 7;
			memcpy(fm->response, buf + 7, fm->response_len);
			fm->command_pending = false;
			complete(&fm->command_done);
		}
	} else if (buf[1] == 0xff && len >= 9 && buf[3] == 0x30 && fm->seek_pending) {
		fm->seek_result = buf[4] ? -ENODATA : 0;
		fm->seek_freq = get_unaligned_le16(buf + 7);
		fm->seek_pending = false;
		complete(&fm->seek_done);
	}
	spin_unlock_irqrestore(&fm->response_lock, flags);
}

/* HCI events can be split over SDIO packets, or share a single packet. */
static void unisoc_fm_receive(struct unisoc_fm *fm, const u8 *data, size_t len)
{
	size_t needed, count;

	while (len) {
		if (!fm->rx_used && *data != 0x04) {
			data++;
			len--;
			continue;
		}
		needed = fm->rx_used < 3 ? 3 : 3 + fm->rx_buf[2];
		count = min(len, needed - fm->rx_used);
		memcpy(fm->rx_buf + fm->rx_used, data, count);
		fm->rx_used += count;
		data += count;
		len -= count;
		if (fm->rx_used >= 3 && fm->rx_used == 3 + fm->rx_buf[2]) {
			unisoc_fm_event(fm, fm->rx_buf, fm->rx_used);
			fm->rx_used = 0;
		}
	}
}

static void unisoc_fm_rx(struct wcn_chan *chan, struct sk_buff *skb)
{
	struct unisoc_fm *fm = wcn_chan_get_drvdata(chan);

	mutex_lock(&fm->rx_lock);
	unisoc_fm_receive(fm, skb->data, skb->len);
	mutex_unlock(&fm->rx_lock);
	kfree_skb(skb);
}

static const struct wcn_chan_ops unisoc_fm_rx_ops = { .rx = unisoc_fm_rx };

/* Caller holds lock; RX only takes rx_lock/response_lock. */
static int unisoc_fm_command(struct unisoc_fm *fm, u8 command, const void *payload,
			     size_t len, void *response, size_t response_len)
{
	unsigned long flags;
	u8 *buf;
	int ret;

	if (fm->disconnected)
		return -ENODEV;
	if (len > 254)
		return -EMSGSIZE;
	buf = kmalloc(5 + len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	buf[0] = 0x01;
	put_unaligned_le16(UNISOC_FM_OPCODE, buf + 1);
	buf[3] = len + 1;
	buf[4] = command;
	if (len)
		memcpy(buf + 5, payload, len);

	spin_lock_irqsave(&fm->response_lock, flags);
	reinit_completion(&fm->command_done);
	fm->command_pending = true;
	fm->response_len = 0;
	spin_unlock_irqrestore(&fm->response_lock, flags);
	ret = wcn_chan_tx(fm->tx, buf, 5 + len);
	kfree(buf);
	if (!ret && !wait_for_completion_timeout(&fm->command_done,
						 msecs_to_jiffies(UNISOC_FM_CMD_TIMEOUT)))
		ret = -ETIMEDOUT;
	spin_lock_irqsave(&fm->response_lock, flags);
	fm->command_pending = false;
	if (!ret)
		ret = fm->command_result;
	if (!ret && response_len) {
		if (fm->response_len < response_len)
			ret = -EPROTO;
		else
			memcpy(response, fm->response, response_len);
	}
	spin_unlock_irqrestore(&fm->response_lock, flags);
	return ret;
}

static int unisoc_fm_byte(struct unisoc_fm *fm, u8 command, u8 value)
{
	return unisoc_fm_command(fm, command, &value, sizeof(value), NULL, 0);
}

static int unisoc_fm_tune(struct unisoc_fm *fm, u16 frequency)
{
	__le16 value = cpu_to_le16(frequency);
	int ret;

	ret = unisoc_fm_command(fm, FM_TUNE, &value, sizeof(value), NULL, 0);
	if (!ret)
		fm->frequency = frequency;
	return ret;
}

static int unisoc_fm_power_off(struct unisoc_fm *fm, bool force)
{
	int ret;

	if (!fm->powered)
		return 0;
	ret = unisoc_fm_byte(fm, FM_POWERDOWN, 0);
	if (ret && !force)
		return ret;
	if (ret && ret != -ENODEV)
		dev_warn(fm->v4l2_dev.dev, "FM shutdown failed: %d\n", ret);
	fm->powered = false;
	wcn_power_put(fm->core, WCN_CHILD_FM);
	return ret;
}

static int unisoc_fm_power_on(struct unisoc_fm *fm)
{
	__le16 payload[1 + UNISOC_FM_CONFIG_WORDS];
	int ret;

	if (fm->powered)
		return 0;
	mutex_lock(&fm->rx_lock);
	fm->rx_used = 0;
	mutex_unlock(&fm->rx_lock);
	ret = wcn_power_get(fm->core, WCN_CHILD_FM);
	if (ret)
		return ret;
	payload[0] = cpu_to_le16(fm->frequency);
	memcpy(payload + 1, fm->config, sizeof(fm->config));
	ret = unisoc_fm_command(fm, FM_POWERUP, payload, sizeof(payload), NULL, 0);
	if (ret) {
		wcn_power_put(fm->core, WCN_CHILD_FM);
		return ret;
	}
	fm->powered = true;
	ret = unisoc_fm_byte(fm, FM_REGION, 1); /* US/Europe: 87.5--108 MHz */
	if (!ret)
		ret = unisoc_fm_byte(fm, FM_AUDIO_MODE, fm->audio_mode);
	if (!ret)
		ret = __v4l2_ctrl_handler_setup(&fm->ctrls);
	if (ret)
		unisoc_fm_power_off(fm, true);
	return ret;
}

static int unisoc_fm_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct unisoc_fm *fm = container_of(ctrl->handler, struct unisoc_fm, ctrls);

	if (!fm->powered)
		return 0;
	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		return unisoc_fm_byte(fm, FM_MUTE, ctrl->val);
	case V4L2_CID_AUDIO_VOLUME:
		return unisoc_fm_byte(fm, FM_VOLUME, ctrl->val);
	case V4L2_CID_TUNE_DEEMPHASIS:
		return unisoc_fm_byte(fm, FM_DEEMPHASIS,
				      ctrl->val == V4L2_DEEMPHASIS_75_uS);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops unisoc_fm_ctrl_ops = { .s_ctrl = unisoc_fm_s_ctrl };

static int unisoc_fm_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct unisoc_fm *fm = video_drvdata(file);

	strscpy(cap->driver, "radio-unisoc", sizeof(cap->driver));
	strscpy(cap->card, "Unisoc Marlin3 FM", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(fm->v4l2_dev.dev));
	return 0;
}

static int unisoc_fm_g_tuner(struct file *file, void *priv, struct v4l2_tuner *tuner)
{
	struct unisoc_fm *fm = video_drvdata(file);
	u8 rssi = 0, mode = 0, zero = 0;
	int ret;

	if (tuner->index)
		return -EINVAL;
	if (!fm->powered)
		return -EHOSTDOWN;
	strscpy(tuner->name, "FM", sizeof(tuner->name));
	tuner->type = V4L2_TUNER_RADIO;
	tuner->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
		V4L2_TUNER_CAP_HWSEEK_WRAP | V4L2_TUNER_CAP_HWSEEK_BOUNDED;
	tuner->rangelow = UNISOC_FM_MIN * UNISOC_FM_FREQ_SCALE;
	tuner->rangehigh = UNISOC_FM_MAX * UNISOC_FM_FREQ_SCALE;
	tuner->audmode = fm->audio_mode == 0 ? V4L2_TUNER_MODE_MONO : V4L2_TUNER_MODE_STEREO;
	ret = unisoc_fm_command(fm, FM_RSSI, &zero, 1, &rssi, 1);
	if (!ret)
		ret = unisoc_fm_command(fm, FM_GET_AUDIO_MODE, NULL, 0, &mode, 1);
	if (ret)
		return ret;
	tuner->signal = rssi * 257;
	tuner->rxsubchans = V4L2_TUNER_SUB_MONO;
	if (mode)
		tuner->rxsubchans |= V4L2_TUNER_SUB_STEREO;
	return 0;
}

static int unisoc_fm_s_tuner(struct file *file, void *priv, const struct v4l2_tuner *tuner)
{
	struct unisoc_fm *fm = video_drvdata(file);
	u8 mode;
	int ret;

	if (tuner->index || (tuner->audmode != V4L2_TUNER_MODE_MONO &&
			     tuner->audmode != V4L2_TUNER_MODE_STEREO))
		return -EINVAL;
	if (!fm->powered)
		return -EHOSTDOWN;
	mode = tuner->audmode == V4L2_TUNER_MODE_MONO ? 0 : 1;
	ret = unisoc_fm_byte(fm, FM_AUDIO_MODE, mode);
	if (!ret)
		fm->audio_mode = mode;
	return ret;
}

static int unisoc_fm_g_frequency(struct file *file, void *priv, struct v4l2_frequency *freq)
{
	struct unisoc_fm *fm = video_drvdata(file);

	if (freq->tuner)
		return -EINVAL;
	freq->type = V4L2_TUNER_RADIO;
	freq->frequency = fm->frequency * UNISOC_FM_FREQ_SCALE;
	return 0;
}

static int unisoc_fm_s_frequency(struct file *file, void *priv,
				 const struct v4l2_frequency *freq)
{
	struct unisoc_fm *fm = video_drvdata(file);
	u32 value;

	if (freq->tuner || freq->type != V4L2_TUNER_RADIO)
		return -EINVAL;
	if (!fm->powered)
		return -EHOSTDOWN;
	value = clamp_val(freq->frequency, UNISOC_FM_MIN * UNISOC_FM_FREQ_SCALE,
			  UNISOC_FM_MAX * UNISOC_FM_FREQ_SCALE);
	value = DIV_ROUND_CLOSEST(value, UNISOC_FM_FREQ_SCALE);
	return unisoc_fm_tune(fm, value);
}

static int unisoc_fm_seek(struct file *file, void *priv,
			  const struct v4l2_hw_freq_seek *seek)
{
	struct unisoc_fm *fm = video_drvdata(file);
	unsigned long flags;
	u16 start = fm->frequency, found;
	u8 payload[3], step;
	int ret;

	if (seek->tuner || seek->type != V4L2_TUNER_RADIO)
		return -EINVAL;
	if (!fm->powered)
		return -EHOSTDOWN;
	if (file->f_flags & O_NONBLOCK)
		return -EWOULDBLOCK;
	if ((seek->rangelow && seek->rangelow != UNISOC_FM_MIN * UNISOC_FM_FREQ_SCALE) ||
	    (seek->rangehigh && seek->rangehigh != UNISOC_FM_MAX * UNISOC_FM_FREQ_SCALE))
		return -EINVAL;
	if (!seek->spacing || seek->spacing == 100000)
		step = 1;
	else if (seek->spacing == 50000)
		step = 0;
	else if (seek->spacing == 200000)
		step = 2;
	else
		return -EINVAL;
	ret = unisoc_fm_byte(fm, FM_SCAN_STEP, step);
	if (ret)
		return ret;
	put_unaligned_le16(start, payload);
	payload[2] = seek->seek_upward ? 0 : 1;
	spin_lock_irqsave(&fm->response_lock, flags);
	reinit_completion(&fm->seek_done);
	fm->seek_pending = true;
	spin_unlock_irqrestore(&fm->response_lock, flags);
	ret = unisoc_fm_command(fm, FM_SEEK, payload, sizeof(payload), NULL, 0);
	if (!ret && !wait_for_completion_timeout(&fm->seek_done,
						 msecs_to_jiffies(UNISOC_FM_SEEK_TIMEOUT)))
		ret = -ETIMEDOUT;
	/* The search may have started even if its command ACK was lost. */
	if (ret == -ETIMEDOUT)
		unisoc_fm_command(fm, FM_SEARCH_ABORT, NULL, 0, NULL, 0);
	spin_lock_irqsave(&fm->response_lock, flags);
	fm->seek_pending = false;
	found = fm->seek_freq;
	if (!ret)
		ret = fm->seek_result;
	spin_unlock_irqrestore(&fm->response_lock, flags);
	if (ret)
		return ret;
	if (found < UNISOC_FM_MIN || found > UNISOC_FM_MAX)
		return -EPROTO;
	if (!seek->wrap_around && ((seek->seek_upward && found < start) ||
				   (!seek->seek_upward && found > start))) {
		ret = unisoc_fm_tune(fm, start);
		return ret ?: -ENODATA;
	}
	fm->frequency = found;
	return 0;
}

static const struct v4l2_ioctl_ops unisoc_fm_ioctl_ops = {
	.vidioc_querycap = unisoc_fm_querycap,
	.vidioc_g_tuner = unisoc_fm_g_tuner,
	.vidioc_s_tuner = unisoc_fm_s_tuner,
	.vidioc_g_frequency = unisoc_fm_g_frequency,
	.vidioc_s_frequency = unisoc_fm_s_frequency,
	.vidioc_s_hw_freq_seek = unisoc_fm_seek,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int unisoc_fm_open(struct file *file)
{
	struct unisoc_fm *fm = video_drvdata(file);
	int ret;

	mutex_lock(&fm->lock);
	if (fm->disconnected) {
		ret = -ENODEV;
		goto unlock;
	}
	ret = v4l2_fh_open(file);
	if (ret)
		goto unlock;
	if (!fm->users) {
		ret = unisoc_fm_power_on(fm);
		if (ret) {
			v4l2_fh_release(file);
			goto unlock;
		}
	}
	fm->users++;
unlock:
	mutex_unlock(&fm->lock);
	return ret;
}

static int unisoc_fm_release(struct file *file)
{
	struct unisoc_fm *fm = video_drvdata(file);

	mutex_lock(&fm->lock);
	if (!--fm->users)
		unisoc_fm_power_off(fm, true);
	v4l2_fh_release(file);
	mutex_unlock(&fm->lock);
	return 0;
}

static const struct v4l2_file_operations unisoc_fm_fops = {
	.owner = THIS_MODULE,
	.open = unisoc_fm_open,
	.release = unisoc_fm_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_ctrl_poll,
};

/* Matches the 64 u16 words used by the vendor's FM power-up command. */
static int unisoc_fm_parse_config(char *text, __le16 *config)
{
	static const char * const names[] = {
		"Fm_modem_work_freq", "Stra_sel", "Seek_ch_th", "Mono_pwr_th1",
		"Mono_pwr_th2", "Mono_pwr_th3", "Seek_chan_mode", "Seek_vldch_fo_th1",
		"Seek_vldch_fo_th2", "Noise_pwr_th0", "Noise_pwr_th1", "Noise_pwr_th2",
		"Pdp_th", "Pdp_dev1", "Pdp_dev2", "Seek_sample_num_div", "Boundary_offset",
		"Fm_db_comp", "Fm_cali_reserved",
	};
	char *line, *key, *values, *token, *comment;
	unsigned int index, count, limit;
	int value, ret;

	while ((line = strsep(&text, "\n"))) {
		comment = strpbrk(line, "#;");
		if (comment)
			*comment = '\0';
		key = strim(line);
		if (!*key || *key == '[')
			continue;
		values = strchr(key, '=');
		if (!values)
			continue;
		*values++ = '\0';
		key = strim(key);
		for (index = 0; index < ARRAY_SIZE(names); index++)
			if (!strcasecmp(key, names[index]))
				break;
		if (index == ARRAY_SIZE(names))
			continue;
		limit = index == ARRAY_SIZE(names) - 1 ? 46 : 1;
		count = 0;
		while ((token = strsep(&values, ", \t\r"))) {
			if (!*token)
				continue;
			if (count >= limit)
				return -EINVAL;
			ret = kstrtoint(token, 0, &value);
			if (ret || value < -32768 || value > 65535)
				return -EINVAL;
			config[index + count++] = cpu_to_le16((u16)value);
		}
		if (!count)
			return -EINVAL;
	}
	return 0;
}

static int unisoc_fm_config(struct device *dev, const char *name, __le16 *config)
{
	const struct firmware *fw;
	char *text;
	int ret;

	device_property_read_string(dev, "firmware-name", &name);
	if (!name)
		return 0;
	ret = firmware_request_nowarn(&fw, name, dev);
	if (ret == -ENOENT)
		return 0; /* The BSP also uses zero/default words without a board INI. */
	if (ret)
		return ret;
	if (fw->size > SZ_64K) {
		ret = -EFBIG;
		goto release;
	}
	text = kmemdup_nul(fw->data, fw->size, GFP_KERNEL);
	if (!text) {
		ret = -ENOMEM;
		goto release;
	}
	ret = unisoc_fm_parse_config(text, config);
	kfree(text);
release:
	release_firmware(fw);
	return ret;
}

static void unisoc_fm_video_release(struct video_device *vdev)
{
	struct unisoc_fm *fm = container_of(vdev, struct unisoc_fm, vdev);

	v4l2_ctrl_handler_free(&fm->ctrls);
	v4l2_device_unregister(&fm->v4l2_dev);
	kfree(fm);
}

static int unisoc_fm_probe(struct platform_device *pdev)
{
	const struct wcn_child_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct unisoc_fm *fm;
	int ret;

	if (!pdata || !pdata->core || !pdata->data)
		return -ENODEV;
	fm = kzalloc_obj(*fm);
	if (!fm)
		return -ENOMEM;
	fm->core = pdata->core;
	fm->frequency = UNISOC_FM_MIN;
	fm->audio_mode = 2;
	mutex_init(&fm->lock);
	mutex_init(&fm->rx_lock);
	spin_lock_init(&fm->response_lock);
	init_completion(&fm->command_done);
	init_completion(&fm->seek_done);
	ret = unisoc_fm_config(&pdev->dev, pdata->data->firmware_name, fm->config);
	if (ret)
		goto free;
	fm->tx = wcn_chan_open(fm->core, pdata->data->channels.tx_cmd, NULL, fm);
	if (IS_ERR(fm->tx)) {
		ret = PTR_ERR(fm->tx);
		goto free;
	}
	fm->rx = wcn_chan_open(fm->core, pdata->data->channels.rx_cmd, &unisoc_fm_rx_ops, fm);
	if (IS_ERR(fm->rx)) {
		ret = PTR_ERR(fm->rx);
		goto close_tx;
	}
	ret = v4l2_device_register(&pdev->dev, &fm->v4l2_dev);
	if (ret)
		goto close_rx;
	v4l2_ctrl_handler_init(&fm->ctrls, 3);
	fm->ctrls.lock = &fm->lock;
	v4l2_ctrl_new_std(&fm->ctrls, &unisoc_fm_ctrl_ops, V4L2_CID_AUDIO_MUTE, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&fm->ctrls, &unisoc_fm_ctrl_ops, V4L2_CID_AUDIO_VOLUME, 0, 15, 1, 8);
	v4l2_ctrl_new_std_menu(&fm->ctrls, &unisoc_fm_ctrl_ops, V4L2_CID_TUNE_DEEMPHASIS,
			       V4L2_DEEMPHASIS_75_uS, BIT(V4L2_DEEMPHASIS_DISABLED),
			       V4L2_DEEMPHASIS_50_uS);
	if (fm->ctrls.error) {
		ret = fm->ctrls.error;
		goto unregister;
	}
	strscpy(fm->vdev.name, "Unisoc Marlin3 FM", sizeof(fm->vdev.name));
	fm->vdev.fops = &unisoc_fm_fops;
	fm->vdev.ioctl_ops = &unisoc_fm_ioctl_ops;
	fm->vdev.release = unisoc_fm_video_release;
	fm->vdev.v4l2_dev = &fm->v4l2_dev;
	fm->vdev.ctrl_handler = &fm->ctrls;
	fm->vdev.lock = &fm->lock;
	fm->vdev.device_caps = V4L2_CAP_RADIO | V4L2_CAP_TUNER | V4L2_CAP_HW_FREQ_SEEK;
	video_set_drvdata(&fm->vdev, fm);
	ret = video_register_device(&fm->vdev, VFL_TYPE_RADIO, -1);
	if (ret)
		goto unregister;
	platform_set_drvdata(pdev, fm);
	return 0;
unregister:
	v4l2_ctrl_handler_free(&fm->ctrls);
	v4l2_device_unregister(&fm->v4l2_dev);
close_rx:
	wcn_chan_close(fm->rx);
close_tx:
	wcn_chan_close(fm->tx);
free:
	kfree(fm);
	return ret;
}

static void unisoc_fm_remove(struct platform_device *pdev)
{
	struct unisoc_fm *fm = platform_get_drvdata(pdev);

	mutex_lock(&fm->lock);
	unisoc_fm_power_off(fm, true);
	fm->disconnected = true;
	wcn_chan_close(fm->rx);
	wcn_chan_close(fm->tx);
	fm->core = NULL;
	mutex_unlock(&fm->lock);
	/* The embedded video device keeps fm alive until the last file closes. */
	video_unregister_device(&fm->vdev);
}

static int unisoc_fm_suspend(struct device *dev)
{
	struct unisoc_fm *fm = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&fm->lock);
	ret = unisoc_fm_power_off(fm, false);
	mutex_unlock(&fm->lock);
	return ret;
}

static int unisoc_fm_resume(struct device *dev)
{
	struct unisoc_fm *fm = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&fm->lock);
	if (fm->users)
		ret = unisoc_fm_power_on(fm);
	mutex_unlock(&fm->lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(unisoc_fm_pm_ops, unisoc_fm_suspend, unisoc_fm_resume);

static const struct of_device_id unisoc_fm_of_match[] = {
	{ .compatible = "unisoc,marlin3-fm" },
	{ }
};
MODULE_DEVICE_TABLE(of, unisoc_fm_of_match);

static struct platform_driver unisoc_fm_driver = {
	.probe = unisoc_fm_probe,
	.remove = unisoc_fm_remove,
	.driver = {
		.name = "radio-unisoc",
		.of_match_table = unisoc_fm_of_match,
		.pm = pm_sleep_ptr(&unisoc_fm_pm_ops),
	},
};
module_platform_driver(unisoc_fm_driver);

MODULE_DESCRIPTION("Unisoc Marlin3 SDIO FM radio receiver");
MODULE_LICENSE("GPL");
