/*
 * This file is part of wcn sysfs debug
 */

#define DRIVER_AUTHOR "Songhe Wei<songhe.wei@unisoc.com>"
#define DRIVER_DESC "support sysfs and uevent for debug wcn"
#define WCN_SYSFS_VERSION	"v0.0"

#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <misc/wcn_bus.h>

#include "../sdio/sdiohal.h"
#include "wcn_dbg.h"
#include "wcn_glb.h"
#include "sysfs.h"

struct wcn_sysfs_info sysfs_info;

void wcn_send_atcmd_lock(void)
{
	mutex_lock(&sysfs_info.mutex);
}

void wcn_send_atcmd_unlock(void)
{
	mutex_unlock(&sysfs_info.mutex);
}

int notify_at_cmd_finish(void *buf, size_t len)
{
	sysfs_info.p = buf;
	sysfs_info.len = len;
	complete(&sysfs_info.cmd_completion);

	return 0;
}

int wcn_send_atcmd(void *cmd, size_t cmd_len,
			  void *response, size_t *response_len)
{
	struct mbuf_t *head = NULL;
	struct mbuf_t *tail = NULL;
	int num = 1;
	int ret;
	unsigned long timeleft;
	struct sdiohal_data_t *p_data = sdiohal_get_data();
	unsigned int pub_head_rsv = SDIOHAL_PUB_HEAD_RSV;
	unsigned char *com_buf = NULL;
	size_t capacity;

	if (!cmd_len || cmd_len > USHRT_MAX)
		return -EINVAL;

	if (!WCN_CARD_EXIST(&p_data->xmit_cnt)) {
		WCN_DBG("%s:already power off\n", __func__);
		return 0;
	}

	wcn_send_atcmd_lock();
	ret = sprdwcn_bus_list_alloc(0, &head, &tail, &num);
	if (ret || !head || !tail) {
		WCN_ERR("%s:%d mbuf_link_alloc fail\n", __func__, __LINE__);
		wcn_send_atcmd_unlock();
		return -1;
	}

	com_buf = kzalloc(cmd_len + pub_head_rsv + 1, GFP_KERNEL);
	if (!com_buf) {
		wcn_send_atcmd_unlock();
		return -ENOMEM;
	}
	memcpy(com_buf + pub_head_rsv, cmd, cmd_len);
	head->buf = com_buf;
	head->len = cmd_len;
	head->next = NULL;

	reinit_completion(&sysfs_info.cmd_completion);
	ret = sprdwcn_bus_push_list(0, head, tail, num);
	if (ret)
		WCN_ERR("sprdwcn_bus_push_list error=%d\n", ret);
	timeleft = wait_for_completion_timeout(&sysfs_info.cmd_completion,
					       3 * HZ);
	if (!WCN_CARD_EXIST(&p_data->xmit_cnt)) {
		WCN_DBG("%s:already power off\n", __func__);
		wcn_send_atcmd_unlock();
		return 0;
	}

	if (!timeleft) {
		WCN_ERR("%s,Timeout(%d sec),didn't get at cmd(%s) response\n",
			__func__, jiffies_to_msecs(3 * HZ) / 1000, (char *)cmd);
		wcn_send_atcmd_unlock();
		return -ETIMEDOUT;
	}
	if (!response) {
		wcn_send_atcmd_unlock();
		return 0;
	}

	if (*response_len == WCN_AT_RSP_RAW_FLAG) {
		memcpy(response, sysfs_info.p, sysfs_info.len);
	} else {
		capacity = *response_len ?: sysfs_info.len;
		if (capacity)
			scnprintf(response, capacity, "%s", (char *)sysfs_info.p);
		else
			((char *)response)[0] = '\0';
	}
	*response_len = sysfs_info.len;

	WCN_DBG("len=%zu, buf=%s\n", *response_len, (char *)(response));
	wcn_send_atcmd_unlock();

	return 0;
}

char *__wcn_get_sw_ver(void)
{
	return sysfs_info.sw_ver_buf;
}
EXPORT_SYMBOL_GPL(__wcn_get_sw_ver);

static int wcn_get_sw_ver(void)
{
	char a[] = "at+spatgetcp2info\r\n";

	wcn_send_atcmd(a, strlen(a), &sysfs_info.sw_ver_buf,
		       &sysfs_info.sw_ver_len);

	WCN_DBG("show:len=%zd, buf=%s\n", sysfs_info.sw_ver_len,
		sysfs_info.sw_ver_buf);

	return 0;
}

int wcn_fw_armlog_set(unsigned int enable)
{
	char a[32];

	if (enable > 1)
		return -EINVAL;

	sysfs_info.armlog_status = enable;
	if (!marlin_get_module_status())
		return 0;

	scnprintf(a, sizeof(a), "at+armlog=%u\r\n", enable);
	WCN_DBG("%s:%s", __func__, a);

	return wcn_send_atcmd(a, strlen(a), NULL, NULL);
}

int wcn_fw_armlog_get(char *buf, size_t size)
{
	size_t len = size;
	int ret;

	if (!buf || size < 2)
		return -EINVAL;

	if (!marlin_get_module_status())
		return scnprintf(buf, size, "+ARMLOG: %u",
				 sysfs_info.armlog_status);

	ret = wcn_send_atcmd("at+armlog?\r\n", strlen("at+armlog?\r\n"),
			     buf, &len);
	if (ret)
		return ret;

	if (!strncmp(buf, "+ARMLOG: ", strlen("+ARMLOG: "))) {
		buf[0] = buf[strlen("+ARMLOG: ")];
		buf[1] = '\0';
	} else {
		buf[0] = '\0';
	}

	return strlen(buf);
}

int wcn_fw_loglevel_set(unsigned int level)
{
	char a[32];
	int ret = 0;

	if (level >= 6)
		return -EINVAL;

	sysfs_info.loglevel = level;
	if (!marlin_get_module_status()) {
		atomic_or(WCN_SYSFS_LOGLEVEL_SET_BIT, &sysfs_info.set_mask);
		return 0;
	}

	scnprintf(a, sizeof(a), "at+loglevel=%u\r\n", level);
	WCN_DBG("%s:%s\n", __func__, a);
	ret = wcn_send_atcmd(a, strlen(a), NULL, NULL);
	if (!ret)
		atomic_and((unsigned int)(~WCN_SYSFS_LOGLEVEL_SET_BIT),
			   &sysfs_info.set_mask);

	return ret;
}

int wcn_fw_loglevel_get(char *buf, size_t size)
{
	unsigned int level;
	size_t len = size;
	int ret;

	if (!buf || !size)
		return -EINVAL;

	if (!marlin_get_module_status()) {
		if (sysfs_info.loglevel >= 6)
			sysfs_info.loglevel = 0;
		return scnprintf(buf, size, "+LOGLEVEL: %u",
				 sysfs_info.loglevel);
	}

	ret = wcn_send_atcmd("at+loglevel?\r\n", strlen("at+loglevel?\r\n"),
			     buf, &len);
	if (ret)
		return ret;

	if (!strncmp(buf, "+LOGLEVEL: ", strlen("+LOGLEVEL: ")) &&
	    !kstrtouint(buf + strlen("+LOGLEVEL: "), 10, &level))
		sysfs_info.loglevel = level;

	WCN_DBG("show:len=%zd, buf=%s, level=%u\n", len, buf,
		sysfs_info.loglevel);

	return strlen(buf);
}

int wcn_firmware_init(void)
{
	char level[64];
	int ret;
	int err = 0;

	wcn_get_sw_ver();

	ret = wcn_fw_armlog_set(sysfs_info.armlog_status);
	if (ret)
		err = ret;

	if (atomic_read(&sysfs_info.set_mask) & WCN_SYSFS_LOGLEVEL_SET_BIT) {
		ret = wcn_fw_loglevel_set(sysfs_info.loglevel);
		if (ret)
			err = ret;
	}

	wcn_fw_loglevel_get(level, sizeof(level));

	return err;
}

void wcn_firmware_init_wq(struct work_struct *work)
{
	wcn_firmware_init();
}

static ssize_t wcn_sysfs_show_sw_ver(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	size_t len = PAGE_SIZE;
	char a[] = "at+spatgetcp2info\r\n";

	WCN_DBG("%s \n", __func__);
	if (!marlin_get_module_status()) {
		memcpy(buf, sysfs_info.sw_ver_buf, sysfs_info.sw_ver_len);
		return sysfs_info.sw_ver_len;
	}

	wcn_send_atcmd(a, strlen(a), buf, &len);

	WCN_DBG("show:len=%zd, buf=%s\n", len, buf);
	len = strlen(buf);
	WCN_DBG("show:len=%zd\n", len);

	return len;
}

static DEVICE_ATTR(sw_ver, 0444,
		   wcn_sysfs_show_sw_ver, NULL);

static ssize_t wcn_sysfs_show_hw_ver(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	ssize_t len;

	len = PAGE_SIZE;
	mutex_lock(&sysfs_info.mutex);
	len = scnprintf(buf, len, "%s\n", wcn_get_chip_name());
	mutex_unlock(&sysfs_info.mutex);

	return len;
}

static DEVICE_ATTR(hw_ver, 0444,
		   wcn_sysfs_show_hw_ver, NULL);

static ssize_t wcn_sysfs_show_armlog_status(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	int len;

	len = wcn_fw_armlog_get(buf, PAGE_SIZE);
	if (len < 0)
		return len;

	WCN_DBG("%s:len=%d, buf=%s\n", __func__, len, buf);
	if (len && buf[len - 1] != '\n')
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static ssize_t wcn_sysfs_store_armlog_status(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	unsigned long res;
	int ret;

	ret = kstrtoul(buf, 10, &res);
	if (ret < 0) {
		WCN_ERR("incorrect value written to armlog\n");
		return -EINVAL;
	}

	if (res > 1) {
		WCN_ERR("incorrect value(%ld) written to armlog\n", res);
		return -EINVAL;
	}

	WCN_DBG("%s:set armlog = %ld\n", __func__, res);
	ret = wcn_fw_armlog_set(res);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(armlog_status, 0644,
		   wcn_sysfs_show_armlog_status,
		   wcn_sysfs_store_armlog_status);

static ssize_t wcn_sysfs_show_loglevel(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int len;

	len = wcn_fw_loglevel_get(buf, PAGE_SIZE);
	if (len < 0)
		return len;

	WCN_DBG("%s:len=%d, buf=%s\n", __func__, len, buf);

	return len;
}

static ssize_t wcn_sysfs_store_loglevel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long res;
	int ret;

	WCN_DBG("%s:buf=%s\n", __func__, buf);
	ret = kstrtoul(buf, 10, &res);
	if (ret < 0) {
		WCN_ERR("incorrect value written to loglevel\n");
		return -EINVAL;
	}

	if (res >= 6) {
		WCN_ERR("incorrect value(%ld) written to loglevel\n", res);
		return -EINVAL;
	}

	ret = wcn_fw_loglevel_set(res);
	if (ret)
		return ret;

	WCN_DBG("%s:set_mask:%d\n", __func__,
		 atomic_read(&sysfs_info.set_mask));

	return count;
}

static DEVICE_ATTR(loglevel, 0644,
		   wcn_sysfs_show_loglevel,
		   wcn_sysfs_store_loglevel);

static long wcn_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int wcn_open(struct inode *inode, struct file *filep)
{
	return 0;
}

static ssize_t wcn_read_data(struct file *filp, char __user *buf,
			     size_t count, loff_t *pos)
{
	return 0;
}

static int wcn_release(struct inode *inode, struct file *filep)
{
	return 0;
}

#ifdef CONFIG_COMPAT
static long wcn_compat_ioctl(struct file *file, unsigned int cmd,
			     unsigned long data)
{
	cmd = cmd & 0xFFF0FFFF;
	cmd = cmd | 0x00080000;
	return wcn_ioctl(file, cmd, data);
}
#endif

static const struct file_operations wcn_misc_fops = {
	.owner = THIS_MODULE,
	.open = wcn_open,
	.read = wcn_read_data,
	.unlocked_ioctl = wcn_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = wcn_compat_ioctl,
#endif
	.release = wcn_release,
};

static struct attribute *wcn_attrs[] = {
	&dev_attr_sw_ver.attr,
	&dev_attr_hw_ver.attr,
	&dev_attr_armlog_status.attr,
	&dev_attr_loglevel.attr,
	NULL,
};

static struct attribute_group wcn_attribute_group = {
	.name = "devices",
	.attrs = wcn_attrs,
};

static struct miscdevice wcn_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "wcn",
	.fops = &wcn_misc_fops,
};

void wcn_notify_fw_error(enum wcn_source_type type, char *buf)
{
	int len;
	char *pbuf;
	char *envp[4];

	WCN_ERR("Notify firmware error:%s\n", buf);
	len = strlen(buf) + strlen(WCN_UEVENT_REASON) + 1;
	if (len > 256)
		len = 256;
	pbuf = kmalloc(len, GFP_KERNEL);
	if (!pbuf) {
		WCN_ERR("no mem for notify fw error\n");
		pbuf = kmalloc(64, GFP_KERNEL);
		if (!pbuf) {
			WCN_ERR("notify fw error faile\n");
			return;
		}
	}

	snprintf(pbuf, len, "%s%s", WCN_UEVENT_REASON, buf);

	if (type == WCN_SOURCE_BTWF)
		envp[0] = "SOURCE=WCN-CP2-EXCEPTION";
	else if (type == WCN_SOURCE_CP2_ALIVE)
		envp[0] = "SOURCE=WCN-CP2-ALIVE";
	else
		envp[0] = "SOURCE=WCN";
	envp[1] = WCN_UEVENT_FW_ERRO;
	envp[2] = pbuf;
	envp[3] = NULL;

	kobject_uevent_env(&wcn_misc_device.this_device->kobj,
			   KOBJ_CHANGE, envp);

	kfree(pbuf);
}

int init_wcn_sysfs(void)
{
	int ret;

	WCN_DBG("%s\n", __func__);
	ret = misc_register(&wcn_misc_device);
	if (ret) {
		WCN_ERR("wcn misc dev register fail\n");
		return ret;
	}

	ret = sysfs_create_group(&wcn_misc_device.this_device->kobj,
				 &wcn_attribute_group);
	if (ret) {
		WCN_ERR("%s failed to create device group.\n",
			__func__);
		misc_deregister(&wcn_misc_device);
		return ret;
	}

	init_completion(&sysfs_info.cmd_completion);
	mutex_init(&sysfs_info.mutex);
	atomic_set(&sysfs_info.set_mask, 0x0);

#ifdef FLAG_WCN_USER
	sysfs_info.armlog_status = 0;
#else
	sysfs_info.armlog_status = 1;
#endif

	return 0;
}

void exit_wcn_sysfs(void)
{
	misc_deregister(&wcn_misc_device);
}
