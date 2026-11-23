// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "core.h"
#include "sdio.h"

#define WCN_SEC_IMAGE_MAGIC	0x42544844
#define WCN_SEC_IMAGE_HDR_SIZE	512

struct wcn_img_header {
	__le32 magic;
	__le32 version;
	u8 payload_hash[32];
	__le64 img_addr;
	__le32 img_size;
	u8 iv_data[16];
	__le32 is_packed;
	__le32 firmware_size;
	__le32 img_real_size;
	__le32 img_signed_size;
	u8 reserved[424];
	__le32 slot;
} __packed;

static_assert(sizeof(struct wcn_img_header) == WCN_SEC_IMAGE_HDR_SIZE);

#define WCN_IMG_PACK_MAGIC	"WCNM"
#define WCN_IMG_PACK_HEAD_SIZE	12
#define WCN_IMG_PACK_ENTRY_SIZE	12

static int wcn_cp_read32(struct wcn_core *core, u32 addr, u32 *val,
			 __le32 *buf)
{
	int ret;

	ret = wcn_sdio_dt_read(core, addr, buf, sizeof(*buf));
	if (!ret)
		*val = le32_to_cpu(*buf);

	return ret;
}

static int wcn_cp_write32(struct wcn_core *core, u32 addr, u32 val,
			  __le32 *buf)
{
	*buf = cpu_to_le32(val);
	return wcn_sdio_dt_write(core, addr, buf, sizeof(*buf));
}

static const char *wcn_fw_chip_tag(struct wcn_core *core, u32 chip_id)
{
	const struct wcn_fw_data *fw = &core->chip->fw;
	size_t i;

	for (i = 0; i < fw->num_ids; i++)
		if (fw->ids[i].chip_id == chip_id)
			return fw->ids[i].tag;

	return NULL;
}

static int wcn_fw_unpack(struct wcn_core *core, const struct firmware *fw,
			 const void **image, size_t *image_len, __le32 *buf)
{
	const u8 *pack = fw->data;
	const char *want;
	u32 chip_id, count, i;
	int ret;

	ret = wcn_cp_read32(core, core->chip->fw.chip_id_reg, &chip_id, buf);
	if (ret) {
		dev_err(core->dev, "failed to read WCN chip ID: %d\n", ret);
		return ret;
	}

	want = wcn_fw_chip_tag(core, chip_id);
	if (!want) {
		dev_err(core->dev, "unsupported WCN chip ID 0x%08x\n", chip_id);
		return -ENODEV;
	}

	count = get_unaligned_le32(pack + 8);
	if (!count || count > (fw->size - WCN_IMG_PACK_HEAD_SIZE) /
			       WCN_IMG_PACK_ENTRY_SIZE) {
		dev_err(core->dev, "invalid WCNM image count %u\n", count);
		return -EINVAL;
	}

	dev_info(core->dev, "WCNM image pack: chip 0x%08x, tag %.4s, %u images\n",
		 chip_id, want, count);

	for (i = 0; i < count; i++) {
		const u8 *entry = pack + WCN_IMG_PACK_HEAD_SIZE +
				  i * WCN_IMG_PACK_ENTRY_SIZE;
		u32 offset = get_unaligned_le32(entry + 4);
		u32 size = get_unaligned_le32(entry + 8);
		char tag[5] = {};

		memcpy(tag, entry, 4);
		dev_info(core->dev, "  image %.4s offset 0x%08x size 0x%08x\n",
			 tag, offset, size);

		if (memcmp(tag, want, 4))
			continue;

		if (!size || offset > fw->size || size > fw->size - offset) {
			dev_err(core->dev, "invalid WCNM image bounds\n");
			return -EINVAL;
		}

		*image = fw->data + offset;
		*image_len = size;
		return 0;
	}

	dev_err(core->dev, "no WCNM image for chip tag %.4s\n", want);
	return -ENOENT;
}

static int wcn_cp_wait_sync(struct wcn_core *core, u32 sync_addr,
			    bool calibration, __le32 *buf)
{
	u32 status, prev = 0;
	int i, ret;

	for (i = 0; i < 200; i++) {
		ret = READ_ONCE(core->pub_error);
		if (ret)
			return ret;
		ret = wcn_cp_read32(core, sync_addr, &status, buf);
		if (ret)
			return ret;

		if (!i || status != prev)
			dev_dbg(core->dev, "CP2 sync status %#010x\n", status);
		prev = status;

		if (status == WCN_SYNC_ALL_FINISHED)
			return 0;

		if (status == WCN_SYNC_CALI_WAITING && calibration) {
			put_unaligned_le16(WCN_SYNC_DAC_DEFAULT, buf);
			ret = wcn_sdio_dt_write(core,
						sync_addr + WCN_SYNC_DAC_OFFSET,
						buf, sizeof(__le16));
			if (ret)
				return ret;

			ret = wcn_cp_write32(core, sync_addr,
					     WCN_SYNC_CALI_WRITE_DONE, buf);
			if (ret)
				return ret;
			dev_dbg(core->dev, "CP2 TSX calibration acknowledged\n");
			return 0;
		}

		usleep_range(3000, 5000);
	}

	dev_err(core->dev, "WCN %s timeout, status %#010x, PUB_INT events %#x\n",
		calibration ? "calibration" : "final sync", prev,
		atomic_read(&core->pub_events));
	return -ETIMEDOUT;
}

static int wcn_cp_hold(struct wcn_core *core, __le32 *buf)
{
	u32 val;
	int ret = wcn_cp_read32(core, core->chip->reset_reg, &val, buf);

	if (ret)
		return ret;
	return wcn_cp_write32(core, core->chip->reset_reg, val | core->chip->reset_mask, buf);
}

int wcn_fw_stop(struct wcn_core *core)
{
	__le32 *buf;
	int ret;

	buf = kmalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;
	ret = wcn_cp_hold(core, buf);
	kfree(buf);
	return ret;
}

static int wcn_cp_boot(struct wcn_core *core, u32 sync_addr, __le32 *buf)
{
	u32 val;
	int ret;

	ret = wcn_sdio_pub_prepare(core);
	if (ret)
		return ret;

	ret = wcn_cp_read32(core, core->chip->reset_reg, &val, buf);
	if (ret)
		return ret;

	dev_dbg(core->dev, "CP2 reset register before release %#010x\n", val);
	val &= ~core->chip->reset_mask;
	ret = wcn_cp_write32(core, core->chip->reset_reg, val, buf);
	if (ret)
		return ret;

	ret = wcn_cp_wait_sync(core, sync_addr, true, buf);
	if (!ret)
		ret = wcn_mem_boot(core, sync_addr, buf);
	if (ret) {
		dev_err(core->dev, "calibration/MEM_PD handshake failed: %d\n", ret);
		return ret;
	}
	return wcn_cp_wait_sync(core, sync_addr, false, buf);
}

int wcn_fw_load(struct wcn_core *core, const char *name)
{
	const struct firmware *fw;
	const struct wcn_img_header *hdr;
	struct device *fwdev = core->btwf_dev ?: core->dev;
	const void *image;
	size_t image_len;
	size_t offset;
	void *buf;
	u32 base = core->chip->fw.base;
	u32 sync_addr = core->chip->fw.sync_addr;
	int ret;

	device_property_read_u32(fwdev, "sprd,btwf-base", &base);
	core->fw_base = base ?: core->chip->fw.base;

	device_property_read_u32(fwdev, "sprd,sync-addr", &sync_addr);
	if (sync_addr > U32_MAX - WCN_SYNC_DAC_OFFSET - sizeof(__le16) + 1)
		return -ERANGE;

	ret = request_firmware(&fw, name, core->dev);
	if (ret) {
		dev_err(core->dev, "failed to load %s: %d\n", name, ret);
		return ret;
	}

	buf = kmalloc(WCN_TX_CHUNK, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	if (fw->size < sizeof(u32)) {
		ret = -EINVAL;
		goto out_free;
	}

	if (!memcmp(fw->data, WCN_IMG_PACK_MAGIC, 4)) {
		if (fw->size < WCN_IMG_PACK_HEAD_SIZE) {
			ret = -EINVAL;
			goto out_free;
		}
		ret = wcn_fw_unpack(core, fw, &image, &image_len, buf);
		if (ret)
			goto out_free;
	} else if (get_unaligned_le32(fw->data) == WCN_SEC_IMAGE_MAGIC) {
		u32 real_size, signed_size;

		if (fw->size < sizeof(*hdr)) {
			ret = -EINVAL;
			goto out_free;
		}

		hdr = (const struct wcn_img_header *)fw->data;
		real_size = le32_to_cpu(hdr->img_real_size);
		signed_size = le32_to_cpu(hdr->img_signed_size);

		if (!real_size ||
		    signed_size > fw->size || signed_size <= sizeof(*hdr) ||
		    real_size >= signed_size - sizeof(*hdr)) {
			ret = -EINVAL;
			goto out_free;
		}

		image = fw->data + sizeof(*hdr);
		image_len = real_size;
	} else {
		image = fw->data;
		image_len = fw->size;
		/* The board image contains two Cortex-M slots. Load slot A. */
		if (core->chip->fw.container_size &&
		    fw->size == core->chip->fw.container_size)
			image_len = core->chip->fw.max_size;
	}

	if (!image_len || image_len > core->chip->fw.max_size) {
		dev_err(core->dev, "firmware image (%zu bytes) exceeds %s max (%u)\n",
			image_len, core->chip->name, core->chip->fw.max_size);
		ret = -EINVAL;
		goto out_free;
	}

	if (core->fw_base > U32_MAX ||
	    image_len - 1 > U32_MAX - core->fw_base) {
		dev_err(core->dev, "firmware at 0x%08llx (%zu bytes) exceeds CP address space\n",
			(u64)core->fw_base, image_len);
		ret = -ERANGE;
		goto out_free;
	}

	wcn_sdio_pub_irq_disable(core);
	cancel_work_sync(&core->pub_fault_work);
	wcn_mem_clear(core);
	ret = wcn_cp_hold(core, buf);
	if (!ret)
		ret = wcn_mem_prepare(core, buf);
	if (ret) {
		dev_err(core->dev, "failed to prepare CP2 for download: %d\n", ret);
		goto out_stop;
	}

	for (offset = 0; offset < image_len; offset += WCN_TX_CHUNK) {
		size_t chunk = min_t(size_t, image_len - offset, WCN_TX_CHUNK);

		memcpy(buf, (const u8 *)image + offset, chunk);
		ret = wcn_sdio_dt_write(core, core->fw_base + offset, buf, chunk);
		if (ret) {
			dev_err(core->dev,
				"failed to write firmware at 0x%08llx (%zu bytes): %d\n",
				(u64)core->fw_base + offset, chunk, ret);
			goto out_stop;
		}
	}

	ret = wcn_cp_boot(core, sync_addr, buf);
	if (ret) {
		dev_err(core->dev, "failed to boot WCN firmware: %d\n", ret);
		goto out_stop;
	}

	ret = wcn_sdio_cp_init(core);
	if (ret)
		goto out_stop;

	dev_info(core->dev, "loaded %s (%zu bytes) at 0x%08llx and booted\n",
		 name, image_len, (u64)core->fw_base);
	goto out_free;
out_stop:
	wcn_sdio_pub_irq_disable(core);
	cancel_work_sync(&core->pub_fault_work);
	if (wcn_cp_hold(core, buf))
		dev_err(core->dev, "failed to hold CP2 after boot failure\n");
	wcn_mem_clear(core);
out_free:
	kfree(buf);
out:
	release_firmware(fw);
	return ret;
}
