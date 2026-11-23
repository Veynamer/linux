// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/limits.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "core.h"
#include "sdio.h"

#define WCN_MEM_SAVE_TIMEOUT_MS	2000
#define WCN_MEM_OPEN_TIMEOUT_MS	30000
#define WCN_MEM_LOCK_TIMEOUT_MS	50

static int wcn_mem_read32(struct wcn_core *core, u32 addr, u32 *val, void *buf)
{
	int ret = wcn_sdio_dt_read(core, addr, buf, sizeof(__le32));

	if (!ret)
		*val = get_unaligned_le32(buf);
	return ret;
}

static int wcn_mem_write32(struct wcn_core *core, u32 addr, u32 val, void *buf)
{
	put_unaligned_le32(val, buf);
	return wcn_sdio_dt_write(core, addr, buf, sizeof(__le32));
}

static int wcn_mem_update(struct wcn_core *core, u32 addr, u32 mask,
			  bool on, void *buf)
{
	u32 val;
	int ret = wcn_mem_read32(core, addr, &val, buf);

	if (ret)
		return ret;
	return wcn_mem_write32(core, addr, on ? val & ~mask : val | mask, buf);
}

static int wcn_mem_power(struct wcn_core *core, unsigned int subsys,
			 bool on, void *buf)
{
	const struct wcn_mem_data *mem = &core->chip->mem;
	unsigned long timeout = jiffies + msecs_to_jiffies(WCN_MEM_LOCK_TIMEOUT_MS);
	u32 val;
	int ret, unlock_ret;

	/* Reading zero claims the CP hardware semaphore, as in BSP. */
	for (;;) {
		ret = wcn_mem_read32(core, mem->semaphore, &val, buf);
		if (ret)
			return ret;
		if (!val)
			break;
		if (time_after_eq(jiffies, timeout)) {
			dev_err(core->dev, "CP2 RAM semaphore timeout\n");
			return -ETIMEDOUT;
		}
		usleep_range(100, 200);
	}

	if (subsys == WCN_CHILD_WLAN) {
		ret = wcn_mem_update(core, mem->clock_gate1, mem->wifi_iram_mask,
				     on, buf);
		if (!ret)
			ret = wcn_mem_update(core, mem->clock_gate2,
					     mem->wifi_dram_mask, on, buf);
	} else {
		ret = wcn_mem_update(core, mem->clock_gate2, mem->bt_dram_mask,
				     on, buf);
	}
	unlock_ret = wcn_mem_write32(core, mem->semaphore, mem->unlock_token, buf);
	return ret ?: unlock_ret;
}

void wcn_mem_clear(struct wcn_core *core)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(core->mem); i++) {
		kvfree(core->mem[i].image);
		memset(&core->mem[i], 0, sizeof(core->mem[i]));
		core->mem_error[i] = 0;
	}
	core->mem_pd = false;
	core->mem_active = 0;
}

int wcn_mem_prepare(struct wcn_core *core, void *buf)
{
	int ret;

	/* Forced RAM power-down can survive a CP-only reset. */
	ret = wcn_mem_power(core, WCN_CHILD_WLAN, true, buf);
	if (ret)
		return ret;
	return wcn_mem_power(core, WCN_CHILD_BT, true, buf);
}

static int wcn_mem_region_set(struct wcn_core *core,
			      struct wcn_mem_region *mem, u32 begin, u32 end)
{
	if (begin >= end || end > core->chip->mem.ram_size ||
	    end - 1 > U32_MAX - core->chip->mem.ram_base)
		return -EINVAL;
	mem->addr = core->chip->mem.ram_base + begin;
	mem->size = end - begin;
	return 0;
}

static int wcn_mem_copy(struct wcn_core *core, struct wcn_mem_region *mem,
			bool to_cp, void *buf)
{
	size_t off;
	int ret;

	/* kvmalloc images use a kmalloc bounce buffer for SDIO DMA. */
	for (off = 0; off < mem->size; off += WCN_TX_CHUNK) {
		size_t len = min_t(size_t, mem->size - off, WCN_TX_CHUNK);

		if (to_cp) {
			memcpy(buf, mem->image + off, len);
			ret = wcn_sdio_dt_write(core, mem->addr + off, buf, len);
		} else {
			ret = wcn_sdio_dt_read(core, mem->addr + off, buf, len);
			if (!ret)
				memcpy(mem->image + off, buf, len);
		}
		if (ret)
			return ret;
	}
	return 0;
}

int wcn_mem_boot(struct wcn_core *core, u32 sync_addr, void *buf)
{
	struct wcn_mem_region *wifi = &core->mem[WCN_CHILD_WLAN];
	struct wcn_mem_region *bt = &core->mem[WCN_CHILD_BT];
	u8 pending;
	unsigned int i;
	int ret;

	if (!wait_for_completion_timeout(&core->mem_save,
			msecs_to_jiffies(WCN_MEM_SAVE_TIMEOUT_MS)) &&
	    !(atomic_read(&core->pub_events) & WCN_PUB_MEM_SAVE)) {
		ret = READ_ONCE(core->pub_error);
		if (ret)
			return ret;
		ret = wcn_sdio_aon_read(core, core->chip->sdio.pub_int_status, &pending);
		if (ret)
			return ret;
		if (pending & WCN_PUB_MEM_SAVE) {
			dev_err(core->dev, "MEM_SAVE pending without PUB_INT delivery\n");
			return -ETIMEDOUT;
		}
		dev_info(core->dev, "firmware did not request MEM_PD\n");
		return 0;
	}
	ret = READ_ONCE(core->pub_error);
	if (ret)
		return ret;

	ret = wcn_sdio_dt_read(core, sync_addr + 4, buf, 4 * sizeof(__le32));
	if (ret)
		return ret;
	ret = wcn_mem_region_set(core, bt, get_unaligned_le32(buf),
				 get_unaligned_le32(buf + 4));
	if (!ret)
		ret = wcn_mem_region_set(core, wifi, get_unaligned_le32(buf + 8),
					 get_unaligned_le32(buf + 12));
	if (ret || (wifi->addr < bt->addr + bt->size &&
		    bt->addr < wifi->addr + wifi->size)) {
		dev_err(core->dev, "invalid MEM_PD ranges: %16ph\n", buf);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(core->mem); i++) {
		core->mem[i].image = kvmalloc(core->mem[i].size, GFP_KERNEL);
		if (!core->mem[i].image)
			return -ENOMEM;
		ret = wcn_mem_copy(core, &core->mem[i], false, buf);
		if (ret)
			return ret;
	}
	ret = wcn_mem_power(core, WCN_CHILD_WLAN, false, buf);
	if (!ret)
		ret = wcn_mem_power(core, WCN_CHILD_BT, false, buf);
	if (ret)
		return ret;
	ret = wcn_sdio_aon_write(core, core->chip->sdio.ap_int, WCN_AP_SAVE_MEM_DONE);
	if (ret)
		return ret;
	core->mem_pd = true;
	dev_info(core->dev, "MEM_PD saved WLAN %zu and BT %zu bytes\n",
		 wifi->size, bt->size);
	return 0;
}

int wcn_mem_start(struct wcn_core *core, unsigned int subsys)
{
	u8 ready = subsys == WCN_CHILD_WLAN ? WCN_AP_WIFI_BIN_READY :
					     WCN_AP_BT_BIN_READY;
	void *buf;
	u8 pending;
	int ret;

	if (!core->mem_pd || (core->mem_active & BIT(subsys)))
		return 0;
	if (core->mem_error[subsys])
		return core->mem_error[subsys];
	buf = kmalloc(WCN_TX_CHUNK, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = wcn_mem_power(core, subsys, true, buf);
	if (!ret)
		ret = wcn_mem_copy(core, &core->mem[subsys], true, buf);
	if (ret)
		goto out;
	reinit_completion(&core->subsys_open[subsys]);
	ret = READ_ONCE(core->pub_error);
	if (ret)
		goto out;
	ret = wcn_sdio_aon_write(core, core->chip->sdio.ap_int, ready);
	if (ret)
		goto out;
	if (!wait_for_completion_timeout(&core->subsys_open[subsys],
			msecs_to_jiffies(WCN_MEM_OPEN_TIMEOUT_MS))) {
		ret = wcn_sdio_aon_read(core, core->chip->sdio.pub_int_status, &pending);
		if (!ret) {
			dev_err(core->dev, "%s open timeout, PUB_INT status %#x\n",
				subsys == WCN_CHILD_WLAN ? "WLAN" : "BT", pending);
			ret = -ETIMEDOUT;
		}
	}
	if (READ_ONCE(core->pub_error))
		ret = READ_ONCE(core->pub_error);
	if (!ret)
		core->mem_active |= BIT(subsys);
out:
	/* Never rewrite a possibly running region after an ambiguous failure. */
	core->mem_error[subsys] = ret;
	kfree(buf);
	return ret;
}
