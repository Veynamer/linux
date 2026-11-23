/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UNISOC_WCN_HW_H
#define _UNISOC_WCN_HW_H

#include <linux/misc/unisoc-wcn.h>
#include <linux/types.h>

struct wcn_fw_id {
	u32 chip_id;
	const char *tag;
};

struct wcn_fw_data {
	const char *name;
	u32 base;
	u32 max_size;
	u32 container_size;
	u32 sync_addr;
	u32 chip_id_reg;
	const struct wcn_fw_id *ids;
	size_t num_ids;
};

struct wcn_mem_data {
	u32 ram_base;
	u32 ram_size;
	u32 clock_gate1;
	u32 clock_gate2;
	u32 wifi_iram_mask;
	u32 wifi_dram_mask;
	u32 bt_dram_mask;
	u32 semaphore;
	u32 unlock_token;
};

struct wcn_sdio_data {
	u16 block_size;
	u16 packet_size;
	u16 sleep_ctl;
	u16 sleep_status;
	u16 ap_int;
	u16 pub_int_en;
	u16 pub_int_clr;
	u16 pub_int_status;
};

struct wcn_child_info {
	const char *name;
	struct wcn_child_data data;
};

struct wcn_gnss_data {
	u32 base;
	u32 max_size;
	u32 log_offset; /* optional host log dictionary offset in the unpacked image */
	u32 reset_reg;
	u32 reset_mask;
	u32 power_reg;
	u32 force_sleep;
	u32 auto_enable;
	u32 chip_sleep;
	u32 power_status;
	u32 power_ready;
	u32 fake_clock;
	u32 fake_clock_mask;
	u32 apb_clock;
	u32 apb_clock_mask;
	u32 cali_addr;
	u32 cali_size;
	u32 cali_done;
	u32 efuse_addr;
	u32 boot_addr;
	u32 boot_done;
};

/* Selected by the SDIO function's OF match; never modified at runtime. */
struct wcn_chip_data {
	const char *name;
	struct wcn_fw_data fw;
	struct wcn_mem_data mem;
	struct wcn_sdio_data sdio;
	struct wcn_gnss_data gnss;
	u32 fm_spur_reg;
	u32 fm_spur_enable;
	u32 fm_spur_disable;
	u32 reset_reg;
	u32 reset_mask;
	u32 data_pin_reg;
	u32 data_pin_mask;
	u8 at_rx_port;
	u8 assert_rx_port;
	struct wcn_child_info children[WCN_CHILD_MAX];
};

extern const struct wcn_chip_data wcn_marlin3lite_data;

#endif
