// SPDX-License-Identifier: GPL-2.0-only
/* Marlin3lite (UMW2652) hardware and firmware interface data. */

#include <linux/bits.h>
#include <linux/kernel.h>

#include "hw.h"

static const struct wcn_fw_id marlin3lite_fw_ids[] = {
	{ .chip_id = 0x2355b000, .tag = "MLAA" },
	{ .chip_id = 0x2355b001, .tag = "MLAB" },
	{ .chip_id = 0x2355b002, .tag = "MLAC" },
	{ .chip_id = 0x2355b003, .tag = "MLAC" },
};

const struct wcn_chip_data wcn_marlin3lite_data = {
	.name = "Marlin3lite",
	.fw = {
		.name = "sprd/wcnmodem.bin",
		.base = 0x40500000,
		.max_size = 0xe7400,
		.container_size = 0x300000,
		.sync_addr = 0x405e73b0,
		.chip_id_reg = 0x4083c208,
		.ids = marlin3lite_fw_ids,
		.num_ids = ARRAY_SIZE(marlin3lite_fw_ids),
	},
	.mem = {
		.ram_base = 0x40400000,
		.ram_size = 0x200000,
		.clock_gate1 = 0x4083c10c,
		.clock_gate2 = 0x4083c110,
		.wifi_iram_mask = 0xfc00,
		.wifi_dram_mask = 0x80,
		.bt_dram_mask = 0xc00,
		.semaphore = 0x40850800,
		.unlock_token = 0x55aa10c5,
	},
	.sdio = {
		.block_size = 840,
		.packet_size = 1680,
		.sleep_ctl = 0x1a2,
		.sleep_status = 0x148,
		.ap_int = 0x1b0,
		.pub_int_en = 0x1c0,
		.pub_int_clr = 0x1d0,
		.pub_int_status = 0x1f0,
	},
	.gnss = {
		.base = 0x40a50000,
		.max_size = 0x2b000,
		.log_offset = 0x88000,
		.reset_reg = 0x40bc8280,
		.reset_mask = BIT(0),
		.power_reg = 0x4083c0c8,
		.force_sleep = BIT(3),
		.auto_enable = BIT(12),
		.chip_sleep = BIT(1),
		.power_status = 0x4083c00c,
		.power_ready = BIT(12) | BIT(13),
		.fake_clock = 0x40844304,
		.fake_clock_mask = 0x3,
		.apb_clock = 0x4084431c,
		.apb_clock_mask = BIT(0),
		.cali_addr = 0x40aabf4c,
		.cali_size = 0x1c,
		.cali_done = 0x01314520,
		.efuse_addr = 0x40aabf40,
		.boot_addr = 0x40aabf6c,
		.boot_done = 0x12345678,
	},
	.fm_spur_reg = 0x40098104,
	.fm_spur_enable = 0x06dcab7c,
	.fm_spur_disable = 0x06dc063c,
	.reset_reg = 0x40088288,
	.reset_mask = BIT(0),
	.data_pin_reg = 0x40840014,
	.data_pin_mask = BIT(8),
	.at_rx_port = 13,
	.assert_rx_port = 14,
	.children = {
		[WCN_CHILD_WLAN] = {
			.name = "wlan",
			.data = {
				.channels = {
					.tx_cmd = 8,
					.tx_data = 10,
					.rx_cmd = 22,
					.rx_data = 24,
					.rx_log = 23,
				},
				.mc_filter_no_empty = true,
			},
		},
		[WCN_CHILD_BT] = {
			.name = "bluetooth",
			.data = {
				.channels = {
					.tx_data = 3,
					.rx_data = 17,
				},
				.pskey_name = "sprd/bt_configure_pskey.ini",
				.rf_name = "sprd/bt_configure_rf.ini",
			},
		},
		[WCN_CHILD_FM] = {
			.name = "fm",
			.data = {
				.channels = {
					.tx_cmd = 6,
					.rx_cmd = 20
				},
				.firmware_name = "sprd/fm_board_config.ini",
			},
		},
		[WCN_CHILD_GNSS] = {
			.name = "gnss",
			.data = { .firmware_name = "sprd/gnssmodem.bin" },
		},
	},
};
