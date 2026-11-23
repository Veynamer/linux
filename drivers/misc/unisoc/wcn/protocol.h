/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UNISOC_WCN_PROTOCOL_H
#define _UNISOC_WCN_PROTOCOL_H

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/types.h>

#define WCN_CHAN_TX_NUM		12
#define WCN_CHAN_RX_NUM		14
#define WCN_CHAN_NUM		(WCN_CHAN_TX_NUM + WCN_CHAN_RX_NUM)

struct wcn_puh {
	u32 pad:6;
	u32 check_sum:1;
	u32 len:16;
	u32 eof:1;
	u32 subtype:4;
	u32 type:4;
} __packed;

static_assert(sizeof(struct wcn_puh) == 4);

#define WCN_SYNC_DAC_OFFSET		24
#define WCN_SYNC_DAC_DEFAULT		0xffff
#define WCN_BTWF_IN_DEEPSLEEP		0x00
#define WCN_BTWF_XLT_WAIT		0x10
#define WCN_BTWF_XLTBUF_WAIT		0x20
#define WCN_BTWF_IN_DEEPSLEEP_XTL_ON	0x30
#define WCN_BTWF_PLL_PWR_WAIT		0x40
#define WCN_AP_ALLOW_SLEEP		BIT(0)
#define WCN_AP_WIFI_BIN_READY		BIT(1)
#define WCN_AP_BT_BIN_READY		BIT(2)
#define WCN_AP_SAVE_MEM_DONE		BIT(3)
#define WCN_PUB_MEM_SAVE			BIT(0)
#define WCN_PUB_WAKE_ACK			BIT(1)
#define WCN_PUB_REQ_SLEEP			BIT(2)
#define WCN_PUB_WIFI_OPEN			BIT(3)
#define WCN_PUB_BT_OPEN			BIT(4)
#define WCN_PUB_WIFI_CLOSE		BIT(5)
#define WCN_PUB_BT_CLOSE			BIT(6)
#define WCN_PUB_BTWF_MASK			GENMASK(6, 0)
#define WCN_SYNC_IN_PROGRESS		0xf0f0f0f0
#define WCN_SYNC_CALI_WAITING		0xf0f0f0f1
#define WCN_SYNC_CALI_WRITE_DONE	0xf0f0f0f2
#define WCN_SYNC_ALL_FINISHED		0xf0f0f0ff

#endif
