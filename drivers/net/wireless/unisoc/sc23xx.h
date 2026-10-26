// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Otto Pflüger
 */

#ifndef __SC23XX_H
#define __SC23XX_H

#include <linux/device.h>
#include <net/cfg80211.h>

#include "protocol.h"

#define SC23XX_MAX_SCAN_SSIDS		12
#define SC23XX_MAX_SCAN_IE_LEN		2304
#define SC23XX_MAX_PFN_LIST_COUNT	9
#define SC23XX_MAX_NUM_PMKIDS		4
#define SC23XX_MAX_TID_NUM		16
#define SC23XX_TX_BA_WIN_SIZE		64

/* Number of TX flow-control "colors" the firmware grants credit for.
 * Only used on chips reporting credit_capa == SC23XX_TX_WITH_CREDIT.
 */
#define SC23XX_TX_COLORS		4
#define SC23XX_TX_WITH_CREDIT		0

#define SC23XX_CAPA_5G			BIT(0)
#define SC23XX_CAPA_MCC			BIT(1)
#define SC23XX_CAPA_ACL			BIT(2)
#define SC23XX_CAPA_AP_SME		BIT(3)
#define SC23XX_CAPA_PMK_OKC_OFFLOAD	BIT(4)
#define SC23XX_CAPA_11R_ROAM_OFFLOAD	BIT(5)
#define SC23XX_CAPA_SCHED_SCAN		BIT(6)
#define SC23XX_CAPA_TDLS		BIT(7)
#define SC23XX_CAPA_MC_FILTER		BIT(8)
#define SC23XX_CAPA_NS_OFFLOAD		BIT(9)
#define SC23XX_CAPA_RA_OFFLOAD		BIT(10)
#define SC23XX_CAPA_LL_STATS		BIT(11)

enum {
	SC23XX_VIF_STA = 0,
	SC23XX_VIF_AP,
	SC23XX_VIF_P2P_DEVICE,
	SC23XX_VIF_NUM,
};

enum {
	SC23XX_STA_IDX_AP_MULTICAST = 4,
	SC23XX_STA_IDX_P2P_GROUP = 5,
	SC23XX_STA_IDX_MIN = 6,
	SC23XX_STA_IDX_MAX = 31,
	SC23XX_STA_IDX_NUM = SC23XX_STA_IDX_MAX - SC23XX_STA_IDX_MIN + 1,
};

enum sc23xx_msg_type {
	SC23XX_MSG_TYPE_CMD,
	SC23XX_MSG_TYPE_DATA,
};

struct sc23xx_dev;
struct sc23xx_vif;

struct sc23xx_bus_ops {
	int (*tx_cmd)(struct sc23xx_dev *dev, struct sk_buff *skb);
	void (*tx_data)(struct sc23xx_dev *dev, struct sk_buff *skb);
	/*
	 * Optional: nudge the transport after the core replenishes TX credit,
	 * so a transport that parked its TX path on credit starvation can
	 * resume. NULL for transports without credit-based flow control.
	 */
	void (*tx_kick)(struct sc23xx_dev *dev);
};

struct sc23xx_reorder_data {
	struct sc23xx_dev *sdev;
	struct timer_list timer;
	u16 win_size;
	u16 seq_start;
	u16 seq_last;
	u16 buf_pos;
	struct list_head *buf;
};

struct sc23xx_sta {
	struct sc23xx_dev *sdev;
	bool valid;
	bool ht_enabled;
	u8 addba_retries;
	u8 ctx_id;
	u8 addr[ETH_ALEN];
	struct sc23xx_reorder_data r[SC23XX_MAX_TID_NUM];
	struct delayed_work tx_ba_setup;
};

enum sc23xx_pwr_state {
	SC23XX_PWR_HOST_IDLE,
	SC23XX_PWR_ACTIVE,
	SC23XX_PWR_SUSPENDED,
};

struct sc23xx_dev {
	struct wiphy *wiphy;
	const struct sc23xx_bus_ops *bus_ops;
	bool use_dma;

	enum sc23xx_pwr_state pwr_state;
	struct mutex pwr_state_lock;

	char country_code[2];

	/* TX credit capability from firmware GET_INFO: 0 = with credit
	 * (color/flow-control required), 1 = no credit.
	 */
	u8 credit_capa;

	u8 mac_addr[ETH_ALEN];
	bool mac_addr_random;
	struct ieee80211_supported_band band_2ghz;
	struct ieee80211_supported_band band_5ghz;
	u8 max_mc_mac_addrs;

	struct mutex cmd_lock;
	struct sk_buff_head rsp_q;
	wait_queue_head_t rsp_wait;
	struct workqueue_struct *evt_wq;

	spinlock_t tx_buf_lock;
	struct list_head tx_buf_cache;
	DECLARE_HASHTABLE(tx_buf_table, 4);
	unsigned int tx_buf_count;
	bool tx_blocked;

	spinlock_t tx_queue_lock;
	struct list_head tx_queue;
	wait_queue_head_t tx_wait;
	struct task_struct *tx_thread;

	spinlock_t rx_buf_lock;
	DECLARE_HASHTABLE(rx_buf_table, 4);
	atomic_t rx_buf_count;

	struct mutex vif_lock;
	struct srcu_struct vif_srcu;
	struct sc23xx_vif __rcu *vif[SC23XX_VIF_NUM];
	atomic_t open_count;

	spinlock_t sta_lock;
	struct sc23xx_sta sta[SC23XX_STA_IDX_NUM];

	/* Credit-based TX flow control (credit_capa == SC23XX_TX_WITH_CREDIT).
	 * The firmware grants per-color credits via EVT_SDIO_FLOWCON; each data
	 * frame consumes one credit and is stamped with its color + a running
	 * sequence number.
	 */
	atomic_t tx_credit[SC23XX_TX_COLORS];
	atomic_t tx_seq;
};

void *sc23xx_alloc_device(struct device *dev, size_t size,
			  const struct sc23xx_bus_ops *bus_ops);
void sc23xx_free_device(struct sc23xx_dev *sdev);
int sc23xx_load_config(struct sc23xx_dev *sdev, const char *fw_name);
int sc23xx_load_hw_param(struct sc23xx_dev *sdev, const char *fw_name);
int sc23xx_sync_version(struct sc23xx_dev *sdev);

/* Reserve one TX credit and return its color (0..SC23XX_TX_COLORS-1), or
 * -EBUSY if no color has credit. Used by the credit-based (SDIO) transport.
 */
int sc23xx_tx_reserve_credit(struct sc23xx_dev *sdev);
void sc23xx_tx_credit_add(struct sc23xx_dev *sdev, const u8 *flow);

/* True if a data frame can be sent now: either the chip needs no credit, or at
 * least one color has a credit to spend. Used by the credit-based transport to
 * decide whether to drain its TX backlog or park until the next grant. */
static inline bool sc23xx_tx_credit_ready(struct sc23xx_dev *sdev)
{
	int i;

	if (sdev->credit_capa != SC23XX_TX_WITH_CREDIT)
		return true;

	for (i = 0; i < SC23XX_TX_COLORS; i++)
		if (atomic_read(&sdev->tx_credit[i]) > 0)
			return true;

	return false;
}

/* Wake (or stop) the netdev TX queues of every open interface. */
void sc23xx_netif_tx(struct sc23xx_dev *sdev, bool wake);

static inline u16 sc23xx_tx_seq_info(struct sc23xx_dev *sdev, int color)
{
	u8 seq = atomic_inc_return(&sdev->tx_seq);

	/* packs as color_bit:2, seq_num:8 in the tx_data_hdr seq_info field */
	return (color & 0x3) | (seq << 2);
}
int sc23xx_register_device(struct sc23xx_dev *sdev);
void sc23xx_unregister_device(struct sc23xx_dev *sdev);

int sc23xx_set_suspend(struct sc23xx_dev *sdev, bool suspend);

void sc23xx_wakeup_fw(struct sc23xx_dev *sdev);

void sc23xx_rx_msg(struct sc23xx_dev *sdev, enum sc23xx_msg_type type,
		   struct sk_buff *skb);

void sc23xx_tx_data_dma(struct sc23xx_dev *sdev, struct sk_buff *skb);

int sc23xx_rx_fill_addr_list(struct sc23xx_dev *sdev,
			     struct sc23xx_rx_addr_free_list *list,
			     unsigned int *len, unsigned int max_count);
int sc23xx_tx_fill_addr_list(struct sc23xx_dev *sdev,
			     struct sc23xx_tx_addr_list *list,
			     unsigned int *len, unsigned int max_count);

bool sc23xx_rx_fill_needed(struct sc23xx_dev *sdev);

static inline bool sc23xx_tx_fill_needed(struct sc23xx_dev *sdev)
{
	return !list_empty(&sdev->tx_queue);
}

static inline int sc23xx_tx_cmd(struct sc23xx_dev *sdev, struct sk_buff *skb)
{
	return sdev->bus_ops->tx_cmd(sdev, skb);
}

static inline void sc23xx_tx_data(struct sc23xx_dev *sdev, struct sk_buff *skb)
{
	return sdev->bus_ops->tx_data(sdev, skb);
}

static inline bool sc23xx_channel_is_2ghz(int channel)
{
	return channel >= 1 && channel <= 14;
}

static inline int sc23xx_channel_to_freq(int chan)
{
	enum nl80211_band band = sc23xx_channel_is_2ghz(chan) ?
		NL80211_BAND_2GHZ : NL80211_BAND_5GHZ;

	return ieee80211_channel_to_frequency(chan, band);
}

#endif
