/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_MISC_UNISOC_WCN_H
#define __LINUX_MISC_UNISOC_WCN_H

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/skbuff.h>
#include <linux/types.h>

struct wcn_core;
struct wcn_chan;

#define WCN_TX_BATCH_MAX	16

struct wcn_tx_packet {
	const void *data;
	size_t len;
};

#define WCN_CHILD_MAX		4
#define WCN_CHILD_WLAN		0
#define WCN_CHILD_BT		1
#define WCN_CHILD_FM		2
#define WCN_CHILD_GNSS		3

struct wcn_channel_map {
	u8 tx_cmd;
	u8 tx_data;
	u8 rx_cmd;
	u8 rx_data;
	u8 rx_log;
};

/* Immutable, variant-specific information for the function drivers. */
struct wcn_child_data {
	struct wcn_channel_map channels;
	const char *firmware_name;
	const char *pskey_name;
	const char *rf_name;
	bool mc_filter_no_empty;
};

struct wcn_child_pdata {
	struct wcn_core *core;
	const struct wcn_child_data *data;
};

struct wcn_chan_ops {
	/*
	 * RX runs in process context and transfers skb ownership to the
	 * callback. Do not call channel or power lifecycle operations here.
	 */
	void (*rx)(struct wcn_chan *chan, struct sk_buff *skb);
};

/* ops and immutable priv are published together before RX can start. */
struct wcn_chan *wcn_chan_open(struct wcn_core *core, unsigned int id,
			       const struct wcn_chan_ops *ops, void *priv);
/* Stop the caller's TX producers before closing their channel. */
void wcn_chan_close(struct wcn_chan *chan);
int wcn_chan_tx(struct wcn_chan *chan, const void *data, size_t len);
/* Synchronous: packets stay owned by the caller, including on failure. */
int wcn_chan_tx_batch(struct wcn_chan *chan, const struct wcn_tx_packet *packets,
		      unsigned int num);
void *wcn_chan_get_drvdata(struct wcn_chan *chan);

/*
 * The first user resumes SDIO, loads firmware and opens its CP subsystem.
 * The last put drains I/O and stops CP2 before releasing MMC runtime power.
 * CP idle sleep and wakeup are handled by the transport while users exist.
 * Call these from process context, outside channel RX callbacks.
 */
int wcn_power_get(struct wcn_core *core, unsigned int subsys);
void wcn_power_put(struct wcn_core *core, unsigned int subsys);

/* GNSS remoteproc operations; firmware bytes remain owned by the caller. */
int wcn_gnss_validate_fw(struct wcn_core *core, const void *data, size_t size);
int wcn_gnss_prepare(struct wcn_core *core);
int wcn_gnss_load(struct wcn_core *core, const void *data, size_t size);
int wcn_gnss_start(struct wcn_core *core);
int wcn_gnss_stop(struct wcn_core *core);
int wcn_gnss_unprepare(struct wcn_core *core);

enum wcn_reset_event {
	WCN_RESET_PREPARE,
	WCN_RESET_DONE,
	WCN_RESET_FAILED,
};

/* Synchronous notifier payload; valid only for the callback duration. */
struct wcn_reset_info {
	struct wcn_core *core;
	int error;
	bool rebind;
	bool ready;
};

int wcn_reset_notifier_register(struct wcn_core *core,
				struct notifier_block *nb);
int wcn_reset_notifier_unregister(struct wcn_core *core,
				  struct notifier_block *nb);
int wcn_core_reset(struct wcn_core *core);

#endif
