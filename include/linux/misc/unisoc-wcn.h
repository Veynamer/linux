/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_MISC_UNISOC_WCN_H
#define __LINUX_MISC_UNISOC_WCN_H

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/skbuff.h>
#include <linux/types.h>

struct wcn_core;
struct wcn_chan;

#define WCN_CHILD_MAX		4
#define WCN_CHILD_WLAN		0
#define WCN_CHILD_BT		1
#define WCN_CHILD_FM		2
#define WCN_CHILD_GNSS		3

struct wcn_child_pdata {
	struct wcn_core *core;
};

struct wcn_chan_ops {
	void (*rx)(struct wcn_chan *chan, struct sk_buff *skb);
};

struct wcn_chan *wcn_chan_open(struct wcn_core *core, unsigned int id,
			       const struct wcn_chan_ops *ops);
void wcn_chan_close(struct wcn_chan *chan);
int wcn_chan_tx(struct wcn_chan *chan, const void *data, size_t len);
void wcn_chan_set_drvdata(struct wcn_chan *chan, void *data);
void *wcn_chan_get_drvdata(struct wcn_chan *chan);

int wcn_power_get(struct wcn_core *core);
void wcn_power_put(struct wcn_core *core);

enum wcn_reset_event {
	WCN_RESET_PREPARE,
	WCN_RESET_DONE,
};

int wcn_reset_notifier_register(struct wcn_core *core,
				struct notifier_block *nb);
int wcn_reset_notifier_unregister(struct wcn_core *core,
				  struct notifier_block *nb);
int wcn_core_reset(struct wcn_core *core);

#endif
