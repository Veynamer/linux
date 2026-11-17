/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UNISOC_WCN_CORE_H
#define _UNISOC_WCN_CORE_H

#include <linux/build_bug.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/misc/unisoc-wcn.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/refcount.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>

#define WCN_CHAN_TX_NUM		12
#define WCN_CHAN_RX_NUM		14
#define WCN_CHAN_NUM		(WCN_CHAN_TX_NUM + WCN_CHAN_RX_NUM)

#define WCN_BLK_SIZE		840
#define WCN_MAX_PAC_SIZE	(WCN_BLK_SIZE * 2)
#define WCN_RX_BUF_SIZE		(16 * 1024)
#define WCN_TX_CHUNK		32768

struct wcn_core;
struct wcn_chan;
struct platform_device;

struct wcn_puh {
	u32 pad:6;
	u32 check_sum:1;
	u32 len:16;
	u32 eof:1;
	u32 subtype:4;
	u32 type:4;
} __packed;

static_assert(sizeof(struct wcn_puh) == 4);

struct wcn_chan {
	struct wcn_core *core;
	unsigned int id;
	bool tx;
	bool closing;
	struct wcn_chan_ops ops;
	refcount_t refs;
	struct completion idle;
	void *priv;
	u64 rx_packets;
	u64 tx_packets;
	u64 rx_dropped;
	u64 tx_dropped;
};

struct wcn_core {
	struct device *dev;
	struct sdio_func *func;
	struct mutex lock; /* protects chans[] */
	struct mutex power_lock; /* protects power_users and fw_loaded */
	struct mutex irq_lock; /* protects data_irq and irq_enabled */
	struct wcn_chan *chans[WCN_CHAN_NUM];
	unsigned int power_users;
	bool fw_loaded;
	bool started;
	bool irq_enabled;
	struct work_struct rx_work;
	struct work_struct reset_work;
	struct atomic_notifier_head reset_chain;
	atomic_t reset_pending;
	u8 *rx_buf;
	int data_irq;
	int pub_irq;
	struct gpio_desc *data_gpio;
	struct wcn_chan *assert_chan;
	struct wcn_chan *at_chan;
	phys_addr_t fw_base;
	struct device *btwf_dev;
	struct device *gnss_dev;
	struct platform_device *child_pdevs[WCN_CHILD_MAX];
	struct dentry *debugfs;
};

struct wcn_core *wcn_core_alloc(struct device *dev, struct sdio_func *func);
void wcn_core_free(struct wcn_core *core);
int wcn_core_start(struct wcn_core *core);
void wcn_core_stop(struct wcn_core *core);
int wcn_core_assert_start(struct wcn_core *core);
void wcn_core_assert_stop(struct wcn_core *core);
void wcn_core_at_start(struct wcn_core *core);
void wcn_core_at_stop(struct wcn_core *core);

void wcn_chan_rx(struct wcn_core *core, unsigned int id,
		 const void *data, size_t len);

int wcn_fw_load(struct wcn_core *core, const char *name);

int wcn_children_create(struct wcn_core *core);
void wcn_children_destroy(struct wcn_core *core);

int wcn_sdio_dt_write(struct wcn_core *core, u32 addr,
		      const void *buf, size_t len);
int wcn_sdio_dt_read(struct wcn_core *core, u32 addr, void *buf, size_t len);
int wcn_sdio_packet_write(struct wcn_core *core, const void *buf, size_t len);
int wcn_sdio_packet_read(struct wcn_core *core, void *buf, size_t len);
int wcn_sdio_enable_slave_irq(struct wcn_core *core);
int wcn_sdio_cp_init(struct wcn_core *core);
void wcn_sdio_irq_enable(struct wcn_core *core);
void wcn_sdio_irq_disable(struct wcn_core *core);
void wcn_sdio_rx_work(struct work_struct *work);

#endif
