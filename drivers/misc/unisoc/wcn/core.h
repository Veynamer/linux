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

#include "hw.h"
#include "protocol.h"

struct wcn_core;
struct wcn_chan;
struct platform_device;

struct wcn_mem_region {
	u32 addr;
	size_t size;
	void *image;
};

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
	const struct wcn_chip_data *chip;
	struct mutex lock; /* protects chans[] */
	struct mutex power_lock; /* protects power_users and fw_loaded */
	struct mutex irq_lock; /* protects data_irq and irq_enabled */
	struct mutex rx_lock; /* serializes RX buffer use and delivery */
	struct mutex io_lock; /* packet I/O barrier during boot/reset/remove */
	struct mutex sleep_lock; /* CP sleep state and in-flight bus users */
	unsigned int io_users;
	bool sleep_enabled;
	bool sleep_requested;
	bool cp_sleeping;
	bool system_suspended;
	bool data_was_enabled;
	bool pub_was_enabled;
	bool wake_armed;
	unsigned long wake_pending;
	bool power_ref;
	int power_error;
	struct work_struct sleep_work;
	struct wcn_chan *chans[WCN_CHAN_NUM];
	unsigned int power_users;
	unsigned int subsys_users[WCN_CHILD_MAX];
	bool fw_loaded;
	bool tx_ready;
	bool removing;
	bool started;
	bool irq_enabled;
	bool pub_irq_enabled;
	int pub_error;
	atomic_t pub_events;
	struct completion mem_save;
	struct completion subsys_open[2];
	struct wcn_mem_region mem[2];
	bool mem_pd;
	bool rx_packet_mode;
	bool tx_packet_mode;
	unsigned int mem_active;
	int mem_error[2];
	struct work_struct rx_work;
	struct work_struct reset_work;
	struct work_struct pub_fault_work;
	struct blocking_notifier_head reset_chain;
	atomic_t reset_pending;
	u8 *rx_buf;
	size_t rx_size;
	int data_irq;
	int pub_irq;
	struct gpio_desc *data_gpio;
	struct gpio_desc *pub_gpio;
	struct wcn_chan *assert_chan;
	struct wcn_chan *at_chan;
	phys_addr_t fw_base;
	struct device *btwf_dev;
	struct device *gnss_dev;
	struct platform_device *child_pdevs[WCN_CHILD_MAX];
	struct dentry *debugfs;
	bool gnss_prepared;
	bool gnss_powered;
	bool gnss_running;
	bool gnss_cali_valid;
	u8 gnss_cali[32];
	u8 gnss_efuse[12];
	void *gnss_image;
	size_t gnss_image_size;
};

struct wcn_core *wcn_core_alloc(struct device *dev, struct sdio_func *func,
				const struct wcn_chip_data *chip);
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
int wcn_fw_stop(struct wcn_core *core);
int wcn_mem_boot(struct wcn_core *core, u32 sync_addr, void *buf);
int wcn_mem_prepare(struct wcn_core *core, void *buf);
int wcn_mem_start(struct wcn_core *core, unsigned int subsys);
void wcn_mem_clear(struct wcn_core *core);

/* power_lock and io_lock held; CP is awake after firmware reload. */
int wcn_gnss_restart_locked(struct wcn_core *core);
int wcn_coexist_update_locked(struct wcn_core *core, unsigned int subsys,
			      bool enable);

int wcn_children_create(struct wcn_core *core);
void wcn_children_destroy(struct wcn_core *core);

void wcn_pm_init(struct wcn_core *core);
void wcn_pm_disable(struct wcn_core *core);
void wcn_pm_enable(struct wcn_core *core);
void wcn_pm_reset(struct wcn_core *core);
int wcn_pm_get(struct wcn_core *core);
void wcn_pm_put(struct wcn_core *core);
void wcn_pm_request_sleep(struct wcn_core *core);

#endif
