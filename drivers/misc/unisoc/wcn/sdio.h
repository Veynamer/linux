/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _UNISOC_WCN_SDIO_H
#define _UNISOC_WCN_SDIO_H

#include "core.h"

#define WCN_RX_BUF_SIZE		(16 * 1024)
#define WCN_TX_CHUNK		32768

#define WCN_SDIO_DT_MODE_ADDR		0x0f
#define WCN_SDIO_PK_MODE_ADDR		0x20
#define WCN_SDIO_CCCR_ABORT		0x06
#define WCN_SDIO_CCCR_ABORT_TRANS	0x01
#define WCN_SDIO_FBR_SYSADDR0		0x15c
#define WCN_SDIO_FBR_SYSADDR1		0x15d
#define WCN_SDIO_FBR_SYSADDR2		0x15e
#define WCN_SDIO_FBR_SYSADDR3		0x15f
#define WCN_SDIO_FBR_DEINT_EN		0x1ca
#define WCN_SDIO_DEINT_ENABLE		0x3

static inline unsigned int wcn_sdio_max_rx_slots(struct wcn_core *core)
{
	return (WCN_RX_BUF_SIZE - core->chip->sdio.block_size) /
		core->chip->sdio.packet_size;
}

int wcn_sdio_start(struct wcn_core *core);
void wcn_sdio_stop(struct wcn_core *core);
int wcn_sdio_dt_write(struct wcn_core *core, u32 addr,
		      const void *buf, size_t len);
int wcn_sdio_dt_read(struct wcn_core *core, u32 addr, void *buf, size_t len);
int wcn_sdio_packet_write(struct wcn_core *core, const void *buf, size_t len);
int wcn_sdio_packet_read(struct wcn_core *core, void *buf, size_t len);
size_t wcn_sdio_rx_size(struct wcn_core *core, u32 dtbs);
int wcn_sdio_enable_slave_irq(struct wcn_core *core);
int wcn_sdio_cp_init(struct wcn_core *core);
void wcn_sdio_irq_enable(struct wcn_core *core);
void wcn_sdio_irq_disable(struct wcn_core *core);
int wcn_sdio_aon_read(struct wcn_core *core, unsigned int addr, u8 *val);
int wcn_sdio_aon_write(struct wcn_core *core, unsigned int addr, u8 val);
int wcn_sdio_pub_prepare(struct wcn_core *core);
void wcn_sdio_pub_irq_disable(struct wcn_core *core);
void wcn_sdio_pub_fault_work(struct work_struct *work);
void wcn_sdio_rx_work(struct work_struct *work);

int wcn_sdio_configure(struct wcn_core *core);
int wcn_sdio_wake(struct wcn_core *core);

extern const struct dev_pm_ops wcn_sdio_pm_ops;

#endif
