/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Defines for the sdio device driver
 */
#ifndef __SPRD_WCN_H__
#define __SPRD_WCN_H__

#include <linux/platform_device.h>

typedef int (*marlin_reset_callback) (void *para);
extern marlin_reset_callback marlin_reset_func;
extern void *marlin_callback_para;

struct wcn_match_data {
	bool unisoc_wcn_sdio;
	bool unisoc_wcn_slp;
	bool unisoc_wcn_m3lite; //UMW2652
	bool unisoc_wcn_m3; //SC2355
};

struct wcn_match_data *get_wcn_match_config(void);

int marlin_probe(struct platform_device *pdev);
int marlin_remove(struct platform_device *pdev);
void marlin_shutdown(struct platform_device *pdev);

int wcn_probe(struct platform_device *pdev);
int wcn_remove(struct platform_device *pdev);
void wcn_shutdown(struct platform_device *pdev);

void module_bus_sdio_init(void);
void module_bus_sdio_deinit(void);

#endif
