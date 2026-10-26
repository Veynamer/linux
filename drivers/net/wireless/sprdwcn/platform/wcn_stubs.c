// SPDX-License-Identifier: GPL-2.0
/*
 * Stubs for WCN subsystems dropped from the boot-milestone build of the
 * external SDIO Marlin3-Lite (SC2355) driver:
 *
 *  - RF calibration (rf) and the whole SoC-integrated WCN path
 *    (wcn_integrate*), which is never taken on this external-SDIO board.
 *
 * Every symbol below is referenced only on runtime-guarded integrated paths or
 * on debug/diagnostic paths that are inactive during bring-up. Providing no-op
 * definitions lets the trimmed module link. Re-add the corresponding source
 * files to the Makefile to restore real functionality.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include "wcn_glb.h"
#include "wcn_boot.h"
#include "wcn_procfs.h"
#include "../boot/wcn_integrate_dev.h"

/* ---- data symbols (were defined in the integrated core) ---------------- */
int is_wcn_shutdown;
struct wcn_device_manage s_wcn_device;

/* ---- integrated WCN core (wcn_integrate*): guarded-out on external SDIO - */
int start_integ_marlin(u32 subsys) { return 0; }
int stop_integ_marlin(u32 subsys) { return 0; }
int integ_marlin_get_power(void) { return 0; }
int integ_marlin_get_module_status(void) { return 0; }
void integ_wcn_chip_power_off(void) { }
const char *integ_wcn_get_chip_name(void) { return ""; }
int integ_wcn_get_module_status_changed(void) { return 0; }
void integ_wcn_set_module_status_changed(bool status) { }
enum wcn_clock_mode integ_wcn_get_xtal_26m_clk_mode(void) { return WCN_CLOCK_MODE_UNKNOWN; }
enum wcn_clock_type integ_wcn_get_xtal_26m_clk_type(void) { return WCN_CLOCK_TYPE_UNKNOWN; }
