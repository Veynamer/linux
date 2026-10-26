# SPDX-License-Identifier: GPL-2.0
#
# Unisoc Marlin3-Lite (SC2355) external WCN combo over SDIO.
# Trimmed from the 5.4 vendor BSP to the external-SDIO path only:
# integrated-chip boot (wcn_integrate*), SIPC and PCIe transports removed.
#
wcn_srcdir := $(srctree)/drivers/net/wireless/sprdwcn
wcn_incdirs := \
	-I $(wcn_srcdir)/include \
	-I $(wcn_srcdir)/boot \
	-I $(wcn_srcdir)/platform \
	-I $(wcn_srcdir)/platform/rf \
	-I $(wcn_srcdir)/sleep

ccflags-y += $(wcn_incdirs)
subdir-ccflags-y += $(wcn_incdirs)

obj-$(CONFIG_UNISOC_WCN_BSP) += wcn_bsp.o

# core bus + unified platform-driver entry
wcn_bsp-y += wcn_bus.o
wcn_bsp-y += sprd_wcn.o

# platform (external marlin3 boot + glue)
wcn_bsp-y += platform/wcn_parn_parser.o
wcn_bsp-y += platform/wcn_txrx.o
wcn_bsp-y += platform/wcn_log.o
wcn_bsp-y += platform/wcn_misc.o
wcn_bsp-y += platform/bufring.o
wcn_bsp-y += platform/wcn_op.o
wcn_bsp-y += platform/sysfs.o
wcn_bsp-y += platform/reset.o
wcn_bsp-y += platform/loopcheck.o
wcn_bsp-y += platform/wcn_boot.o
wcn_bsp-y += platform/mem_pd_mgr.o
wcn_bsp-y += platform/wcn_procfs.o
# Stubs for the remaining integrated-only symbols.
wcn_bsp-y += platform/wcn_stubs.o

# sleep / power
wcn_bsp-y += sleep/sdio_int.o
wcn_bsp-y += sleep/slp_mgr.o
wcn_bsp-y += sleep/slp_sdio.o

# sdio transport (sdiohal)
wcn_bsp-y += sdio/sdiohal_main.o
wcn_bsp-y += sdio/sdiohal_common.o
wcn_bsp-y += sdio/sdiohal_tx.o
wcn_bsp-y += sdio/sdiohal_rx.o
wcn_bsp-y += sdio/sdiohal_if.o
