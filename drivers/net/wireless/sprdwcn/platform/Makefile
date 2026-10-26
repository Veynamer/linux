# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Unisoc wcn platform configuration
#

#common obj
#WCN buf log loopcheck ...
WCN_MARLIN_COMMON_FILES += bufring.o \
			   wcn_txrx.o \
			   wcn_misc.o \
			   wcn_parn_parser.o \
			   wcn_log.o \
			   wcn_procfs.o \
			   loopcheck.o \
			   reset.o
#			   sysfs.o

#common obj end

ifeq ($(TARGET_BUILD_VARIANT),user)
ccflags-y += -DFLAG_WCN_USER
endif

obj-y := marlin.o
marlin-y += pcie_boot.o
marlin-y += wcn_swd_dap.o

#marlin3:sdio pcie
MARLINOFILES += wcn_boot.o \
		wcn_op.o

marlin-y += mem_pd_mgr.o

marlin-objs += $(WCN_MARLIN_COMMON_FILES)
marlin-objs += $(MARLINOFILES)

#WCN_PLATFORM
obj-y := wcn_platform.o

wcn_platform-c-objs += $(WCN_MARLIN_COMMON_FILES)

wcn_platform-c-objs += rf/rf.o

wcn_platform-objs += $(wcn_platform-c-objs)
