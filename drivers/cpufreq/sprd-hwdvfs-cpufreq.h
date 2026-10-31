/* SPDX-License-Identifier: GPL-2.0 */
//
// Unisoc APCPU HW DVFS driver
//
// Copyright (C) 2020 Unisoc, Inc.
// Author: Jack Liu <jack.liu@unisoc.com>

#ifndef SPRD_HW_CPUFREQ_H
#define SPRD_HW_CPUFREQ_H

#include "sprd-hwdvfs-normal.h"

struct sprd_cpu_cluster_info {
	struct mutex opp_mutex;
	bool arch_dvfs_enabled;
	struct cpudvfs_device *parchdev;
	unsigned int clu_id;
	unsigned int policy_trans;
};

struct sprd_cpufreq_info {
	struct device *pdev;
	struct cpumask cpus;
	struct device *cpu_dev;
	struct cpudvfs_device *parchdev;
	unsigned int clu_id, cpu_id;
	struct sprd_cpu_cluster_info *pcluster;
};

#endif
