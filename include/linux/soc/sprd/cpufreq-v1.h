/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SPRD_CPUFREQ_V1_H__
#define __SPRD_CPUFREQ_V1_H__

#include <linux/cpumask.h>
#include <linux/types.h>

struct cpufreq_frequency_table;
struct device;
struct regmap;
struct regmap_config;
struct sprd_cpufreq_v1;
struct sprd_cpufreq_v1_cluster;

/*
 * Callbacks own the register layout and the safe hardware handover sequence.
 * Resources used by restore() must be allocated through priv_size or during
 * validate(), before init() starts programming the controller. The core keeps
 * the OPP tables and these resources alive until restore() completes, including
 * when init() fails part way through.
 */
struct sprd_cpufreq_v1_ops {
	int (*validate)(struct sprd_cpufreq_v1 *dvfs);
	int (*init)(struct sprd_cpufreq_v1 *dvfs);
	void (*restore)(struct sprd_cpufreq_v1 *dvfs);
	int (*set_target)(struct sprd_cpufreq_v1_cluster *cluster, unsigned int index);
	unsigned int (*get_frequency)(struct sprd_cpufreq_v1_cluster *cluster);
	int (*resume)(struct sprd_cpufreq_v1 *dvfs);
};

struct sprd_cpufreq_v1_cluster_data {
	const unsigned long *freq_hz;
	unsigned int num_opps;
	unsigned int max_grades;
	u32 voltage_base_uv;
	u32 voltage_step_uv;
	u32 voltage_selector_mask;
};

/* Map zero is the platform MMIO resource; the rest are named syscon phandles. */
struct sprd_cpufreq_v1_soc_data {
	const struct sprd_cpufreq_v1_cluster_data *clusters;
	unsigned int num_clusters;
	const char * const *bin_names;
	unsigned int num_bins;
	const char * const *syscons;
	unsigned int num_maps;
	const struct regmap_config *regmap_config;
	size_t priv_size;
	unsigned int settle_us;
	unsigned int transition_delay_us;
	const struct sprd_cpufreq_v1_ops *ops;
};

struct sprd_cpufreq_v1_cluster {
	struct sprd_cpufreq_v1 *parent;
	const struct sprd_cpufreq_v1_cluster_data *data;
	struct device *cpu_dev;
	cpumask_t cpus;
	struct cpufreq_frequency_table *table;
	u32 *grades;
	unsigned int id, bin, grade_count;
	int opp_token;
	bool opp_added;
};

struct sprd_cpufreq_v1 {
	struct device *dev;
	const struct sprd_cpufreq_v1_soc_data *soc;
	void __iomem *base;
	struct regmap **maps;
	unsigned long clock_rate;
	struct sprd_cpufreq_v1_cluster *clusters;
	void *priv;
};

extern const struct sprd_cpufreq_v1_soc_data ums512_cpufreq_data;

#endif /* __SPRD_CPUFREQ_V1_H__ */
