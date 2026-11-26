/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SPRD_CPU_DVFS_H__
#define __SPRD_CPU_DVFS_H__

#include <linux/cpumask.h>
#include <linux/types.h>

struct cpufreq_frequency_table;
struct device;
struct platform_device;
struct regmap;
struct regmap_config;
struct sprd_cpu_dvfs;
struct sprd_cpu_dvfs_cluster;

/*
 * Callbacks own the register layout and the safe hardware handover sequence.
 * Resources used by restore() must be allocated through priv_size or during
 * validate(), before init() starts programming the controller. The core keeps
 * the OPP tables and these resources alive until restore() completes, including
 * when init() fails part way through. validate() must resolve each OPP's hardware
 * index before the core builds the CPUFreq table mapping.
 */
struct sprd_cpu_dvfs_ops {
	int (*validate)(struct sprd_cpu_dvfs *dvfs);
	int (*init)(struct sprd_cpu_dvfs *dvfs);
	void (*restore)(struct sprd_cpu_dvfs *dvfs);
	int (*set_target)(struct sprd_cpu_dvfs_cluster *cluster, unsigned int index);
	unsigned int (*get_frequency)(struct sprd_cpu_dvfs_cluster *cluster);
	int (*resume)(struct sprd_cpu_dvfs *dvfs);
};

struct sprd_cpu_dvfs_cluster_data {
	unsigned int max_opps;
	unsigned int max_grades;
	u32 voltage_base_uv;
	u32 voltage_step_uv;
	u32 voltage_selector_mask;
};

/* Map zero is the platform MMIO resource; the rest are named syscon phandles. */
struct sprd_cpu_dvfs_soc_data {
	const struct sprd_cpu_dvfs_cluster_data *clusters;
	unsigned int num_clusters;
	const char * const *bin_names;
	unsigned int num_bins;
	const char * const *syscons;
	unsigned int num_maps;
	const struct regmap_config *regmap_config;
	size_t priv_size;
	unsigned int settle_us;
	unsigned int transition_delay_us;
	unsigned int max_saved_regs;
	const struct sprd_cpu_dvfs_ops *ops;
};

struct sprd_cpu_dvfs_opp {
	unsigned long rate;
	u32 voltage;
	unsigned int index;
};

struct sprd_cpu_dvfs_cluster {
	struct sprd_cpu_dvfs *parent;
	const struct sprd_cpu_dvfs_cluster_data *data;
	struct device *cpu_dev;
	cpumask_t cpus;
	struct cpufreq_frequency_table *table;
	struct sprd_cpu_dvfs_opp *opps;
	u32 *grades;
	unsigned int id, bin, grade_count, opp_count, table_count;
	int opp_token;
	bool opp_added;
};

struct sprd_cpu_dvfs_saved_reg {
	u32 reg, mask, value;
	unsigned int block;
};

struct sprd_cpu_dvfs {
	struct device *dev;
	const struct sprd_cpu_dvfs_soc_data *soc;
	void __iomem *base;
	struct regmap **maps;
	unsigned long clock_rate;
	struct sprd_cpu_dvfs_cluster *clusters;
	void *priv;
	struct sprd_cpu_dvfs_saved_reg *saved;
	unsigned int saved_count;
};

extern const struct sprd_cpu_dvfs_soc_data ums512_cpu_dvfs_data;

int sprd_cpu_dvfs_save(struct sprd_cpu_dvfs *dvfs, unsigned int block,
		       u32 reg, u32 mask);
int sprd_cpu_dvfs_update(struct sprd_cpu_dvfs *dvfs, unsigned int block,
			 u32 reg, u32 mask, u32 value);

struct sprd_cpu_dvfs *devm_sprd_cpu_dvfs_init(struct platform_device *pdev);
int sprd_cpu_dvfs_set_target(struct sprd_cpu_dvfs_cluster *cluster, unsigned int index);
unsigned int sprd_cpu_dvfs_get_frequency(struct sprd_cpu_dvfs_cluster *cluster);
int sprd_cpu_dvfs_resume(struct sprd_cpu_dvfs *dvfs);

#endif /* __SPRD_CPU_DVFS_H__ */
