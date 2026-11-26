// SPDX-License-Identifier: GPL-2.0-only
/*
 * Common Unisoc hardware CPU DVFS support.
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/mfd/syscon.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sort.h>

#include <linux/soc/sprd/cpu-dvfs.h>

/* Save each register once, before modifying any of its owned fields. */
int sprd_cpu_dvfs_save(struct sprd_cpu_dvfs *dvfs, unsigned int block,
		       u32 reg, u32 mask)
{
	struct sprd_cpu_dvfs_saved_reg *saved;
	unsigned int i;
	int ret;

	for (i = 0; i < dvfs->saved_count; i++) {
		saved = &dvfs->saved[i];
		if (saved->block == block && saved->reg == reg)
			break;
	}
	if (i == dvfs->saved_count) {
		if (i == dvfs->soc->max_saved_regs)
			return -E2BIG;
		saved = &dvfs->saved[i];
		ret = regmap_read(dvfs->maps[block], reg, &saved->value);
		if (ret)
			return ret;
		saved->block = block;
		saved->reg = reg;
		dvfs->saved_count++;
	}
	saved->mask |= mask;

	return 0;
}

int sprd_cpu_dvfs_update(struct sprd_cpu_dvfs *dvfs, unsigned int block,
			 u32 reg, u32 mask, u32 value)
{
	int ret;

	ret = sprd_cpu_dvfs_save(dvfs, block, reg, mask);
	if (ret)
		return ret;
	return regmap_update_bits(dvfs->maps[block], reg, mask, value);
}

static void sprd_cpu_dvfs_free_opps(void *arg)
{
	struct sprd_cpu_dvfs_cluster *cluster = arg;

	dev_pm_opp_free_cpufreq_table(cluster->cpu_dev, &cluster->table);
	if (cluster->opp_added)
		dev_pm_opp_of_cpumask_remove_table(&cluster->cpus);
	if (cluster->opp_token >= 0)
		dev_pm_opp_put_prop_name(cluster->opp_token);
}

static int sprd_cpu_dvfs_read_bin(struct device_node *np, unsigned int num_bins,
				  unsigned int *bin)
{
	struct nvmem_cell *cell;
	size_t len;
	u8 *value;
	int ret = 0;

	cell = of_nvmem_cell_get(np, "dvfs_bin");
	if (IS_ERR(cell))
		return PTR_ERR(cell);
	value = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(value))
		return PTR_ERR(value);
	if (len != 1 || *value >= num_bins)
		ret = -EINVAL;
	else
		*bin = *value;
	kfree(value);

	return ret;
}

static int sprd_cpu_dvfs_compare_opps(const void *a, const void *b)
{
	const struct sprd_cpu_dvfs_opp *opp_a = a, *opp_b = b;

	return (opp_a->rate > opp_b->rate) - (opp_a->rate < opp_b->rate);
}

static int sprd_cpu_dvfs_read_opps(struct sprd_cpu_dvfs_cluster *cluster,
				   const char *profile)
{
	const struct sprd_cpu_dvfs_cluster_data *data = cluster->data;
	struct device_node *table;
	struct sprd_cpu_dvfs_opp *opp;
	unsigned long previous = 0;
	unsigned int i, selector;
	u32 voltage;
	u64 rate;
	char *property;
	int ret = 0;

	property = profile ? devm_kasprintf(cluster->parent->dev, GFP_KERNEL,
					  "opp-microvolt-%s", profile) : "opp-microvolt";
	if (!property)
		return -ENOMEM;
	table = dev_pm_opp_of_get_opp_desc_node(cluster->cpu_dev);
	if (!table)
		return -ENODEV;

	/* Disabled CPU OPPs can still supply voltage grades needed by the SCU. */
	for_each_child_of_node_scoped(table, np) {
		if (cluster->opp_count == data->max_opps) {
			ret = -E2BIG;
			break;
		}
		ret = of_property_read_u64(np, "opp-hz", &rate);
		if (ret)
			break;
		ret = of_property_read_u32(np, property, &voltage);
		if (ret)
			break;
		if (!rate || rate > ULONG_MAX || rate % 1000 || !voltage ||
		    voltage < data->voltage_base_uv ||
		    voltage - data->voltage_base_uv >
		    (u64)data->voltage_selector_mask * data->voltage_step_uv) {
			ret = -EINVAL;
			break;
		}
		opp = &cluster->opps[cluster->opp_count++];
		opp->rate = rate;
		selector = DIV_ROUND_UP(voltage - data->voltage_base_uv, data->voltage_step_uv);
		opp->voltage = data->voltage_base_uv + selector * data->voltage_step_uv;
	}
	of_node_put(table);
	if (ret)
		return ret;
	if (!cluster->opp_count)
		return -ENODEV;

	sort(cluster->opps, cluster->opp_count, sizeof(*cluster->opps),
	     sprd_cpu_dvfs_compare_opps, NULL);
	for (i = 0; i < cluster->opp_count; i++) {
		opp = &cluster->opps[i];
		if ((i && opp->rate == cluster->opps[i - 1].rate) || opp->voltage < previous)
			return -EINVAL;
		if (opp->voltage != previous) {
			if (cluster->grade_count == data->max_grades)
				return -E2BIG;
			cluster->grades[cluster->grade_count++] = opp->voltage;
		}
		previous = opp->voltage;
	}
	return 0;
}

static int sprd_cpu_dvfs_prepare_opps(struct sprd_cpu_dvfs_cluster *cluster)
{
	const struct sprd_cpu_dvfs_soc_data *soc = cluster->parent->soc;
	const char *profile = soc->bin_names[cluster->bin];
	struct device *cpu_dev = cluster->cpu_dev;
	int ret;

	ret = sprd_cpu_dvfs_read_opps(cluster, profile);
	if (ret)
		return ret;
	if (profile) {
		ret = dev_pm_opp_set_prop_name(cpu_dev, profile);
		if (ret < 0)
			return ret;
		cluster->opp_token = ret;
	}
	ret = dev_pm_opp_of_cpumask_add_table(&cluster->cpus);
	if (ret)
		goto free_opps;
	cluster->opp_added = true;
	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		ret = ret ?: -ENODEV;
		goto free_opps;
	}
	cluster->table_count = ret;
	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &cluster->table);
	if (ret)
		goto free_opps;

	return devm_add_action_or_reset(cluster->parent->dev, sprd_cpu_dvfs_free_opps, cluster);

free_opps:
	sprd_cpu_dvfs_free_opps(cluster);
	return ret;
}

static int sprd_cpu_dvfs_map_table(struct sprd_cpu_dvfs_cluster *cluster)
{
	struct cpufreq_frequency_table *entry;
	unsigned int i;

	cpufreq_for_each_valid_entry(entry, cluster->table) {
		for (i = 0; i < cluster->opp_count; i++)
			if (entry->frequency == cluster->opps[i].rate / 1000)
				break;
		if (i == cluster->opp_count)
			return -EINVAL;
		entry->driver_data = cluster->opps[i].index;
	}
	return 0;
}

static int sprd_cpu_dvfs_parse_clusters(struct sprd_cpu_dvfs *dvfs)
{
	struct of_phandle_args args;
	struct sprd_cpu_dvfs_cluster *cluster;
	struct device *cpu_dev;
	unsigned int prepared = 0;
	unsigned int cpu, id;
	int ret;

	for_each_possible_cpu(cpu) {
		cpu_dev = get_cpu_device(cpu);
		if (!cpu_dev)
			return -ENODEV;
		ret = of_parse_phandle_with_args(cpu_dev->of_node, "performance-domains",
						 "#performance-domain-cells", 0, &args);
		if (ret)
			return ret;
		if (args.np != dvfs->dev->of_node || args.args_count != 1) {
			of_node_put(args.np);
			return -EINVAL;
		}
		id = args.args[0];
		ret = id < dvfs->soc->num_clusters ? 0 : -EINVAL;
		of_node_put(args.np);
		if (ret)
			return ret;
		cluster = &dvfs->clusters[id];
		if (!cluster->cpu_dev)
			cluster->cpu_dev = cpu_dev;
		cpumask_set_cpu(cpu, &cluster->cpus);
	}
	for_each_available_child_of_node_scoped(dvfs->dev->of_node, np) {
		ret = of_property_read_u32(np, "reg", &id);
		if (ret || id >= dvfs->soc->num_clusters ||
		    dvfs->clusters[id].opp_added)
			return -EINVAL;
		cluster = &dvfs->clusters[id];
		if (!cluster->cpu_dev)
			return -ENODEV;
		ret = sprd_cpu_dvfs_read_bin(np, dvfs->soc->num_bins, &cluster->bin);
		if (ret)
			return ret;
		ret = sprd_cpu_dvfs_prepare_opps(cluster);
		if (ret)
			return ret;
		prepared++;
	}

	return prepared == dvfs->soc->num_clusters ? 0 : -EINVAL;
}

static void sprd_cpu_dvfs_restore(void *arg)
{
	struct sprd_cpu_dvfs *dvfs = arg;

	dvfs->soc->ops->restore(dvfs);
}

static int sprd_cpu_dvfs_alloc_clusters(struct sprd_cpu_dvfs *dvfs)
{
	const struct sprd_cpu_dvfs_soc_data *soc = dvfs->soc;
	struct sprd_cpu_dvfs_cluster *cluster;
	unsigned int i;

	dvfs->clusters = devm_kcalloc(dvfs->dev, soc->num_clusters,
				      sizeof(*dvfs->clusters), GFP_KERNEL);
	if (!dvfs->clusters)
		return -ENOMEM;
	for (i = 0; i < soc->num_clusters; i++) {
		cluster = &dvfs->clusters[i];
		cluster->parent = dvfs;
		cluster->data = &soc->clusters[i];
		cluster->id = i;
		cluster->opp_token = -1;
		cluster->grades = devm_kcalloc(dvfs->dev, cluster->data->max_grades,
					       sizeof(*cluster->grades), GFP_KERNEL);
		cluster->opps = devm_kcalloc(dvfs->dev, cluster->data->max_opps,
					     sizeof(*cluster->opps), GFP_KERNEL);
		if (!cluster->grades || !cluster->opps)
			return -ENOMEM;
	}
	return 0;
}

struct sprd_cpu_dvfs *devm_sprd_cpu_dvfs_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct sprd_cpu_dvfs_soc_data *soc = of_device_get_match_data(dev);
	struct sprd_cpu_dvfs *dvfs;
	struct clk *clk;
	unsigned int i;
	int ret;

	if (!soc)
		return ERR_PTR(-ENODEV);
	dvfs = devm_kzalloc(dev, sizeof(*dvfs), GFP_KERNEL);
	if (!dvfs)
		return ERR_PTR(-ENOMEM);
	dvfs->dev = dev;
	dvfs->soc = soc;
	dvfs->priv = devm_kzalloc(dev, soc->priv_size, GFP_KERNEL);
	dvfs->maps = devm_kcalloc(dev, soc->num_maps, sizeof(*dvfs->maps), GFP_KERNEL);
	dvfs->saved = devm_kcalloc(dev, soc->max_saved_regs, sizeof(*dvfs->saved), GFP_KERNEL);
	if (!dvfs->priv || !dvfs->maps || !dvfs->saved)
		return ERR_PTR(-ENOMEM);
	ret = sprd_cpu_dvfs_alloc_clusters(dvfs);
	if (ret)
		return ERR_PTR(ret);
	dvfs->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dvfs->base))
		return ERR_CAST(dvfs->base);
	dvfs->maps[0] = devm_regmap_init_mmio(dev, dvfs->base, soc->regmap_config);
	if (IS_ERR(dvfs->maps[0]))
		return ERR_CAST(dvfs->maps[0]);
	for (i = 1; i < soc->num_maps; i++) {
		dvfs->maps[i] = syscon_regmap_lookup_by_phandle(dev->of_node, soc->syscons[i]);
		if (IS_ERR(dvfs->maps[i]))
			return ERR_PTR(dev_err_probe(dev, PTR_ERR(dvfs->maps[i]),
					     "failed to get %s\n", soc->syscons[i]));
	}
	clk = devm_clk_get_enabled(dev, "dvfs");
	if (IS_ERR(clk))
		return ERR_PTR(dev_err_probe(dev, PTR_ERR(clk), "failed to enable DVFS clock\n"));
	dvfs->clock_rate = clk_get_rate(clk);
	if (!dvfs->clock_rate)
		return ERR_PTR(-EINVAL);
	ret = sprd_cpu_dvfs_parse_clusters(dvfs);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret, "invalid CPU DVFS configuration\n"));
	ret = soc->ops->validate(dvfs);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret, "invalid hardware DVFS configuration\n"));
	for (i = 0; i < soc->num_clusters; i++) {
		ret = sprd_cpu_dvfs_map_table(&dvfs->clusters[i]);
		if (ret)
			return ERR_PTR(ret);
	}
	ret = devm_add_action_or_reset(dev, sprd_cpu_dvfs_restore, dvfs);
	if (ret)
		return ERR_PTR(ret);
	ret = soc->ops->init(dvfs);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret, "failed to initialize DVFS hardware\n"));

	return dvfs;
}

int sprd_cpu_dvfs_set_target(struct sprd_cpu_dvfs_cluster *cluster, unsigned int index)
{
	if (index >= cluster->table_count)
		return -EINVAL;
	return cluster->parent->soc->ops->set_target(cluster, cluster->table[index].driver_data);
}

unsigned int sprd_cpu_dvfs_get_frequency(struct sprd_cpu_dvfs_cluster *cluster)
{
	return cluster->parent->soc->ops->get_frequency(cluster);
}

int sprd_cpu_dvfs_resume(struct sprd_cpu_dvfs *dvfs)
{
	if (dvfs->soc->ops->resume)
		return dvfs->soc->ops->resume(dvfs);
	return 0;
}
