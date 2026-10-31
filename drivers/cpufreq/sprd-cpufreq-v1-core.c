// SPDX-License-Identifier: GPL-2.0-only
/*
 * Unisoc hardware CPUFreq v1 core.
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/soc/sprd/cpufreq-v1.h>

static struct cpufreq_driver sprd_cpufreq_v1_driver;

static void sprd_cpufreq_v1_free_opps(void *arg)
{
	struct sprd_cpufreq_v1_cluster *cluster = arg;

	dev_pm_opp_free_cpufreq_table(cluster->cpu_dev, &cluster->table);
	if (cluster->opp_added)
		dev_pm_opp_of_cpumask_remove_table(&cluster->cpus);
	if (cluster->opp_token >= 0)
		dev_pm_opp_put_prop_name(cluster->opp_token);
}

static int sprd_cpufreq_v1_read_bin(struct device_node *np, unsigned int num_bins,
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

static int sprd_cpufreq_v1_prepare_opps(struct sprd_cpufreq_v1_cluster *cluster)
{
	const struct sprd_cpufreq_v1_cluster_data *data = cluster->data;
	const struct sprd_cpufreq_v1_soc_data *soc = cluster->parent->soc;
	const char *profile = soc->bin_names[cluster->bin];
	struct device *cpu_dev = cluster->cpu_dev;
	unsigned long rate = 0, voltage, previous = 0;
	struct device_node *opp_np;
	struct dev_pm_opp *opp;
	unsigned int i, selector;
	char *prop_name = NULL;
	int ret;

	cluster->opp_token = -1;
	if (profile) {
		prop_name = devm_kasprintf(cluster->parent->dev, GFP_KERNEL,
					   "opp-microvolt-%s", profile);
		if (!prop_name)
			return -ENOMEM;
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
	if (ret != data->num_opps) {
		ret = ret < 0 ? ret : -EINVAL;
		goto free_opps;
	}
	for (i = 0; i < data->num_opps; i++, rate++) {
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
		if (IS_ERR(opp)) {
			ret = PTR_ERR(opp);
			goto free_opps;
		}
		voltage = dev_pm_opp_get_voltage(opp);
		opp_np = dev_pm_opp_get_of_node(opp);
		/* Falling back to the default voltage would invalidate bin maps. */
		ret = profile && !of_property_present(opp_np, prop_name) ?
			-EINVAL : 0;
		of_node_put(opp_np);
		dev_pm_opp_put(opp);
		if (ret)
			goto free_opps;
		if (rate != data->freq_hz[i] || !voltage ||
		    voltage < previous || voltage < data->voltage_base_uv ||
		    voltage - data->voltage_base_uv >
		    (u64)data->voltage_selector_mask * data->voltage_step_uv) {
			ret = -EINVAL;
			goto free_opps;
		}
		if (voltage != previous) {
			if (cluster->grade_count == data->max_grades) {
				ret = -E2BIG;
				goto free_opps;
			}
			selector = DIV_ROUND_UP(voltage - data->voltage_base_uv,
						data->voltage_step_uv);
			cluster->grades[cluster->grade_count++] = data->voltage_base_uv +
				selector * data->voltage_step_uv;
		}
		previous = voltage;
	}
	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &cluster->table);
	if (ret)
		goto free_opps;

	return devm_add_action_or_reset(cluster->parent->dev, sprd_cpufreq_v1_free_opps, cluster);

free_opps:
	sprd_cpufreq_v1_free_opps(cluster);
	return ret;
}

static int sprd_cpufreq_v1_parse_clusters(struct sprd_cpufreq_v1 *dvfs)
{
	struct of_phandle_args args;
	struct sprd_cpufreq_v1_cluster *cluster;
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
		ret = sprd_cpufreq_v1_read_bin(np, dvfs->soc->num_bins, &cluster->bin);
		if (ret)
			return ret;
		ret = sprd_cpufreq_v1_prepare_opps(cluster);
		if (ret)
			return ret;
		prepared++;
	}

	return prepared == dvfs->soc->num_clusters ? 0 : -EINVAL;
}

static int sprd_cpufreq_v1_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct sprd_cpufreq_v1_cluster *cluster = policy->driver_data;

	if (index >= cluster->data->num_opps)
		return -EINVAL;
	return cluster->parent->soc->ops->set_target(cluster, index);
}

static unsigned int sprd_cpufreq_v1_get_frequency(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct sprd_cpufreq_v1_cluster *cluster;

	if (!policy || !policy->driver_data)
		return 0;
	cluster = policy->driver_data;
	return cluster->parent->soc->ops->get_frequency(cluster);
}

static int sprd_cpufreq_v1_policy_init(struct cpufreq_policy *policy)
{
	struct sprd_cpufreq_v1 *dvfs = cpufreq_get_driver_data();
	struct sprd_cpufreq_v1_cluster *cluster;
	unsigned int i;

	for (i = 0; i < dvfs->soc->num_clusters; i++) {
		cluster = &dvfs->clusters[i];
		if (!cpumask_test_cpu(policy->cpu, &cluster->cpus))
			continue;
		cpumask_copy(policy->cpus, &cluster->cpus);
		policy->driver_data = cluster;
		policy->freq_table = cluster->table;
		policy->suspend_freq = cluster->table[0].frequency;
		policy->transition_delay_us = dvfs->soc->transition_delay_us;
		policy->cpuinfo.transition_latency = dvfs->soc->settle_us * NSEC_PER_USEC;
		policy->dvfs_possible_from_any_cpu = true;
		return 0;
	}
	return -ENODEV;
}

static void sprd_cpufreq_v1_policy_exit(struct cpufreq_policy *policy)
{
	policy->freq_table = NULL;
	policy->driver_data = NULL;
}

static struct cpufreq_driver sprd_cpufreq_v1_driver = {
	.name = "sprd-cpufreq-v1",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sprd_cpufreq_v1_set_target,
	.get = sprd_cpufreq_v1_get_frequency,
	.init = sprd_cpufreq_v1_policy_init,
	.exit = sprd_cpufreq_v1_policy_exit,
	.suspend = cpufreq_generic_suspend,
	.register_em = cpufreq_register_em_with_opp,
};

static void sprd_cpufreq_v1_unregister(void *arg)
{
	cpufreq_unregister_driver(&sprd_cpufreq_v1_driver);
	sprd_cpufreq_v1_driver.driver_data = NULL;
}

static void sprd_cpufreq_v1_restore(void *arg)
{
	struct sprd_cpufreq_v1 *dvfs = arg;

	dvfs->soc->ops->restore(dvfs);
}

static int sprd_cpufreq_v1_alloc_clusters(struct sprd_cpufreq_v1 *dvfs)
{
	const struct sprd_cpufreq_v1_soc_data *soc = dvfs->soc;
	struct sprd_cpufreq_v1_cluster *cluster;
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
		if (!cluster->grades)
			return -ENOMEM;
	}
	return 0;
}

static int sprd_cpufreq_v1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct sprd_cpufreq_v1_soc_data *soc = of_device_get_match_data(dev);
	struct sprd_cpufreq_v1 *dvfs;
	struct clk *clk;
	unsigned int i;
	int ret;

	if (!soc)
		return -ENODEV;
	if (sprd_cpufreq_v1_driver.driver_data)
		return -EBUSY;
	dvfs = devm_kzalloc(dev, sizeof(*dvfs), GFP_KERNEL);
	if (!dvfs)
		return -ENOMEM;
	dvfs->dev = dev;
	dvfs->soc = soc;
	dvfs->priv = devm_kzalloc(dev, soc->priv_size, GFP_KERNEL);
	dvfs->maps = devm_kcalloc(dev, soc->num_maps, sizeof(*dvfs->maps), GFP_KERNEL);
	if (!dvfs->priv || !dvfs->maps)
		return -ENOMEM;
	ret = sprd_cpufreq_v1_alloc_clusters(dvfs);
	if (ret)
		return ret;
	dvfs->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dvfs->base))
		return PTR_ERR(dvfs->base);
	dvfs->maps[0] = devm_regmap_init_mmio(dev, dvfs->base, soc->regmap_config);
	if (IS_ERR(dvfs->maps[0]))
		return PTR_ERR(dvfs->maps[0]);
	for (i = 1; i < soc->num_maps; i++) {
		dvfs->maps[i] = syscon_regmap_lookup_by_phandle(dev->of_node, soc->syscons[i]);
		if (IS_ERR(dvfs->maps[i]))
			return dev_err_probe(dev, PTR_ERR(dvfs->maps[i]),
					     "failed to get %s\n", soc->syscons[i]);
	}
	clk = devm_clk_get_enabled(dev, "dvfs");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable DVFS clock\n");
	dvfs->clock_rate = clk_get_rate(clk);
	if (!dvfs->clock_rate)
		return -EINVAL;
	ret = sprd_cpufreq_v1_parse_clusters(dvfs);
	if (ret)
		return dev_err_probe(dev, ret, "invalid CPU DVFS configuration\n");
	ret = soc->ops->validate(dvfs);
	if (ret)
		return dev_err_probe(dev, ret, "invalid DVFS voltage grade map\n");
	ret = devm_add_action_or_reset(dev, sprd_cpufreq_v1_restore, dvfs);
	if (ret)
		return ret;
	ret = soc->ops->init(dvfs);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize DVFS hardware\n");
	platform_set_drvdata(pdev, dvfs);
	sprd_cpufreq_v1_driver.driver_data = dvfs;
	ret = cpufreq_register_driver(&sprd_cpufreq_v1_driver);
	if (ret) {
		sprd_cpufreq_v1_driver.driver_data = NULL;
		return dev_err_probe(dev, ret, "failed to register cpufreq driver\n");
	}
	return devm_add_action_or_reset(dev, sprd_cpufreq_v1_unregister, dvfs);
}

static int sprd_cpufreq_v1_resume(struct device *dev)
{
	struct sprd_cpufreq_v1 *dvfs = dev_get_drvdata(dev);

	if (dvfs->soc->ops->resume)
		return dvfs->soc->ops->resume(dvfs);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sprd_cpufreq_v1_pm_ops, NULL, sprd_cpufreq_v1_resume);

static const struct of_device_id sprd_cpufreq_v1_of_match[] = {
	{ .compatible = "sprd,ums512-cpufreq", .data = &ums512_cpufreq_data },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_cpufreq_v1_of_match);

static struct platform_driver sprd_cpufreq_v1_platform_driver = {
	.probe = sprd_cpufreq_v1_probe,
	.driver = {
		.name = "sprd-cpufreq-v1",
		.of_match_table = sprd_cpufreq_v1_of_match,
		.pm = pm_sleep_ptr(&sprd_cpufreq_v1_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sprd_cpufreq_v1_platform_driver);

MODULE_DESCRIPTION("Unisoc hardware CPU frequency scaling v1");
MODULE_LICENSE("GPL");
