// SPDX-License-Identifier: GPL-2.0
//
// Unisoc APCPU HW DVFS driver
//
// Copyright (C) 2020 Unisoc, Inc.
// Author: Jack Liu <jack.liu@unisoc.com>

#include <linux/arch_topology.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "sprd-hwdvfs-cpufreq.h"

static struct cpufreq_driver sprd_hardware_cpufreq_driver;
static struct sprd_cpufreq_info *global_cpufreq_info;

static const struct of_device_id sprd_hardware_cpufreq_of_match[] = {
	{.compatible = "sprd,cpufreq-v1",},
	{},
};

static inline struct sprd_cpufreq_info *sprd_cpufreq_info_lookup(u32 cpu)
{
	if (cpu >= nr_cpu_ids) {
		pr_err("invalid cpu id\n");
		return ERR_PTR(-EINVAL);
	}

	if (!global_cpufreq_info) {
		pr_err("the cpufreq information for all cpus is null\n");
		return ERR_PTR(-ENODEV);
	}

	return &global_cpufreq_info[cpu];
}

static int sprd_related_cpus_cluster_alloc(struct sprd_cpufreq_info *info,
					   struct cpufreq_policy *policy)
{
	struct sprd_cpufreq_info *temp;
	int cpu;

	if (!info || !policy)
		return -ENODEV;

	for_each_cpu(cpu, policy->cpus) {
		if (cpu == policy->cpu)
			continue;
		temp = sprd_cpufreq_info_lookup(cpu);
		if (IS_ERR(temp))
			return PTR_ERR(temp);

		temp->pcluster = info->pcluster;
	}

	return 0;
}

static int sprd_cpufreq_arch_update(struct sprd_cpufreq_info *info)
{
	struct cpudvfs_phy_ops *driver;
	struct dev_pm_opp *opp;
	unsigned long rate, volt, freq;
	int opp_num, i, ret = 0;

	if (!info) {
		pr_err("the info for cpu is null\n");
		return -EINVAL;
	}

	if (!info->cpu_dev || !info->pcluster || !info->parchdev) {
		dev_err(info->pdev, "cpu or cluster information is missing\n");
		return -EINVAL;
	}

	driver = info->parchdev->phy_ops;
	if (!driver) {
		dev_err(info->pdev, "the way to operate dvfs device is null\n");
		return -EINVAL;
	}

	if (!driver->udelay_update || !driver->idle_pd_volt_update ||
	    !driver->index_map_table_update || !driver->volt_grade_table_update) {
		dev_err(info->pdev,
			"no way to update cpu hw dvfs device parameters\n");
		return -EINVAL;
	}

	opp_num = dev_pm_opp_get_opp_count(info->cpu_dev);
	if (opp_num < 0) {
		dev_err(info->pdev,
			"failed to get entry num of opp tbl(%d)\n", opp_num);
		return opp_num;
	}

	for (i = 0, rate = 0; i < opp_num; i++, rate++) {
		opp = dev_pm_opp_find_freq_ceil(info->cpu_dev, &rate);
		if (IS_ERR(opp))
			return PTR_ERR(opp);

		volt = dev_pm_opp_get_voltage(opp);
		freq = dev_pm_opp_get_freq(opp);		/* in HZ */

		dev_pm_opp_put(opp);

		ret = driver->volt_grade_table_update(info->parchdev,
						      info->clu_id, freq,
						      volt, i);
		if (ret) {
			dev_err(info->pdev, "failed to update vol grade val\n");
			return ret;
		}
	}

	/*
	 * Because the opp table is changed, some dvfs hardare parameters need
	 * to be changed meanwhile
	 */
	ret = driver->udelay_update(info->parchdev, info->clu_id);
	if (ret) {
		dev_err(info->pdev, "failed to update hw delay time\n");
		return ret;
	}

	ret = driver->index_map_table_update(info->parchdev, info->clu_id);
	if (ret) {
		dev_err(info->pdev, "failed to update dvfs index table\n");
		return ret;
	}

	ret = driver->idle_pd_volt_update(info->parchdev, info->clu_id);
	if (ret)
		dev_err(info->pdev, "failed to set idle pd voltage\n");

	return ret;
}

static
int sprd_hardware_cpufreq_set_target_index(struct cpufreq_policy *policy,
					   unsigned int index)
{
	struct sprd_cpufreq_info *info;
	struct cpudvfs_device *pdev;
	struct cpudvfs_phy_ops *driver;
	struct dev_pm_opp *opp;
	unsigned long freq_hz;
	int ret;

	info = policy->driver_data;
	if (IS_ERR(info))
		return PTR_ERR(info);

	pdev = info->parchdev;
	if (!pdev) {
		dev_err(info->pdev, "no dvfs device found\n");
		return -ENODEV;
	}

	driver = pdev->phy_ops;
	if (!driver || !driver->target_set) {
		dev_err(info->pdev, "no way to operate dvfs device\n");
		return -EINVAL;
	}

	if (!info->pcluster) {
		dev_err(info->pdev, "cluster info for cpu%d is missing\n",
			policy->cpu);
		return -EINVAL;
	}

	freq_hz = policy->freq_table[index].frequency * 1000;
	opp = dev_pm_opp_find_freq_ceil(info->cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		dev_err(info->pdev, "cpu%d: failed to find OPP for %lu\n",
			info->cpu_id, freq_hz);
		return PTR_ERR(opp);
	}
	freq_hz = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	mutex_lock(&info->pcluster->opp_mutex);
	ret = driver->target_set(pdev, info->clu_id, freq_hz);
	mutex_unlock(&info->pcluster->opp_mutex);

	if (ret) {
		dev_err(info->pdev, "failed to set target %lu for cpu%d\n",
			freq_hz, info->cpu_id);
		return ret;
	}

	arch_set_freq_scale(policy->related_cpus,
			    policy->freq_table[index].frequency,
			    policy->cpuinfo.max_freq);
	return 0;
}

static
int sprd_hw_dvfs_enable(struct device *dev, struct cpudvfs_device *pdev,
			u32 clu_id)
{
	struct cpudvfs_phy_ops *driver;

	if (!pdev) {
		dev_err(dev, "no cpu dvfs device found\n");
		return -ENODEV;
	}

	driver = pdev->phy_ops;
	if (!driver || !driver->dvfs_enable) {
		dev_err(dev, "no way to operate cpu dvfs device\n");
		return -EINVAL;
	}

	return driver->dvfs_enable(pdev, clu_id, true);
}

static int sprd_cpu_cluster_static_init(struct device *pdev,
					struct sprd_cpu_cluster_info *clu)
{
	if (!clu) {
		dev_err(pdev, "the cluster to access is null\n");
		return -EINVAL;
	}

	mutex_init(&clu->opp_mutex);

	return 0;
}

static int sprd_hardware_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_table;
	struct sprd_cpufreq_info *info;
	struct of_phandle_args args;
	int ret;

	info = sprd_cpufreq_info_lookup(policy->cpu);
	if (IS_ERR(info))
		return PTR_ERR(info);

	/* CPUs in the same cluster share same clock and power domain */
	ret = of_perf_domain_get_sharing_cpumask(policy->cpu,
						 "performance-domains",
						 "#performance-domain-cells",
						 policy->cpus, &args);
	if (ret) {
		dev_err(info->pdev, "cpu%d: no performance domain: %d\n",
			policy->cpu, ret);
		return ret;
	}
	of_node_put(args.np);
	cpumask_copy(&info->cpus, policy->cpus);

	if (!info->pcluster) { /* Just do once for every policy */
		struct sprd_cpu_cluster_info *clu;

		clu = kzalloc(sizeof(*clu), GFP_KERNEL);
		if (!clu)
			return -ENOMEM;

		info->pcluster = clu;

		ret = sprd_cpu_cluster_static_init(info->pdev, clu);
		if (ret)
			return ret;

		ret = sprd_related_cpus_cluster_alloc(info, policy);
		if (ret)
			return ret;
	}

	if (info->policy_trans)
		policy->transition_delay_us = info->policy_trans;

	mutex_lock(&info->pcluster->opp_mutex);
	ret = dev_pm_opp_of_cpumask_add_table(policy->cpus);
	if (ret) {
		dev_err(info->pdev, "failed to add OPP table: %d\n", ret);
		mutex_unlock(&info->pcluster->opp_mutex);
		return ret;
	}

	ret = sprd_cpufreq_arch_update(info);
	if (ret) {
		dev_err(info->pdev, "failed to update hw dvfs parameters\n");
		mutex_unlock(&info->pcluster->opp_mutex);
		goto free_opp_table;
	}

	ret = dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
	if (ret) {
		dev_err(info->pdev, "error in initializing frequency table (%d)\n",
			ret);
		mutex_unlock(&info->pcluster->opp_mutex);
		goto free_opp_table;
	}

	policy->freq_table = freq_table;
	policy->suspend_freq = freq_table[0].frequency;
	mutex_unlock(&info->pcluster->opp_mutex);

	policy->dvfs_possible_from_any_cpu = true;
	dev_pm_opp_of_register_em(info->cpu_dev, policy->cpus);
	policy->driver_data = info;

	if (!info->pcluster->arch_dvfs_enabled) {
		ret = sprd_hw_dvfs_enable(info->pdev, info->parchdev,
					  info->clu_id);
		if (ret) {
			dev_pm_opp_free_cpufreq_table(info->cpu_dev,
						      &policy->freq_table);
			dev_pm_opp_of_cpumask_remove_table(&info->cpus);
			policy->driver_data = NULL;
			return ret;
		}
		info->pcluster->arch_dvfs_enabled = true;
	}

	return 0;

free_opp_table:
	dev_pm_opp_of_cpumask_remove_table(policy->cpus);
	return ret;
}

static void sprd_hardware_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct sprd_cpufreq_info *info = policy->driver_data;

	if (IS_ERR_OR_NULL(info))
		return;

	mutex_lock(&info->pcluster->opp_mutex);
	dev_pm_opp_free_cpufreq_table(info->cpu_dev, &policy->freq_table);
	dev_pm_opp_of_cpumask_remove_table(&info->cpus);

	policy->driver_data = NULL;
	mutex_unlock(&info->pcluster->opp_mutex);
}

static int sprd_hardware_cpufreq_table_verify(struct cpufreq_policy_data
					      *policy_data)
{
	return cpufreq_generic_frequency_table_verify(policy_data);
}

static unsigned int sprd_hardware_cpufreq_get(unsigned int cpu)
{
	struct sprd_cpufreq_info *info;
	struct cpudvfs_device *pdev;
	struct cpudvfs_phy_ops *driver;

	info = sprd_cpufreq_info_lookup(cpu);
	if (IS_ERR(info))
		return 0;

	pdev = info->parchdev;
	if (!pdev) {
		dev_err(info->pdev, "no cpu dvfs device found.\n");
		return  0;
	}
	driver = pdev->phy_ops;
	if (!driver || !driver->freq_get) {
		dev_err(info->pdev, "no way to access cpu dvfs device\n");
		return 0;
	}

	return driver->freq_get(pdev, info->clu_id);
}

static int sprd_hardware_cpufreq_suspend(struct cpufreq_policy *policy)
{
	return cpufreq_generic_suspend(policy);
}

static int sprd_hardware_cpufreq_resume(struct cpufreq_policy *policy)
{
	/*
	 * Re-apply the suspend frequency so the hardware DVFS engine is
	 * programmed to a known state after resume.
	 */
	return cpufreq_generic_suspend(policy);
}

static struct cpufreq_driver sprd_hardware_cpufreq_driver = {
	.name = "sprd-cpufreq",
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK
			| CPUFREQ_HAVE_GOVERNOR_PER_POLICY
			| CPUFREQ_IS_COOLING_DEV,
	.init = sprd_hardware_cpufreq_init,
	.exit = sprd_hardware_cpufreq_exit,
	.verify = sprd_hardware_cpufreq_table_verify,
	.target_index = sprd_hardware_cpufreq_set_target_index,
	.get = sprd_hardware_cpufreq_get,
	.suspend = sprd_hardware_cpufreq_suspend,
	.resume = sprd_hardware_cpufreq_resume,
};

static int sprd_cpufreq_info_init(struct sprd_cpufreq_info *info, int cpu)
{
	struct device_node *cluster_np = NULL, *dev_np;
	struct of_phandle_args args;
	struct device *cpu_dev;
	struct platform_device *pdev;
	unsigned int policy_trans_delay;
	u32 cluster_id;
	int ret;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		dev_err(info->pdev, "failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}

	ret = parse_perf_domain(cpu, "performance-domains",
				"#performance-domain-cells", &args);
	if (ret) {
		dev_err(info->pdev, "cpu%d: no performance domain: %d\n",
			cpu, ret);
		return ret;
	}

	cluster_id = args.args[0];
	for_each_available_child_of_node(args.np, cluster_np) {
		u32 reg;

		if (!of_property_read_u32(cluster_np, "reg", &reg) &&
		    reg == cluster_id)
			break;
	}
	of_node_put(args.np);
	if (!cluster_np) {
		dev_err(info->pdev, "cpu%d: performance domain %u not found\n",
			cpu, cluster_id);
		return -ENODEV;
	}

	dev_np = of_parse_phandle(cluster_np, "sprd,hw-dvfs-device", 0);
	if (!dev_np) {
		dev_err(info->pdev, "no associated hw dvfs device appointed\n");
		ret = -EINVAL;
		goto cluster_np_put;
	}

	if (!of_property_read_u32(cluster_np, "transition_delay_us",
				  &policy_trans_delay))
		info->policy_trans = policy_trans_delay;

	pdev = of_find_device_by_node(dev_np);
	if (IS_ERR_OR_NULL(pdev)) {
		ret = -EPROBE_DEFER;
		goto dev_np_put;
	}

	info->parchdev = platform_get_drvdata(pdev);
	platform_device_put(pdev);
	if (!info->parchdev) {
		ret = -EPROBE_DEFER;
		goto dev_np_put;
	}

	info->cpu_dev = cpu_dev;
	info->clu_id = cluster_id;
	info->cpu_id = cpu;
	ret = 0;

dev_np_put:
	of_node_put(dev_np);
cluster_np_put:
	of_node_put(cluster_np);

	return ret;
}

static int sprd_hardware_cpufreq_probe(struct platform_device *pdev)
{
	struct sprd_cpufreq_info *info;
	int ret, cpu;

	global_cpufreq_info = devm_kcalloc(&pdev->dev, nr_cpu_ids,
					   sizeof(struct sprd_cpufreq_info),
					   GFP_KERNEL);
	if (!global_cpufreq_info)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		info = sprd_cpufreq_info_lookup(cpu);
		if (IS_ERR(info))
			return PTR_ERR(info);

		info->pdev = &pdev->dev;
		ret = sprd_cpufreq_info_init(info, cpu);
		if (ret)
			return ret;
	}

	ret = cpufreq_register_driver(&sprd_hardware_cpufreq_driver);
	if (ret)
		dev_err(&pdev->dev, "Failed to reigister cpufreq driver\n");
	else
		dev_info(&pdev->dev, "Succeeded to register cpufreq driver\n");

	return 0;
}

static void sprd_hardware_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&sprd_hardware_cpufreq_driver);
}

static struct platform_driver sprd_hardware_cpufreq_platdrv = {
	.driver = {
		.name	= "sprd-hw-cpufreq",
		.of_match_table = sprd_hardware_cpufreq_of_match,
	},
	.probe		= sprd_hardware_cpufreq_probe,
	.remove		= sprd_hardware_cpufreq_remove,
};

static int __init sprd_hardware_cpufreq_common_init(void)
{
	return platform_driver_register(&sprd_hardware_cpufreq_platdrv);
}

static void __exit sprd_hardware_cpufreq_common_exit(void)
{
	platform_driver_unregister(&sprd_hardware_cpufreq_platdrv);
}

device_initcall(sprd_hardware_cpufreq_common_init);
module_exit(sprd_hardware_cpufreq_common_exit);

MODULE_DESCRIPTION("sprd hardware cpufreq driver");
MODULE_LICENSE("GPL v2");
