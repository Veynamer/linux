// SPDX-License-Identifier: GPL-2.0-only
/*
 * Unisoc hardware CPUFreq v1 driver.
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/soc/sprd/cpu-dvfs.h>

static struct cpufreq_driver sprd_cpufreq_driver;

static int sprd_cpufreq_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct sprd_cpu_dvfs_cluster *cluster = policy->driver_data;

	return sprd_cpu_dvfs_set_target(cluster, index);
}

static unsigned int sprd_cpufreq_get_frequency(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct sprd_cpu_dvfs_cluster *cluster;

	if (!policy || !policy->driver_data)
		return 0;
	cluster = policy->driver_data;
	return sprd_cpu_dvfs_get_frequency(cluster);
}

static int sprd_cpufreq_policy_init(struct cpufreq_policy *policy)
{
	struct sprd_cpu_dvfs *dvfs = cpufreq_get_driver_data();
	struct sprd_cpu_dvfs_cluster *cluster;
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

static void sprd_cpufreq_policy_exit(struct cpufreq_policy *policy)
{
	policy->freq_table = NULL;
	policy->driver_data = NULL;
}

static struct cpufreq_driver sprd_cpufreq_driver = {
	.name = "sprd-cpufreq-v1",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sprd_cpufreq_set_target,
	.get = sprd_cpufreq_get_frequency,
	.init = sprd_cpufreq_policy_init,
	.exit = sprd_cpufreq_policy_exit,
	.suspend = cpufreq_generic_suspend,
	.register_em = cpufreq_register_em_with_opp,
};

static void sprd_cpufreq_unregister(void *arg)
{
	cpufreq_unregister_driver(&sprd_cpufreq_driver);
	sprd_cpufreq_driver.driver_data = NULL;
}

static int sprd_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sprd_cpu_dvfs *dvfs;
	int ret;

	if (sprd_cpufreq_driver.driver_data)
		return -EBUSY;
	dvfs = devm_sprd_cpu_dvfs_init(pdev);
	if (IS_ERR(dvfs))
		return PTR_ERR(dvfs);

	platform_set_drvdata(pdev, dvfs);
	sprd_cpufreq_driver.driver_data = dvfs;
	ret = cpufreq_register_driver(&sprd_cpufreq_driver);
	if (ret) {
		sprd_cpufreq_driver.driver_data = NULL;
		return dev_err_probe(dev, ret, "failed to register cpufreq driver\n");
	}
	return devm_add_action_or_reset(dev, sprd_cpufreq_unregister, dvfs);
}

static int sprd_cpufreq_resume(struct device *dev)
{
	struct sprd_cpu_dvfs *dvfs = dev_get_drvdata(dev);

	return sprd_cpu_dvfs_resume(dvfs);
}

static DEFINE_SIMPLE_DEV_PM_OPS(sprd_cpufreq_pm_ops, NULL, sprd_cpufreq_resume);

static const struct of_device_id sprd_cpufreq_of_match[] = {
	{ .compatible = "sprd,ums512-cpufreq", .data = &ums512_cpu_dvfs_data },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_cpufreq_of_match);

static struct platform_driver sprd_cpufreq_platform_driver = {
	.probe = sprd_cpufreq_probe,
	.driver = {
		.name = "sprd-cpufreq-v1",
		.of_match_table = sprd_cpufreq_of_match,
		.pm = pm_sleep_ptr(&sprd_cpufreq_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sprd_cpufreq_platform_driver);

MODULE_DESCRIPTION("Unisoc hardware CPU frequency scaling v1");
MODULE_LICENSE("GPL");
