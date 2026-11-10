// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 lemona DeV
 *
 * Based on sprd-cpufreq-v2.c,
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/arm-smccc.h>
#include <linux/cpufreq.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SPRD_CPUFREQ_MIN_TEMPERATURE	(-274)

#define SPRD_SIP_SVC_DVFS_REV						\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0600))

#define SPRD_SIP_SVC_DVFS_CHIP_CONFIG					\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0601))

#define SPRD_SIP_SVC_DVFS_PHY_ENABLE					\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0602))

#define SPRD_SIP_SVC_DVFS_TABLE_UPDATE					\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0603))

#define SPRD_SIP_SVC_DVFS_CLUSTER_CONFIG				\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0604))

#define SPRD_SIP_SVC_DVFS_FREQ_SET					\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0605))

#define SPRD_SIP_SVC_DVFS_FREQ_GET					\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0606))

#define SPRD_SIP_SVC_DVFS_INDEX_INFO					\
	(ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,			\
			    ARM_SMCCC_SMC_32,				\
			    ARM_SMCCC_OWNER_SIP,			\
			    0x0607))

struct sprd_cpufreq_data {
	struct device *cpu_dev;
	u32 cluster;
};

static int sprd_cpufreq_set_target(struct cpufreq_policy *policy,
				   unsigned int index)
{
	struct sprd_cpufreq_data *data = policy->driver_data;
	struct arm_smccc_res res;

	arm_smccc_smc(SPRD_SIP_SVC_DVFS_FREQ_SET, data->cluster,
		      index, 0, 0, 0, 0, 0, &res);

	if (res.a0)
		return -EINVAL;

	return 0;
}

static unsigned int sprd_cpufreq_get_freq(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct sprd_cpufreq_data *data;
	struct arm_smccc_res res;

	if (!policy)
		return 0;

	data = policy->driver_data;

	arm_smccc_smc(SPRD_SIP_SVC_DVFS_FREQ_GET, data->cluster,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return 0;

	do_div(res.a1, 1000);

	return (u32)res.a1;
}

static int sprd_cpufreq_table_init(struct cpufreq_policy *policy,
				   int temperature)
{
	struct sprd_cpufreq_data *data = policy->driver_data;
	struct arm_smccc_res res;
	unsigned int i, length;
	int ret;

	arm_smccc_smc(SPRD_SIP_SVC_DVFS_TABLE_UPDATE, data->cluster,
		      temperature, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(data->cpu_dev,
			"failed to get table for temperature %d\n",
			temperature);
		return -EINVAL;
	}

	length = res.a1;

	for (i = 0; i < length; i++) {
		/* gets the pair of frequency and voltage */
		arm_smccc_smc(SPRD_SIP_SVC_DVFS_INDEX_INFO, data->cluster,
			      i, 0, 0, 0, 0, 0, &res);
		if (res.a0) {
			dev_err(data->cpu_dev, "failed to get freq %d\n", i);
			return -EINVAL;
		}

		dev_dbg(data->cpu_dev, "[%d] freq: %lu voltage: %lu\n",
			i, res.a1, res.a2);

		ret = dev_pm_opp_add(data->cpu_dev, res.a1, res.a2);
		if (ret) {
			dev_warn(data->cpu_dev, "failed to add freq %lu: %d\n",
				 res.a1, ret);
		}
	}

	arm_smccc_smc(SPRD_SIP_SVC_DVFS_FREQ_GET, data->cluster,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(data->cpu_dev, "failed to get initial frequency\n");
		return -EINVAL;
	}

	dev_dbg(data->cpu_dev, "initial frequency: %lu\n", res.a1);

	arm_smccc_smc(SPRD_SIP_SVC_DVFS_PHY_ENABLE, data->cluster,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(data->cpu_dev, "failed to enable DVFS\n");
		return -EINVAL;
	}

	ret = dev_pm_opp_init_cpufreq_table(data->cpu_dev,
					    &policy->freq_table);
	if (ret) {
		dev_err(data->cpu_dev, "failed to init cpufreq table\n");
		return ret;
	}

	return 0;
}
/*
static int sprd_cpufreq_ver_init(struct device *dev)
{
	struct arm_smccc_res res;
	u64 version;
	int ret;

	ret = of_property_read_u64(dev->of_node, "sprd,soc-version", &version);
	if (!ret) {
		arm_smccc_smc(SPRD_SIP_SVC_DVFS_CHIP_CONFIG,
			      version, 0, 0, 0, 0, 0, 0, &res);
		if (res.a0) {
			dev_err(dev, "%pOF: failed to set SoC version\n", np);
			return -EINVAL;
		}
	}

	return 0;
}
*/
static int sprd_cpufreq_cluster_init(struct device *dev,
				     struct device_node *np,
				     u32 cluster)
{
	struct arm_smccc_res res;
	struct nvmem_cell *cell;
	u32 margin, step;
	u64 version;
	size_t len;
	u8 *buf;
	int ret;

	ret = of_property_read_u32(np, "sprd,voltage-step", &step);
	if (ret) {
		dev_warn(dev, "sprd,voltage-step is not found\n");
		step = 0U;
	}

	ret = of_property_read_u32(np, "sprd,voltage-margin", &margin);
	if (ret) {
		dev_warn(dev, "sprd,voltage-margin get error\n");
		margin = 0U;
	}

	cell = of_nvmem_cell_get(np, "dvfs_bin");
	if (IS_ERR(cell))
		return dev_err_probe(dev, PTR_ERR(cell),
				     "%pOF: failed to get DVFS bin\n", np);

	buf = nvmem_cell_read(cell, &len);
	if (IS_ERR(buf)) {
		dev_err(dev, "%pOF: failed to read DVFS bin\n", np);
		nvmem_cell_put(cell);
		return PTR_ERR(buf);
	}

	dev_dbg(dev, "%pOF: DVFS bin is %d\n", np, *buf);

	arm_smccc_smc(SPRD_SIP_SVC_DVFS_CLUSTER_CONFIG, cluster,
		      *buf, margin, step, 0, 0, 0, &res);
	kfree(buf);
	nvmem_cell_put(cell);
	if (res.a0) {
		dev_err(dev, "%pOF: failed to set DVFS_CLUSTER_CONFIG\n", np);
		return -EINVAL;
	}

	ret = of_property_read_u64(dev->of_node, "sprd,soc-version", &version);
	if (!ret) {
		arm_smccc_smc(SPRD_SIP_SVC_DVFS_CHIP_CONFIG,
			      version, 0, 0, 0, 0, 0, 0, &res);
		if (res.a0) {
			dev_err(dev, "%pOF: failed to set SoC version\n", np);
			return -EINVAL;
		}
	}

	return 0;
}

static int sprd_cpufreq_init(struct cpufreq_policy *policy)
{
	struct device *cpufreq_dev = cpufreq_get_driver_data();
	struct sprd_cpufreq_data *data;
	struct of_phandle_args args;
	struct device *cpu_dev;
	struct device_node *np;
	int ret, temperature;
	u32 val;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev)
		return -ENODEV;

	ret = of_perf_domain_get_sharing_cpumask(policy->cpu,
						 "performance-domains",
						 "#performance-domain-cells",
						 policy->cpus, &args);
	of_node_put(args.np);
	if (ret)
		return ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpu_dev = cpu_dev;
	data->cluster = args.args[0];

	policy->driver_data = data;
	policy->dvfs_possible_from_any_cpu = true;

	for_each_available_child_of_node(cpufreq_dev->of_node, np) {
		ret = of_property_read_u32(np, "reg", &val);
		if (!ret && val == args.args[0])
			break;
	}

	if (!np) {
		dev_err(cpu_dev, "cluster %d not found\n", args.args[0]);
		kfree(data);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "sprd,transition-delay-us", &val);
	if (!ret)
		policy->transition_delay_us = val;

	ret = of_property_read_u32(np, "sprd,default-temperature", &val);
	if (!ret)
		temperature = val;
	else
		temperature = SPRD_CPUFREQ_MIN_TEMPERATURE;

	ret = sprd_cpufreq_table_init(policy, temperature);
	if (ret) {
		dev_pm_opp_remove_all_dynamic(cpu_dev);
		goto unlock_ret_error;
	}

	dev_pm_opp_set_sharing_cpus(cpu_dev, policy->cpus);

	return 0;

unlock_ret_error:
	kfree(data);
	return ret;
}

static void sprd_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct sprd_cpufreq_data *data = policy->driver_data;

	dev_pm_opp_free_cpufreq_table(data->cpu_dev, &policy->freq_table);
	dev_pm_opp_remove_all_dynamic(data->cpu_dev);
	kfree(data);
}

static struct cpufreq_driver sprd_cpufreq_driver = {
	.name = "sprd",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
		 CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sprd_cpufreq_set_target,
	.get = sprd_cpufreq_get_freq,
	.init = sprd_cpufreq_init,
	.exit = sprd_cpufreq_exit,
	.register_em = cpufreq_register_em_with_opp,
	.set_boost = cpufreq_boost_set_sw,
};

static int sprd_cpufreq_probe(struct platform_device *pdev)
{
	struct arm_smccc_res res;
	struct device_node *np;
	u32 cluster;
	int ret;

	arm_smccc_smc(SPRD_SIP_SVC_DVFS_REV,
		      0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(&pdev->dev, "failed to initialize DVFS\n");
		return -EINVAL;
	}

	for_each_available_child_of_node(pdev->dev.of_node, np) {
		ret = of_property_read_u32(np, "reg", &cluster);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "%pOF: missing reg\n", np);

		ret = sprd_cpufreq_cluster_init(&pdev->dev, np, cluster);
		if (ret)
			return ret;
	}

	sprd_cpufreq_driver.driver_data = &pdev->dev;

	return cpufreq_register_driver(&sprd_cpufreq_driver);
}

static const struct of_device_id sprd_cpufreq_of_match[] = {
	{ .compatible = "sprd,cpufreq-v1" },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_cpufreq_of_match);

static struct platform_driver sprd_cpufreq_platform_driver = {
	.probe = sprd_cpufreq_probe,
	.driver = {
		.name = "sprd-cpufreq-v1",
		.of_match_table = sprd_cpufreq_of_match,
	},
};
module_platform_driver(sprd_cpufreq_platform_driver);

MODULE_DESCRIPTION("Spreadtrum cpufreq driver");
MODULE_LICENSE("GPL");
