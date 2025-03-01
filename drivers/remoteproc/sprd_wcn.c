// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Otto Pflüger
 */

#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>

#include "sprd_common.h"

struct sprd_wcn_subsys_info {
	u32 init_status_offset;
	u32 init_status_boot_magic;
	u32 sys_rst_reg;
	u32 sys_rst_mask;
	u32 sync_addr_reg;
	u32 deep_sleep_reg;
	u32 deep_sleep_mask;
	u32 boot_sel_reg;
};

struct sprd_wcn_info {
	const struct regulator_bulk_data *regulators;
	unsigned int num_regulators;
	u32 dram_offset;
	u32 mem_remap_reg;
	u32 mem_remap_val;
	u32 ap_wcn_sleep_ctrl_reg;
	u32 ap_wcn_sleep_allow_mask;
	u32 deep_sleep_ctrl_reg;
	u32 deep_sleep_ctrl_mask;
	struct sprd_wcn_subsys_info btwf;
	struct sprd_wcn_subsys_info gnss;
};

struct sprd_wcn_shared {
	struct device *dev;
	struct device btwf_dev;
	struct device gnss_dev;

	const struct sprd_wcn_info *info;
	struct regulator_bulk_data *regulators;
	struct gpio_desc *en_gpio;
	struct gpio_desc *rst_gpio;
	struct reset_control *reset;
	struct regmap *ap_pub_apb_regs;
	struct regmap *ap_aon_apb_regs;
	struct regmap *wcn_aon_apb_regs;
	struct regmap *wcn_aon_ahb_regs;

	unsigned int rf_power_count;
	unsigned int sleep_disable_count;

	struct mutex lock;
};

struct sprd_wcn_proc {
	struct device *dev;
	struct sprd_wcn_shared *wcn;
	struct sprd_sipc_subdev sipc;

	const struct sprd_wcn_subsys_info *info;
	struct regmap *wcn_ahb_regs;

	phys_addr_t mem_base;
	size_t mem_size;

	phys_addr_t sync_base;
	void __iomem *sync_virt;
	size_t sync_size;
};

static int sprd_wcn_rf_power_on(struct sprd_wcn_shared *wcn)
{
	int ret;

	if (wcn->rf_power_count++ != 0)
		return 0;

	ret = regulator_bulk_enable(wcn->info->num_regulators, wcn->regulators);
	if (ret < 0) {
		wcn->rf_power_count--;
		return ret;
	}

	gpiod_set_value(wcn->en_gpio, 1);
	usleep_range(500, 1000);
	gpiod_set_value(wcn->rst_gpio, 0);

	return 0;
}

static void sprd_wcn_rf_power_off(struct sprd_wcn_shared *wcn)
{
	if (WARN_ON(wcn->rf_power_count == 0))
		return;

	if (wcn->rf_power_count-- != 1)
		return;

	gpiod_set_value(wcn->rst_gpio, 1);
	gpiod_set_value(wcn->en_gpio, 0);

	regulator_bulk_disable(wcn->info->num_regulators, wcn->regulators);
}

static void sprd_wcn_sleep_disable(struct sprd_wcn_shared *wcn)
{
	if (wcn->sleep_disable_count++ != 0)
		return;

	regmap_write(wcn->ap_aon_apb_regs, wcn->info->ap_wcn_sleep_ctrl_reg + 0x2000,
		     wcn->info->ap_wcn_sleep_allow_mask);
}

static void sprd_wcn_sleep_enable(struct sprd_wcn_shared *wcn)
{
	if (WARN_ON(wcn->sleep_disable_count == 0))
		return;

	if (wcn->sleep_disable_count-- != 1)
		return;

	regmap_write(wcn->ap_aon_apb_regs, wcn->info->ap_wcn_sleep_ctrl_reg + 0x1000,
		     wcn->info->ap_wcn_sleep_allow_mask);
}

static int sprd_wcn_load(struct rproc *rproc, const struct firmware *fw)
{
	struct sprd_wcn_proc *wp = rproc->priv;
	struct sprd_wcn_shared *wcn = wp->wcn;
	void *mem;

	if (fw->size > wp->mem_size) {
		dev_err(wcn->dev, "firmware too large\n");
		return -EINVAL;
	}

	mem = memremap(wp->mem_base, wp->mem_size, MEMREMAP_WC);
	if (!mem) {
		dev_err(wp->dev, "failed to map memory region\n");
		return -EBUSY;
	}
	memcpy(mem, fw->data, fw->size);
	memunmap(mem);

	return 0;
}

static int sprd_wcn_poll_boot_status(struct sprd_wcn_proc *wp)
{
	unsigned int retries = 100;
	u32 init_status;
	int ret = -ETIMEDOUT;

	while (retries--) {
		init_status = readl(wp->sync_virt + wp->info->init_status_offset);
		dev_dbg(wp->dev, "init status: %08x\n", init_status);
		if (init_status == wp->info->init_status_boot_magic) {
			ret = 0;
			break;
		}
		msleep(20);
	}

	return ret;
}

static int sprd_wcn_start(struct rproc *rproc)
{
	struct sprd_wcn_proc *wp = rproc->priv;
	struct sprd_wcn_shared *wcn = wp->wcn;
	int ret;

	mutex_lock(&wcn->lock);

	ret = sprd_wcn_rf_power_on(wcn);
	if (ret < 0)
		goto err_out;

	regmap_write(wcn->ap_pub_apb_regs, wcn->info->mem_remap_reg,
		     wcn->info->mem_remap_val);

	memset_io(wp->sync_virt, 0, wp->sync_size);

	ret = pm_runtime_resume_and_get(wcn->dev);
	if (ret < 0)
		goto err_rf_off;

	ret = reset_control_deassert(wcn->reset);
	if (ret < 0)
		goto err_pm_put;

	sprd_wcn_sleep_disable(wcn);

	/* wait for stable WCN power */
	msleep(20);

	/* assert cpu reset to avoid accidental startup */
	regmap_set_bits(wcn->wcn_aon_ahb_regs, wp->info->sys_rst_reg,
			wp->info->sys_rst_mask);

	/* disable force deep sleep */
	regmap_clear_bits(wcn->wcn_aon_apb_regs, wp->info->deep_sleep_reg,
			  wp->info->deep_sleep_mask);
	/* allow wcn to control deep sleep */
	regmap_set_bits(wcn->wcn_aon_apb_regs, wcn->info->deep_sleep_ctrl_reg,
			wcn->info->deep_sleep_ctrl_mask);
	/* wait for stable subsystem power */
	msleep(20);
	/* select boot from dram */
	regmap_update_bits(wp->wcn_ahb_regs, wp->info->boot_sel_reg,
			   0x3ffffff, 0x3000000 |
			   (u32)(wp->mem_base - wcn->info->dram_offset));
	/* write shared memory address */
	regmap_write(wcn->wcn_aon_ahb_regs, wp->info->sync_addr_reg,
		     (u32)(wp->sync_base - wp->mem_base));
	/* deassert cpu reset */
	regmap_clear_bits(wcn->wcn_aon_ahb_regs, wp->info->sys_rst_reg,
			  wp->info->sys_rst_mask);

	sprd_wcn_sleep_enable(wcn);

	ret = sprd_wcn_poll_boot_status(wp);
	if (ret < 0)
		goto err_cpu_stop;

	mutex_unlock(&wcn->lock);

	return 0;

err_cpu_stop:
	/* assert cpu reset */
	regmap_set_bits(wcn->wcn_aon_ahb_regs, wp->info->sys_rst_reg,
			wp->info->sys_rst_mask);
	/* force deep sleep */
	regmap_set_bits(wcn->wcn_aon_apb_regs, wp->info->deep_sleep_reg,
			wp->info->deep_sleep_mask);
	reset_control_assert(wcn->reset);
err_pm_put:
	pm_runtime_put_sync(wcn->dev);
err_rf_off:
	sprd_wcn_rf_power_off(wcn);
err_out:
	mutex_unlock(&wcn->lock);
	return ret;
}

static int sprd_wcn_stop(struct rproc *rproc)
{
	struct sprd_wcn_proc *wp = rproc->priv;
	struct sprd_wcn_shared *wcn = wp->wcn;

	mutex_lock(&wcn->lock);

	/* needed to access registers from AP */
	sprd_wcn_sleep_disable(wcn);
	/* wait for stable power */
	msleep(40);

	/* assert reset */
	regmap_set_bits(wcn->wcn_aon_ahb_regs, wp->info->sys_rst_reg,
			wp->info->sys_rst_mask);
	/* force deep sleep */
	regmap_set_bits(wcn->wcn_aon_apb_regs, wp->info->deep_sleep_reg,
			wp->info->deep_sleep_mask);

	sprd_wcn_sleep_enable(wcn);
	reset_control_assert(wcn->reset);
	pm_runtime_put_sync_suspend(wcn->dev);
	sprd_wcn_rf_power_off(wcn);

	mutex_unlock(&wcn->lock);

	return 0;
}

static const struct rproc_ops wcn_ops = {
	.load		= sprd_wcn_load,
	.start		= sprd_wcn_start,
	.stop		= sprd_wcn_stop,
};

static int sprd_wcn_parse_memory_region(struct sprd_wcn_proc *wp)
{
	struct device_node *np;
	struct reserved_mem *rmem;

	np = of_parse_phandle(wp->dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(wp->dev, "no memory region specified for\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(np);
	if (!rmem) {
		of_node_put(np);
		dev_err(wp->dev, "failed to look up memory region\n");
		return -EINVAL;
	}

	wp->mem_base = rmem->base;
	wp->mem_size = rmem->size;

	of_node_put(np);

	np = of_parse_phandle(wp->dev->of_node, "sprd,sync-region", 0);
	if (!np) {
		dev_err(wp->dev, "no sync region specified\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(np);
	if (!rmem) {
		of_node_put(np);
		dev_err(wp->dev, "failed to look up sync region\n");
		return -EINVAL;
	}

	wp->sync_base = rmem->base;
	wp->sync_size = rmem->size;
	wp->sync_virt = devm_ioremap_wc(wp->dev, rmem->base, rmem->size);
	if (!wp->sync_virt) {
		dev_err(wp->dev, "failed to map sync region\n");
		return -EBUSY;
	}

	of_node_put(np);

	return 0;
}

static int sprd_wcn_init_proc(struct sprd_wcn_shared *wcn,
			      const struct sprd_wcn_subsys_info *info,
			      const char *node_name,
			      struct device *dev)
{
	struct sprd_wcn_proc *wp;
	struct rproc *rproc;
	const char *fw_name;
	int ret;

	dev->of_node = of_get_child_by_name(wcn->dev->of_node, node_name);
	if (!dev->of_node) {
		dev_err(wcn->dev, "unable to get %s child node\n", node_name);
		return -ENOENT;
	}

	dev->parent = wcn->dev;
	dev_set_name(dev, "%s:%s", dev_name(wcn->dev), node_name);

	ret = device_register(dev);
	if (ret < 0)
		return ret;

	ret = of_property_read_string_index(dev->of_node, "firmware-name",
					    0, &fw_name);
	if (ret < 0) {
		dev_err(dev, "unable to read firmware-name\n");
		goto err;
	}

	rproc = devm_rproc_alloc(dev, node_name, &wcn_ops,
				 fw_name, sizeof(*wp));
	if (!rproc)
		goto err;

	rproc->auto_boot = !strcmp(node_name, "btwf");

	wp = rproc->priv;
	wp->wcn = wcn;
	wp->dev = dev;
	wp->info = info;

	ret = sprd_wcn_parse_memory_region(wp);
	if (ret < 0)
		goto err;

	wp->wcn_ahb_regs = syscon_regmap_lookup_by_phandle(dev->of_node,
							"sprd,syscon-wcn-ahb");
	if (IS_ERR(wp->wcn_ahb_regs)) {
		dev_err(wp->dev, "failed to get wcn-ahb syscon handle\n");
		ret = PTR_ERR(wp->wcn_ahb_regs);
		goto err;
	}

	ret = devm_sprd_rproc_add_sipc_subdev(rproc, dev->of_node, &wp->sipc);
	if (ret < 0) {
		dev_err(wp->dev, "failed to add SIPC subdev\n");
		goto err;
	}

	ret = devm_rproc_add(wp->dev, rproc);
	if (ret < 0) {
		dev_err(wp->dev, "failed to register remoteproc\n");
		goto err;
	}

	dev_set_drvdata(wp->dev, rproc);

	return 0;

err:
	device_unregister(dev);
	return ret;
}

static int sprd_wcn_probe(struct platform_device *pdev)
{
	struct sprd_wcn_shared *wcn;
	int ret;

	wcn = devm_kzalloc(&pdev->dev, sizeof(*wcn), GFP_KERNEL);
	if (!wcn)
		return -ENOMEM;

	mutex_init(&wcn->lock);

	wcn->dev = &pdev->dev;
	platform_set_drvdata(pdev, wcn);

	wcn->info = of_device_get_match_data(wcn->dev);
	if (!wcn->info)
		return -EINVAL;

	ret = devm_pm_runtime_enable(wcn->dev);
	if (ret < 0) {
		dev_err(wcn->dev, "failed to enable runtime PM\n");
		return ret;
	}

	ret = devm_regulator_bulk_get_const(wcn->dev, wcn->info->num_regulators,
					    wcn->info->regulators, &wcn->regulators);
	if (ret < 0) {
		dev_err(wcn->dev, "failed to get regulators\n");
		return ret;
	}

	wcn->en_gpio = devm_gpiod_get(wcn->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(wcn->en_gpio)) {
		dev_err(wcn->dev, "failed to get enable gpio\n");
		return ret;
	}

	wcn->rst_gpio = devm_gpiod_get(wcn->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(wcn->rst_gpio)) {
		dev_err(wcn->dev, "failed to get reset gpio\n");
		return ret;
	}

	wcn->ap_pub_apb_regs = syscon_regmap_lookup_by_phandle(wcn->dev->of_node,
							       "sprd,syscon-ap-pub-apb");
	if (IS_ERR(wcn->ap_pub_apb_regs)) {
		dev_err(wcn->dev, "failed to get ap-pub-apb syscon handle\n");
		return PTR_ERR(wcn->ap_pub_apb_regs);
	}

	wcn->ap_aon_apb_regs = syscon_regmap_lookup_by_phandle(wcn->dev->of_node,
							       "sprd,syscon-ap-aon-apb");
	if (IS_ERR(wcn->ap_aon_apb_regs)) {
		dev_err(wcn->dev, "failed to get ap-aon-apb syscon handle\n");
		return PTR_ERR(wcn->ap_aon_apb_regs);
	}

	wcn->wcn_aon_apb_regs = syscon_regmap_lookup_by_phandle(wcn->dev->of_node,
								"sprd,syscon-wcn-aon-apb");
	if (IS_ERR(wcn->wcn_aon_apb_regs)) {
		dev_err(wcn->dev, "failed to get wcn-aon-apb syscon handle\n");
		return PTR_ERR(wcn->wcn_aon_apb_regs);
	}

	wcn->wcn_aon_ahb_regs = syscon_regmap_lookup_by_phandle(wcn->dev->of_node,
								"sprd,syscon-wcn-aon-ahb");
	if (IS_ERR(wcn->wcn_aon_ahb_regs)) {
		dev_err(wcn->dev, "failed to get wcn-aon-ahb syscon handle\n");
		return PTR_ERR(wcn->wcn_aon_ahb_regs);
	}

	wcn->reset = devm_reset_control_get_shared(wcn->dev, NULL);
	if (IS_ERR(wcn->reset)) {
		dev_err(wcn->dev, "failed to get wcn reset\n");
		return PTR_ERR(wcn->reset);
	}

	device_set_wakeup_capable(wcn->dev, true);

	if (of_property_read_bool(wcn->dev->of_node, "wakeup-source"))
		device_wakeup_enable(wcn->dev);

	ret = sprd_wcn_init_proc(wcn, &wcn->info->btwf, "btwf", &wcn->btwf_dev);
	if (ret < 0)
		return ret;

	ret = sprd_wcn_init_proc(wcn, &wcn->info->gnss, "gnss", &wcn->gnss_dev);
	if (ret < 0) {
		device_unregister(&wcn->btwf_dev);
		return ret;
	}

	return 0;
}

static void sprd_wcn_remove(struct platform_device *pdev)
{
	struct sprd_wcn_shared *wcn = platform_get_drvdata(pdev);

	device_unregister(&wcn->btwf_dev);
	device_unregister(&wcn->gnss_dev);
}

static const struct regulator_bulk_data ums9230_wcn_regulators[] = {
	{ "vdd" },
	{ "vddwifipa" },
	{ "dcxo1v8" },
};

static const struct sprd_wcn_info ums9230_wcn_info = {
	.regulators = ums9230_wcn_regulators,
	.num_regulators = ARRAY_SIZE(ums9230_wcn_regulators),

	.dram_offset = 0x87000000,

	/* ap-pub-apb */
	.mem_remap_reg = 0x9018,
	.mem_remap_val = 0x70,

	/* ap-aon-apb */
	.ap_wcn_sleep_ctrl_reg = 0x034c,
	.ap_wcn_sleep_allow_mask = BIT(16),

	/* wcn-aon-apb */
	.deep_sleep_ctrl_reg = 0x0090,
	.deep_sleep_ctrl_mask = BIT(30),

	.btwf = {
		.init_status_offset = 0,
		.init_status_boot_magic = 0xf0f0f0ff,

		/* wcn-aon-ahb */
		.sys_rst_reg = 0x000c,
		.sys_rst_mask = BIT(6) | BIT(4) | BIT(2) | BIT(0),
		.sync_addr_reg = 0x004c,

		/* wcn-aon-apb */
		.deep_sleep_reg = 0x0098,
		.deep_sleep_mask = BIT(12) | BIT(3) | BIT(2),

		/* wcn-btwf-ahb */
		.boot_sel_reg = 0x0410,
	},

	.gnss = {
		.init_status_offset = 0x34,
		.init_status_boot_magic = 0x12345678,

		/* wcn-aon-ahb */
		.sys_rst_reg = 0x000c,
		.sys_rst_mask = BIT(5) | BIT(1),
		.sync_addr_reg = 0x0050,

		/* wcn-aon-apb */
		.deep_sleep_reg = 0x00c8,
		.deep_sleep_mask = BIT(12) | BIT(3),

		/* wcn-gnss-ahb */
		.boot_sel_reg = 0x0404,
	},
};

static const struct of_device_id sprd_wcn_of_match[] = {
	{ .compatible = "sprd,ums9230-wcn", .data = &ums9230_wcn_info },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_wcn_of_match);

static int sprd_wcn_suspend(struct device *dev)
{
	struct sprd_wcn_shared *wcn = dev_get_drvdata(dev);
	struct rproc *proc;

	if (device_may_wakeup(dev))
		return 0;

	proc = dev_get_drvdata(&wcn->btwf_dev);
	if (proc->state == RPROC_RUNNING)
		sprd_wcn_stop(proc);

	proc = dev_get_drvdata(&wcn->gnss_dev);
	if (proc->state == RPROC_RUNNING)
		sprd_wcn_stop(proc);

	return 0;
}

static int sprd_wcn_resume(struct device *dev)
{
	struct sprd_wcn_shared *wcn = dev_get_drvdata(dev);
	struct rproc *proc;
	int ret;

	if (device_may_wakeup(dev))
		return 0;

	proc = dev_get_drvdata(&wcn->gnss_dev);
	if (proc->state == RPROC_RUNNING) {
		ret = sprd_wcn_start(proc);
		if (ret < 0)
			return ret;
	}

	proc = dev_get_drvdata(&wcn->btwf_dev);
	if (proc->state == RPROC_RUNNING) {
		ret = sprd_wcn_start(proc);
		if (ret < 0)
			goto stop_gnss;
	}

	return 0;

stop_gnss:
	proc = dev_get_drvdata(&wcn->gnss_dev);
	if (proc->state == RPROC_RUNNING)
		sprd_wcn_stop(proc);

	return ret;
}

static struct dev_pm_ops sprd_wcn_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sprd_wcn_suspend, sprd_wcn_resume)
};

static struct platform_driver sprd_wcn_driver = {
	.probe = sprd_wcn_probe,
	.remove = sprd_wcn_remove,
	.driver = {
		.name = "sprd-wcn",
		.of_match_table = sprd_wcn_of_match,
		.pm = &sprd_wcn_pm_ops,
	},
};

module_platform_driver(sprd_wcn_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Unisoc WCN remoteproc driver");
