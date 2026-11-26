// SPDX-License-Identifier: GPL-2.0-only
/*
 * UMS512 GPU clock and voltage requests are sequenced by hardware DVFS.
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/sprd,ums512-clk.h>

#define GPU_MIN_VOLTAGE		0x10
#define GPU_CLOCK_DBG		0x34
#define GPU_DVFS_STATE		0x38
#define GPU_INDEX_MAP		0x48
#define GPU_WORK_INDEX		0x8c
#define GPU_UPDATE_TYPE		0xa4
#define GPU_CLOCK_SEL		GENMASK(14, 12)
#define GPU_CLOCK_DIV		GENMASK(11, 9)
#define GPU_MEM_DIV		GENMASK(5, 3)
#define GPU_SYS_DIV		GENMASK(2, 0)
#define GPU_DVFS_BUSY		BIT(19)
#define GPU_INDEX_MASK		GENMASK(2, 0)
#define GPU_MAP_MASK		(GENMASK(22, 14) | GENMASK(5, 0))
#define GPU_SUPPLY_CLOCKS	2
#define GPU_GATE_CLOCKS		3
#define GPU_SETTLE_US		200

#define TOP_MM_CTRL		0x18
#define TOP_MODEM_CTRL		0x3c
#define TOP_SUBSYS_CTRL		0x150
#define TOP_SHARE_CTRL		0x170
#define TOP_SHARED_CTRL		0x18c
#define TOP_SW_TUNE		BIT(20)
#define TOP_JUDGE_VOLTAGE	GENMASK(3, 1)

static const unsigned long ums512_gpu_rates[] = {
	384000000, 512000000, 614400000, 768000000, 850000000,
};

static const struct clk_parent_data ums512_gpu_parents[] = {
	{ .fw_name = "source-384m" },
	{ .fw_name = "source-512m" },
	{ .fw_name = "source-614m4" },
	{ .fw_name = "source-768m" },
	{ .fw_name = "source-850m" },
};

struct ums512_gpu_dvfs {
	struct device *dev;
	struct clk_hw hw;
	struct regmap *gpu;
	struct regmap *top;
	struct clk_bulk_data clocks[GPU_SUPPLY_CLOCKS + GPU_GATE_CLOCKS];
	u32 saved_top[4];
	unsigned int index;
};

#define to_ums512_gpu(_hw) container_of(_hw, struct ums512_gpu_dvfs, hw)

static int ums512_gpu_wait(struct ums512_gpu_dvfs *gpu, unsigned int index)
{
	unsigned int value;
	int ret;

	/* The request can complete before BUSY is observed by software. */
	fsleep(GPU_SETTLE_US);
	ret = regmap_read_poll_timeout(gpu->gpu, GPU_DVFS_STATE, value,
				       !(value & GPU_DVFS_BUSY), 10, 1000);
	if (ret)
		return ret;
	ret = regmap_read(gpu->gpu, GPU_CLOCK_DBG, &value);
	if (ret)
		return ret;
	if (FIELD_GET(GPU_CLOCK_SEL, value) != index + 1 ||
	    FIELD_GET(GPU_CLOCK_DIV, value) || FIELD_GET(GPU_MEM_DIV, value) ||
	    FIELD_GET(GPU_SYS_DIV, value) != 3)
		return -EIO;

	return 0;
}

static int ums512_gpu_request(struct ums512_gpu_dvfs *gpu, unsigned int index)
{
	unsigned int previous = gpu->index;
	int ret;

	ret = regmap_update_bits(gpu->gpu, GPU_WORK_INDEX, GPU_INDEX_MASK, index);
	if (!ret)
		ret = ums512_gpu_wait(gpu, index);
	if (ret) {
		regmap_update_bits(gpu->gpu, GPU_WORK_INDEX, GPU_INDEX_MASK, previous);
		if (ums512_gpu_wait(gpu, previous))
			dev_err(gpu->dev, "failed to restore GPU DVFS index %u\n", previous);
		return ret;
	}
	gpu->index = index;
	return 0;
}

static int ums512_gpu_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	unsigned long distance = ULONG_MAX;
	unsigned int i, best = 0;
	bool found = false;

	for (i = 0; i < ARRAY_SIZE(ums512_gpu_rates); i++) {
		unsigned long rate = ums512_gpu_rates[i];
		unsigned long delta = rate > req->rate ? rate - req->rate : req->rate - rate;

		if (rate < req->min_rate || rate > req->max_rate || delta >= distance)
			continue;
		best = i;
		distance = delta;
		found = true;
	}
	if (!found)
		return -EINVAL;
	req->best_parent_hw = clk_hw_get_parent_by_index(hw, best);
	if (!req->best_parent_hw)
		return -ENOENT;
	req->best_parent_rate = clk_hw_get_rate(req->best_parent_hw);
	if (req->best_parent_rate != ums512_gpu_rates[best])
		return -EINVAL;
	req->rate = req->best_parent_rate;
	return 0;
}

static unsigned long ums512_gpu_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	return ums512_gpu_rates[to_ums512_gpu(hw)->index];
}

static u8 ums512_gpu_get_parent(struct clk_hw *hw)
{
	return to_ums512_gpu(hw)->index;
}

static int ums512_gpu_set_parent(struct clk_hw *hw, u8 index)
{
	if (index >= ARRAY_SIZE(ums512_gpu_rates))
		return -EINVAL;
	return ums512_gpu_request(to_ums512_gpu(hw), index);
}

static int ums512_gpu_set_rate_and_parent(struct clk_hw *hw, unsigned long rate,
					  unsigned long parent_rate, u8 index)
{
	if (index >= ARRAY_SIZE(ums512_gpu_rates) ||
	    rate != ums512_gpu_rates[index] || rate != parent_rate)
		return -EINVAL;
	return ums512_gpu_set_parent(hw, index);
}

static int ums512_gpu_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	return ums512_gpu_set_rate_and_parent(hw, rate, parent_rate,
					      to_ums512_gpu(hw)->index);
}

static int ums512_gpu_enable(struct clk_hw *hw)
{
	struct ums512_gpu_dvfs *gpu = to_ums512_gpu(hw);

	return clk_bulk_enable(GPU_GATE_CLOCKS, gpu->clocks + GPU_SUPPLY_CLOCKS);
}

static void ums512_gpu_disable(struct clk_hw *hw)
{
	struct ums512_gpu_dvfs *gpu = to_ums512_gpu(hw);

	clk_bulk_disable(GPU_GATE_CLOCKS, gpu->clocks + GPU_SUPPLY_CLOCKS);
}

static const struct clk_ops ums512_gpu_ops = {
	.enable = ums512_gpu_enable,
	.disable = ums512_gpu_disable,
	.get_parent = ums512_gpu_get_parent,
	.set_parent = ums512_gpu_set_parent,
	.recalc_rate = ums512_gpu_recalc_rate,
	.determine_rate = ums512_gpu_determine_rate,
	.set_rate = ums512_gpu_set_rate,
	.set_rate_and_parent = ums512_gpu_set_rate_and_parent,
};

static int ums512_gpu_runtime_resume(struct device *dev)
{
	struct ums512_gpu_dvfs *gpu = dev_get_drvdata(dev);
	unsigned int i, voltage;
	int ret;

	ret = clk_bulk_prepare_enable(GPU_SUPPLY_CLOCKS, gpu->clocks);
	if (ret)
		return ret;
	ret = clk_bulk_prepare(GPU_GATE_CLOCKS, gpu->clocks + GPU_SUPPLY_CLOCKS);
	if (ret)
		goto disable_clocks;

	/* Core and memory run together; the system clock is divided by four. */
	for (i = 0; i < 8; i++) {
		unsigned int index = min(i, 4U);

		voltage = index ? (index < 3 ? 1 : 2) : 0;
		ret = regmap_update_bits(gpu->gpu, GPU_INDEX_MAP + i * 4,
					 GPU_MAP_MASK,
					 (3 << 20) | (voltage << 14) | (index + 1));
		if (ret)
			goto unprepare_gates;
	}
	ret = regmap_update_bits(gpu->gpu, GPU_MIN_VOLTAGE, GPU_INDEX_MASK, 0);
	if (!ret)
		ret = regmap_update_bits(gpu->gpu, GPU_UPDATE_TYPE, GENMASK(1, 0), 1);
	if (!ret)
		ret = ums512_gpu_request(gpu, gpu->index);
	if (!ret)
		return 0;

unprepare_gates:
	clk_bulk_unprepare(GPU_GATE_CLOCKS, gpu->clocks + GPU_SUPPLY_CLOCKS);
disable_clocks:
	clk_bulk_disable_unprepare(GPU_SUPPLY_CLOCKS, gpu->clocks);
	return ret;
}

static int ums512_gpu_runtime_suspend(struct device *dev)
{
	struct ums512_gpu_dvfs *gpu = dev_get_drvdata(dev);

	clk_bulk_unprepare(GPU_GATE_CLOCKS, gpu->clocks + GPU_SUPPLY_CLOCKS);
	clk_bulk_disable_unprepare(GPU_SUPPLY_CLOCKS, gpu->clocks);
	return 0;
}

static int ums512_gpu_validate_voltages(struct ums512_gpu_dvfs *gpu)
{
	static const struct {
		u16 reg, minimum[3];
	} grades[] = {
		{ 0xe4, { 700000 / 3125, 750000 / 3125, 800000 / 3125 } },
		{ 0x1a4, { 750000 / 3125, 800000 / 3125, 850000 / 3125 } },
	};
	unsigned int value, i, j;
	int ret;

	/* These banks are shared with firmware-owned MM/modem consumers. */
	for (i = 0; i < ARRAY_SIZE(grades); i++) {
		ret = regmap_read(gpu->top, grades[i].reg, &value);
		if (ret)
			return ret;
		for (j = 0; j < 3; j++)
			if (((value >> (j * 9)) & 0x1ff) < grades[i].minimum[j])
				return -EINVAL;
	}
	ret = regmap_read(gpu->top, TOP_SHARE_CTRL, &value);
	if (ret)
		return ret;
	return value & BIT(0) ? 0 : -EINVAL;
}

/* CPU DVFS owns bits 6/7 of TOP_SUBSYS_CTRL; leave them intact. */
static const struct {
	u16 reg;
	u32 mask, value;
} ums512_gpu_top_config[] = {
	{ TOP_MM_CTRL, TOP_JUDGE_VOLTAGE | TOP_SW_TUNE, 2 << 1 },
	{ TOP_MODEM_CTRL, TOP_JUDGE_VOLTAGE | TOP_SW_TUNE, 2 << 1 },
	{ TOP_SHARED_CTRL, TOP_JUDGE_VOLTAGE | TOP_SW_TUNE, 2 << 1 },
	{ TOP_SUBSYS_CTRL, BIT(0) | BIT(1), 0 },
};

static void ums512_gpu_restore_top(void *data)
{
	struct ums512_gpu_dvfs *gpu = data;
	int i, ret;

	for (i = ARRAY_SIZE(ums512_gpu_top_config) - 1; i >= 0; i--) {
		ret = regmap_update_bits(gpu->top, ums512_gpu_top_config[i].reg,
					 ums512_gpu_top_config[i].mask, gpu->saved_top[i]);
		if (ret)
			dev_err(gpu->dev, "failed to restore TOP register %#x: %d\n",
				ums512_gpu_top_config[i].reg, ret);
	}
}

static int ums512_gpu_init_top(struct ums512_gpu_dvfs *gpu)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ums512_gpu_top_config); i++) {
		ret = regmap_read(gpu->top, ums512_gpu_top_config[i].reg, &gpu->saved_top[i]);
		if (ret)
			return ret;
	}
	ret = devm_add_action_or_reset(gpu->dev, ums512_gpu_restore_top, gpu);
	if (ret)
		return ret;

	/* Preserve firmware MM/modem votes through the shared TOP arbiter. */
	for (i = 0; i < ARRAY_SIZE(ums512_gpu_top_config); i++) {
		ret = regmap_update_bits(gpu->top, ums512_gpu_top_config[i].reg,
					 ums512_gpu_top_config[i].mask,
					 ums512_gpu_top_config[i].value);
		if (ret)
			return ret;
	}
	return 0;
}

static int ums512_gpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ums512_gpu_dvfs *gpu;
	struct clk_hw_onecell_data *clocks;
	struct clk_init_data init = {
		.name = "ums512-gpu-dvfs",
		.ops = &ums512_gpu_ops,
		.parent_data = ums512_gpu_parents,
		.num_parents = ARRAY_SIZE(ums512_gpu_parents),
	};
	struct clk *top_clk;
	int ret;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	clocks = devm_kzalloc(dev, struct_size(clocks, hws, CLK_GPU_DVFS_NUM), GFP_KERNEL);
	if (!gpu || !clocks)
		return -ENOMEM;
	gpu->dev = dev;
	gpu->gpu = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(gpu->gpu))
		return PTR_ERR(gpu->gpu);
	gpu->top = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,top-dvfs");
	if (IS_ERR(gpu->top))
		return PTR_ERR(gpu->top);
	gpu->clocks[0].id = "enable";
	gpu->clocks[1].id = "dvfs";
	gpu->clocks[2].id = "core-gate";
	gpu->clocks[3].id = "mem-gate";
	gpu->clocks[4].id = "bus-gate";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(gpu->clocks), gpu->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get DVFS clocks\n");
	top_clk = devm_clk_get_enabled(dev, "dvfs");
	if (IS_ERR(top_clk))
		return PTR_ERR(top_clk);
	ret = ums512_gpu_validate_voltages(gpu);
	if (ret)
		return dev_err_probe(dev, ret, "incompatible firmware voltage banks\n");

	ret = ums512_gpu_init_top(gpu);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, gpu);
	pm_runtime_set_suspended(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	gpu->hw.init = &init;
	ret = devm_clk_hw_register(dev, &gpu->hw);
	if (ret)
		return ret;
	clocks->num = CLK_GPU_DVFS_NUM;
	clocks->hws[CLK_GPU_DVFS_CORE] = &gpu->hw;
	clocks->hws[CLK_GPU_DVFS_MEM] =
		devm_clk_hw_register_fixed_factor_parent_hw(dev, "ums512-gpu-dvfs-mem",
							    &gpu->hw, 0, 1, 1);
	if (IS_ERR(clocks->hws[CLK_GPU_DVFS_MEM]))
		return PTR_ERR(clocks->hws[CLK_GPU_DVFS_MEM]);
	clocks->hws[CLK_GPU_DVFS_SYS] =
		devm_clk_hw_register_fixed_factor_parent_hw(dev, "ums512-gpu-dvfs-bus",
							    &gpu->hw, 0, 1, 4);
	if (IS_ERR(clocks->hws[CLK_GPU_DVFS_SYS]))
		return PTR_ERR(clocks->hws[CLK_GPU_DVFS_SYS]);
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clocks);
}

static const struct of_device_id ums512_gpu_of_match[] = {
	{ .compatible = "sprd,ums512-gpu-dvfs" },
	{ }
};
MODULE_DEVICE_TABLE(of, ums512_gpu_of_match);

static const struct dev_pm_ops ums512_gpu_pm_ops = {
	RUNTIME_PM_OPS(ums512_gpu_runtime_suspend, ums512_gpu_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static struct platform_driver ums512_gpu_driver = {
	.probe = ums512_gpu_probe,
	.driver = {
		.name = "ums512-gpu-dvfs",
		.of_match_table = ums512_gpu_of_match,
		.pm = pm_ptr(&ums512_gpu_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ums512_gpu_driver);

MODULE_DESCRIPTION("UMS512 GPU hardware DVFS clocks");
MODULE_LICENSE("GPL");
