// SPDX-License-Identifier: GPL-2.0-only
/*
 * Unisoc SoC information driver
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/bitops.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sys_soc.h>

#define SPRD_SOCINFO_MAX_ID_REGS	2

struct sprd_socinfo_regs {
	u32 chip_id[SPRD_SOCINFO_MAX_ID_REGS];
	u32 plat_id[SPRD_SOCINFO_MAX_ID_REGS];
	u32 impl_id;
	u32 mft_id;
	u32 ver_id;
	u32 chip_id_mask;
	u32 plat_id_mask;
	u32 impl_id_mask;
	u32 mft_id_mask;
	u32 ver_id_mask;
	u8 chip_id_count;
	u8 plat_id_count;
};

struct sprd_socinfo_desc {
	const char *soc_id;
	const char *family;
	const struct sprd_socinfo_regs regs;
};

struct sprd_socinfo {
	struct soc_device *soc_dev;
};

/*
 * UMS512 AON APB registers:
 *   REG_AON_APB_RF_AON_CHIP_ID0	0x00e0
 *   REG_AON_APB_RF_AON_CHIP_ID1	0x00e4
 *   REG_AON_APB_RF_AON_PLAT_ID0	0x00e8
 *   REG_AON_APB_RF_AON_PLAT_ID1	0x00ec
 *   REG_AON_APB_RF_AON_IMPL_ID	0x00f0
 *   REG_AON_APB_RF_AON_MFT_ID	0x00f4
 *   REG_AON_APB_RF_AON_VER_ID	0x00f8
 */
static const struct sprd_socinfo_desc ums512_socinfo_desc = {
	.soc_id = "UMS512",
	.family = "Unisoc UMS512",
	.regs = {
		.chip_id = { 0x00e0, 0x00e4 },
		.plat_id = { 0x00e8, 0x00ec },
		.impl_id = 0x00f0,
		.mft_id = 0x00f4,
		.ver_id = 0x00f8,
		.chip_id_mask = 0xffffffff,
		.plat_id_mask = 0xffffffff,
		.impl_id_mask = 0xffffffff,
		.mft_id_mask = 0xffffffff,
		.ver_id_mask = 0xffffffff,
		.chip_id_count = 2,
		.plat_id_count = 2,
	},
};

/*
 * UMS9360/UMS9621/UMS9632 AON APB registers:
 *   REG_AON_APB_PLATFORM_ID_L	0x00e0
 *   REG_AON_APB_PROJECT_ID_L	0x00e8
 *   REG_AON_APB_DERIVED_ID	0x00f0
 *   REG_AON_APB_MANUFACTURE_ID	0x00f4
 *   REG_AON_APB_IMPLEMENTATION_ID	0x00f8
 */
#define SPRD_UMS9XXX_SOCINFO_REGS					\
	.regs = {							\
		.chip_id = { 0x00e8 },					\
		.plat_id = { 0x00e0 },					\
		.impl_id = 0x00f8,					\
		.mft_id = 0x00f4,					\
		.ver_id = 0x00f0,					\
		.chip_id_mask = 0xffffffff,				\
		.plat_id_mask = 0xffffffff,				\
		.impl_id_mask = 0x00ff,					\
		.mft_id_mask = 0x00ff,					\
		.ver_id_mask = 0x001f,					\
		.chip_id_count = 1,					\
		.plat_id_count = 1,					\
	}

/*
 * UMS9230 (qogirl6) AON APB registers:
 *   REG_AON_APB_AON_CHIP_ID0	0x00e0
 *   REG_AON_APB_AON_CHIP_ID1	0x00e4
 *   REG_AON_APB_AON_PLAT_ID0	0x00e8
 *   REG_AON_APB_AON_PLAT_ID1	0x00ec
 *   REG_AON_APB_AON_IMPL_ID	0x00f0
 *   REG_AON_APB_AON_MFT_ID	0x00f4
 *   REG_AON_APB_AON_VER_ID	0x00f8
 */
static const struct sprd_socinfo_desc ums9230_socinfo_desc = {
	.soc_id = "UMS9230",
	.family = "Unisoc UMS9230",
	.regs = {
		.chip_id = { 0x00e0, 0x00e4 },
		.plat_id = { 0x00e8, 0x00ec },
		.impl_id = 0x00f0,
		.mft_id = 0x00f4,
		.ver_id = 0x00f8,
		.chip_id_mask = 0xffffffff,
		.plat_id_mask = 0xffffffff,
		.impl_id_mask = 0xffffffff,
		.mft_id_mask = 0xffffffff,
		.ver_id_mask = 0xffffffff,
		.chip_id_count = 2,
		.plat_id_count = 2,
	},
};

static const struct sprd_socinfo_desc ums9360_socinfo_desc = {
	.soc_id = "UMS9360",
	.family = "Unisoc UMS9360",
	SPRD_UMS9XXX_SOCINFO_REGS,
};

static const struct sprd_socinfo_desc ums9621_socinfo_desc = {
	.soc_id = "UMS9621",
	.family = "Unisoc UMS9621",
	SPRD_UMS9XXX_SOCINFO_REGS,
};

static const struct sprd_socinfo_desc ums9632_socinfo_desc = {
	.soc_id = "UMS9632",
	.family = "Unisoc UMS9632",
	SPRD_UMS9XXX_SOCINFO_REGS,
};

static int sprd_socinfo_read_id(struct regmap *map, u32 reg, u32 mask, u32 *val)
{
	u32 tmp;
	int ret;

	ret = regmap_read(map, reg, &tmp);
	if (ret)
		return ret;

	if (!mask)
		mask = U32_MAX;

	*val = (tmp & mask) >> __ffs(mask);

	return 0;
}

static int sprd_socinfo_read_wide(struct regmap *map, const u32 *regs,
				  u8 count, u32 mask, u64 *val)
{
	u64 id = 0;
	u32 tmp;
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = sprd_socinfo_read_id(map, regs[i], mask, &tmp);
		if (ret)
			return ret;

		id |= (u64)tmp << (i * 32);
	}

	*val = id;

	return 0;
}

static int sprd_socinfo_probe(struct platform_device *pdev)
{
	const struct sprd_socinfo_desc *desc;
	struct soc_device_attribute *attr;
	struct sprd_socinfo *priv;
	struct regmap *map;
	u64 chip_id, plat_id;
	u32 impl_id, mft_id, ver_id;
	int ret;

	desc = of_device_get_match_data(&pdev->dev);
	if (!desc)
		return -ENODEV;

	map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
					      "sprd,syscon-aon-apb");
	if (IS_ERR(map))
		return dev_err_probe(&pdev->dev, PTR_ERR(map),
				     "failed to get AON APB syscon\n");

	ret = sprd_socinfo_read_wide(map, desc->regs.chip_id,
				     desc->regs.chip_id_count,
				     desc->regs.chip_id_mask, &chip_id);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to read chip ID\n");

	ret = sprd_socinfo_read_wide(map, desc->regs.plat_id,
				     desc->regs.plat_id_count,
				     desc->regs.plat_id_mask, &plat_id);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to read platform ID\n");

	ret = sprd_socinfo_read_id(map, desc->regs.impl_id,
				   desc->regs.impl_id_mask, &impl_id);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to read implementation ID\n");

	ret = sprd_socinfo_read_id(map, desc->regs.mft_id,
				   desc->regs.mft_id_mask, &mft_id);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to read manufacturer ID\n");

	ret = sprd_socinfo_read_id(map, desc->regs.ver_id,
				   desc->regs.ver_id_mask, &ver_id);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to read version ID\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	attr = devm_kzalloc(&pdev->dev, sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	attr->family = desc->family;
	attr->soc_id = desc->soc_id;
	attr->revision = devm_kasprintf(&pdev->dev, GFP_KERNEL, "0x%x",
					ver_id);
	if (!attr->revision)
		return -ENOMEM;

	attr->serial_number = devm_kasprintf(&pdev->dev, GFP_KERNEL,
					     "0x%016llx", chip_id);
	if (!attr->serial_number)
		return -ENOMEM;

	priv->soc_dev = soc_device_register(attr);
	if (IS_ERR(priv->soc_dev))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->soc_dev),
				     "failed to register SoC device\n");

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev,
		 "%s SoC detected: chip 0x%016llx, plat 0x%016llx, impl 0x%x, mft 0x%x, ver 0x%x\n",
		 desc->soc_id, chip_id, plat_id, impl_id, mft_id, ver_id);

	return 0;
}

static void sprd_socinfo_remove(struct platform_device *pdev)
{
	struct sprd_socinfo *priv = platform_get_drvdata(pdev);

	soc_device_unregister(priv->soc_dev);
}

static const struct of_device_id sprd_socinfo_of_match[] = {
	{ .compatible = "sprd,ums512-socinfo", .data = &ums512_socinfo_desc },
	{ .compatible = "sprd,ums9230-socinfo", .data = &ums9230_socinfo_desc },
	{ .compatible = "sprd,ums9360-socinfo", .data = &ums9360_socinfo_desc },
	{ .compatible = "sprd,ums9621-socinfo", .data = &ums9621_socinfo_desc },
	{ .compatible = "sprd,ums9632-socinfo", .data = &ums9632_socinfo_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_socinfo_of_match);

static struct platform_driver sprd_socinfo_driver = {
	.probe = sprd_socinfo_probe,
	.remove = sprd_socinfo_remove,
	.driver = {
		.name = "sprd-socinfo",
		.of_match_table = sprd_socinfo_of_match,
	},
};
module_platform_driver(sprd_socinfo_driver);

MODULE_DESCRIPTION("Unisoc SoC information driver");
MODULE_LICENSE("GPL");
