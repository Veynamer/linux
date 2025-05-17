// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Unisoc camera subsystem media device driver - UMS9230 specific code
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/regmap.h>

#include "camsys.h"

/* Platform-specific syscon registers */
#define UMS9230_MM_AHB_CSI_PHY_SEL			0x0030

#define UMS9230_MM_AHB_DCAM_CSI_SEL(id)			(0x0048 + (id) * 4)
#define UMS9230_MM_AHB_DCAM_CSI_SEL_FORCE		BIT(11)
#define UMS9230_MM_AHB_DCAM_CSI_SEL_INDEX		GENMASK(10, 9)
#define UMS9230_MM_AHB_DCAM_CSI_SEL_FORCE_TRIGGER	BIT(8)

#define UMS9230_ANLG_PHY_G10_CSI_2P2_RO_CTRL		0x000C
#define UMS9230_ANLG_PHY_G10_CSI_2P2_CTRL		0x0038
#define UMS9230_ANLG_PHY_G10_CSI_2P2			BIT(4)

#define UMS9230_ANLG_PHY_G10_CSI_BIST_TEST		0x00B4

static void ums9230_csi_power_phy(struct sprd_csi *csi, int enable)
{
	u32 val;

	switch (csi->index) {
	case 0:
		val = BIT(26);
		break;
	case 1:
		val = BIT(24);
		break;
	case 2:
		val = BIT(25);
		break;
	default:
		return;
	}

	regmap_assign_bits(csi->camsys->anlg_phy_regs,
			   UMS9230_ANLG_PHY_G10_CSI_BIST_TEST,
			   val, enable);
}

static void ums9230_csi_setup_phy(struct sprd_csi *csi, int enable)
{
	u32 mask, val, id = enable ? csi->index + 1 : 0;
	int shift1, shift2 = -1;

	switch (csi->phy_id) {
	case CAMSYS_CSI_M_S:
		if (enable)
			regmap_set_bits(csi->camsys->anlg_phy_regs,
					UMS9230_ANLG_PHY_G10_CSI_2P2_CTRL,
					UMS9230_ANLG_PHY_G10_CSI_2P2);
		shift1 = 0;
		shift2 = 2;
		break;
	case CAMSYS_CSI_M:
		if (enable)
			regmap_clear_bits(csi->camsys->anlg_phy_regs,
					  UMS9230_ANLG_PHY_G10_CSI_2P2_CTRL,
					  UMS9230_ANLG_PHY_G10_CSI_2P2);
		shift1 = 0;
		break;
	case CAMSYS_CSI_S:
		if (enable)
			regmap_clear_bits(csi->camsys->anlg_phy_regs,
					  UMS9230_ANLG_PHY_G10_CSI_2P2_CTRL,
					  UMS9230_ANLG_PHY_G10_CSI_2P2);
		shift1 = 2;
		break;
	case CAMSYS_CSI_4LANE:
		shift1 = 4;
		break;
	case CAMSYS_CSI_RO_M_S:
		if (enable)
			regmap_set_bits(csi->camsys->anlg_phy_regs,
					UMS9230_ANLG_PHY_G10_CSI_2P2_RO_CTRL,
					UMS9230_ANLG_PHY_G10_CSI_2P2);
		shift1 = 6;
		shift2 = 8;
		break;
	case CAMSYS_CSI_RO_M:
		if (enable)
			regmap_clear_bits(csi->camsys->anlg_phy_regs,
					  UMS9230_ANLG_PHY_G10_CSI_2P2_RO_CTRL,
					  UMS9230_ANLG_PHY_G10_CSI_2P2);
		shift1 = 6;
		break;
	case CAMSYS_CSI_RO_S:
		if (enable)
			regmap_clear_bits(csi->camsys->anlg_phy_regs,
					  UMS9230_ANLG_PHY_G10_CSI_2P2_RO_CTRL,
					  UMS9230_ANLG_PHY_G10_CSI_2P2);
		shift1 = 8;
		break;
	default:
		return;
	}

	mask = 3 << shift1;
	val = id << shift1;
	if (shift2 >= 0) {
		mask |= 3 << shift2;
		val |= id << shift2;
	}

	regmap_update_bits(csi->camsys->cam_ahb_regs,
			   UMS9230_MM_AHB_CSI_PHY_SEL, mask, val);
}

static void ums9230_csi_connect_dcam(struct sprd_csi *csi, int enable)
{
	u32 val, reg = UMS9230_MM_AHB_DCAM_CSI_SEL(csi->dcam_id);
	u32 index = enable ? csi->index : 3;

	val = UMS9230_MM_AHB_DCAM_CSI_SEL_FORCE |
	      FIELD_PREP(UMS9230_MM_AHB_DCAM_CSI_SEL_INDEX, index) |
	      UMS9230_MM_AHB_DCAM_CSI_SEL_FORCE_TRIGGER;
	regmap_update_bits(csi->camsys->cam_ahb_regs, reg,
			   UMS9230_MM_AHB_DCAM_CSI_SEL_FORCE |
			   UMS9230_MM_AHB_DCAM_CSI_SEL_INDEX |
			   UMS9230_MM_AHB_DCAM_CSI_SEL_FORCE_TRIGGER,
			   val);

	regmap_clear_bits(csi->camsys->cam_ahb_regs, reg,
			  UMS9230_MM_AHB_DCAM_CSI_SEL_FORCE_TRIGGER);
}

static const struct sprd_csi_hw_ops ums9230_csi_ops = {
	.power_phy = ums9230_csi_power_phy,
	.setup_phy = ums9230_csi_setup_phy,
	.connect_dcam = ums9230_csi_connect_dcam,
};

static const struct sprd_camsys_fmt_config ums9230_camsys_formats[] = {
	SPRD_CAMSYS_FMT(SBGGR8_1X8, SBGGR8, 1, 8),
	SPRD_CAMSYS_FMT(SGBRG8_1X8, SGBRG8, 1, 8),
	SPRD_CAMSYS_FMT(SGRBG8_1X8, SGRBG8, 1, 8),
	SPRD_CAMSYS_FMT(SRGGB8_1X8, SRGGB8, 1, 8),
	SPRD_CAMSYS_FMT(SBGGR10_1X10, SBGGR10P, 4, 10),
	SPRD_CAMSYS_FMT(SGBRG10_1X10, SGBRG10P, 4, 10),
	SPRD_CAMSYS_FMT(SGRBG10_1X10, SGRBG10P, 4, 10),
	SPRD_CAMSYS_FMT(SRGGB10_1X10, SRGGB10P, 4, 10),
};

const struct sprd_camsys_hardware ums9230_camsys_info = {
	.csi_ops = &ums9230_csi_ops,
	.dcam_ops = &ums9230_dcam_ops,
	.max_width = 8048,
	.max_height = 6036,
	.formats = ums9230_camsys_formats,
	.num_formats = ARRAY_SIZE(ums9230_camsys_formats),
};
