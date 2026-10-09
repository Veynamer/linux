// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc USB2 PHY driver
 *
 * Copyright (C) 2024 Otto Pflüger
 */

#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* otg_test_reg */
#define BIT_AON_USB2_PHY_IDDIG		BIT(3)
#define BIT_AON_OTG_VBUS_VALID_PHYREG	BIT(24)

/* otg_ctrl_reg */
#define BIT_AON_UTMI_WIDTH_SEL		BIT(30)

/* pll_reg */
#define BIT_ANLG_USB20_ISO_SW_EN	BIT(0)

/* pd_reg */
#define BIT_ANLG_USB20_PS_PD_L		BIT(3)
#define BIT_ANLG_USB20_PS_PD_S		BIT(4)

/* utmi_ctl1_reg */
#define BIT_ANLG_USB20_RESERVED		GENMASK(15, 0) /* undocumented */
#define BIT_ANLG_USB20_VBUSVLDEXT	BIT(16)
#define BIT_ANLG_USB20_DATABUS16_8	BIT(28)

/* utmi_ctl2_reg */
#define BIT_ANLG_USB20_DMPULLDOWN	BIT(3)
#define BIT_ANLG_USB20_DPPULLDOWN	BIT(4)

#define DEFAULT_EYE_PATTERN	0x04f3d1c0

/* AON APB APB_EB1: module enables */
#define BIT_AON_APB_ANA_EB		BIT(12)	/* analog PHY block clock */

/* AON APB CGM_REG1: USB reference-clock gates */
#define BIT_AON_APB_CGM_OTG_REF_EN	BIT(12)
#define BIT_AON_APB_CGM_DPHY_REF_EN	BIT(10)

/* AON APB APB_RST1: PHY/UTMI soft reset */
#define BIT_AON_APB_OTG_PHY_SOFT_RST	BIT(9)
#define BIT_AON_APB_OTG_UTMI_SOFT_RST	BIT(8)

struct sprd_hsphy_data {
	/* AON APB regs */
	u32 otg_test_reg;
	u32 otg_ctrl_reg;
	u32 apb_eb1;	/* module enables (ANA_EB); 0 = not wired */
	u32 cgm_reg1;	/* USB ref-clock gates; 0 = not wired */
	u32 apb_rst1;	/* PHY soft-reset; 0 = not wired */

	/* analog regs */
	u32 pll_reg;
	u32 pd_reg;
	u32 utmi_ctl1_reg;
	u32 utmi_ctl2_reg;
	u32 trimming_reg;
	u32 reg_sel_cfg_reg;
	u32 reg_sel_mask;
};

struct sprd_hsphy {
	struct device *dev;
	struct regmap *aon_apb;
	struct regmap *ana_regs;
	const struct sprd_hsphy_data *data;
};

static int sprd_hsphy_init(struct phy *phy)
{
	struct sprd_hsphy *hsphy = phy_get_drvdata(phy);

	dev_dbg(hsphy->dev, "%s()\n", __func__);

	/*
	 * Enable the USB reference-clock gates. On a USB-plug boot U-Boot sets
	 * these, which is why mainline got away without it; on a cold
	 * (power-button) boot nothing does, leaving the PHY reference clock
	 * dead so the gadget's EP0 never answers and the host fails every
	 * descriptor read with -71. Mirror the vendor BSP and assert them here.
	 */
	if (hsphy->data->cgm_reg1)
		regmap_set_bits(hsphy->aon_apb, hsphy->data->cgm_reg1,
				BIT_AON_APB_CGM_OTG_REF_EN |
				BIT_AON_APB_CGM_DPHY_REF_EN);

	regmap_set_bits(hsphy->aon_apb, hsphy->data->otg_ctrl_reg,
			BIT_AON_UTMI_WIDTH_SEL);
	regmap_set_bits(hsphy->ana_regs, hsphy->data->utmi_ctl1_reg,
			BIT_ANLG_USB20_DATABUS16_8);

	/*
	 * Soft-reset the PHY/UTMI once, as the vendor BSP does, so a cold boot
	 * starts from a known state instead of inheriting whatever U-Boot left
	 * (or didn't leave). The vendor delay is 20-30ms.
	 */
	if (hsphy->data->apb_rst1) {
		regmap_set_bits(hsphy->aon_apb, hsphy->data->apb_rst1,
				BIT_AON_APB_OTG_PHY_SOFT_RST |
				BIT_AON_APB_OTG_UTMI_SOFT_RST);
		usleep_range(20000, 30000);
		regmap_clear_bits(hsphy->aon_apb, hsphy->data->apb_rst1,
				  BIT_AON_APB_OTG_PHY_SOFT_RST |
				  BIT_AON_APB_OTG_UTMI_SOFT_RST);
	}

	return 0;
}

static int sprd_hsphy_power_on(struct phy *phy)
{
	struct sprd_hsphy *hsphy = phy_get_drvdata(phy);

	dev_dbg(hsphy->dev, "%s()\n", __func__);

	regmap_clear_bits(hsphy->ana_regs, hsphy->data->pll_reg,
			  BIT_ANLG_USB20_ISO_SW_EN);
	regmap_clear_bits(hsphy->ana_regs, hsphy->data->pd_reg,
			  BIT_ANLG_USB20_PS_PD_L | BIT_ANLG_USB20_PS_PD_S);

	/*
	 * Enable the analog PHY block clock (APB_EB1.ANA_EB). Nothing else in our
	 * USB path enables it; U-Boot does it first thing. Without the analog
	 * clock the PHY can't drive HS signalling and EP0 answers garbage (-71).
	 */
	if (hsphy->data->apb_eb1)
		regmap_set_bits(hsphy->aon_apb, hsphy->data->apb_eb1,
				BIT_AON_APB_ANA_EB);

	/* VBUS-valid (both the AON test reg and the analog VBUSVLDEXT), early. */
	regmap_set_bits(hsphy->aon_apb, hsphy->data->otg_test_reg,
			BIT_AON_OTG_VBUS_VALID_PHYREG);
	regmap_set_bits(hsphy->ana_regs, hsphy->data->utmi_ctl1_reg,
			BIT_ANLG_USB20_VBUSVLDEXT);

	/* 16-bit UTMI width. */
	regmap_set_bits(hsphy->aon_apb, hsphy->data->otg_ctrl_reg,
			BIT_AON_UTMI_WIDTH_SEL);
	regmap_set_bits(hsphy->ana_regs, hsphy->data->utmi_ctl1_reg,
			BIT_ANLG_USB20_DATABUS16_8);

	return 0;
}

static int sprd_hsphy_power_off(struct phy *phy)
{
	struct sprd_hsphy *hsphy = phy_get_drvdata(phy);

	dev_dbg(hsphy->dev, "%s()\n", __func__);

	regmap_clear_bits(hsphy->aon_apb, hsphy->data->otg_test_reg,
			  BIT_AON_OTG_VBUS_VALID_PHYREG);
	regmap_clear_bits(hsphy->ana_regs, hsphy->data->utmi_ctl1_reg,
			  BIT_ANLG_USB20_VBUSVLDEXT);

	regmap_set_bits(hsphy->ana_regs, hsphy->data->pll_reg,
			BIT_ANLG_USB20_ISO_SW_EN);
	regmap_set_bits(hsphy->ana_regs, hsphy->data->pd_reg,
			BIT_ANLG_USB20_PS_PD_L | BIT_ANLG_USB20_PS_PD_S);

	return 0;
}

static int sprd_hsphy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct sprd_hsphy *hsphy = phy_get_drvdata(phy);

	switch (mode) {
	case PHY_MODE_USB_HOST:
		dev_dbg(hsphy->dev, "%s(host)\n", __func__);
		regmap_clear_bits(hsphy->aon_apb, hsphy->data->otg_ctrl_reg,
				  BIT_AON_USB2_PHY_IDDIG);
		regmap_set_bits(hsphy->ana_regs, hsphy->data->reg_sel_cfg_reg,
				hsphy->data->reg_sel_mask);
		regmap_set_bits(hsphy->ana_regs, hsphy->data->utmi_ctl2_reg,
				BIT_ANLG_USB20_DMPULLDOWN |
				BIT_ANLG_USB20_DPPULLDOWN);
		regmap_update_bits(hsphy->ana_regs, hsphy->data->utmi_ctl1_reg,
				   BIT_ANLG_USB20_RESERVED, 0x200);
		break;

	case PHY_MODE_USB_DEVICE:
		dev_dbg(hsphy->dev, "%s(device)\n", __func__);
		regmap_set_bits(hsphy->aon_apb, hsphy->data->otg_ctrl_reg,
				BIT_AON_USB2_PHY_IDDIG);
		regmap_set_bits(hsphy->ana_regs, hsphy->data->reg_sel_cfg_reg,
				hsphy->data->reg_sel_mask);
		regmap_clear_bits(hsphy->ana_regs, hsphy->data->utmi_ctl2_reg,
				  BIT_ANLG_USB20_DMPULLDOWN |
				  BIT_ANLG_USB20_DPPULLDOWN);
		regmap_update_bits(hsphy->ana_regs, hsphy->data->utmi_ctl1_reg,
				   BIT_ANLG_USB20_RESERVED, 0);
		break;

	default:
		dev_dbg(hsphy->dev, "%s(other)\n", __func__);
		break;
	}

	/* enable to activate mode */
	regmap_set_bits(hsphy->aon_apb, hsphy->data->otg_test_reg,
			BIT_AON_OTG_VBUS_VALID_PHYREG);
	regmap_set_bits(hsphy->ana_regs, hsphy->data->utmi_ctl1_reg,
			BIT_ANLG_USB20_VBUSVLDEXT);

	return 0;
}

static const struct phy_ops sprd_hsphy_ops = {
	.init = sprd_hsphy_init,
	.power_on = sprd_hsphy_power_on,
	.power_off = sprd_hsphy_power_off,
	.set_mode = sprd_hsphy_set_mode,
	.owner = THIS_MODULE,
};

static int sprd_hsphy_probe(struct platform_device *pdev)
{
	struct sprd_hsphy *hsphy;
	struct phy_provider *provider;
	struct phy *phy;

	hsphy = devm_kzalloc(&pdev->dev, sizeof(*hsphy), GFP_KERNEL);
	if (!hsphy)
		return -ENOMEM;

	hsphy->dev = &pdev->dev;

	hsphy->data = of_device_get_match_data(hsphy->dev);
	if (!hsphy->data)
		return -EINVAL;

	hsphy->aon_apb = syscon_regmap_lookup_by_phandle(hsphy->dev->of_node,
							 "sprd,syscon-aon-apb");
	if (IS_ERR(hsphy->aon_apb))
		return dev_err_probe(hsphy->dev, PTR_ERR(hsphy->aon_apb),
				     "failed to get AON APB syscon\n");

	hsphy->ana_regs = syscon_node_to_regmap(hsphy->dev->of_node->parent);
	if (IS_ERR(hsphy->ana_regs))
		return dev_err_probe(hsphy->dev, PTR_ERR(hsphy->ana_regs),
				     "failed to get ANLG_PHY_G2 syscon\n");

	phy = devm_phy_create(hsphy->dev, NULL, &sprd_hsphy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(hsphy->dev, PTR_ERR(phy),
				     "failed to create phy\n");

	phy_set_drvdata(phy, hsphy);

	provider = devm_of_phy_provider_register(hsphy->dev,
						 of_phy_simple_xlate);
	if (IS_ERR(provider))
		return dev_err_probe(hsphy->dev, PTR_ERR(provider),
				     "failed to register phy provider\n");

	return 0;
}

static const struct sprd_hsphy_data ums9230_data = {
	/* AON APB regs */
	.otg_test_reg		= 0x0204,
	.otg_ctrl_reg		= 0x0208,

	/* analog g2 regs */
	.pll_reg		= 0x001c,
	.pd_reg			= 0x0008,
	.utmi_ctl1_reg		= 0x0004,
	.utmi_ctl2_reg		= 0x000c,
	.trimming_reg		= 0x0010,
	.reg_sel_cfg_reg	= 0x0020,
	.reg_sel_mask		= BIT(2) | BIT(1),
};

static const struct sprd_hsphy_data ums512_data = {
	/* AON APB regs */
	.otg_test_reg		= 0x0204,
	.otg_ctrl_reg		= 0x0208,
	.apb_eb1		= 0x0004,
	.cgm_reg1		= 0x0138,
	.apb_rst1		= 0x0010,

	/* analog g2 regs (offsets from anlg_phy_g2 syscon base 0x323b0000) */
	.pll_reg		= 0x0070,	/* ISO_SW */
	.pd_reg			= 0x005c,	/* BATTER_PLL (PS_PD_L/S) */
	.utmi_ctl1_reg		= 0x0058,	/* UTMI_CTL1 */
	.utmi_ctl2_reg		= 0x0060,	/* UTMI_CTL2 */
	.trimming_reg		= 0x0064,	/* TRIMMING */
	.reg_sel_cfg_reg	= 0x0074,	/* REG_SEL_CFG_0 */
	.reg_sel_mask		= BIT(2) | BIT(1),
};

static const struct of_device_id sprd_hsphy_of_match[] = {
	{ .compatible = "sprd,ums512-hsphy", .data = &ums512_data },
	{ .compatible = "sprd,ums9230-hsphy", .data = &ums9230_data },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_hsphy_of_match);

static struct platform_driver sprd_hsphy_driver = {
	.probe	= sprd_hsphy_probe,
	.driver	= {
		.name		= "sprd-usb2-phy",
		.of_match_table	= sprd_hsphy_of_match,
	},
};

module_platform_driver(sprd_hsphy_driver);

MODULE_AUTHOR("Otto Pflüger <otto.pflueger@abscue.de>");
MODULE_DESCRIPTION("Unisoc USB2 PHY driver");
MODULE_LICENSE("GPL");
