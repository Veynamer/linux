// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spreadtrum SC27xx PMIC BC1.2 USB charger type detection
 *
 * The SC27xx PMICs embed a BC1.2 detection block. When VBUS is applied the
 * PMIC runs the handshake autonomously and latches the result in the charger
 * status register: a "detection done" bit plus one bit for each of the SDP,
 * DCP and CDP charger types.
 *
 * This driver polls that result and exposes it as a power supply so that a
 * charger following the legacy "ti,usb-charger-detection" binding (e.g.
 * bq2415x) can react to the detected input current limit.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SC2720_CHARGE_STATUS		0xe14
#define SC2720_CHG_DET_FGU_CTRL		0xe18
#define SC2721_CHARGE_STATUS		0xec8
#define SC2721_CHG_DET_FGU_CTRL		0xecc
#define SC2730_CHARGE_STATUS		0x1b9c
#define SC2730_CHG_DET_FGU_CTRL		0x1ba0
#define SC2730_CHARGER_PD		0x1b98
#define SC2731_CHARGE_STATUS		0xedc
#define SC2731_CHG_DET_FGU_CTRL		0xed8

/* Charger status register bits */
#define SC27XX_CHG_VBUS_BIT		BIT(2)
#define SC27XX_CHG_DET_DONE		BIT(11)
#define SC27XX_CHG_CDP_TYPE		BIT(5)
#define SC27XX_CHG_DCP_TYPE		BIT(6)
#define SC27XX_CHG_SDP_TYPE		BIT(7)
#define SC27XX_CHG_TYPE_MASK		GENMASK(7, 5)

/*
 * Detection interrupt delay, in 64 ms steps, at the top of the control reg.
 * The vendor uses factor 7 (448 ms) so the BC1.2 handshake and the VBUS
 * debounce have completed before the result is latched.
 */
#define SC27XX_CHG_INT_DELAY_MASK	GENMASK(11, 9)
#define SC27XX_CHG_INT_DELAY_SHIFT	9
#define SC27XX_CHG_INT_DELAY_FACTOR	7
#define SC27XX_CHG_DET_STEP_MS		64

/* CHG_DET_FGU_CTRL bit 0: 1 disables BC1.2 detection, 0 enables it. */
#define SC27XX_CHG_BC1P2_DISABLE	BIT(0)

#define SC27XX_BC1P2_POLL_INTERVAL	msecs_to_jiffies(2000)

struct sc27xx_bc1p2_data {
	u32 charge_status;
	u32 chg_det_fgu_ctrl;
	u32 charger_pd;
	u32 charger_pd_mask;
};

struct sc27xx_bc1p2 {
	struct device *dev;
	const struct sc27xx_bc1p2_data *data;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work work;
	struct mutex lock;
	int online;
	int usb_type;
};

static const struct sc27xx_bc1p2_data sc2720_data = {
	.charge_status = SC2720_CHARGE_STATUS,
	.chg_det_fgu_ctrl = SC2720_CHG_DET_FGU_CTRL,
};

static const struct sc27xx_bc1p2_data sc2721_data = {
	.charge_status = SC2721_CHARGE_STATUS,
	.chg_det_fgu_ctrl = SC2721_CHG_DET_FGU_CTRL,
};

static const struct sc27xx_bc1p2_data sc2730_data = {
	.charge_status = SC2730_CHARGE_STATUS,
	.chg_det_fgu_ctrl = SC2730_CHG_DET_FGU_CTRL,
	.charger_pd = SC2730_CHARGER_PD,
	.charger_pd_mask = BIT(0),
};

static const struct sc27xx_bc1p2_data sc2731_data = {
	.charge_status = SC2731_CHARGE_STATUS,
	.chg_det_fgu_ctrl = SC2731_CHG_DET_FGU_CTRL,
};

/*
 * CURRENT_MAX is reported in mA. This matches the bq2415x and isp1704
 * "usb-charger-detection" consumers, which compare it against 500/1800 mA
 * thresholds rather than the usual microamp convention.
 */
static int sc27xx_bc1p2_current(const struct sc27xx_bc1p2 *info)
{
	switch (info->usb_type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		return 500;
	case POWER_SUPPLY_USB_TYPE_CDP:
		return 1500;
	case POWER_SUPPLY_USB_TYPE_DCP:
		return 2000;
	default:
		return info->online ? 100 : 0;
	}
}

static int sc27xx_bc1p2_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct sc27xx_bc1p2 *info = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = info->usb_type;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (info->usb_type) {
		case POWER_SUPPLY_USB_TYPE_DCP:
		case POWER_SUPPLY_USB_TYPE_CDP:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case POWER_SUPPLY_USB_TYPE_SDP:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = sc27xx_bc1p2_current(info);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&info->lock);

	return ret;
}

static void sc27xx_bc1p2_update(struct sc27xx_bc1p2 *info)
{
	unsigned int val;
	int online, usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int ret;

	ret = regmap_read(info->regmap, info->data->charge_status, &val);
	if (ret) {
		dev_warn(info->dev, "failed to read charger status: %d\n", ret);
		return;
	}

	online = !!(val & SC27XX_CHG_VBUS_BIT);

	if (online && (val & SC27XX_CHG_DET_DONE)) {
		switch (val & SC27XX_CHG_TYPE_MASK) {
		case SC27XX_CHG_SDP_TYPE:
			usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case SC27XX_CHG_DCP_TYPE:
			usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case SC27XX_CHG_CDP_TYPE:
			usb_type = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		default:
			break;
		}
	}

	mutex_lock(&info->lock);
	if (info->online == online && info->usb_type == usb_type) {
		mutex_unlock(&info->lock);
		return;
	}

	info->online = online;
	info->usb_type = usb_type;
	mutex_unlock(&info->lock);

	dev_info(info->dev, "VBUS %d, USB type %d (status 0x%x)\n",
		 online, usb_type, val);

	power_supply_changed(info->psy);
}

static void sc27xx_bc1p2_work(struct work_struct *work)
{
	struct sc27xx_bc1p2 *info = container_of(to_delayed_work(work),
						 struct sc27xx_bc1p2, work);

	sc27xx_bc1p2_update(info);
	schedule_delayed_work(&info->work, SC27XX_BC1P2_POLL_INTERVAL);
}

static enum power_supply_property sc27xx_bc1p2_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc sc27xx_bc1p2_desc = {
	.name = "sc27xx-bc1p2",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP),
	.properties = sc27xx_bc1p2_props,
	.num_properties = ARRAY_SIZE(sc27xx_bc1p2_props),
	.get_property = sc27xx_bc1p2_get_property,
};

static int sc27xx_bc1p2_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = { };
	struct sc27xx_bc1p2 *info;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->data = of_device_get_match_data(&pdev->dev);
	if (!info->data) {
		dev_err(&pdev->dev, "no matching driver data found\n");
		return -EINVAL;
	}

	info->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!info->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "failed to get PMIC regmap\n");

	mutex_init(&info->lock);

	ret = regmap_update_bits(info->regmap, info->data->chg_det_fgu_ctrl,
				 SC27XX_CHG_INT_DELAY_MASK,
				 SC27XX_CHG_INT_DELAY_FACTOR
				 << SC27XX_CHG_INT_DELAY_SHIFT);
	if (ret)
		dev_warn(&pdev->dev, "failed to configure detection delay: %d\n",
			 ret);

	/* Make sure the BC1.2 block is enabled (the vendor disables it for OTG). */
	ret = regmap_update_bits(info->regmap, info->data->chg_det_fgu_ctrl,
				 SC27XX_CHG_BC1P2_DISABLE, 0);
	if (ret)
		dev_warn(&pdev->dev, "failed to enable BC1.2 detection: %d\n",
			 ret);

	if (info->data->charger_pd) {
		ret = regmap_update_bits(info->regmap, info->data->charger_pd,
					 info->data->charger_pd_mask, 0);
		if (ret)
			dev_warn(&pdev->dev,
				 "failed to enable charger path: %d\n", ret);
	}

	cfg.drv_data = info;
	cfg.fwnode = dev_fwnode(&pdev->dev);
	info->psy = devm_power_supply_register(&pdev->dev, &sc27xx_bc1p2_desc,
					       &cfg);
	if (IS_ERR(info->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(info->psy),
				     "failed to register power supply\n");

	platform_set_drvdata(pdev, info);

	INIT_DELAYED_WORK(&info->work, sc27xx_bc1p2_work);
	sc27xx_bc1p2_update(info);
	schedule_delayed_work(&info->work, SC27XX_BC1P2_POLL_INTERVAL);

	return 0;
}

static void sc27xx_bc1p2_remove(struct platform_device *pdev)
{
	struct sc27xx_bc1p2 *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->work);
}

static const struct of_device_id sc27xx_bc1p2_of_match[] = {
	{ .compatible = "sprd,sc2730-bc1p2", .data = &sc2730_data },
	{ .compatible = "sprd,sc2731-bc1p2", .data = &sc2731_data },
	{ .compatible = "sprd,sc2721-bc1p2", .data = &sc2721_data },
	{ .compatible = "sprd,sc2720-bc1p2", .data = &sc2720_data },
	{ }
};
MODULE_DEVICE_TABLE(of, sc27xx_bc1p2_of_match);

static struct platform_driver sc27xx_bc1p2_driver = {
	.driver = {
		.name = "sc27xx-bc1p2",
		.of_match_table = sc27xx_bc1p2_of_match,
	},
	.probe = sc27xx_bc1p2_probe,
	.remove = sc27xx_bc1p2_remove,
};
module_platform_driver(sc27xx_bc1p2_driver);

MODULE_DESCRIPTION("Spreadtrum SC27xx PMIC BC1.2 charger detection driver");
MODULE_LICENSE("GPL v2");
