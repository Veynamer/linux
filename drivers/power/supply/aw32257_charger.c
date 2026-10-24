// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * aw32257 charger driver
 *
 * Copyright (C) 2011-2013  Pali Rohár <pali@kernel.org>
 *
 * Datasheets:
 * https://www.ti.com/product/bq24150
 * https://www.ti.com/product/bq24150a
 * https://www.ti.com/product/bq24152
 * https://www.ti.com/product/bq24153
 * https://www.ti.com/product/bq24153a
 * https://www.ti.com/product/bq24155
 * https://www.ti.com/product/bq24157s
 * https://www.ti.com/product/bq24158
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/kconfig.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#if IS_REACHABLE(CONFIG_REGULATOR)
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#endif



enum aw32257_mode {
	AW32257_MODE_OFF,
	AW32257_MODE_NONE,
	AW32257_MODE_HOST_CHARGER,
	AW32257_MODE_DEDICATED_CHARGER,
	AW32257_MODE_BOOST,
};

struct aw32257_platform_data {
	int current_limit;
	int weak_battery_voltage;
	int battery_regulation_voltage;
	int charge_current;
	int charge_current_dcp;
	int charge_current_cdp;
	int charge_current_sdp;
	int charge_current_unknown;
	int termination_current;
	int resistor_sense;
	const char *notify_device;
};

/* timeout for resetting chip timer */
#define AW32257_TIMER_TIMEOUT		10

#define AW32257_REG_STATUS		0x00
#define AW32257_REG_CONTROL		0x01
#define AW32257_REG_VOLTAGE		0x02
#define AW32257_REG_VENDER		0x03
#define AW32257_REG_CURRENT		0x04

/* reset state for all registers */
#define AW32257_RESET_STATUS		BIT(6)
#define AW32257_RESET_CONTROL		(BIT(4)|BIT(5))
#define AW32257_RESET_VOLTAGE		(BIT(1)|BIT(3))
#define AW32257_RESET_CURRENT		(BIT(0)|BIT(3)|BIT(7))

/* status register */
#define AW32257_BIT_TMR_RST		7
#define AW32257_BIT_OTG			7
#define AW32257_BIT_EN_STAT		6
#define AW32257_MASK_STAT		(BIT(4)|BIT(5))
#define AW32257_SHIFT_STAT		4
#define AW32257_BIT_BOOST		3
#define AW32257_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define AW32257_SHIFT_FAULT		0

/* control register */
#define AW32257_MASK_LIMIT		(BIT(6)|BIT(7))
#define AW32257_SHIFT_LIMIT		6
#define AW32257_MASK_VLOWV		(BIT(4)|BIT(5))
#define AW32257_SHIFT_VLOWV		4
#define AW32257_BIT_TE			3
#define AW32257_BIT_CE			2
#define AW32257_BIT_HZ_MODE		1
#define AW32257_BIT_OPA_MODE		0

/* voltage register */
#define AW32257_MASK_VO		(BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define AW32257_SHIFT_VO		2
#define AW32257_BIT_OTG_PL		1
#define AW32257_BIT_OTG_EN		0

/* vender register */
#define AW32257_MASK_VENDER		(BIT(5)|BIT(6)|BIT(7))
#define AW32257_SHIFT_VENDER		5
#define AW32257_MASK_PN			(BIT(3)|BIT(4))
#define AW32257_SHIFT_PN		3
#define AW32257_MASK_REVISION		(BIT(0)|BIT(1)|BIT(2))
#define AW32257_SHIFT_REVISION		0

/* current register */
#define AW32257_MASK_RESET		BIT(7)
#define AW32257_MASK_VI_CHRG		(BIT(4)|BIT(5)|BIT(6))
#define AW32257_SHIFT_VI_CHRG		4
/* N/A					BIT(3) */
#define AW32257_MASK_VI_TERM		(BIT(0)|BIT(1)|BIT(2))
#define AW32257_SHIFT_VI_TERM		0

/*
 * The AW32257 (Awinic) is a register clone of the bq24157/bq24158, but the
 * charge current field of REG04 is four bits wide (bits 3..6) and is encoded
 * as a sense-resistor voltage instead of the three-bit aw32257 field.
 */
#define AW32257_MASK_VI_CHRG_EXT		(BIT(3)|BIT(4)|BIT(5)|BIT(6))
#define AW32257_SHIFT_VI_CHRG_EXT		3

/* REG04[6:3] charge current levels, in 0.01 mA. */
static const u32 aw32257_cs_vth[] = {
	47500, 60000, 85000, 97500, 110000, 122500, 135000, 147500,
	160000, 172500, 185000, 197500, 210000,
};

static u32 aw32257_find_closest_level(const u32 *list, u32 num, u32 level)
{
	u32 i;

	for (i = num - 1; i != 0; i--)
		if (list[i] <= level)
			return list[i];

	return list[0];
}

static u32 aw32257_level_to_index(const u32 *list, u32 num, u32 val)
{
	u32 i;

	for (i = 0; i < num; i++)
		if (val == list[i])
			return i;

	return 0;
}

enum aw32257_command {
	AW32257_TIMER_RESET,
	AW32257_OTG_STATUS,
	AW32257_STAT_PIN_STATUS,
	AW32257_STAT_PIN_ENABLE,
	AW32257_STAT_PIN_DISABLE,
	AW32257_CHARGE_STATUS,
	AW32257_BOOST_STATUS,
	AW32257_FAULT_STATUS,

	AW32257_CHARGE_TERMINATION_STATUS,
	AW32257_CHARGE_TERMINATION_ENABLE,
	AW32257_CHARGE_TERMINATION_DISABLE,
	AW32257_CHARGER_STATUS,
	AW32257_CHARGER_ENABLE,
	AW32257_CHARGER_DISABLE,
	AW32257_HIGH_IMPEDANCE_STATUS,
	AW32257_HIGH_IMPEDANCE_ENABLE,
	AW32257_HIGH_IMPEDANCE_DISABLE,
	AW32257_BOOST_MODE_STATUS,
	AW32257_BOOST_MODE_ENABLE,
	AW32257_BOOST_MODE_DISABLE,

	AW32257_OTG_LEVEL,
	AW32257_OTG_ACTIVATE_HIGH,
	AW32257_OTG_ACTIVATE_LOW,
	AW32257_OTG_PIN_STATUS,
	AW32257_OTG_PIN_ENABLE,
	AW32257_OTG_PIN_DISABLE,

	AW32257_VENDER_CODE,
	AW32257_PART_NUMBER,
	AW32257_REVISION,
};

enum aw32257_chip {
	BQUNKNOWN,
	BQ24150,
	BQ24150A,
	BQ24151,
	BQ24151A,
	BQ24152,
	BQ24153,
	BQ24153A,
	BQ24155,
	BQ24156,
	BQ24156A,
	BQ24157S,
	BQ24158,
	AW32257,
};

static char *aw32257_chip_name[] = {
	"unknown",
	"bq24150",
	"bq24150a",
	"bq24151",
	"bq24151a",
	"bq24152",
	"bq24153",
	"bq24153a",
	"bq24155",
	"bq24156",
	"bq24156a",
	"bq24157s",
	"bq24158",
	"aw32257",
};

enum aw32257_jeita_type {
	AW32257_JEITA_UNKNOWN,
	AW32257_JEITA_SDP,
	AW32257_JEITA_CDP,
	AW32257_JEITA_DCP,
	AW32257_JEITA_MAX,
};

static const char * const aw32257_jeita_names[AW32257_JEITA_MAX] = {
	[AW32257_JEITA_UNKNOWN] = "unknown-jeita-temp-table",
	[AW32257_JEITA_SDP] = "sdp-jeita-temp-table",
	[AW32257_JEITA_CDP] = "cdp-jeita-temp-table",
	[AW32257_JEITA_DCP] = "dcp-jeita-temp-table",
};

struct aw32257_jeita_table {
	int temp;		/* 0.1 C */
	int recovery;		/* 0.1 C */
	int current_ma;
	int voltage_mv;
};

struct aw32257_device {
	struct device *dev;
	struct aw32257_platform_data init_data;
	struct power_supply *charger;
	struct power_supply_desc charger_desc;
	struct delayed_work work;
	struct delayed_work jeita_work;
	struct aw32257_jeita_table *jeita[AW32257_JEITA_MAX];
	int jeita_len[AW32257_JEITA_MAX];
	struct power_supply *fgu;
	struct device_node *notify_node;
	int usb_type;
	struct notifier_block nb;
	enum aw32257_mode reported_mode;/* mode reported by hook function */
	enum aw32257_mode mode;		/* currently configured mode */
	enum aw32257_chip chip;
	const char *timer_error;
	char *model;
	char *name;
	int autotimer;	/* 1 - if driver automatically reset timer, 0 - not */
	int automode;	/* 1 - enabled, 0 - disabled; -1 - not supported */
	int charge_status;
	int id;
#if IS_REACHABLE(CONFIG_REGULATOR)
	struct regulator_dev *usb_vbus_reg;
#endif
};

/* each registered chip must have unique id */
static DEFINE_IDR(aw32257_id);

static DEFINE_MUTEX(aw32257_id_mutex);
static DEFINE_MUTEX(aw32257_timer_mutex);
static DEFINE_MUTEX(aw32257_i2c_mutex);

/**** i2c read functions ****/

/* read value from register */
static int aw32257_i2c_read(struct aw32257_device *bq, u8 reg)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[2];
	u8 val;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = sizeof(val);

	mutex_lock(&aw32257_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&aw32257_i2c_mutex);

	if (ret < 0)
		return ret;

	return val;
}

/* read value from register, apply mask and right shift it */
static int aw32257_i2c_read_mask(struct aw32257_device *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = aw32257_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}

/* read value from register and return one specified bit */
static int aw32257_i2c_read_bit(struct aw32257_device *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return aw32257_i2c_read_mask(bq, reg, BIT(bit), bit);
}

/**** i2c write functions ****/

/* write value to register */
static int aw32257_i2c_write(struct aw32257_device *bq, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	mutex_lock(&aw32257_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&aw32257_i2c_mutex);

	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}

/* read value from register, change it with mask left shifted and write back */
static int aw32257_i2c_write_mask(struct aw32257_device *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = aw32257_i2c_read(bq, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return aw32257_i2c_write(bq, reg, ret);
}

/* change only one bit in register */
static int aw32257_i2c_write_bit(struct aw32257_device *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return aw32257_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

/**** global functions ****/

/* exec command function */
static int aw32257_exec_command(struct aw32257_device *bq,
				enum aw32257_command command)
{
	int ret;

	switch (command) {
	case AW32257_TIMER_RESET:
		return aw32257_i2c_write_bit(bq, AW32257_REG_STATUS,
				1, AW32257_BIT_TMR_RST);
	case AW32257_OTG_STATUS:
		return aw32257_i2c_read_bit(bq, AW32257_REG_STATUS,
				AW32257_BIT_OTG);
	case AW32257_STAT_PIN_STATUS:
		return aw32257_i2c_read_bit(bq, AW32257_REG_STATUS,
				AW32257_BIT_EN_STAT);
	case AW32257_STAT_PIN_ENABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_STATUS, 1,
				AW32257_BIT_EN_STAT);
	case AW32257_STAT_PIN_DISABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_STATUS, 0,
				AW32257_BIT_EN_STAT);
	case AW32257_CHARGE_STATUS:
		return aw32257_i2c_read_mask(bq, AW32257_REG_STATUS,
				AW32257_MASK_STAT, AW32257_SHIFT_STAT);
	case AW32257_BOOST_STATUS:
		return aw32257_i2c_read_bit(bq, AW32257_REG_STATUS,
				AW32257_BIT_BOOST);
	case AW32257_FAULT_STATUS:
		return aw32257_i2c_read_mask(bq, AW32257_REG_STATUS,
			AW32257_MASK_FAULT, AW32257_SHIFT_FAULT);

	case AW32257_CHARGE_TERMINATION_STATUS:
		return aw32257_i2c_read_bit(bq, AW32257_REG_CONTROL,
				AW32257_BIT_TE);
	case AW32257_CHARGE_TERMINATION_ENABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				1, AW32257_BIT_TE);
	case AW32257_CHARGE_TERMINATION_DISABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				0, AW32257_BIT_TE);
	case AW32257_CHARGER_STATUS:
		ret = aw32257_i2c_read_bit(bq, AW32257_REG_CONTROL,
			AW32257_BIT_CE);
		if (ret < 0)
			return ret;
		return ret > 0 ? 0 : 1;
	case AW32257_CHARGER_ENABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				0, AW32257_BIT_CE);
	case AW32257_CHARGER_DISABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				1, AW32257_BIT_CE);
	case AW32257_HIGH_IMPEDANCE_STATUS:
		return aw32257_i2c_read_bit(bq, AW32257_REG_CONTROL,
				AW32257_BIT_HZ_MODE);
	case AW32257_HIGH_IMPEDANCE_ENABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				1, AW32257_BIT_HZ_MODE);
	case AW32257_HIGH_IMPEDANCE_DISABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				0, AW32257_BIT_HZ_MODE);
	case AW32257_BOOST_MODE_STATUS:
		return aw32257_i2c_read_bit(bq, AW32257_REG_CONTROL,
				AW32257_BIT_OPA_MODE);
	case AW32257_BOOST_MODE_ENABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				1, AW32257_BIT_OPA_MODE);
	case AW32257_BOOST_MODE_DISABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_CONTROL,
				0, AW32257_BIT_OPA_MODE);

	case AW32257_OTG_LEVEL:
		return aw32257_i2c_read_bit(bq, AW32257_REG_VOLTAGE,
				AW32257_BIT_OTG_PL);
	case AW32257_OTG_ACTIVATE_HIGH:
		return aw32257_i2c_write_bit(bq, AW32257_REG_VOLTAGE,
				1, AW32257_BIT_OTG_PL);
	case AW32257_OTG_ACTIVATE_LOW:
		return aw32257_i2c_write_bit(bq, AW32257_REG_VOLTAGE,
				0, AW32257_BIT_OTG_PL);
	case AW32257_OTG_PIN_STATUS:
		return aw32257_i2c_read_bit(bq, AW32257_REG_VOLTAGE,
				AW32257_BIT_OTG_EN);
	case AW32257_OTG_PIN_ENABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_VOLTAGE,
				1, AW32257_BIT_OTG_EN);
	case AW32257_OTG_PIN_DISABLE:
		return aw32257_i2c_write_bit(bq, AW32257_REG_VOLTAGE,
				0, AW32257_BIT_OTG_EN);

	case AW32257_VENDER_CODE:
		return aw32257_i2c_read_mask(bq, AW32257_REG_VENDER,
			AW32257_MASK_VENDER, AW32257_SHIFT_VENDER);
	case AW32257_PART_NUMBER:
		return aw32257_i2c_read_mask(bq, AW32257_REG_VENDER,
				AW32257_MASK_PN, AW32257_SHIFT_PN);
	case AW32257_REVISION:
		return aw32257_i2c_read_mask(bq, AW32257_REG_VENDER,
			AW32257_MASK_REVISION, AW32257_SHIFT_REVISION);
	}
	return -EINVAL;
}

/* detect chip type */
static enum aw32257_chip aw32257_detect_chip(struct aw32257_device *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	int ret = aw32257_exec_command(bq, AW32257_PART_NUMBER);

	if (ret < 0)
		return ret;

	switch (client->addr) {
	case 0x6b:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24151A)
				return bq->chip;
			return BQ24151;
		case 1:
			if (bq->chip == BQ24150A ||
				bq->chip == BQ24152 ||
				bq->chip == BQ24155)
				return bq->chip;
			return BQ24150;
		case 2:
			if (bq->chip == BQ24153A)
				return bq->chip;
			return BQ24153;
		default:
			return BQUNKNOWN;
		}
		break;

	case 0x6a:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24156A)
				return bq->chip;
			return BQ24156;
		case 2:
			if (bq->chip == BQ24157S || bq->chip == AW32257)
				return bq->chip;
			return BQ24158;
		default:
			return BQUNKNOWN;
		}
		break;
	}

	return BQUNKNOWN;
}

/* detect chip revision */
static int aw32257_detect_revision(struct aw32257_device *bq)
{
	int ret = aw32257_exec_command(bq, AW32257_REVISION);
	int chip = aw32257_detect_chip(bq);

	if (ret < 0 || chip < 0)
		return -1;

	switch (chip) {
	case BQ24150:
	case BQ24150A:
	case BQ24151:
	case BQ24151A:
	case BQ24152:
		if (ret >= 0 && ret <= 3)
			return ret;
		return -1;
	case BQ24153:
	case BQ24153A:
	case BQ24156:
	case BQ24156A:
	case BQ24157S:
	case BQ24158:
	case AW32257:
		if (ret == 3)
			return 0;
		else if (ret == 1)
			return 1;
		return -1;
	case BQ24155:
		if (ret == 3)
			return 3;
		return -1;
	case BQUNKNOWN:
		return -1;
	}

	return -1;
}

/* return chip vender code */
static int aw32257_get_vender_code(struct aw32257_device *bq)
{
	int ret;

	ret = aw32257_exec_command(bq, AW32257_VENDER_CODE);
	if (ret < 0)
		return 0;

	/* convert to binary */
	return (ret & 0x1) +
	       ((ret >> 1) & 0x1) * 10 +
	       ((ret >> 2) & 0x1) * 100;
}

/* reset all chip registers to default state */
static void aw32257_reset_chip(struct aw32257_device *bq)
{
	aw32257_i2c_write(bq, AW32257_REG_CURRENT, AW32257_RESET_CURRENT);
	aw32257_i2c_write(bq, AW32257_REG_VOLTAGE, AW32257_RESET_VOLTAGE);
	aw32257_i2c_write(bq, AW32257_REG_CONTROL, AW32257_RESET_CONTROL);
	aw32257_i2c_write(bq, AW32257_REG_STATUS, AW32257_RESET_STATUS);
	bq->timer_error = NULL;
}

/**** properties functions ****/

/* set current limit in mA */
static int aw32257_set_current_limit(struct aw32257_device *bq, int mA)
{
	int val;

	if (mA <= 100)
		val = 0;
	else if (mA <= 500)
		val = 1;
	else if (mA <= 800)
		val = 2;
	else
		val = 3;

	return aw32257_i2c_write_mask(bq, AW32257_REG_CONTROL, val,
			AW32257_MASK_LIMIT, AW32257_SHIFT_LIMIT);
}

/* get current limit in mA */
static int aw32257_get_current_limit(struct aw32257_device *bq)
{
	int ret;

	ret = aw32257_i2c_read_mask(bq, AW32257_REG_CONTROL,
			AW32257_MASK_LIMIT, AW32257_SHIFT_LIMIT);
	if (ret < 0)
		return ret;
	else if (ret == 0)
		return 100;
	else if (ret == 1)
		return 500;
	else if (ret == 2)
		return 800;
	else if (ret == 3)
		return 1800;
	return -EINVAL;
}

/* set weak battery voltage in mV */
static int aw32257_set_weak_battery_voltage(struct aw32257_device *bq, int mV)
{
	int val;

	/* round to 100mV */
	if (mV <= 3400 + 50)
		val = 0;
	else if (mV <= 3500 + 50)
		val = 1;
	else if (mV <= 3600 + 50)
		val = 2;
	else
		val = 3;

	return aw32257_i2c_write_mask(bq, AW32257_REG_CONTROL, val,
			AW32257_MASK_VLOWV, AW32257_SHIFT_VLOWV);
}

/* get weak battery voltage in mV */
static int aw32257_get_weak_battery_voltage(struct aw32257_device *bq)
{
	int ret;

	ret = aw32257_i2c_read_mask(bq, AW32257_REG_CONTROL,
			AW32257_MASK_VLOWV, AW32257_SHIFT_VLOWV);
	if (ret < 0)
		return ret;
	return 100 * (34 + ret);
}

/* set battery regulation voltage in mV */
static int aw32257_set_battery_regulation_voltage(struct aw32257_device *bq,
						  int mV)
{
	int val = (mV/10 - 350) / 2;

	/*
	 * According to datasheet, maximum battery regulation voltage is
	 * 4440mV which is b101111 = 47.
	 */
	if (val < 0)
		val = 0;
	else if (val > 47)
		return -EINVAL;

	return aw32257_i2c_write_mask(bq, AW32257_REG_VOLTAGE, val,
			AW32257_MASK_VO, AW32257_SHIFT_VO);
}

/* get battery regulation voltage in mV */
static int aw32257_get_battery_regulation_voltage(struct aw32257_device *bq)
{
	int ret = aw32257_i2c_read_mask(bq, AW32257_REG_VOLTAGE,
			AW32257_MASK_VO, AW32257_SHIFT_VO);

	if (ret < 0)
		return ret;
	return 10 * (350 + 2*ret);
}

/* set charge current in mA (platform data must provide resistor sense) */
static int aw32257_set_charge_current(struct aw32257_device *bq, int mA)
{
	u32 level, val;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	if (mA > 1200)
		mA = 1200;

	level = aw32257_find_closest_level(aw32257_cs_vth,
					   ARRAY_SIZE(aw32257_cs_vth),
					   mA * 100);
	val = aw32257_level_to_index(aw32257_cs_vth,
				     ARRAY_SIZE(aw32257_cs_vth), level);

	return aw32257_i2c_write_mask(bq, AW32257_REG_CURRENT, val,
			AW32257_MASK_VI_CHRG_EXT | AW32257_MASK_RESET,
			AW32257_SHIFT_VI_CHRG_EXT);
}

/* get charge current in mA (platform data must provide resistor sense) */
static int aw32257_get_charge_current(struct aw32257_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	ret = aw32257_i2c_read_mask(bq, AW32257_REG_CURRENT,
			AW32257_MASK_VI_CHRG_EXT,
			AW32257_SHIFT_VI_CHRG_EXT);
	if (ret < 0)
		return ret;

	return 496 + 124 * ret;
}

/* set termination current in mA (platform data must provide resistor sense) */
static int aw32257_set_termination_current(struct aw32257_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	val = mA / 60;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;

	return aw32257_i2c_write_mask(bq, AW32257_REG_CURRENT, val,
			AW32257_MASK_VI_TERM | AW32257_MASK_RESET,
			AW32257_SHIFT_VI_TERM);
}

/* get termination current in mA (platform data must provide resistor sense) */
static int aw32257_get_termination_current(struct aw32257_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -EINVAL;

	ret = aw32257_i2c_read_mask(bq, AW32257_REG_CURRENT,
			AW32257_MASK_VI_TERM, AW32257_SHIFT_VI_TERM);
	if (ret < 0)
		return ret;

	return ret * 60;
}

/* set default value of property */
#define aw32257_set_default_value(bq, prop) \
	do { \
		int ret = 0; \
		if (bq->init_data.prop != -1) \
			ret = aw32257_set_##prop(bq, bq->init_data.prop); \
		if (ret < 0) \
			return ret; \
	} while (0)

/* set default values of all properties */
static int aw32257_set_defaults(struct aw32257_device *bq)
{
	aw32257_exec_command(bq, AW32257_BOOST_MODE_DISABLE);
	aw32257_exec_command(bq, AW32257_CHARGER_DISABLE);
	aw32257_exec_command(bq, AW32257_CHARGE_TERMINATION_DISABLE);

	aw32257_set_default_value(bq, current_limit);
	aw32257_set_default_value(bq, weak_battery_voltage);
	aw32257_set_default_value(bq, battery_regulation_voltage);

	if (bq->init_data.resistor_sense > 0) {
		aw32257_set_default_value(bq, charge_current);
		aw32257_set_default_value(bq, termination_current);
		aw32257_exec_command(bq, AW32257_CHARGE_TERMINATION_ENABLE);
	}

	aw32257_exec_command(bq, AW32257_CHARGER_ENABLE);
	return 0;
}

static int aw32257_parse_jeita(struct aw32257_device *bq,
			       struct device_node *bat_np)
{
	int i;

	for (i = 0; i < AW32257_JEITA_MAX; i++) {
		const __be32 *list;
		int size, n, j;

		list = of_get_property(bat_np, aw32257_jeita_names[i], &size);
		if (!list || !size)
			continue;

		n = size / (4 * sizeof(__be32));
		bq->jeita[i] = devm_kcalloc(bq->dev, n,
					    sizeof(*bq->jeita[i]), GFP_KERNEL);
		if (!bq->jeita[i])
			return -ENOMEM;

		bq->jeita_len[i] = n;
		for (j = 0; j < n; j++) {
			bq->jeita[i][j].temp =
				be32_to_cpu(list[4 * j]) - 1000;
			bq->jeita[i][j].recovery =
				be32_to_cpu(list[4 * j + 1]) - 1000;
			bq->jeita[i][j].current_ma =
				be32_to_cpu(list[4 * j + 2]) / 1000;
			bq->jeita[i][j].voltage_mv =
				be32_to_cpu(list[4 * j + 3]) / 1000;
		}
	}

	return 0;
}

static int aw32257_apply_jeita(struct aw32257_device *bq)
{
	struct aw32257_jeita_table *table;
	union power_supply_propval prop;
	int type, len, i, ret;

	switch (bq->mode) {
	case AW32257_MODE_DEDICATED_CHARGER:
		if (bq->usb_type == POWER_SUPPLY_USB_TYPE_CDP)
			type = AW32257_JEITA_CDP;
		else
			type = AW32257_JEITA_DCP;
		break;
	case AW32257_MODE_HOST_CHARGER:
		type = AW32257_JEITA_SDP;
		break;
	case AW32257_MODE_NONE:
		type = AW32257_JEITA_UNKNOWN;
		break;
	default:
		return 0;
	}

	table = bq->jeita[type];
	len = bq->jeita_len[type];
	if (!table || !len)
		return 0;

	if (!bq->fgu) {
		bq->fgu = power_supply_get_by_name("sc27xx-fgu");
		if (!bq->fgu)
			return 0;
	}

	ret = power_supply_get_property(bq->fgu, POWER_SUPPLY_PROP_TEMP,
					&prop);
	if (ret)
		return ret;

	for (i = 0; i < len; i++)
		if (prop.intval <= table[i].temp)
			break;

	if (i == len || table[i].current_ma == 0) {
		aw32257_exec_command(bq, AW32257_CHARGER_DISABLE);
		return 0;
	}

	aw32257_exec_command(bq, AW32257_CHARGER_ENABLE);
	aw32257_set_charge_current(bq, table[i].current_ma);
	if (table[i].voltage_mv > 0)
		aw32257_set_battery_regulation_voltage(bq,
						       table[i].voltage_mv);

	return 0;
}

static bool aw32257_refresh_input(struct aw32257_device *bq);

static void aw32257_jeita_work(struct work_struct *work)
{
	struct aw32257_device *bq = container_of(work, struct aw32257_device,
						 jeita_work.work);
	bool changed;

	/*
	 * Re-read the charger detector on every poll. The power supply
	 * notifier can be missed when the detector probes after us, so this
	 * is the fallback that keeps the input current in sync.
	 */
	changed = aw32257_refresh_input(bq);
	if (!changed || bq->automode <= 0)
		aw32257_apply_jeita(bq);

	if (bq->mode == AW32257_MODE_DEDICATED_CHARGER ||
	    bq->mode == AW32257_MODE_HOST_CHARGER ||
	    bq->mode == AW32257_MODE_NONE)
		schedule_delayed_work(&bq->jeita_work, 30 * HZ);
}

/**** charger mode functions ****/

/* set charger mode */
static int aw32257_set_mode(struct aw32257_device *bq, enum aw32257_mode mode)
{
	int ret = 0;
	int charger = 0;
	int boost = 0;
	int charge_current = 0;

	if (mode == AW32257_MODE_BOOST)
		boost = 1;
	else if (mode != AW32257_MODE_OFF)
		charger = 1;

	if (!charger)
		ret = aw32257_exec_command(bq, AW32257_CHARGER_DISABLE);

	if (!boost) {
		ret = aw32257_exec_command(bq, AW32257_BOOST_MODE_DISABLE);
		if (ret < 0)
			return ret;
		ret = aw32257_exec_command(bq, AW32257_OTG_PIN_DISABLE);
		if (ret < 0)
			return ret;
		ret = aw32257_exec_command(bq, AW32257_OTG_ACTIVATE_LOW);
		if (ret < 0)
			return ret;
	}

	if (ret < 0)
		return ret;

	switch (mode) {
	case AW32257_MODE_OFF:
		dev_dbg(bq->dev, "changing mode to: Offline\n");
		ret = aw32257_set_current_limit(bq, 100);
		break;
	case AW32257_MODE_NONE:
		dev_dbg(bq->dev, "changing mode to: N/A\n");
		ret = aw32257_set_current_limit(bq, 100);
		charge_current = bq->init_data.charge_current_unknown;
		break;
	case AW32257_MODE_HOST_CHARGER:
		dev_dbg(bq->dev, "changing mode to: Host/HUB charger\n");
		ret = aw32257_set_current_limit(bq, 500);
		charge_current = bq->init_data.charge_current_sdp;
		break;
	case AW32257_MODE_DEDICATED_CHARGER:
		dev_dbg(bq->dev, "changing mode to: Dedicated charger\n");
		ret = aw32257_set_current_limit(bq, 1800);
		if (bq->usb_type == POWER_SUPPLY_USB_TYPE_CDP &&
		    bq->init_data.charge_current_cdp > 0)
			charge_current = bq->init_data.charge_current_cdp;
		else
			charge_current = bq->init_data.charge_current_dcp;
		break;
	case AW32257_MODE_BOOST: /* Boost mode */
		dev_dbg(bq->dev, "changing mode to: Boost\n");
		ret = aw32257_set_current_limit(bq, 100);
		break;
	}

	if (ret < 0)
		return ret;

	if (charge_current > 0 && bq->init_data.resistor_sense > 0) {
		ret = aw32257_set_charge_current(bq, charge_current);
		if (ret < 0)
			return ret;
	}

	if (charger) {
		ret = aw32257_exec_command(bq, AW32257_CHARGER_ENABLE);
	} else if (boost) {
		ret = aw32257_exec_command(bq, AW32257_BOOST_MODE_ENABLE);
		if (ret < 0)
			return ret;
		ret = aw32257_exec_command(bq, AW32257_OTG_ACTIVATE_HIGH);
		if (ret < 0)
			return ret;
		ret = aw32257_exec_command(bq, AW32257_OTG_PIN_ENABLE);
	}

	if (ret < 0)
		return ret;

	aw32257_set_default_value(bq, weak_battery_voltage);
	aw32257_set_default_value(bq, battery_regulation_voltage);

	bq->mode = mode;

	if (mode == AW32257_MODE_DEDICATED_CHARGER ||
	    mode == AW32257_MODE_HOST_CHARGER ||
	    mode == AW32257_MODE_NONE) {
		aw32257_apply_jeita(bq);
		schedule_delayed_work(&bq->jeita_work, 30 * HZ);
	} else {
		cancel_delayed_work(&bq->jeita_work);
	}

	sysfs_notify(&bq->charger->dev.kobj, NULL, "mode");

	return 0;

}

/**** USB OTG VBUS regulator (charger boost mode) ****/

#if IS_REACHABLE(CONFIG_REGULATOR)
static int aw32257_vbus_enable(struct regulator_dev *rdev)
{
	struct aw32257_device *bq = rdev_get_drvdata(rdev);

	return aw32257_set_mode(bq, AW32257_MODE_BOOST);
}

static int aw32257_vbus_disable(struct regulator_dev *rdev)
{
	struct aw32257_device *bq = rdev_get_drvdata(rdev);

	return aw32257_set_mode(bq, AW32257_MODE_OFF);
}

static int aw32257_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct aw32257_device *bq = rdev_get_drvdata(rdev);
	int ret;

	ret = aw32257_exec_command(bq, AW32257_BOOST_MODE_STATUS);
	if (ret < 0)
		return ret;

	return !!ret;
}

static const struct regulator_ops aw32257_vbus_ops = {
	.enable		= aw32257_vbus_enable,
	.disable	= aw32257_vbus_disable,
	.is_enabled	= aw32257_vbus_is_enabled,
};

static const struct regulator_desc aw32257_vbus_desc = {
	.name		= "aw32257-otg-vbus",
	.of_match	= of_match_ptr("usb-otg-vbus"),
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.ops		= &aw32257_vbus_ops,
	.fixed_uV	= 5000000,
	.n_voltages	= 1,
};

static int aw32257_register_vbus_regulator(struct aw32257_device *bq)
{
	struct device_node *np;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = { };

	/* Optional: no subnode means the board does not use charger boost. */
	np = of_get_child_by_name(bq->dev->of_node, "usb-otg-vbus");
	if (!np)
		return 0;

	init_data = of_get_regulator_init_data(bq->dev, np,
					       &aw32257_vbus_desc);
	if (!init_data) {
		of_node_put(np);
		return -ENOMEM;
	}

	cfg.dev = bq->dev;
	cfg.init_data = init_data;
	cfg.driver_data = bq;
	cfg.of_node = np;

	bq->usb_vbus_reg = devm_regulator_register(bq->dev,
						   &aw32257_vbus_desc, &cfg);
	of_node_put(np);

	return PTR_ERR_OR_ZERO(bq->usb_vbus_reg);
}
#else
static inline int aw32257_register_vbus_regulator(struct aw32257_device *bq)
{
	return 0;
}
#endif

static bool aw32257_update_reported_mode(struct aw32257_device *bq, int mA)
{
	enum aw32257_mode mode;

	switch (bq->usb_type) {
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_CDP:
		mode = AW32257_MODE_DEDICATED_CHARGER;
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
		mode = AW32257_MODE_HOST_CHARGER;
		break;
	default:
		if (mA == 0)
			mode = AW32257_MODE_OFF;
		else if (mA < 500)
			mode = AW32257_MODE_NONE;
		else if (mA < 1800)
			mode = AW32257_MODE_HOST_CHARGER;
		else
			mode = AW32257_MODE_DEDICATED_CHARGER;
		break;
	}

	if (bq->reported_mode == mode)
		return false;

	bq->reported_mode = mode;
	return true;
}

static struct power_supply *aw32257_get_notify_psy(struct aw32257_device *bq)
{
	if (dev_fwnode(bq->dev))
		return power_supply_get_by_reference(dev_fwnode(bq->dev),
						"ti,usb-charger-detection");
	if (bq->init_data.notify_device)
		return power_supply_get_by_name(bq->init_data.notify_device);

	return NULL;
}

static bool aw32257_refresh_input(struct aw32257_device *bq)
{
	struct power_supply *psy;
	union power_supply_propval prop;
	int mA = 0;
	bool changed;

	psy = aw32257_get_notify_psy(bq);
	if (IS_ERR_OR_NULL(psy))
		return false;

	/* Never treat a read error as "no current", that would stop charging. */
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop)) {
		power_supply_put(psy);
		return false;
	}
	mA = prop.intval;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &prop))
		bq->usb_type = prop.intval;
	power_supply_put(psy);

	changed = aw32257_update_reported_mode(bq, mA);
	if (changed && bq->automode > 0) {
		dev_dbg(bq->dev, "USB type %d, limit %d mA, mode %d\n",
			bq->usb_type, mA, bq->reported_mode);
		aw32257_set_mode(bq, bq->reported_mode);
	}

	return changed;
}

static int aw32257_notifier_call(struct notifier_block *nb,
		unsigned long val, void *v)
{
	struct aw32257_device *bq =
		container_of(nb, struct aw32257_device, nb);
	struct power_supply *psy = v;
	union power_supply_propval prop, type;
	int ret;

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	/* Ignore event if it was not send by notify_node/notify_device */
	if (bq->notify_node) {
		if (!psy->dev.of_node || psy->dev.of_node != bq->notify_node)
			return NOTIFY_OK;
	} else if (bq->init_data.notify_device) {
		if (strcmp(psy->desc->name, bq->init_data.notify_device) != 0)
			return NOTIFY_OK;
	}

	dev_dbg(bq->dev, "notifier call was called\n");

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX,
			&prop);
	if (ret != 0)
		return NOTIFY_OK;

	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &type))
		bq->usb_type = type.intval;

	if (!aw32257_update_reported_mode(bq, prop.intval))
		return NOTIFY_OK;

	power_supply_changed(bq->charger);

	/* if automode is not enabled do not tell about reported_mode */
	if (bq->automode < 1)
		return NOTIFY_OK;

	mod_delayed_work(system_percpu_wq, &bq->work, 0);

	return NOTIFY_OK;
}

/**** timer functions ****/

/* enable/disable auto resetting chip timer */
static void aw32257_set_autotimer(struct aw32257_device *bq, int state)
{
	mutex_lock(&aw32257_timer_mutex);

	if (bq->autotimer == state) {
		mutex_unlock(&aw32257_timer_mutex);
		return;
	}

	bq->autotimer = state;

	if (state) {
		schedule_delayed_work(&bq->work, AW32257_TIMER_TIMEOUT * HZ);
		aw32257_exec_command(bq, AW32257_TIMER_RESET);
		bq->timer_error = NULL;
	} else {
		cancel_delayed_work_sync(&bq->work);
	}

	mutex_unlock(&aw32257_timer_mutex);
}

/* called by aw32257_timer_work on timer error */
static void aw32257_timer_error(struct aw32257_device *bq, const char *msg)
{
	bq->timer_error = msg;
	sysfs_notify(&bq->charger->dev.kobj, NULL, "timer");
	dev_err(bq->dev, "%s\n", msg);
	if (bq->automode > 0)
		bq->automode = 0;
	aw32257_set_mode(bq, AW32257_MODE_OFF);
	aw32257_set_autotimer(bq, 0);
}

/* delayed work function for auto resetting chip timer */
static void aw32257_timer_work(struct work_struct *work)
{
	struct aw32257_device *bq = container_of(work, struct aw32257_device,
						 work.work);
	int ret;
	int error;
	int boost;
	int charge;

	/*
	 * If the detector probed after us, the notifier was never delivered.
	 * Poll it here so reported_mode and the input current catch up.
	 */
	if (bq->automode > 0)
		aw32257_refresh_input(bq);

	if (bq->automode > 0 && (bq->reported_mode != bq->mode)) {
		sysfs_notify(&bq->charger->dev.kobj, NULL, "reported_mode");
		aw32257_set_mode(bq, bq->reported_mode);
	}

	charge = aw32257_exec_command(bq, AW32257_CHARGE_STATUS);
	if (bq->charge_status != charge) {
		power_supply_changed(bq->charger);
		bq->charge_status = charge;
	}

	if (!bq->autotimer)
		return;

	ret = aw32257_exec_command(bq, AW32257_TIMER_RESET);
	if (ret < 0) {
		aw32257_timer_error(bq, "Resetting timer failed");
		return;
	}

	boost = aw32257_exec_command(bq, AW32257_BOOST_MODE_STATUS);
	if (boost < 0) {
		aw32257_timer_error(bq, "Unknown error");
		return;
	}

	error = aw32257_exec_command(bq, AW32257_FAULT_STATUS);
	if (error < 0) {
		aw32257_timer_error(bq, "Unknown error");
		return;
	}

	if (boost) {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 3: /* Battery voltage too low */
			dev_err(bq->dev, "Battery voltage to low\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			aw32257_timer_error(bq,
				"Overvoltage protection (chip fried)");
			return;
		case 2: /* Overload */
			aw32257_timer_error(bq, "Overload");
			return;
		case 4: /* Battery overvoltage protection */
			aw32257_timer_error(bq,
				"Battery overvoltage protection");
			return;
		case 5: /* Thermal shutdown (too hot) */
			aw32257_timer_error(bq,
					"Thermal shutdown (too hot)");
			return;
		case 7: /* N/A */
			aw32257_timer_error(bq, "Unknown error");
			return;
		}
	} else {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 2: /* Sleep mode */
			dev_err(bq->dev, "Sleep mode\n");
			break;
		case 3: /* Poor input source */
			dev_err(bq->dev, "Poor input source\n");
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 7: /* No battery */
			dev_err(bq->dev, "No battery\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			aw32257_timer_error(bq,
				"Overvoltage protection (chip fried)");
			return;
		case 4: /* Battery overvoltage protection */
			aw32257_timer_error(bq,
				"Battery overvoltage protection");
			return;
		case 5: /* Thermal shutdown (too hot) */
			aw32257_timer_error(bq,
				"Thermal shutdown (too hot)");
			return;
		}
	}

	schedule_delayed_work(&bq->work, AW32257_TIMER_TIMEOUT * HZ);
}

/**** power supply interface code ****/

static enum power_supply_property aw32257_power_supply_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int aw32257_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = aw32257_exec_command(bq, AW32257_CHARGE_STATUS);
		if (ret < 0)
			return ret;
		else if (ret == 0) /* Ready */
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (ret == 1) /* Charge in progress */
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (ret == 2) /* Charge done */
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (bq->usb_type) {
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
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* VBUS is present for all charging and fault states,
		 * except the 'Ready' state.
		 */
		ret = aw32257_exec_command(bq, AW32257_CHARGE_STATUS);
		if (ret < 0)
			return ret;
		val->intval = ret > 0;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = bq->usb_type;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		switch (bq->usb_type) {
		case POWER_SUPPLY_USB_TYPE_SDP:
			val->intval = 500000;
			break;
		case POWER_SUPPLY_USB_TYPE_CDP:
			val->intval = 1500000;
			break;
		case POWER_SUPPLY_USB_TYPE_DCP:
			val->intval = 2000000;
			break;
		default:
			val->intval = 0;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = aw32257_get_charge_current(bq);
		if (ret < 0)
			return ret;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = aw32257_get_battery_regulation_voltage(bq);
		if (ret < 0)
			return ret;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = aw32257_get_current_limit(bq);
		if (ret < 0)
			return ret;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = aw32257_get_termination_current(bq);
		if (ret < 0)
			return ret;
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Awinic";
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int aw32257_power_supply_set_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     const union power_supply_propval *val)
{
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (val->intval == POWER_SUPPLY_STATUS_CHARGING)
			ret = aw32257_exec_command(bq, AW32257_CHARGER_ENABLE);
		else
			ret = aw32257_exec_command(bq, AW32257_CHARGER_DISABLE);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = aw32257_set_charge_current(bq, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = aw32257_set_battery_regulation_voltage(bq,
							     val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = aw32257_set_current_limit(bq, val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int aw32257_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		return 0;
	}
}

static void aw32257_power_supply_exit(struct aw32257_device *bq)
{
	bq->autotimer = 0;
	if (bq->automode > 0)
		bq->automode = 0;
	cancel_delayed_work_sync(&bq->work);
	power_supply_unregister(bq->charger);
	kfree(bq->model);
}

/**** additional sysfs entries for power supply interface ****/

/* show *_status entries */
static ssize_t aw32257_sysfs_show_status(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	enum aw32257_command command;
	int ret;

	if (strcmp(attr->attr.name, "otg_status") == 0)
		command = AW32257_OTG_STATUS;
	else if (strcmp(attr->attr.name, "charge_status") == 0)
		command = AW32257_CHARGE_STATUS;
	else if (strcmp(attr->attr.name, "boost_status") == 0)
		command = AW32257_BOOST_STATUS;
	else if (strcmp(attr->attr.name, "fault_status") == 0)
		command = AW32257_FAULT_STATUS;
	else
		return -EINVAL;

	ret = aw32257_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return sysfs_emit(buf, "%d\n", ret);
}

/*
 * set timer entry:
 *    auto - enable auto mode
 *    off - disable auto mode
 *    (other values) - reset chip timer
 */
static ssize_t aw32257_sysfs_set_timer(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0)
		aw32257_set_autotimer(bq, 1);
	else if (strncmp(buf, "off", 3) == 0)
		aw32257_set_autotimer(bq, 0);
	else
		ret = aw32257_exec_command(bq, AW32257_TIMER_RESET);

	if (ret < 0)
		return ret;
	return count;
}

/* show timer entry (auto or off) */
static ssize_t aw32257_sysfs_show_timer(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);

	if (bq->timer_error)
		return sysfs_emit(buf, "%s\n", bq->timer_error);

	if (bq->autotimer)
		return sysfs_emit(buf, "auto\n");
	return sysfs_emit(buf, "off\n");
}

/*
 * set mode entry:
 *    auto - if automode is supported, enable it and set mode to reported
 *    none - disable charger and boost mode
 *    host - charging mode for host/hub chargers (current limit 500mA)
 *    dedicated - charging mode for dedicated chargers (unlimited current limit)
 *    boost - disable charger and enable boost mode
 */
static ssize_t aw32257_sysfs_set_mode(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	enum aw32257_mode mode;
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0) {
		if (bq->automode < 0)
			return -EINVAL;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else if (strncmp(buf, "off", 3) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = AW32257_MODE_OFF;
	} else if (strncmp(buf, "none", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = AW32257_MODE_NONE;
	} else if (strncmp(buf, "host", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = AW32257_MODE_HOST_CHARGER;
	} else if (strncmp(buf, "dedicated", 9) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = AW32257_MODE_DEDICATED_CHARGER;
	} else if (strncmp(buf, "boost", 5) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = AW32257_MODE_BOOST;
	} else if (strncmp(buf, "reset", 5) == 0) {
		aw32257_reset_chip(bq);
		aw32257_set_defaults(bq);
		if (bq->automode <= 0)
			return count;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else {
		return -EINVAL;
	}

	ret = aw32257_set_mode(bq, mode);
	if (ret < 0)
		return ret;
	return count;
}

/* show mode entry (auto, none, host, dedicated or boost) */
static ssize_t aw32257_sysfs_show_mode(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	if (bq->automode > 0)
		ret += sysfs_emit_at(buf, ret, "auto (");

	switch (bq->mode) {
	case AW32257_MODE_OFF:
		ret += sysfs_emit_at(buf, ret, "off");
		break;
	case AW32257_MODE_NONE:
		ret += sysfs_emit_at(buf, ret, "none");
		break;
	case AW32257_MODE_HOST_CHARGER:
		ret += sysfs_emit_at(buf, ret, "host");
		break;
	case AW32257_MODE_DEDICATED_CHARGER:
		ret += sysfs_emit_at(buf, ret, "dedicated");
		break;
	case AW32257_MODE_BOOST:
		ret += sysfs_emit_at(buf, ret, "boost");
		break;
	}

	if (bq->automode > 0)
		ret += sysfs_emit_at(buf, ret, ")");

	ret += sysfs_emit_at(buf, ret, "\n");
	return ret;
}

/* show reported_mode entry (none, host, dedicated or boost) */
static ssize_t aw32257_sysfs_show_reported_mode(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);

	if (bq->automode < 0)
		return -EINVAL;

	switch (bq->reported_mode) {
	case AW32257_MODE_OFF:
		return sysfs_emit(buf, "off\n");
	case AW32257_MODE_NONE:
		return sysfs_emit(buf, "none\n");
	case AW32257_MODE_HOST_CHARGER:
		return sysfs_emit(buf, "host\n");
	case AW32257_MODE_DEDICATED_CHARGER:
		return sysfs_emit(buf, "dedicated\n");
	case AW32257_MODE_BOOST:
		return sysfs_emit(buf, "boost\n");
	}

	return -EINVAL;
}

/* directly set raw value to chip register, format: 'register value' */
static ssize_t aw32257_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;

	if (reg > 4 || val > 255)
		return -EINVAL;

	ret = aw32257_i2c_write(bq, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/* print value of chip register, format: 'register=value' */
static ssize_t aw32257_sysfs_print_reg(struct aw32257_device *bq,
				       u8 reg,
				       char *buf)
{
	int ret = aw32257_i2c_read(bq, reg);

	if (ret < 0)
		return sysfs_emit(buf, "%#.2x=error %d\n", reg, ret);
	return sysfs_emit(buf, "%#.2x=%#.2x\n", reg, ret);
}

/* show all raw values of chip register, format per line: 'register=value' */
static ssize_t aw32257_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	ret += aw32257_sysfs_print_reg(bq, AW32257_REG_STATUS, buf+ret);
	ret += aw32257_sysfs_print_reg(bq, AW32257_REG_CONTROL, buf+ret);
	ret += aw32257_sysfs_print_reg(bq, AW32257_REG_VOLTAGE, buf+ret);
	ret += aw32257_sysfs_print_reg(bq, AW32257_REG_VENDER, buf+ret);
	ret += aw32257_sysfs_print_reg(bq, AW32257_REG_CURRENT, buf+ret);
	return ret;
}

/* set current and voltage limit entries (in mA or mV) */
static ssize_t aw32257_sysfs_set_limit(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (strcmp(attr->attr.name, "current_limit") == 0)
		ret = aw32257_set_current_limit(bq, val);
	else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		ret = aw32257_set_weak_battery_voltage(bq, val);
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		ret = aw32257_set_battery_regulation_voltage(bq, val);
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		ret = aw32257_set_charge_current(bq, val);
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		ret = aw32257_set_termination_current(bq, val);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;
	return count;
}

/* show current and voltage limit entries (in mA or mV) */
static ssize_t aw32257_sysfs_show_limit(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	int ret;

	if (strcmp(attr->attr.name, "current_limit") == 0)
		ret = aw32257_get_current_limit(bq);
	else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		ret = aw32257_get_weak_battery_voltage(bq);
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		ret = aw32257_get_battery_regulation_voltage(bq);
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		ret = aw32257_get_charge_current(bq);
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		ret = aw32257_get_termination_current(bq);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;
	return sysfs_emit(buf, "%d\n", ret);
}

/* set *_enable entries */
static ssize_t aw32257_sysfs_set_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	enum aw32257_command command;
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0)
		command = val ? AW32257_CHARGE_TERMINATION_ENABLE :
			AW32257_CHARGE_TERMINATION_DISABLE;
	else if (strcmp(attr->attr.name, "high_impedance_enable") == 0)
		command = val ? AW32257_HIGH_IMPEDANCE_ENABLE :
			AW32257_HIGH_IMPEDANCE_DISABLE;
	else if (strcmp(attr->attr.name, "otg_pin_enable") == 0)
		command = val ? AW32257_OTG_PIN_ENABLE :
			AW32257_OTG_PIN_DISABLE;
	else if (strcmp(attr->attr.name, "stat_pin_enable") == 0)
		command = val ? AW32257_STAT_PIN_ENABLE :
			AW32257_STAT_PIN_DISABLE;
	else
		return -EINVAL;

	ret = aw32257_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return count;
}

/* show *_enable entries */
static ssize_t aw32257_sysfs_show_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_to_psy(dev);
	struct aw32257_device *bq = power_supply_get_drvdata(psy);
	enum aw32257_command command;
	int ret;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0)
		command = AW32257_CHARGE_TERMINATION_STATUS;
	else if (strcmp(attr->attr.name, "high_impedance_enable") == 0)
		command = AW32257_HIGH_IMPEDANCE_STATUS;
	else if (strcmp(attr->attr.name, "otg_pin_enable") == 0)
		command = AW32257_OTG_PIN_STATUS;
	else if (strcmp(attr->attr.name, "stat_pin_enable") == 0)
		command = AW32257_STAT_PIN_STATUS;
	else
		return -EINVAL;

	ret = aw32257_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return sysfs_emit(buf, "%d\n", ret);
}

static DEVICE_ATTR(current_limit, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_limit, aw32257_sysfs_set_limit);
static DEVICE_ATTR(weak_battery_voltage, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_limit, aw32257_sysfs_set_limit);
static DEVICE_ATTR(battery_regulation_voltage, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_limit, aw32257_sysfs_set_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_limit, aw32257_sysfs_set_limit);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_limit, aw32257_sysfs_set_limit);

static DEVICE_ATTR(charge_termination_enable, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_enable, aw32257_sysfs_set_enable);
static DEVICE_ATTR(high_impedance_enable, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_enable, aw32257_sysfs_set_enable);
static DEVICE_ATTR(otg_pin_enable, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_enable, aw32257_sysfs_set_enable);
static DEVICE_ATTR(stat_pin_enable, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_enable, aw32257_sysfs_set_enable);

static DEVICE_ATTR(reported_mode, S_IRUGO,
		aw32257_sysfs_show_reported_mode, NULL);
static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_mode, aw32257_sysfs_set_mode);
static DEVICE_ATTR(timer, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_timer, aw32257_sysfs_set_timer);

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		aw32257_sysfs_show_registers, aw32257_sysfs_set_registers);

static DEVICE_ATTR(otg_status, S_IRUGO, aw32257_sysfs_show_status, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, aw32257_sysfs_show_status, NULL);
static DEVICE_ATTR(boost_status, S_IRUGO, aw32257_sysfs_show_status, NULL);
static DEVICE_ATTR(fault_status, S_IRUGO, aw32257_sysfs_show_status, NULL);

static struct attribute *aw32257_sysfs_attrs[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use power supply class props.
	 */
	&dev_attr_current_limit.attr,
	&dev_attr_weak_battery_voltage.attr,
	&dev_attr_battery_regulation_voltage.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_termination_current.attr,

	&dev_attr_charge_termination_enable.attr,
	&dev_attr_high_impedance_enable.attr,
	&dev_attr_otg_pin_enable.attr,
	&dev_attr_stat_pin_enable.attr,

	&dev_attr_reported_mode.attr,
	&dev_attr_mode.attr,
	&dev_attr_timer.attr,

	&dev_attr_registers.attr,

	&dev_attr_otg_status.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_boost_status.attr,
	&dev_attr_fault_status.attr,
	NULL,
};

ATTRIBUTE_GROUPS(aw32257_sysfs);

static int aw32257_power_supply_init(struct aw32257_device *bq)
{
	int ret;
	int chip;
	char revstr[8];
	struct power_supply_config psy_cfg = {
		.drv_data = bq,
		.fwnode = dev_fwnode(bq->dev),
		.attr_grp = aw32257_sysfs_groups,
	};

	bq->charger_desc.name = bq->name;
	bq->charger_desc.type = POWER_SUPPLY_TYPE_USB;
	bq->charger_desc.properties = aw32257_power_supply_props;
	bq->charger_desc.num_properties =
			ARRAY_SIZE(aw32257_power_supply_props);
	bq->charger_desc.get_property = aw32257_power_supply_get_property;
	bq->charger_desc.set_property = aw32257_power_supply_set_property;
	bq->charger_desc.property_is_writeable =
			aw32257_property_is_writeable;
	bq->charger_desc.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
				     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
				     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
				     BIT(POWER_SUPPLY_USB_TYPE_CDP);

	chip = AW32257;

	ret = aw32257_detect_revision(bq);
	if (ret < 0)
		strscpy(revstr, "unknown", sizeof(revstr));
	else
		sprintf(revstr, "1.%d", ret);

	bq->model = kasprintf(GFP_KERNEL,
				"chip %s, revision %s, vender code %.3d",
				aw32257_chip_name[chip], revstr,
				aw32257_get_vender_code(bq));
	if (!bq->model) {
		dev_err(bq->dev, "failed to allocate model name\n");
		return -ENOMEM;
	}

	bq->charger = power_supply_register(bq->dev, &bq->charger_desc,
					    &psy_cfg);
	if (IS_ERR(bq->charger)) {
		kfree(bq->model);
		return PTR_ERR(bq->charger);
	}

	return 0;
}

/* main aw32257 probe function */
static int aw32257_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	int ret;
	int num;
	char *name = NULL;
	struct aw32257_device *bq;
	struct device_node *np = client->dev.of_node;
	struct aw32257_platform_data *pdata = client->dev.platform_data;
	const struct acpi_device_id *acpi_id = NULL;
	struct power_supply *notify_psy = NULL;
	union power_supply_propval prop, type;

	if (!np && !pdata && !ACPI_HANDLE(&client->dev)) {
		dev_err(&client->dev, "Neither devicetree, nor platform data, nor ACPI support\n");
		return -ENODEV;
	}

	/* Get new ID for the new device */
	mutex_lock(&aw32257_id_mutex);
	num = idr_alloc(&aw32257_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&aw32257_id_mutex);
	if (num < 0)
		return num;

	if (id) {
		name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	} else if (ACPI_HANDLE(&client->dev)) {
		acpi_id =
			acpi_match_device(client->dev.driver->acpi_match_table,
					  &client->dev);
		if (!acpi_id) {
			dev_err(&client->dev, "failed to match device name\n");
			ret = -ENODEV;
			goto error_1;
		}
		name = kasprintf(GFP_KERNEL, "%s-%d", acpi_id->id, num);
	}
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		ret = -ENOMEM;
		goto error_1;
	}

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		ret = -ENOMEM;
		goto error_2;
	}

	i2c_set_clientdata(client, bq);

	bq->id = num;
	bq->dev = &client->dev;
	if (id)
		bq->chip = id->driver_data;
	else if (ACPI_HANDLE(bq->dev))
		bq->chip = acpi_id->driver_data;
	bq->chip = AW32257;
	bq->name = name;
	bq->mode = AW32257_MODE_OFF;
	bq->reported_mode = AW32257_MODE_OFF;
	bq->autotimer = 0;
	bq->automode = 0;
	INIT_DELAYED_WORK(&bq->jeita_work, aw32257_jeita_work);

	if (np || ACPI_HANDLE(bq->dev)) {
		ret = device_property_read_u32(bq->dev,
					       "ti,current-limit",
					       &bq->init_data.current_limit);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
					"ti,weak-battery-voltage",
					&bq->init_data.weak_battery_voltage);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
				"ti,battery-regulation-voltage",
				&bq->init_data.battery_regulation_voltage);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
					       "ti,charge-current",
					       &bq->init_data.charge_current);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
				"ti,termination-current",
				&bq->init_data.termination_current);
		if (ret)
			goto error_2;
		ret = device_property_read_u32(bq->dev,
					       "ti,resistor-sense",
					       &bq->init_data.resistor_sense);
		if (ret)
			goto error_2;
		if (np)
			bq->notify_node = of_parse_phandle(np,
						"ti,usb-charger-detection", 0);
		if (np) {
			struct device_node *bat_np;
			u32 ua;

			bat_np = of_parse_phandle(np, "monitored-battery", 0);
			if (bat_np) {
				if (!of_property_read_u32(bat_np,
						"charge-dcp-current-microamp",
						&ua))
					bq->init_data.charge_current_dcp =
						ua / 1000;
				if (!of_property_read_u32(bat_np,
						"charge-cdp-current-microamp",
						&ua))
					bq->init_data.charge_current_cdp =
						ua / 1000;
				if (!of_property_read_u32(bat_np,
						"charge-sdp-current-microamp",
						&ua))
					bq->init_data.charge_current_sdp =
						ua / 1000;
				if (!of_property_read_u32(bat_np,
					"charge-unknown-current-microamp",
					&ua))
					bq->init_data.charge_current_unknown =
						ua / 1000;
				if (!of_property_read_u32(bat_np,
					"constant-charge-current-max-microamp",
					&ua)) {
					if (!bq->init_data.charge_current_dcp)
						bq->init_data.charge_current_dcp =
							ua / 1000;
					if (!bq->init_data.charge_current_cdp)
						bq->init_data.charge_current_cdp =
							ua / 1000;
				}
				if (!bq->init_data.charge_current_sdp)
					bq->init_data.charge_current_sdp = 500;
				if (!bq->init_data.charge_current_unknown)
					bq->init_data.charge_current_unknown = 1000;
				if (!of_property_read_u32(bat_np,
					"constant-charge-voltage-max-microvolt",
					&ua))
					bq->init_data.battery_regulation_voltage =
						ua / 1000;
				aw32257_parse_jeita(bq, bat_np);
				of_node_put(bat_np);
			}
		}
	} else {
		memcpy(&bq->init_data, pdata, sizeof(bq->init_data));
	}

	aw32257_reset_chip(bq);

	ret = aw32257_power_supply_init(bq);
	if (ret) {
		dev_err(bq->dev, "failed to register power supply: %d\n", ret);
		goto error_2;
	}

	ret = aw32257_set_defaults(bq);
	if (ret) {
		dev_err(bq->dev, "failed to set default values: %d\n", ret);
		goto error_3;
	}

	/*
	 * Expose the charger's boost mode as a USB OTG VBUS regulator so a
	 * USB role switch (or TCPM) can source 5V on VBUS in host mode.
	 * Fails probe (for -EPROBE_DEFER) so vbus-supply consumers resolve.
	 */
	ret = aw32257_register_vbus_regulator(bq);
	if (ret) {
		dev_err(bq->dev,
			"failed to register usb-otg-vbus regulator: %d\n", ret);
		goto error_3;
	}

	if (bq->notify_node || bq->init_data.notify_device) {
		bq->nb.notifier_call = aw32257_notifier_call;
		ret = power_supply_reg_notifier(&bq->nb);
		if (ret) {
			dev_err(bq->dev, "failed to reg notifier: %d\n", ret);
			goto error_3;
		}

		bq->automode = 1;
		dev_info(bq->dev, "automode supported, waiting for events\n");
	} else {
		bq->automode = -1;
		dev_info(bq->dev, "automode not supported\n");
	}

	/* Query for initial reported_mode and set it */
	if (bq->nb.notifier_call) {
		notify_psy = aw32257_get_notify_psy(bq);
		if (IS_ERR(notify_psy)) {
			dev_warn(bq->dev,
				 "charger detector not ready (%ld), will poll\n",
				 PTR_ERR(notify_psy));
			notify_psy = NULL;
		}
	}
	if (notify_psy) {
		ret = power_supply_get_property(notify_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
		if (!power_supply_get_property(notify_psy,
					POWER_SUPPLY_PROP_USB_TYPE, &type))
			bq->usb_type = type.intval;
		power_supply_put(notify_psy);

		if (ret == 0) {
			dev_info(bq->dev, "detected USB type %d, limit %d mA\n",
				 bq->usb_type, prop.intval);
			aw32257_update_reported_mode(bq, prop.intval);
			aw32257_set_mode(bq, bq->reported_mode);
		}
	}

	INIT_DELAYED_WORK(&bq->work, aw32257_timer_work);
	aw32257_set_autotimer(bq, 1);

	dev_info(bq->dev, "driver registered\n");
	return 0;

error_3:
	aw32257_power_supply_exit(bq);
error_2:
	if (bq)
		of_node_put(bq->notify_node);
	kfree(name);
error_1:
	mutex_lock(&aw32257_id_mutex);
	idr_remove(&aw32257_id, num);
	mutex_unlock(&aw32257_id_mutex);

	return ret;
}

/* main aw32257 remove function */

static void aw32257_remove(struct i2c_client *client)
{
	struct aw32257_device *bq = i2c_get_clientdata(client);

	bq->mode = AW32257_MODE_OFF;
	cancel_delayed_work_sync(&bq->jeita_work);
	if (bq->fgu) {
		power_supply_put(bq->fgu);
		bq->fgu = NULL;
	}

	if (bq->nb.notifier_call)
		power_supply_unreg_notifier(&bq->nb);

	of_node_put(bq->notify_node);
	aw32257_power_supply_exit(bq);

	aw32257_reset_chip(bq);

	mutex_lock(&aw32257_id_mutex);
	idr_remove(&aw32257_id, bq->id);
	mutex_unlock(&aw32257_id_mutex);

	dev_info(bq->dev, "driver unregistered\n");

	kfree(bq->name);
}

static const struct i2c_device_id aw32257_i2c_id_table[] = {
	{ "aw32257", AW32257 },
	{}
};
MODULE_DEVICE_TABLE(i2c, aw32257_i2c_id_table);

#ifdef CONFIG_OF
static const struct of_device_id aw32257_of_match_table[] = {
	{ .compatible = "awinic,aw32257" },
	{}
};
MODULE_DEVICE_TABLE(of, aw32257_of_match_table);
#endif

static struct i2c_driver aw32257_driver = {
	.driver = {
		.name = "aw32257-charger",
		.of_match_table = of_match_ptr(aw32257_of_match_table),
	},
	.probe = aw32257_probe,
	.remove = aw32257_remove,
	.id_table = aw32257_i2c_id_table,
};
module_i2c_driver(aw32257_driver);

MODULE_AUTHOR("Pali Rohár <pali@kernel.org>");
MODULE_DESCRIPTION("aw32257 charger driver");
MODULE_LICENSE("GPL");
