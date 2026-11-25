// SPDX-License-Identifier: GPL-2.0-only
/* Orient Chip OCP81375 dual flash LED controller. */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <media/v4l2-flash-led-class.h>

#define OCP81375_ENABLE		0x01
#define OCP81375_IVFM		0x02
#define OCP81375_BOOST		0x07
#define OCP81375_FLASH_CURRENT	0x03
#define OCP81375_TORCH_CURRENT	0x05
#define OCP81375_TIMING		0x08
#define OCP81375_FLAGS1		0x0a
#define OCP81375_FLAGS2		0x0b
#define OCP81375_ID		0x0c
#define OCP81375_MODE_MASK	GENMASK(3, 2)
#define OCP81375_TORCH		BIT(3)
#define OCP81375_FLASH		GENMASK(3, 2)
#define OCP81375_CHANNELS		GENMASK(1, 0)
#define OCP81375_TORCH_STEP_UA	3900
#define OCP81375_FLASH_STEP_UA	15630
#define OCP81375_TIMEOUT_US	600000

struct ocp81375;

struct ocp81375_led {
	struct led_classdev_flash flash;
	struct ocp81375 *chip;
	u32 id;
};

struct ocp81375 {
	struct regmap *regmap;
	struct gpio_desc *enable;
	struct mutex lock; /* Serializes both outputs and their shared mode. */
	bool suspended;
	unsigned long registered;
	u32 faults[2];
	struct ocp81375_led leds[2];
};

/* Fault registers clear on read; preserve reports for both LED devices. */
static int ocp81375_read_faults(struct ocp81375 *chip)
{
	unsigned int flags;
	u32 common = 0;
	int ret;

	ret = regmap_read(chip->regmap, OCP81375_FLAGS1, &flags);
	if (ret)
		return ret;
	if (flags & BIT(0))
		common |= LED_FAULT_TIMEOUT;
	if (flags & BIT(2))
		common |= LED_FAULT_OVER_TEMPERATURE;
	if (flags & BIT(6))
		common |= LED_FAULT_SHORT_CIRCUIT;
	for (unsigned int i = 0; i < ARRAY_SIZE(chip->leds); i++) {
		chip->faults[i] |= common;
		if (flags & BIT(5 - i))
			chip->faults[i] |= LED_FAULT_SHORT_CIRCUIT;
	}
	ret = regmap_read(chip->regmap, OCP81375_FLAGS2, &flags);
	if (ret)
		return ret;
	common = 0;
	if (flags & BIT(1))
		common |= LED_FAULT_OVER_VOLTAGE;
	if (flags & BIT(2))
		common |= LED_FAULT_INPUT_VOLTAGE;
	for (unsigned int i = 0; i < ARRAY_SIZE(chip->leds); i++)
		chip->faults[i] |= common;
	return 0;
}

/* The two outputs share a mode register and a hardware flash timer. */
static int ocp81375_torch_set(struct led_classdev *cdev,
			      enum led_brightness brightness)
{
	struct ocp81375_led *led = container_of(cdev, struct ocp81375_led, flash.led_cdev);
	struct ocp81375 *chip = led->chip;
	unsigned int val, channels = 0;
	int ret;

	mutex_lock(&chip->lock);
	if (chip->suspended) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = regmap_read(chip->regmap, OCP81375_ENABLE, &val);
	if (ret)
		goto unlock;
	if ((val & OCP81375_CHANNELS) && (val & OCP81375_MODE_MASK) == OCP81375_FLASH) {
		ret = brightness ? -EBUSY :
			regmap_update_bits(chip->regmap, OCP81375_ENABLE, BIT(led->id), 0);
		goto unlock;
	}
	if ((val & OCP81375_MODE_MASK) == OCP81375_TORCH)
		channels = val & OCP81375_CHANNELS;
	if (brightness) {
		ret = ocp81375_read_faults(chip);
		if (ret)
			goto unlock;
		ret = regmap_write(chip->regmap, OCP81375_TORCH_CURRENT + led->id,
				   brightness - 1);
		if (ret)
			goto unlock;
		channels |= BIT(led->id);
	} else {
		channels &= ~BIT(led->id);
	}
	ret = regmap_write(chip->regmap, OCP81375_ENABLE,
			   channels ? OCP81375_TORCH | channels : 0);
unlock:
	mutex_unlock(&chip->lock);
	return ret;
}

static int ocp81375_flash_set(struct led_classdev_flash *flash, u32 brightness)
{
	struct ocp81375_led *led = container_of(flash, struct ocp81375_led, flash);
	struct ocp81375 *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);
	ret = chip->suspended ? -EBUSY :
		regmap_write(chip->regmap, OCP81375_FLASH_CURRENT + led->id,
			     (brightness - flash->brightness.min) / flash->brightness.step);
	mutex_unlock(&chip->lock);
	return ret;
}

static int ocp81375_timeout_set(struct led_classdev_flash *flash, u32 timeout)
{
	/* This controller uses a fixed 600-ms hardware flash timeout. */
	return timeout == OCP81375_TIMEOUT_US ? 0 : -EINVAL;
}

static int ocp81375_strobe_set(struct led_classdev_flash *flash, bool state)
{
	struct ocp81375_led *led = container_of(flash, struct ocp81375_led, flash);
	struct ocp81375 *chip = led->chip;
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);
	if (chip->suspended) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = regmap_read(chip->regmap, OCP81375_ENABLE, &val);
	if (ret)
		goto unlock;
	if (!state) {
		/* Do not disturb another channel's torch or flash. */
		ret = 0;
		if ((val & OCP81375_MODE_MASK) == OCP81375_FLASH)
			ret = regmap_update_bits(chip->regmap, OCP81375_ENABLE, BIT(led->id), 0);
		goto unlock;
	}
	/* The timer and operating mode are shared; allow only one flash. */
	if ((val & OCP81375_CHANNELS) && (val & OCP81375_MODE_MASK)) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = ocp81375_read_faults(chip);
	if (ret)
		goto unlock;
	val = 0x19; /* 1-ms ramp, 600-ms hardware timeout. */
	ret = regmap_write(chip->regmap, OCP81375_TIMING, val);
	if (!ret)
		ret = regmap_write(chip->regmap, OCP81375_FLASH_CURRENT + led->id,
				   (flash->brightness.val - flash->brightness.min) /
				   flash->brightness.step);
	if (!ret)
		ret = regmap_write(chip->regmap, OCP81375_ENABLE,
				   OCP81375_FLASH | BIT(led->id));
unlock:
	mutex_unlock(&chip->lock);
	return ret;
}

static int ocp81375_strobe_get(struct led_classdev_flash *flash, bool *state)
{
	struct ocp81375_led *led = container_of(flash, struct ocp81375_led, flash);
	struct ocp81375 *chip = led->chip;
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);
	ret = regmap_read(chip->regmap, OCP81375_ENABLE, &val);
	if (!ret)
		*state = (val & OCP81375_MODE_MASK) == OCP81375_FLASH && (val & BIT(led->id));
	mutex_unlock(&chip->lock);
	return ret;
}

static int ocp81375_fault_get(struct led_classdev_flash *flash, u32 *fault)
{
	struct ocp81375_led *led = container_of(flash, struct ocp81375_led, flash);
	struct ocp81375 *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);
	ret = ocp81375_read_faults(chip);
	if (!ret) {
		*fault = chip->faults[led->id];
		chip->faults[led->id] = 0;
	}
	mutex_unlock(&chip->lock);
	return ret;
}

static const struct led_flash_ops ocp81375_flash_ops = {
	.flash_brightness_set = ocp81375_flash_set,
	.timeout_set = ocp81375_timeout_set,
	.strobe_set = ocp81375_strobe_set,
	.strobe_get = ocp81375_strobe_get,
	.fault_get = ocp81375_fault_get,
};

static void ocp81375_gpio_disable(void *arg)
{
	struct ocp81375 *chip = arg;

	gpiod_set_value_cansleep(chip->enable, 0);
}

static void ocp81375_disable(void *arg)
{
	struct ocp81375 *chip = arg;

	mutex_lock(&chip->lock);
	chip->suspended = true;
	regmap_write(chip->regmap, OCP81375_ENABLE, 0);
	gpiod_set_value_cansleep(chip->enable, 0);
	mutex_unlock(&chip->lock);
}

static void ocp81375_v4l2_release(void *arg)
{
	v4l2_flash_release(arg);
}

static int ocp81375_register_led(struct device *dev, struct ocp81375 *chip,
				 struct fwnode_handle *child)
{
	struct led_init_data init = { .fwnode = child };
	struct v4l2_flash_config config = {};
	struct ocp81375_led *led;
	struct led_classdev_flash *flash;
	struct v4l2_flash *v4l2;
	u32 id, torch, flash_ua, timeout, levels;
	int ret;

	ret = fwnode_property_read_u32(child, "reg", &id);
	if (ret || id >= ARRAY_SIZE(chip->leds) || test_bit(id, &chip->registered))
		return -EINVAL;
	ret = fwnode_property_read_u32(child, "led-max-microamp", &torch);
	if (ret)
		return ret;
	ret = fwnode_property_read_u32(child, "flash-max-microamp", &flash_ua);
	if (ret)
		return ret;
	ret = fwnode_property_read_u32(child, "flash-max-timeout-us", &timeout);
	if (ret)
		return ret;
	if (torch < OCP81375_TORCH_STEP_UA || flash_ua < OCP81375_FLASH_STEP_UA ||
	    timeout != OCP81375_TIMEOUT_US)
		return -EINVAL;

	led = &chip->leds[id];
	led->chip = chip;
	led->id = id;
	flash = &led->flash;
	flash->ops = &ocp81375_flash_ops;
	flash->led_cdev.flags = LED_DEV_CAP_FLASH | LED_CORE_SUSPENDRESUME;
	flash->led_cdev.brightness_set_blocking = ocp81375_torch_set;
	levels = torch / OCP81375_TORCH_STEP_UA;
	flash->led_cdev.max_brightness = min(levels, 128U);
	flash->brightness.min = OCP81375_FLASH_STEP_UA;
	flash->brightness.step = OCP81375_FLASH_STEP_UA;
	levels = min(flash_ua, 2000000U) / OCP81375_FLASH_STEP_UA;
	flash->brightness.max = levels * OCP81375_FLASH_STEP_UA;
	flash->brightness.val = flash->brightness.min;
	flash->timeout.min = OCP81375_TIMEOUT_US;
	flash->timeout.step = 1;
	flash->timeout.max = OCP81375_TIMEOUT_US;
	flash->timeout.val = flash->timeout.max;

	ret = devm_led_classdev_flash_register_ext(dev, flash, &init);
	if (ret)
		return ret;
	set_bit(id, &chip->registered);
	strscpy(config.dev_name, flash->led_cdev.dev->kobj.name, sizeof(config.dev_name));
	config.intensity.min = OCP81375_TORCH_STEP_UA;
	config.intensity.step = OCP81375_TORCH_STEP_UA;
	config.intensity.max = OCP81375_TORCH_STEP_UA +
		(flash->led_cdev.max_brightness - 1) * OCP81375_TORCH_STEP_UA;
	config.intensity.val = OCP81375_TORCH_STEP_UA;
	config.flash_faults = LED_FAULT_TIMEOUT | LED_FAULT_OVER_TEMPERATURE |
		LED_FAULT_SHORT_CIRCUIT;
	config.flash_faults |= LED_FAULT_OVER_VOLTAGE | LED_FAULT_INPUT_VOLTAGE;
	v4l2 = v4l2_flash_init(dev, child, flash, NULL, &config);
	if (IS_ERR(v4l2))
		return PTR_ERR(v4l2);
	if (v4l2)
		return devm_add_action_or_reset(dev, ocp81375_v4l2_release, v4l2);
	return 0;
}

static const struct regmap_config ocp81375_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = OCP81375_ID,
};

static int ocp81375_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ocp81375 *chip;
	unsigned int id;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	mutex_init(&chip->lock);
	chip->regmap = devm_regmap_init_i2c(client, &ocp81375_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap), "Cannot initialize regmap\n");
	chip->enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->enable))
		return dev_err_probe(dev, PTR_ERR(chip->enable), "Cannot get enable GPIO\n");
	usleep_range(2000, 3000);
	ret = devm_add_action_or_reset(dev, ocp81375_gpio_disable, chip);
	if (ret)
		return ret;
	ret = regmap_read(chip->regmap, OCP81375_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot read chip ID\n");
	if (id != 0x3a)
		return dev_err_probe(dev, -ENODEV, "Unexpected chip ID %#x\n", id);
	ret = devm_add_action_or_reset(dev, ocp81375_disable, chip);
	if (ret)
		return ret;
	ret = regmap_write(chip->regmap, OCP81375_ENABLE, 0);
	if (ret)
		return ret;
	ret = regmap_write(chip->regmap, OCP81375_IVFM, 0x59);
	/* LED1 brightness overrides LED2 after reset; configure them separately. */
	if (!ret)
		ret = regmap_update_bits(chip->regmap, OCP81375_FLASH_CURRENT, BIT(7), 0);
	if (!ret)
		ret = regmap_update_bits(chip->regmap, OCP81375_TORCH_CURRENT, BIT(7), 0);
	if (!ret)
		ret = regmap_write(chip->regmap, OCP81375_BOOST, 0x0b);
	if (ret)
		return ret;
	i2c_set_clientdata(client, chip);
	device_for_each_child_node_scoped(dev, child) {
		ret = ocp81375_register_led(dev, chip, child);
		if (ret)
			return dev_err_probe(dev, ret, "Cannot register LED\n");
	}
	return chip->registered ? 0 : -EINVAL;
}

static void ocp81375_shutdown(struct i2c_client *client)
{
	ocp81375_disable(i2c_get_clientdata(client));
}

static int ocp81375_suspend(struct device *dev)
{
	struct ocp81375 *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->lock);
	ret = regmap_write(chip->regmap, OCP81375_ENABLE, 0);
	if (!ret)
		chip->suspended = true;
	mutex_unlock(&chip->lock);
	return ret;
}

static int ocp81375_resume(struct device *dev)
{
	struct ocp81375 *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	chip->suspended = false;
	mutex_unlock(&chip->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ocp81375_pm_ops, ocp81375_suspend, ocp81375_resume);

static const struct of_device_id ocp81375_of_match[] = {
	{ .compatible = "ocs,ocp81375" },
	{}
};
MODULE_DEVICE_TABLE(of, ocp81375_of_match);

static const struct i2c_device_id ocp81375_ids[] = {
	{ "ocp81375", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ocp81375_ids);

static struct i2c_driver ocp81375_driver = {
	.driver = {
		.name = "ocp81375",
		.of_match_table = ocp81375_of_match,
		.pm = pm_sleep_ptr(&ocp81375_pm_ops),
	},
	.probe = ocp81375_probe,
	.shutdown = ocp81375_shutdown,
	.id_table = ocp81375_ids,
};
module_i2c_driver(ocp81375_driver);

MODULE_DESCRIPTION("OCP81375 flash LED driver");
MODULE_LICENSE("GPL");
