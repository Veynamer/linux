/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compat shims for building the 5.4-era Marlin3 WCN driver against 7.1.
 */
#ifndef _WCN_COMPAT_H
#define _WCN_COMPAT_H

#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/string.h>

/*
 * <linux/of_gpio.h> and of_get_named_gpio() were removed upstream in favour of
 * the descriptor API. The Marlin3 driver still uses the legacy integer GPIO API
 * (gpio_request()/gpio_direction_*()), so emulate the old parse-only helper:
 * resolve the GPIO number via the descriptor lookup, then release the desc so
 * the driver's later gpio_request() on the number still succeeds.
 */
#ifndef _LINUX_OF_GPIO_H
static inline int wcn_of_get_named_gpio(struct device_node *np,
					const char *propname, int index)
{
	struct gpio_desc *desc;
	char con_id[64];
	size_t len = strlen(propname);
	int gpio;

	/* strip a trailing "-gpios" / "-gpio" to form the con_id */
	if (len > 6 && !strcmp(propname + len - 6, "-gpios"))
		len -= 6;
	else if (len > 5 && !strcmp(propname + len - 5, "-gpio"))
		len -= 5;
	if (len >= sizeof(con_id))
		len = sizeof(con_id) - 1;
	memcpy(con_id, propname, len);
	con_id[len] = '\0';

	desc = fwnode_gpiod_get_index(of_fwnode_handle(np), con_id, index,
				      GPIOD_ASIS, propname);
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	gpio = desc_to_gpio(desc);
	gpiod_put(desc);
	return gpio;
}
#define of_get_named_gpio(np, propname, index) \
	wcn_of_get_named_gpio(np, propname, index)
#endif /* _LINUX_OF_GPIO_H */

#endif /* _WCN_COMPAT_H */
