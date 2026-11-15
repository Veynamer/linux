#ifndef __WCN_DBG_H__
#define __WCN_DBG_H__

#include <linux/device.h>
#include <linux/kernel.h>

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "wcn: " fmt

extern struct device *wcn_dev;

#define WCN_INFO(fmt, args...) do { \
	if (wcn_dev) \
		dev_info(wcn_dev, fmt, ## args); \
	else \
		pr_info(fmt, ## args); \
} while (0)

#define WCN_ERR(fmt, args...) do { \
	if (wcn_dev) \
		dev_err(wcn_dev, fmt, ## args); \
	else \
		pr_err(fmt, ## args); \
} while (0)

#define WCN_WARN(fmt, args...) do { \
	if (wcn_dev) \
		dev_warn(wcn_dev, fmt, ## args); \
	else \
		pr_warn(fmt, ## args); \
} while (0)

#define WCN_DBG(fmt, args...) do { \
	if (wcn_dev) \
		dev_dbg(wcn_dev, fmt, ## args); \
	else \
		pr_debug(fmt, ## args); \
} while (0)

#define WCN_ERR_RL(fmt, args...) do { \
	if (wcn_dev) \
		dev_err_ratelimited(wcn_dev, fmt, ## args); \
	else \
		printk_ratelimited(KERN_ERR pr_fmt(fmt), ## args); \
} while (0)

#define WCN_WARN_RL(fmt, args...) do { \
	if (wcn_dev) \
		dev_warn_ratelimited(wcn_dev, fmt, ## args); \
	else \
		printk_ratelimited(KERN_WARNING pr_fmt(fmt), ## args); \
} while (0)

#define MDBG_FUNC_ENTERY		WCN_DBG("ENTER")
#define MDBG_FUNC_EXIT			WCN_DBG("EXIT")

#endif
