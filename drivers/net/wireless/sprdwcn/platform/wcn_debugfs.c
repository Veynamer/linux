#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "bufring.h"
#include "loopcheck.h"
#include "sysfs.h"
#include "wcn_debugfs.h"
#include "wcn_log.h"
#include "wcn_misc.h"
#include "wcn_procfs.h"
#include "wcn_txrx.h"
#include "../include/wcn_dbg.h"

enum wcn_dbg_type {
	WCN_DBG_FW_LOG_LEVEL,
	WCN_DBG_ARMLOG,
	WCN_DBG_ATCMD,
	WCN_DBG_LOOPCHECK,
	WCN_DBG_SLP_INFO,
	WCN_DBG_RING_STATS,
};

static struct dentry *wcn_debugfs_root;

static int wcn_dbg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t wcn_dbg_read(struct file *file, char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	enum wcn_dbg_type type = (enum wcn_dbg_type)(unsigned long)file->private_data;
	char buf[512];
	int len;

	switch (type) {
	case WCN_DBG_FW_LOG_LEVEL:
		len = wcn_fw_loglevel_get(buf, sizeof(buf));
		break;
	case WCN_DBG_ARMLOG:
		len = wcn_fw_armlog_get(buf, sizeof(buf));
		break;
	case WCN_DBG_LOOPCHECK:
		len = scnprintf(buf, sizeof(buf), "status=0x%x fail_count=%d\n",
				loopcheck_status(), get_loopcheck_status());
		break;
	case WCN_DBG_SLP_INFO:
		if (wcn_slpinfo_get(sysfs_info.slpinfo_sys, NULL))
			len = scnprintf(buf, sizeof(buf), "Device busy\n");
		else
			len = scnprintf(buf, sizeof(buf), "OK\n");
		break;
	case WCN_DBG_RING_STATS: {
		struct mdbg_ring_stats stats;
		int open_count = mdbg_dev ? mdbg_dev->open_count : -1;

		mdbg_get_ring_stats(&stats);
		len = scnprintf(buf, sizeof(buf),
				"used=%ld\nwritten=%llu\nread=%llu\ndropped=%llu\nopen=%d\n",
				stats.used, stats.written, stats.read,
				stats.dropped, open_count);
		break;
	}
	default:
		return -EINVAL;
	}

	if (len < 0)
		return len;
	if (len && buf[len - 1] != '\n')
		len += scnprintf(buf + len, sizeof(buf) - len, "\n");

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t wcn_dbg_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	enum wcn_dbg_type type = (enum wcn_dbg_type)(unsigned long)file->private_data;
	char buf[256];
	char cmd[260];
	unsigned int val;
	size_t ret = count;
	int err;

	if (!count || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	count = strcspn(buf, "\r\n");
	buf[count] = '\0';

	switch (type) {
	case WCN_DBG_FW_LOG_LEVEL:
		err = kstrtouint(buf, 10, &val);
		if (err)
			return err;
		err = wcn_fw_loglevel_set(val);
		break;
	case WCN_DBG_ARMLOG:
		err = kstrtouint(buf, 10, &val);
		if (err)
			return err;
		err = wcn_fw_armlog_set(val);
		break;
	case WCN_DBG_ATCMD:
		scnprintf(cmd, sizeof(cmd), "%s\r\n", buf);
		err = wcn_send_atcmd(cmd, strlen(cmd), NULL, NULL);
		break;
	case WCN_DBG_LOOPCHECK:
		if (!strcmp(buf, "1") || !strcmp(buf, "on") ||
		    !strcmp(buf, "start")) {
			start_loopcheck();
			err = 0;
		} else if (!strcmp(buf, "0") || !strcmp(buf, "off") ||
			   !strcmp(buf, "stop")) {
			stop_loopcheck();
			err = 0;
		} else {
			err = -EINVAL;
		}
		break;
	case WCN_DBG_SLP_INFO:
		err = kstrtouint(buf, 10, &val);
		if (err)
			return err;
		if (val > WCN_SOURCE_BTWF)
			return -EINVAL;
		sysfs_info.slpinfo_sys = val;
		err = 0;
		break;
	case WCN_DBG_RING_STATS:
	default:
		err = -EINVAL;
		break;
	}

	return err ? err : ret;
}

static const struct file_operations wcn_dbg_fops = {
	.owner = THIS_MODULE,
	.open = wcn_dbg_open,
	.read = wcn_dbg_read,
	.write = wcn_dbg_write,
	.llseek = default_llseek,
};

static void wcn_debugfs_add(const char *name, umode_t mode,
			    enum wcn_dbg_type type)
{
	debugfs_create_file(name, mode, wcn_debugfs_root,
			    (void *)(unsigned long)type, &wcn_dbg_fops);
}

int wcn_debugfs_init(void)
{
	wcn_debugfs_root = debugfs_create_dir("wcn", NULL);
	if (IS_ERR(wcn_debugfs_root)) {
		int ret = PTR_ERR(wcn_debugfs_root);

		wcn_debugfs_root = NULL;
		return ret;
	}

	wcn_debugfs_add("fw_log_level", 0600, WCN_DBG_FW_LOG_LEVEL);
	wcn_debugfs_add("armlog", 0600, WCN_DBG_ARMLOG);
	wcn_debugfs_add("atcmd", 0200, WCN_DBG_ATCMD);
	wcn_debugfs_add("loopcheck", 0600, WCN_DBG_LOOPCHECK);
	wcn_debugfs_add("slp_info", 0600, WCN_DBG_SLP_INFO);
	wcn_debugfs_add("ring_stats", 0400, WCN_DBG_RING_STATS);

	return 0;
}

void wcn_debugfs_exit(void)
{
	debugfs_remove_recursive(wcn_debugfs_root);
	wcn_debugfs_root = NULL;
}
