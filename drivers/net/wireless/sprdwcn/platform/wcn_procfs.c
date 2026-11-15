/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * File:		wcn_procfs.c
 * Description:	Marlin Debug System main file. Module,device &
 * driver related defination.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the	1
 * GNU General Public License for more details.
 */

#include <linux/devcoredump.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <misc/marlin_platform.h>
#include <misc/wcn_bus.h>

#include "wcn_misc.h"
#include "wcn_glb.h"
#include "wcn_log.h"
#include "wcn_misc.h"
#include "wcn_procfs.h"
#include "wcn_txrx.h"
#include "mdbg_type.h"
#include "../include/wcn_dbg.h"
#include "../sdio/sdiohal.h"

static u32 g_loopcheck_switch;

struct mdbg_proc_entry {
	char *name;
	struct proc_dir_entry *entry;
	struct completion completed;
	wait_queue_head_t	rxwait;
	unsigned int	rcv_len;
	void *buf;
};

struct async_assert_t {
	struct work_struct assert_wq;
	/*device_suspend/device_resume device_lock()*/
	atomic_t dev_lock;
	atomic_t flag;
	enum wcn_source_type async_assert_type;
	char async_assert_str[128];
};

struct mdbg_proc_t {
	char *dir_name;
	struct proc_dir_entry		*procdir;
	struct mdbg_proc_entry		assert;
	struct mdbg_proc_entry		loopcheck;
	struct mdbg_proc_entry		at_cmd;
	struct mutex		mutex;
	char write_buf[MDBG_WRITE_SIZE];
	int fail_count;
	int assert_notify_flag;
	bool loopcheck_flag;

	/*see device_lock*/
	struct async_assert_t async_assert;
};

static struct mdbg_proc_t *mdbg_proc;

void mdbg_device_lock_notify(void)
{
	WARN_ON(atomic_read(&mdbg_proc->async_assert.dev_lock));
	atomic_set(&mdbg_proc->async_assert.dev_lock, 1);
}
EXPORT_SYMBOL_GPL(mdbg_device_lock_notify);

void mdbg_device_unlock_notify(void)
{
	WARN_ON(!atomic_read(&mdbg_proc->async_assert.dev_lock));
	atomic_set(&mdbg_proc->async_assert.dev_lock, 0);
}
EXPORT_SYMBOL_GPL(mdbg_device_unlock_notify);

static void wcn_reset_process(void)
{
	WCN_INFO("%s reset begin\n", __func__);
	wcn_reset_cp2();
	mdbg_proc->assert_notify_flag = 0;
	sprdwcn_bus_set_carddump_status(false);
	/*notify slogmodem to restore the state of saving dump*/
	wcn_notify_fw_error(WCN_SOURCE_CP2_ALIVE, "cp2 alive");
	WCN_INFO("%s reset end\n", __func__);
}

bool wcn_is_assert(void)
{
	return mutex_is_locked(&mdbg_proc->mutex);
}
EXPORT_SYMBOL_GPL(wcn_is_assert);

static void wcn_assert_coredump(const char *str)
{
	size_t len;
	char *dump;

	if (!wcn_dev || !str)
		return;

	len = strlen(str) + 1;

	dump = vzalloc(len);
	if (!dump)
		return;

	memcpy(dump, str, len);
	dev_coredumpv(wcn_dev, dump, len, GFP_KERNEL);
}

static void __wcn_assert_interface(enum wcn_source_type type, char *str)
{
	WCN_INFO("wcn_assert_interface\n");
	WCN_ERR("wcn_source_type:%d\n", type);
	WCN_ERR("fw assert:%s\n", str);
	wcn_assert_coredump(str);

	if (!mutex_trylock(&mdbg_proc->mutex)) {
		WCN_ERR("fw assert hanppend already\n");
		return;
	}
	if (!marlin_get_power()) {
		WCN_INFO("no modules open\n");
		goto out;
	}

	if (!mdbg_proc->assert_notify_flag) {
		wcn_notify_fw_error(type, str);
		mdbg_proc->assert_notify_flag = 1;
	}

	sprdwcn_bus_debug_point_show();
	stop_loopcheck();
	wcnlog_clear_log();
	wcn_reset_process();
out:
	mutex_unlock(&mdbg_proc->mutex);
}
EXPORT_SYMBOL_GPL(wcn_assert_interface);

void wcn_assert_interface(enum wcn_source_type type, char *str)
{
	if (in_interrupt() || unlikely(atomic_read(&mdbg_proc->async_assert.flag)) ||
		unlikely(atomic_read(&mdbg_proc->async_assert.dev_lock))) {
		/* dump: dev_lock(resume), cause deadlock in system resume. see mdbg_dump_data() */
		WCN_INFO("Async assert! %ld,%d,%d\n", in_interrupt(),
			atomic_read(&mdbg_proc->async_assert.flag),
			atomic_read(&mdbg_proc->async_assert.dev_lock));
		mdbg_proc->async_assert.async_assert_type = type;
		strscpy(mdbg_proc->async_assert.async_assert_str, str);
		schedule_work(&mdbg_proc->async_assert.assert_wq);
	} else
		__wcn_assert_interface(type, str);
}

void wcn_assert_interface_async(enum wcn_source_type type, char *str)
{
	atomic_set(&mdbg_proc->async_assert.flag, 1);
	wcn_assert_interface(type, str);
	atomic_set(&mdbg_proc->async_assert.flag, 0);
}
EXPORT_SYMBOL_GPL(wcn_assert_interface_async);

static void wcn_assert_wq(struct work_struct *work)
{
	struct async_assert_t *as = container_of(work, struct async_assert_t, assert_wq);

	if (IS_ERR_OR_NULL(as))
		return;

	__wcn_assert_interface(as->async_assert_type, as->async_assert_str);
}

void mdbg_assert_interface(char *str)
{
	wcn_assert_interface(WCN_SOURCE_BTWF, str);
}
EXPORT_SYMBOL_GPL(mdbg_assert_interface);

int mdbg_assert_read(int channel, struct mbuf_t *head,
		     struct mbuf_t *tail, int num)
{
	struct bus_puh_t *puh = NULL;
	unsigned int pub_head_rsv = SDIOHAL_PUB_HEAD_RSV;

	puh = (struct bus_puh_t *)head->buf;
	if (puh->len > MDBG_ASSERT_SIZE) {
		WCN_ERR("assert data len:%d,beyond max read:%d",
			puh->len, MDBG_ASSERT_SIZE);
		sprdwcn_bus_push_list(channel, head, tail, num);
		return -1;
	}

	wcn_assert_interface(WCN_SOURCE_BTWF, head->buf + pub_head_rsv);
	sprdwcn_bus_push_list(channel, head, tail, num);

	return 0;
}
EXPORT_SYMBOL_GPL(mdbg_assert_read);

int mdbg_loopcheck_read(int channel, struct mbuf_t *head,
			struct mbuf_t *tail, int num)
{
	struct bus_puh_t *puh = NULL;
	unsigned int pub_head_rsv = SDIOHAL_PUB_HEAD_RSV;

	puh = (struct bus_puh_t *)head->buf;
	if (puh->len > MDBG_LOOPCHECK_SIZE) {
		WCN_ERR("The loopcheck data len:%d,beyond max read:%d",
			puh->len, MDBG_LOOPCHECK_SIZE);
		sprdwcn_bus_push_list(channel, head, tail, num);
		return -1;
	}

	memset(mdbg_proc->loopcheck.buf, 0, MDBG_LOOPCHECK_SIZE);
	memcpy(mdbg_proc->loopcheck.buf, head->buf + pub_head_rsv, puh->len);
	mdbg_proc->loopcheck.rcv_len = puh->len;
	WCN_DBG("rx:%s\n", (char *)(mdbg_proc->loopcheck.buf));
	mdbg_proc->fail_count = 0;
	complete(&mdbg_proc->loopcheck.completed);
	complete_kernel_loopcheck();
	sprdwcn_bus_push_list(channel, head, tail, num);

	return 0;
}
EXPORT_SYMBOL_GPL(mdbg_loopcheck_read);

int mdbg_at_cmd_read(int channel, struct mbuf_t *head,
		     struct mbuf_t *tail, int num)
{
	struct bus_puh_t *puh = NULL;
	unsigned int pub_head_rsv = SDIOHAL_PUB_HEAD_RSV;

	puh = (struct bus_puh_t *)head->buf;
	if (puh->len > MDBG_AT_CMD_SIZE) {
		WCN_ERR("The at cmd data len:%d,beyond max read:%d",
			puh->len, MDBG_AT_CMD_SIZE);
		sprdwcn_bus_push_list(channel, head, tail, num);
		return -1;
	}

	memset(mdbg_proc->at_cmd.buf, 0, MDBG_AT_CMD_SIZE);
	memcpy(mdbg_proc->at_cmd.buf, head->buf + pub_head_rsv, puh->len);
	mdbg_proc->at_cmd.rcv_len = puh->len;
	WCN_DBG("at cmd read:%s\n",
		(char *)(mdbg_proc->at_cmd.buf));
	complete(&mdbg_proc->at_cmd.completed);
	notify_at_cmd_finish(mdbg_proc->at_cmd.buf, mdbg_proc->at_cmd.rcv_len);
	sprdwcn_bus_push_list(channel, head, tail, num);

	return 0;
}
EXPORT_SYMBOL_GPL(mdbg_at_cmd_read);

static int mdbg_proc_open(struct inode *inode, struct file *filp)
{
	struct mdbg_proc_entry *entry =
		(struct mdbg_proc_entry *)pde_data(inode);
	filp->private_data = entry;

	return 0;
}

static int mdbg_proc_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t mdbg_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct mdbg_proc_entry *entry =
		(struct mdbg_proc_entry *)filp->private_data;
	char *type = entry->name;
	int timeout = -1;
	int len = 0;
	int ret;

	if (filp->f_flags & O_NONBLOCK)
		timeout = 0;

	if (strcmp(type, "assert") == 0) {
		if (timeout < 0) {
			while (1) {
				ret = wait_for_completion_timeout(
						&mdbg_proc->assert.completed,
						msecs_to_jiffies(1000));
				if (ret != -ERESTARTSYS)
					break;
			}
		}

		if (copy_to_user((void __user *)buf,
				mdbg_proc->assert.buf,
				min(count, (size_t)MDBG_ASSERT_SIZE)))
			WCN_ERR("Read assert info error\n");
		len = mdbg_proc->assert.rcv_len;
		mdbg_proc->assert.rcv_len = 0;
		memset(mdbg_proc->assert.buf, 0, MDBG_ASSERT_SIZE);
	}

	if (strcmp(type, "loopcheck") == 0) {
		if (unlikely(g_loopcheck_switch != 0)) {
			if (marlin_get_module_status() == 1) {
				WCN_DBG("fake loopcheck\n");
				if (copy_to_user((void __user *)buf,
							"loopcheck_ack", 13))
					WCN_ERR("fake loopcheck reply error\n");
				len = 13;
			} else {
				if (copy_to_user((void __user *)buf,
							"poweroff", 8))
					WCN_ERR("read loopcheck error\n");
				len = 8;
				WCN_DBG("loopcheck poweroff\n");
			}
			return len;
		}

		if (timeout < 0) {
			while (1) {
				ret = wait_for_completion_timeout(
						&mdbg_proc->loopcheck.completed,
						msecs_to_jiffies(1000));
				if (ret != -ERESTARTSYS)
					break;
			}
		}
		if (marlin_get_module_status() == 1) {
			/* tell wcnd str"loopcheck_ack" to start loopcheck */
			if (mdbg_proc->loopcheck_flag) {
				if (copy_to_user((void __user *)buf,
					"loopcheck_ack", 13))
					return -EFAULT;
				loopcheck_ready_clear();
				WCN_DBG("CP power on first time\n");
				len = 13;
			} else if (mdbg_rx_count_change()) {
			/* fix the error(ack slow),use rx count to verify CP */
				WCN_DBG("CP run well with rx_cnt change\n");
				if (copy_to_user((void __user *)buf,
							"loopcheck_ack", 13))
					return -EFAULT;
				len = 13;
			} else {
				if (copy_to_user((void __user *)buf,
					mdbg_proc->loopcheck.buf, min(count,
						(size_t)MDBG_LOOPCHECK_SIZE)))
					return -EFAULT;
				len = mdbg_proc->loopcheck.rcv_len;
				if (strncmp(mdbg_proc->loopcheck.buf,
					"loopcheck_ack", 13) != 0)
					mdbg_proc->fail_count++;
				WCN_DBG("loopcheck status:%d\n",
					mdbg_proc->fail_count);
			}
		} else {
			if (copy_to_user((void __user *)buf, "poweroff", 8))
				return -EFAULT;
			len = 8;
			WCN_DBG("mdbg loopcheck poweroff\n");
		}
		memset(mdbg_proc->loopcheck.buf, 0, MDBG_LOOPCHECK_SIZE);
		mdbg_proc->loopcheck.rcv_len = 0;
	}

	if (strcmp(type, "at_cmd") == 0) {
		if (timeout < 0) {
			while (1) {
				ret = wait_for_completion_timeout(
						&mdbg_proc->at_cmd.completed,
						msecs_to_jiffies(1000));
				if (ret != -ERESTARTSYS)
					break;
			}
		}

		if (copy_to_user((void __user *)buf,
					mdbg_proc->at_cmd.buf,
					min(count, (size_t)MDBG_AT_CMD_SIZE)))
			WCN_ERR("Read at cmd ack info error\n");

		len = mdbg_proc->at_cmd.rcv_len;
		mdbg_proc->at_cmd.rcv_len = 0;
		memset(mdbg_proc->at_cmd.buf, 0, MDBG_AT_CMD_SIZE);
	}
	return len;
}

/**************************************************
 * Marlin3 crash
 *   |-user:       rebootmarlin
 *   `-userdebug : dumpmem for btwifi
 *
 *  rebootmarlin: reset gpio enable
 *****************************************************/
static ssize_t mdbg_proc_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char x;
#ifdef MDBG_PROC_CMD_DEBUG
	char *tempbuf = NULL;
	int ret = -1, i;
#endif
	if (count < 1)
		return -EINVAL;
	if (copy_from_user(&x, buf, 1))
		return -EFAULT;
#ifdef MDBG_PROC_CMD_DEBUG
/* for test boot */

	if (x == 'B') {
		WCN_INFO("wsh proc write =%c\n", x);
		tempbuf = kzalloc(10, GFP_KERNEL);
		memset(tempbuf, 0, 10);
		ret = sprdwcn_bus_direct_read(get_cp_start_addr(), tempbuf, 10);
		if (ret < 0)
			WCN_ERR("wsh debug get_cp_start_addr error:%d\n", ret);
		WCN_INFO("\nwsh debug get_cp_start_addr(10) :\n");
		for (i = 0; i < 10; i++)
			WCN_INFO("0x%x\n", tempbuf[i]);

		memset(tempbuf, 0, 10);
		ret = sprdwcn_bus_reg_read(get_cp_reset_reg(), tempbuf, 4);
		if (ret < 0)
			WCN_ERR("wsh debug get_cp_reset_reg error:%d\n", ret);
		WCN_INFO("\nwsh debug get_cp_reset_reg(4) :\n");
		for (i = 0; i < 4; i++)
			WCN_INFO(":0x%x\n", tempbuf[i]);

		kfree(tempbuf);
	}

#ifdef MDBG_PROC_CMD_DEBUG
/* for test cdev */
	if (x == '1') {
		WCN_INFO("wsh proc write =%c\n", x);
		WCN_INFO("start char release test\n");
		log_cdev_exit();
	}
	if (x == '2') {
		WCN_INFO("wsh proc write =%c\n", x);
		WCN_INFO("start char register test\n");
		log_cdev_init();
	}
#endif
/* for test power on/off frequently */
#ifdef MDBG_PROC_CMD_DEBUG
	if (x == '0')
		start_marlin(MARLIN_BLUETOOTH);
	if (x == '2')
		start_marlin(MARLIN_WIFI);
	if (x == '3')
		start_marlin(MARLIN_MDBG);

	if (x == '5')
		stop_marlin(MARLIN_BLUETOOTH);
	if (x == '7')
		stop_marlin(MARLIN_WIFI);
	if (x == '8')
		stop_marlin(MARLIN_MDBG);
	if (x == 'x')
		open_power_ctl();
#endif
	if (x == 'V')
		inform_cp_wifi_download();
	if (x == 'U')
		sprdwcn_bus_aon_writeb(0X1B0, 0X10);
	if (x == 'T')
		mem_pd_mgr(MARLIN_WIFI, 0X1);
	if (x == 'Q')
		mem_pd_save_bin();
	if (x == 'N')
		start_marlin(MARLIN_WIFI);
	if (x == 'R')
		stop_marlin(MARLIN_WIFI);

#endif
	if (count > MDBG_WRITE_SIZE) {
		WCN_ERR("%s count > MDBG_WRITE_SIZE\n", __func__);
		return -ENOMEM;
	}
	memset(mdbg_proc->write_buf, 0, MDBG_WRITE_SIZE);

	if (copy_from_user(mdbg_proc->write_buf, buf, count))
		return -EFAULT;

	WCN_DBG("mdbg_proc->write_buf:%s\n", mdbg_proc->write_buf);

	if (strncmp(mdbg_proc->write_buf, "startwcn", 8) == 0) {
		if (start_marlin(MARLIN_MDBG)) {
			WCN_ERR("%s power on failed\n", __func__);
			return -EIO;
		}
		return count;
	}

	if (strncmp(mdbg_proc->write_buf, "stopwcn", 7) == 0) {
		if (stop_marlin(MARLIN_MDBG)) {
			WCN_ERR("%s power off failed\n", __func__);
			return -EIO;
		}
		return count;
	}
	if (strncmp(mdbg_proc->write_buf, "loopcheckoff",
		strlen("loopcheckoff")) == 0) {
		stop_loopcheck();
		g_loopcheck_switch = 1;
		WCN_INFO("loopcheck debug:switch(%d)\n",
				g_loopcheck_switch);
		return count;
	}
	if (strncmp(mdbg_proc->write_buf, "loopcheckon",
		strlen("loopcheckon")) == 0) {
		g_loopcheck_switch = 0;
		start_loopcheck();
		WCN_INFO("loopcheck debug:switch(%d)\n",
				g_loopcheck_switch);
		return count;
	}

	if (strncmp(mdbg_proc->write_buf, "poweroff_wcn", 12) == 0) {
		marlin_power_off(MARLIN_ALL);
		return count;
	}

	if (strncmp(mdbg_proc->write_buf, "rebootmarlin", 12) == 0) {
		flag_reset = 1;
		WCN_INFO("marlin need reset\n");
		WCN_INFO("fail_count is value %d\n", mdbg_proc->fail_count);
		WCN_INFO("fail_reset is value %d\n", flag_reset);
		mdbg_proc->fail_count = 0;
		marlin_set_download_status(0);
		sprdwcn_bus_set_carddump_status(false);
		marlin_chip_en(false, true);
		if (marlin_reset_func != NULL)
			marlin_reset_func(marlin_callback_para);
		return count;
	}
	if (strncmp(mdbg_proc->write_buf, "at+getchipversion", 17) == 0) {
		WCN_INFO("wcn get chip version: %s\n", wcn_get_chip_name());
		return count;
	}

	/* loopcheck add kernel time ms/1000 */
	if (strncmp(mdbg_proc->write_buf, "at+loopcheck", 12) == 0) {
		/* struct timespec now; */
		unsigned long long loopcheck_tx_ns = local_clock();
		unsigned long long marlin_boot_t = marlin_bootup_time_get();

		MARLIN_64B_NS_TO_32B_MS(loopcheck_tx_ns);
		MARLIN_64B_NS_TO_32B_MS(marlin_boot_t);

		sprintf(mdbg_proc->write_buf, "at+loopcheck=%llu,%llu\r",
			loopcheck_tx_ns, marlin_boot_t);
		/* Be care the count value changed here before send to CP2 */
		count = strlen(mdbg_proc->write_buf);
		WCN_DBG("%s, count = %d", mdbg_proc->write_buf, (int)count);
	}

	mdbg_send(mdbg_proc->write_buf, count, MDBG_SUBTYPE_AT);

	return count;
}

static unsigned int mdbg_proc_poll(struct file *filp, poll_table *wait)
{
	struct mdbg_proc_entry *entry =
		(struct mdbg_proc_entry *)filp->private_data;
	char *type = entry->name;
	unsigned int mask = 0;

	if (strcmp(type, "assert") == 0) {
		poll_wait(filp, &mdbg_proc->assert.rxwait, wait);
		if (mdbg_proc->assert.rcv_len > 0)
			mask |= POLLIN | POLLRDNORM;
	}

	if (strcmp(type, "loopcheck") == 0) {
		poll_wait(filp, &mdbg_proc->loopcheck.rxwait, wait);
		WCN_DBG("loopcheck:power_state_changed:%d\n",
			wcn_get_module_status_changed());
		if (wcn_get_module_status_changed()) {
			wcn_set_module_status_changed(false);
			mask |= POLLIN | POLLRDNORM;
		}
	}

	return mask;
}

static const struct proc_ops mdbg_proc_fops = {
	.proc_open	= mdbg_proc_open,
	.proc_release	= mdbg_proc_release,
	.proc_read	= mdbg_proc_read,
	.proc_write	= mdbg_proc_write,
	.proc_poll	= mdbg_proc_poll,
};

int mdbg_memory_alloc(void)
{
	mdbg_proc->assert.buf =  kzalloc(MDBG_ASSERT_SIZE, GFP_KERNEL);
	if (!mdbg_proc->assert.buf)
		return -ENOMEM;

	mdbg_proc->loopcheck.buf =  kzalloc(MDBG_LOOPCHECK_SIZE, GFP_KERNEL);
	if (!mdbg_proc->loopcheck.buf) {
		kfree(mdbg_proc->assert.buf);
		return -ENOMEM;
	}
	mdbg_proc->at_cmd.buf =  kzalloc(MDBG_AT_CMD_SIZE, GFP_KERNEL);
	if (!mdbg_proc->at_cmd.buf) {
		kfree(mdbg_proc->assert.buf);
		kfree(mdbg_proc->loopcheck.buf);
		return -ENOMEM;
	}

	return 0;
}

struct mchn_ops_t mdbg_proc_ops[MDBG_ASSERT_RX_OPS + 1] = {
	{
		.channel = WCN_AT_TX,
		.inout = WCNBUS_TX,
		.pool_size = 5,
		.pop_link = mdbg_tx_cb,
	},
	{
		.channel = WCN_LOOPCHECK_RX,
		.inout = WCNBUS_RX,
		.pool_size = 1,
		.pop_link = mdbg_loopcheck_read,
	},
	{
		.channel = WCN_AT_RX,
		.inout = WCNBUS_RX,
		.pool_size = 1,
		.pop_link = mdbg_at_cmd_read,
	},
	{
		.channel = WCN_ASSERT_RX,
		.inout = WCNBUS_RX,
		.pool_size = 1,
		.pop_link = mdbg_assert_read,
	},
};

struct mchn_ops_t *get_mdbg_proc_op(void)
{
	return mdbg_proc_ops;
}

void mdbg_fs_channel_destroy(void)
{
	int i;
	struct mchn_ops_t *p_mdbg_proc_ops = get_mdbg_proc_op();

	for (i = 0; i <= MDBG_ASSERT_RX_OPS; i++)
		sprdwcn_bus_chn_deinit(p_mdbg_proc_ops + i);
}

void mdbg_fs_channel_init(void)
{
	int i;
	struct mchn_ops_t *p_mdbg_proc_ops = get_mdbg_proc_op();

	for (i = 0; i <= MDBG_ASSERT_RX_OPS; i++)
		sprdwcn_bus_chn_init(p_mdbg_proc_ops + i);
}

static  void mdbg_memory_free(void)
{
	kfree(mdbg_proc->assert.buf);
	mdbg_proc->assert.buf = NULL;

	kfree(mdbg_proc->loopcheck.buf);
	mdbg_proc->loopcheck.buf = NULL;

	kfree(mdbg_proc->at_cmd.buf);
	mdbg_proc->at_cmd.buf = NULL;
}

int proc_fs_init(void)
{
	mdbg_proc = kzalloc(sizeof(struct mdbg_proc_t), GFP_KERNEL);
	if (!mdbg_proc)
		return -ENOMEM;

	mdbg_proc->dir_name = "mdbg";
	mdbg_proc->procdir = proc_mkdir(mdbg_proc->dir_name, NULL);

	mdbg_proc->assert.name = "assert";
	mdbg_proc->assert.entry = proc_create_data(mdbg_proc->assert.name,
						0600,
						mdbg_proc->procdir,
						&mdbg_proc_fops,
						&(mdbg_proc->assert));

	mdbg_proc->loopcheck.name = "loopcheck";
	mdbg_proc->loopcheck.entry = proc_create_data(mdbg_proc->loopcheck.name,
						0600,
						mdbg_proc->procdir,
						&mdbg_proc_fops,
						&(mdbg_proc->loopcheck));

	mdbg_proc->at_cmd.name = "at_cmd";
	mdbg_proc->at_cmd.entry = proc_create_data(mdbg_proc->at_cmd.name,
						0600,
						mdbg_proc->procdir,
						&mdbg_proc_fops,
						&(mdbg_proc->at_cmd));

	mdbg_fs_channel_init();

	init_completion(&mdbg_proc->assert.completed);
	init_completion(&mdbg_proc->loopcheck.completed);
	init_completion(&mdbg_proc->at_cmd.completed);
	init_waitqueue_head(&mdbg_proc->assert.rxwait);
	init_waitqueue_head(&mdbg_proc->loopcheck.rxwait);
	mutex_init(&mdbg_proc->mutex);
	INIT_WORK(&mdbg_proc->async_assert.assert_wq, wcn_assert_wq);

	if (mdbg_memory_alloc() < 0) {
		kfree(mdbg_proc);
		return -ENOMEM;
	}

	return 0;
}

void proc_fs_exit(void)
{
	mdbg_memory_free();
	mutex_destroy(&mdbg_proc->mutex);
	mdbg_fs_channel_destroy();
	remove_proc_entry(mdbg_proc->assert.name, mdbg_proc->procdir);
	remove_proc_entry(mdbg_proc->loopcheck.name, mdbg_proc->procdir);
	remove_proc_entry(mdbg_proc->at_cmd.name, mdbg_proc->procdir);
	remove_proc_entry(mdbg_proc->dir_name, NULL);

	kfree(mdbg_proc);
	mdbg_proc = NULL;
}

int get_loopcheck_status(void)
{
	return mdbg_proc->fail_count;
}

void wakeup_loopcheck_int(void)
{
	wake_up_interruptible(&mdbg_proc->loopcheck.rxwait);
}

void loopcheck_ready_clear(void)
{
	mdbg_proc->loopcheck_flag = false;
}

void loopcheck_ready_set(void)
{
	mdbg_proc->loopcheck_flag = true;
}
