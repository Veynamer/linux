/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <misc/wcn_bus.h>

#include "bufring.h"
#include "wcn_glb.h"
#include "wcn_procfs.h"
#include "mdbg_type.h"
#include "../include/wcn_dbg.h"

static struct ring_device *ring_dev;
static unsigned long long rx_count;
static unsigned long long rx_count_last;
static atomic_t ring_reg_flag;

static int mdbg_log_read(int channel, struct mbuf_t *head,
			 struct mbuf_t *tail, int num)
{
	struct ring_rx_data *rx;

	if (ring_dev) {
		mutex_lock(&ring_dev->mdbg_read_mutex);
		rx = kmalloc(sizeof(*rx), GFP_KERNEL);
		if (!rx) {
			WCN_ERR("mdbg ring low memory\n");
			mutex_unlock(&ring_dev->mdbg_read_mutex);
			sprdwcn_bus_push_list(channel, head, tail, num);
			return 0;
		}
		mutex_unlock(&ring_dev->mdbg_read_mutex);
		spin_lock_bh(&ring_dev->rw_lock);
		rx->channel = channel;
		rx->head = head;
		rx->tail = tail;
		rx->num = num;
		list_add_tail(&rx->entry, &ring_dev->rx_head);
		spin_unlock_bh(&ring_dev->rw_lock);
		schedule_work(&ring_dev->rx_task);
	}

	return 0;
}

static struct mchn_ops_t mdbg_ringc_ops = {
	.channel = WCN_RING_RX,
	.inout = WCNBUS_RX,
	.pool_size = 1,
	.pop_link = mdbg_log_read,
};

static struct mchn_ops_t *get_mdbg_ringc_ops(void)
{
	return &mdbg_ringc_ops;
}

bool mdbg_rx_count_change(void)
{
	rx_count = sprdwcn_bus_get_rx_total_cnt();

	WCN_DBG("rx_count:0x%llx rx_count_last:0x%llx\n",
		rx_count, rx_count_last);

	if ((rx_count == 0) && (rx_count_last == 0)) {
		return true;
	} else if (rx_count != rx_count_last) {
		rx_count_last = rx_count;
		return true;
	} else {
		return false;
	}
}

int mdbg_read_release(unsigned int fifo_id)
{
	return 0;
}

long mdbg_content_len(void)
{
	if (unlikely(!ring_dev))
		return 0;

	return mdbg_ring_readable_len(ring_dev->ring);
}

static long int mdbg_comm_write(char *buf,
				size_t len, unsigned int subtype)
{
	unsigned char *send_buf = NULL;
	char *str = NULL;
	struct mbuf_t *head = NULL;
	struct mbuf_t *tail = NULL;
	int num = 1;
	size_t rsvlen = SDIOHAL_PUB_HEAD_RSV;
	struct mchn_ops_t *p_mdbg_proc_ops = get_mdbg_proc_op();

	if (unlikely(marlin_get_module_status() != true)) {
		WCN_WARN("WCN module have not open\n");
		return -EIO;
	}

	send_buf = kzalloc(len + rsvlen + 1, GFP_KERNEL);
	if (!send_buf)
		return -ENOMEM;
	memcpy(send_buf + rsvlen, buf, len);

	str = strstr(send_buf + rsvlen, SMP_HEAD_STR);
	if (!str)
		if (len > ARMLOG_HEAD - 1)
			str = strstr(send_buf + rsvlen + ARMLOG_HEAD,
				     SMP_HEAD_STR);

	if (str) {
		int ret;

		/* for arm log to pc */
		WCN_INFO("smp len:%lu,str:%s\n", len, str);
		str[sizeof(SMP_HEAD_STR)] = 0;
		ret = kstrtol(&str[sizeof(SMP_HEAD_STR) - 1], 10,
							&ring_dev->flag_smp);
		WCN_INFO("smp ret:%d, flag_smp:%ld\n", ret,
			 ring_dev->flag_smp);
		kfree(send_buf);
	} else {
		if (!sprdwcn_bus_list_alloc(
				p_mdbg_proc_ops[MDBG_AT_TX_OPS].channel,
				&head, &tail, &num)) {
			head->buf = send_buf;
			head->len = len;
			head->next = NULL;
			sprdwcn_bus_push_list(
				p_mdbg_proc_ops[MDBG_AT_TX_OPS].channel,
				head, tail, num);
		}
	}

	return len;
}

static void mdbg_ring_rx_task(struct work_struct *work)
{
	struct ring_rx_data *rx = NULL;
	struct mdbg_ring_t *ring = NULL;
	struct mbuf_t *mbuf_node;
	int i;
	struct bus_puh_t *puh = NULL;
	struct mchn_ops_t *p_mdbg_ringc_ops = get_mdbg_ringc_ops();
	unsigned int pub_head_rsv = SDIOHAL_PUB_HEAD_RSV;

	if (unlikely(!ring_dev)) {
		WCN_ERR("ring_dev is NULL\n");
		return;
	}

	spin_lock_bh(&ring_dev->rw_lock);
	rx = list_first_entry_or_null(&ring_dev->rx_head,
				      struct ring_rx_data, entry);
	if (rx) {
		list_del(&rx->entry);
	} else {
		WCN_WARN("tasklet something err\n");
		spin_unlock_bh(&ring_dev->rw_lock);
		return;
	}
	if (!list_empty(&ring_dev->rx_head))
		schedule_work(&ring_dev->rx_task);
	ring = ring_dev->ring;
	spin_unlock_bh(&ring_dev->rw_lock);

	for (i = 0, mbuf_node = rx->head; i < rx->num; i++,
		mbuf_node = mbuf_node->next) {
		rx->addr = mbuf_node->buf + pub_head_rsv;
		puh = (struct bus_puh_t *)mbuf_node->buf;
		mdbg_ring_write(ring, rx->addr, puh->len);
	}
	sprdwcn_bus_push_list(p_mdbg_ringc_ops->channel,
			      rx->head, rx->tail, rx->num);
	wake_up_log_wait();
	kfree(rx);
}

long int mdbg_send(char *buf, size_t len, unsigned int subtype)
{
	long int sent_size = 0;

	WCN_DEBUG("BYTE MODE");
	__pm_stay_awake(ring_dev->rw_wake_lock);
	sent_size = mdbg_comm_write(buf, len, subtype);
	__pm_relax(ring_dev->rw_wake_lock);

	return sent_size;
}
EXPORT_SYMBOL_GPL(mdbg_send);

long int mdbg_receive(void *buf, int len)
{
	return mdbg_ring_read(ring_dev->ring, buf, len);
}

int mdbg_tx_cb(int channel, struct mbuf_t *head,
	       struct mbuf_t *tail, int num)
{
	struct mbuf_t *mbuf_node = head;
	int i;

	for (i = 0; i < num; i++, mbuf_node = mbuf_node->next) {
		kfree(mbuf_node->buf);
		mbuf_node->buf = NULL;
	}

	WCN_DBG("%s, chn:%d\n", __func__, channel);
	sprdwcn_bus_list_free(channel, head, tail, num);

	return 0;
}

void mdbg_pt_ring_reg(void)
{
	struct mchn_ops_t *p_mdbg_ringc_ops = get_mdbg_ringc_ops();

	WCN_INFO("%s\n", __func__);
	atomic_set(&ring_reg_flag, 0x1);
	sprdwcn_bus_chn_init(p_mdbg_ringc_ops);
}

void mdbg_pt_ring_unreg(void)
{
	struct mchn_ops_t *p_mdbg_ringc_ops = get_mdbg_ringc_ops();

	WCN_INFO("%s\n", __func__);
	atomic_set(&ring_reg_flag, 0x0);
	sprdwcn_bus_chn_deinit(p_mdbg_ringc_ops);
}

int mdbg_ring_init(void)
{
	int err = 0;
	unsigned int mdbg_rx_ring_size;
	struct wcn_match_data *g_match_config = get_wcn_match_config();

	if (g_match_config && g_match_config->unisoc_wcn_m3)
		mdbg_rx_ring_size = M3_MDBG_RX_RING_SIZE;
	else if (g_match_config && g_match_config->unisoc_wcn_m3lite)
		mdbg_rx_ring_size = M3L_MDBG_RX_RING_SIZE;
	else
		mdbg_rx_ring_size = MDBG_RX_RING_SIZE;

	ring_dev = kmalloc(sizeof(struct ring_device), GFP_KERNEL);
	if (!ring_dev)
		return -ENOMEM;
	ring_dev->ring = mdbg_ring_alloc(mdbg_rx_ring_size);
	if (!(ring_dev->ring)) {
		WCN_ERR("Ring malloc error.");
		kfree(ring_dev);
		return -MDBG_ERR_MALLOC_FAIL;
	}

	ring_dev->rw_wake_lock = wakeup_source_register(NULL, "mdbg_wake_lock");
	spin_lock_init(&ring_dev->rw_lock);
	mutex_init(&ring_dev->mdbg_read_mutex);
	INIT_LIST_HEAD(&ring_dev->rx_head);
	INIT_WORK(&ring_dev->rx_task, mdbg_ring_rx_task);
	ring_dev->flag_smp = 0;
	mdbg_pt_ring_reg();
	WCN_INFO("success!");

	mdbg_dev->ring_dev = ring_dev;

	return err;
}

void mdbg_ring_remove(void)
{
	struct ring_rx_data *pos, *next;

	MDBG_FUNC_ENTERY;
	mdbg_pt_ring_unreg();
	wakeup_source_unregister(ring_dev->rw_wake_lock);
	cancel_work_sync(&ring_dev->rx_task);
	mdbg_ring_destroy(ring_dev->ring);
	list_for_each_entry_safe(pos, next, &ring_dev->rx_head, entry) {
		list_del(&pos->entry);
		kfree(pos);
	}
	kfree(ring_dev);
	ring_dev = NULL;
}
