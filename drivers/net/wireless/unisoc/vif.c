// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/inetdevice.h>
#include <linux/module.h>

#include "cmd.h"
#include "txrx.h"
#include "vif.h"

static int sc23xx_iftype_to_mode(enum nl80211_iftype iftype,
				 enum sc23xx_vif_mode *mode,
				 u8 *idx)
{
	switch (iftype) {
	case NL80211_IFTYPE_STATION:
		*mode = SC23XX_MODE_STATION;
		*idx = SC23XX_VIF_STA;
		break;
	case NL80211_IFTYPE_AP:
		*mode = SC23XX_MODE_AP;
		*idx = SC23XX_VIF_AP;
		break;
	case NL80211_IFTYPE_P2P_DEVICE:
		*mode = SC23XX_MODE_P2P_DEVICE;
		*idx = SC23XX_VIF_P2P_DEVICE;
		break;
	case NL80211_IFTYPE_P2P_CLIENT:
		*mode = SC23XX_MODE_P2P_CLIENT;
		*idx = SC23XX_VIF_AP;
		break;
	case NL80211_IFTYPE_P2P_GO:
		*mode = SC23XX_MODE_P2P_GO;
		*idx = SC23XX_VIF_AP;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const char *sc23xx_vif_idx_to_string(u8 idx)
{
	switch (idx) {
	case SC23XX_VIF_STA:
		return "STA";
	case SC23XX_VIF_AP:
		return "AP, P2P or 2nd STA";
	case SC23XX_VIF_P2P_DEVICE:
		return "P2P device";
	default:
		break;
	}

	return "unknown type";
}

/*
 * Tell the firmware about the addresses the interface already has.
 *
 * The inetaddr notifier below only fires when an address is added or removed,
 * and IPv4 addresses survive ndo_stop/ndo_open - so after "ip link set wlan0
 * down; ip link set wlan0 up", or when an address was configured before the
 * link came up, no event ever arrives and the firmware is never told. It then
 * goes back to delivering no broadcast to the host, which is the failure this
 * whole patch exists to fix. ndo_open runs under RTNL.
 */
static void sc23xx_notify_configured_ips(struct sc23xx_vif *vif,
					 struct net_device *ndev)
{
	const struct in_ifaddr *ifa;
	struct in_device *in_dev;
	int ret;

	if (vif->wdev.iftype != NL80211_IFTYPE_STATION &&
	    vif->wdev.iftype != NL80211_IFTYPE_P2P_CLIENT)
		return;

	in_dev = __in_dev_get_rtnl(ndev);
	if (!in_dev)
		return;

	in_dev_for_each_ifa_rtnl(ifa, in_dev) {
		ret = sc23xx_cmd_notify_ip(vif, (const u8 *)&ifa->ifa_address,
					   sizeof(ifa->ifa_address));
		if (ret)
			netdev_warn(ndev,
				    "failed to notify firmware of %pI4: %d\n",
				    &ifa->ifa_address, ret);
	}
}

static int sc23xx_open(struct net_device *ndev)
{
	struct sc23xx_vif *vif = netdev_priv(ndev);
	int ret;

	ret = sc23xx_cmd_open(vif, ndev->dev_addr);
	if (ret)
		return ret;

	spin_lock_irq(&vif->sdev->tx_buf_lock);
	if (vif->sdev->tx_blocked)
		netif_stop_queue(ndev);
	else
		netif_start_queue(ndev);
	set_bit(SC23XX_FLAG_OPENED, &vif->flags);
	spin_unlock_irq(&vif->sdev->tx_buf_lock);

	atomic_inc(&vif->sdev->open_count);
	wake_up(&vif->sdev->tx_wait);

	ret = sc23xx_cmd_set_regdom(vif->sdev);
	if (ret) {
		wiphy_err(vif->sdev->wiphy,
			  "failed to send initial regdomain to firmware\n");
		return ret;
	}

	sc23xx_notify_configured_ips(vif, ndev);

	return 0;
}

static int sc23xx_close(struct net_device *ndev)
{
	struct sc23xx_vif *vif = netdev_priv(ndev);
	int ret;

	/*
	 * Retire any outstanding scan before cfg80211 gets NETDEV_DOWN and
	 * finds rdev->scan_req un-notified (WARN in core.c), and before the
	 * firmware's scan-done event can hand the stale request back.
	 */
	sc23xx_notify_scan_done(vif, true);
	flush_workqueue(vif->sdev->evt_wq);

	ret = sc23xx_cmd_close(vif);
	if (ret)
		return ret;

	clear_bit(SC23XX_FLAG_OPENED, &vif->flags);
	if (atomic_dec_return(&vif->sdev->open_count) == 0)
		sc23xx_free_txrx_buffers(vif->sdev);

	return 0;
}

static void sc23xx_multicast_work(struct work_struct *work)
{
	struct sc23xx_vif *vif = container_of(work, struct sc23xx_vif, mc_work);
	struct net_device *ndev = vif->wdev.netdev;
	int ret;

	if (!vif->sdev->max_mc_mac_addrs ||
	    !test_bit(SC23XX_FLAG_OPENED, &vif->flags))
		return;

	if (ndev->mc.count > vif->sdev->max_mc_mac_addrs) {
		netdev_err(ndev, "too many multicast addresses set\n");
		return;
	}

	ret = sc23xx_cmd_set_multicast(vif);
	if (ret)
		netdev_err(ndev, "failed to set multicast list\n");
}

static void sc23xx_set_multicast(struct net_device *ndev)
{
	struct sc23xx_vif *vif = netdev_priv(ndev);

	schedule_work(&vif->mc_work);
}

static int sc23xx_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct sc23xx_vif *vif = netdev_priv(ndev);
	struct sockaddr *sa = addr;
	int ret;

	if (test_bit(SC23XX_FLAG_OPENED, &vif->flags)) {
		ret = sc23xx_cmd_set_mac_addr(vif, sa->sa_data);
		if (ret)
			return ret;
	}

	eth_commit_mac_addr_change(ndev, addr);

	return 0;
}

static netdev_tx_t sc23xx_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct sc23xx_vif *vif = netdev_priv(ndev);

	if (vif->state != SC23XX_STATE_CONNECTED) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	/* 802.1X (EAPOL) frames must go through the command channel: the
	 * firmware will not transmit a handshake frame as a plain data MSDU
	 * before the pairwise key is installed, which stalls the 4-way
	 * handshake. */
	if (skb->protocol == htons(ETH_P_PAE)) {
		sc23xx_tx_data2cmd(vif, skb);
		return NETDEV_TX_OK;
	}

	if (sc23xx_tx_prepare(vif, skb)) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	sc23xx_tx_data(vif->sdev, skb);

	return NETDEV_TX_OK;
}

static const struct net_device_ops sc23xx_netdev_ops = {
	.ndo_open = sc23xx_open,
	.ndo_stop = sc23xx_close,
	.ndo_start_xmit = sc23xx_start_xmit,
	.ndo_set_mac_address = sc23xx_set_mac_addr,
	.ndo_set_rx_mode = sc23xx_set_multicast,
};

struct sc23xx_vif *sc23xx_vif_new(struct sc23xx_dev *sdev, const char *name,
				  enum nl80211_iftype iftype)
{
	struct wireless_dev *wdev;
	struct net_device *ndev;
	struct sc23xx_vif *vif;
	u8 mac_addr[ETH_ALEN];
	int i, ret;

	ndev = alloc_netdev(sizeof(*vif), name, NET_NAME_UNKNOWN, ether_setup);
	if (!ndev)
		return ERR_PTR(-ENOMEM);

	vif = netdev_priv(ndev);
	mutex_init(&vif->scan_lock);
	vif->sdev = sdev;

	INIT_WORK(&vif->mc_work, sc23xx_multicast_work);

	ret = sc23xx_iftype_to_mode(iftype, &vif->mode, &vif->idx);
	if (ret)
		goto err_free_netdev;

	mutex_lock(&sdev->vif_lock);
	for (i = 0; i < SC23XX_VIF_NUM; i++) {
		struct sc23xx_vif *other_vif;
		
		other_vif = srcu_dereference_check(sdev->vif[i],
					&sdev->vif_srcu,
					lockdep_is_held(&sdev->vif_lock));
		if (other_vif && !test_bit(SC23XX_FLAG_USE_ALT_MAC,
					   &other_vif->flags))
			set_bit(SC23XX_FLAG_USE_ALT_MAC, &vif->flags);
	}
	/* AP interface may also act as second STA interface */
	if (rcu_access_pointer(sdev->vif[vif->idx]) &&
	    vif->idx == SC23XX_VIF_STA) {
		vif->idx = SC23XX_VIF_AP;
		vif->mode = SC23XX_MODE_STATION_SECOND;
	}
	if (rcu_access_pointer(sdev->vif[vif->idx])) {
		wiphy_err(sdev->wiphy, "%s interface already exists\n",
			  sc23xx_vif_idx_to_string(vif->idx));
		mutex_unlock(&sdev->vif_lock);
		ret = -EINVAL;
		goto err_free_netdev;
	}
	rcu_assign_pointer(sdev->vif[vif->idx], vif);
	mutex_unlock(&sdev->vif_lock);

	wdev = &vif->wdev;
	wdev->wiphy = sdev->wiphy;
	wdev->netdev = ndev;
	wdev->iftype = iftype;

	SET_NETDEV_DEV(ndev, wiphy_dev(sdev->wiphy));
	ndev->needs_free_netdev = true;
	ndev->ieee80211_ptr = wdev;
	ndev->netdev_ops = &sc23xx_netdev_ops;
	ndev->needed_headroom = sizeof(struct sc23xx_tx_data_hdr) + ETH_HLEN;
	if (sdev->use_dma)
		ndev->needed_headroom += sizeof(struct sc23xx_addr);

	memcpy(mac_addr, sdev->mac_addr, ETH_ALEN);
	if (test_bit(SC23XX_FLAG_USE_ALT_MAC, &vif->flags))
		mac_addr[5] ^= 0x70;
	eth_hw_addr_set(ndev, mac_addr);
	/*
	 * A made-up address has to be reported as such, or userspace takes it
	 * for a burned-in one and leaves it alone. With NET_ADDR_RANDOM set,
	 * systemd's default MACAddressPolicy=persistent replaces it before the
	 * interface comes up with one derived from the machine-id, so the
	 * address - and with it the DHCP lease and every peer's ARP entry -
	 * stops changing on each boot.
	 */
	if (sdev->mac_addr_random)
		ndev->addr_assign_type = NET_ADDR_RANDOM;

	ret = cfg80211_register_netdevice(ndev);
	if (ret)
		goto err_remove_vif;

	return vif;

err_remove_vif:
	mutex_lock(&sdev->vif_lock);
	sdev->vif[vif->idx] = NULL;
	mutex_unlock(&sdev->vif_lock);
err_free_netdev:
	free_netdev(ndev);
	return ERR_PTR(ret);
}

int sc23xx_vif_reopen(struct sc23xx_vif *vif, enum nl80211_iftype iftype)
{
	enum sc23xx_vif_mode mode;
	struct sc23xx_vif *other;
	int ret;
	u8 idx;

	ret = sc23xx_iftype_to_mode(iftype, &mode, &idx);
	if (ret)
		return ret;

	mutex_lock(&vif->sdev->vif_lock);
	/* AP interface may also act as second STA interface. Only fall back to
	 * the second-STA context when a *different* vif holds the STA slot: on
	 * a reopen this vif is already installed there itself, and demoting it
	 * to STATION_SECOND lands it in a context the firmware cannot
	 * authenticate in (connect times out with status 16). */
	other = rcu_access_pointer(vif->sdev->vif[idx]);
	if (other && other != vif && idx == SC23XX_VIF_STA) {
		idx = SC23XX_VIF_AP;
		mode = SC23XX_MODE_STATION_SECOND;
	}
	other = rcu_access_pointer(vif->sdev->vif[idx]);
	if (other && other != vif) {
		wiphy_err(vif->sdev->wiphy, "%s interface already exists\n",
			  sc23xx_vif_idx_to_string(idx));
		mutex_unlock(&vif->sdev->vif_lock);
		return -EINVAL;
	}

	if (!test_bit(SC23XX_FLAG_OPENED, &vif->flags)) {
		ret = sc23xx_cmd_close(vif);
		if (ret) {
			mutex_unlock(&vif->sdev->vif_lock);
			return ret;
		}
	}

	rcu_assign_pointer(vif->sdev->vif[vif->idx], NULL);
	rcu_assign_pointer(vif->sdev->vif[idx], vif);
	vif->idx = idx;
	mutex_unlock(&vif->sdev->vif_lock);

	vif->wdev.iftype = iftype;
	vif->mode = mode;

	if (test_bit(SC23XX_FLAG_OPENED, &vif->flags)) {
		ret = sc23xx_cmd_open(vif, vif->wdev.netdev->dev_addr);
		if (ret)
			return ret;
	}

	return 0;
}

void sc23xx_notify_scan_done(struct sc23xx_vif *vif, bool aborted)
{
	struct cfg80211_scan_info scan_info = { .aborted = aborted };

	mutex_lock(&vif->scan_lock);
	if (vif->scan_req) {
		cfg80211_scan_done(vif->scan_req, &scan_info);
		vif->scan_req = NULL;
	}
	mutex_unlock(&vif->scan_lock);
}

struct sc23xx_vif *sc23xx_get_vif(struct sc23xx_dev *sdev, int idx)
{
	struct sc23xx_vif *vif = NULL;

	if (idx < SC23XX_VIF_NUM)
		vif = srcu_dereference(sdev->vif[idx], &sdev->vif_srcu);

	if (!vif)
		wiphy_warn(sdev->wiphy, "invalid interface index %d\n", idx);

	return vif;
}

/*
 * Hand the station's IPv4 address to the firmware once the stack configures
 * it (CMD_NOTIFY_IP_ACQUIRED). The vendor driver does this from the same
 * notifier; without it the firmware never learns the host's address.
 */
static int sc23xx_inetaddr_event(struct notifier_block *nb,
				 unsigned long event, void *ptr)
{
	struct in_ifaddr *ifa = ptr;
	struct net_device *ndev = ifa->ifa_dev->dev;
	struct sc23xx_vif *vif;
	int ret;

	if (ndev->netdev_ops != &sc23xx_netdev_ops || event != NETDEV_UP)
		return NOTIFY_DONE;

	vif = netdev_priv(ndev);
	if (vif->wdev.iftype != NL80211_IFTYPE_STATION &&
	    vif->wdev.iftype != NL80211_IFTYPE_P2P_CLIENT)
		return NOTIFY_DONE;
	if (!test_bit(SC23XX_FLAG_OPENED, &vif->flags))
		return NOTIFY_DONE;

	ret = sc23xx_cmd_notify_ip(vif, (const u8 *)&ifa->ifa_address,
				   sizeof(ifa->ifa_address));
	if (ret)
		netdev_warn(ndev, "failed to notify firmware of %pI4: %d\n",
			    &ifa->ifa_address, ret);
	else
		netdev_dbg(ndev, "notified firmware of %pI4\n",
			   &ifa->ifa_address);

	return NOTIFY_DONE;
}

static struct notifier_block sc23xx_inetaddr_nb = {
	.notifier_call = sc23xx_inetaddr_event,
};

static int __init sc23xx_vif_init(void)
{
	return register_inetaddr_notifier(&sc23xx_inetaddr_nb);
}
module_init(sc23xx_vif_init);

static void __exit sc23xx_vif_exit(void)
{
	unregister_inetaddr_notifier(&sc23xx_inetaddr_nb);
}
module_exit(sc23xx_vif_exit);
