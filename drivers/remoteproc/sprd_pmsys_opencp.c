// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Otto Pflüger
 */

#include <linux/dma-direct.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"

struct sprd_pmsys_info {
	u32 corereset_reg;
	u32 corereset_mask;
	u32 bus_cfg_reg;
	u32 bus_sleep_mask;
};

struct sprd_pmsys {
	struct device *dev;
	struct reset_control *reset;
	struct regmap *aon_apb_regs;
	const struct sprd_pmsys_info *info;

	phys_addr_t mem_phys;
	size_t mem_size;
	void *mem_virt;

	phys_addr_t bootmem_phys;
	size_t bootmem_size;
	void *bootmem_virt;

	struct mbox_client mbox_cl;
	struct mbox_chan *mbox_ch;
	u64 mbox_msg;
	unsigned long mbox_state;
};

static int sprd_pmsys_prepare(struct rproc *rproc)
{
	struct sprd_pmsys *p = rproc->priv;

	/* disable bus sleep */
	regmap_clear_bits(p->aon_apb_regs, p->info->bus_cfg_reg,
			  p->info->bus_sleep_mask);

	return 0;
}

static int sprd_pmsys_start(struct rproc *rproc)
{
	struct sprd_pmsys *p = rproc->priv;
	int ret;

	ret = reset_control_deassert(p->reset);
	if (ret < 0)
		return ret;

	/* start processor */
	regmap_clear_bits(p->aon_apb_regs, p->info->corereset_reg,
			  p->info->corereset_mask);

	return 0;
}

static int sprd_pmsys_stop(struct rproc *rproc)
{
	struct sprd_pmsys *p = rproc->priv;

	/* stop processor */
	regmap_set_bits(p->aon_apb_regs, p->info->corereset_reg,
			p->info->corereset_mask);

	reset_control_assert(p->reset);

	return 0;
}

static void *sprd_pmsys_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct sprd_pmsys *p = rproc->priv;
	phys_addr_t paddr;

	paddr = dma_to_phys(p->dev, da);

	dev_dbg(p->dev, "da 0x%08llx -> pa 0x%08llx\n", da, paddr);

	if (paddr >= p->mem_phys && (paddr + len) <=
	    (p->mem_phys + p->mem_size)) {
		return p->mem_virt + (paddr - p->mem_phys);
	}

	if (paddr >= p->bootmem_phys && (paddr + len) <=
	    (p->bootmem_phys + p->bootmem_size)) {
		if (is_iomem)
			*is_iomem = true;
		return p->bootmem_virt + (paddr - p->bootmem_phys);
	}

	return NULL;
}

static void sprd_pmsys_kick(struct rproc *rproc, int vqid)
{
	struct sprd_pmsys *p = rproc->priv;
	u32 *msg;
	int ret;

	msg = kmalloc(2 * sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return;

	msg[0] = 0;
	msg[1] = vqid;

	ret = mbox_send_message(p->mbox_ch, msg);
	if (ret < 0)
		dev_err(p->dev, "sending kick failed: %d\n", ret);
}

static void sprd_pmsys_rx_callback(struct mbox_client *cl, void *msg_data)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	u32 *msg = msg_data;

	if (msg[0] == 0)
		rproc_vq_interrupt(rproc, msg[1]);
	else if (msg[0] == 1)
		rproc_report_crash(rproc, RPROC_FATAL_ERROR);
	else
		dev_err(cl->dev, "invalid mbox message: 0x%08x 0x%08x\n",
			msg[0], msg[1]);
}

static void sprd_pmsys_tx_done(struct mbox_client *cl, void *msg_data, int r)
{
	if (r)
		dev_err(cl->dev, "kick failed: %d\n", r);

	kfree(msg_data);
}

static const struct rproc_ops sprd_pmsys_ops = {
	.prepare	= sprd_pmsys_prepare,
	.start		= sprd_pmsys_start,
	.stop		= sprd_pmsys_stop,
	.da_to_va	= sprd_pmsys_da_to_va,
	.kick		= sprd_pmsys_kick,
};

static int sprd_pmsys_map_memory_region(struct sprd_pmsys *p)
{
	struct device_node *np;
	struct reserved_mem *rmem;

	np = of_parse_phandle(p->dev->of_node, "memory-region", 1);
	if (!np) {
		dev_err(p->dev, "no firmware image memory region specified\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(np);
	if (!rmem) {
		of_node_put(np);
		dev_err(p->dev, "failed to look up memory region\n");
		return -EINVAL;
	}

	p->mem_phys = rmem->base;
	p->mem_size = rmem->size;

	p->mem_virt = memremap(p->mem_phys, p->mem_size, MEMREMAP_WC);
	if (!p->mem_virt) {
		dev_err(p->dev, "failed to map memory region\n");
		return -EBUSY;
	}

	of_node_put(np);

	return 0;
}

static int sprd_pmsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *bootmem;
	struct sprd_pmsys *p;
	struct rproc *rproc;
	const char *fw_name;
	struct mbox_client *cl;
	int ret;

	ret = of_property_read_string(dev->of_node, "firmware-name", &fw_name);
	if (ret < 0) {
		dev_err(dev, "unable to read firmware-name\n");
		return ret;
	}

	rproc = devm_rproc_alloc(dev, "pmsys", &sprd_pmsys_ops, fw_name, sizeof(*p));
	if (!rproc)
		return -ENOMEM;

	platform_set_drvdata(pdev, rproc);

	p = rproc->priv;
	p->dev = dev;

	p->info = of_device_get_match_data(dev);
	if (!p->info) {
		dev_err(dev, "failed to assign memory region: %d\n", ret);
		return -EINVAL;
	}

	p->bootmem_virt = devm_platform_get_and_ioremap_resource(pdev, 0, &bootmem);
	if (IS_ERR(p->bootmem_virt))
		return PTR_ERR(p->bootmem_virt);

	p->bootmem_phys = bootmem->start;
	p->bootmem_size = bootmem->end - bootmem->start + 1;

	ret = sprd_pmsys_map_memory_region(p);
	if (ret < 0)
		return ret;

	p->aon_apb_regs = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,syscon-aon-apb");
	if (IS_ERR(p->aon_apb_regs)) {
		dev_err(dev, "failed to get aon-apb syscon handle\n");
		return PTR_ERR(p->aon_apb_regs);
	}

	p->reset = devm_reset_control_get_optional(dev, NULL);
	if (IS_ERR(p->reset)) {
		dev_err(dev, "failed to get pmsys reset\n");
		return PTR_ERR(p->reset);
	}

	cl = &p->mbox_cl;
	cl->dev = dev;
	cl->rx_callback = sprd_pmsys_rx_callback;
	cl->tx_done = sprd_pmsys_tx_done;

	p->mbox_ch = mbox_request_channel(cl, 0);
	if (IS_ERR(p->mbox_ch))
		return dev_err_probe(dev, PTR_ERR(p->mbox_ch),
				     "failed to request mailbox channel\n");

	ret = devm_rproc_add(dev, rproc);
	if (ret < 0) {
		dev_err(dev, "failed to add rproc\n");
		mbox_free_channel(p->mbox_ch);
		return ret;
	}

	return 0;
}

static void sprd_pmsys_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sprd_pmsys *p = rproc->priv;

	mbox_free_channel(p->mbox_ch);
}

static const struct sprd_pmsys_info ums9230_pmsys_info = {
	.corereset_reg = 0x008c,
	.corereset_mask = BIT(0),
	.bus_cfg_reg = 0x0124,
	.bus_sleep_mask = BIT(0),
};

static const struct of_device_id sprd_pmsys_of_match[] = {
	{ .compatible = "sprd,ums9230-pmsys-opencp", .data = &ums9230_pmsys_info },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_pmsys_of_match);

static struct platform_driver sprd_pmsys_driver = {
	.probe = sprd_pmsys_probe,
	.remove = sprd_pmsys_remove,
	.driver = {
		.name = "sprd-pmsys-opencp",
		.of_match_table = sprd_pmsys_of_match,
	},
};

module_platform_driver(sprd_pmsys_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Unisoc PMSYS remoteproc driver (OpenCP version)");
