// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "core.h"

struct wcn_child_container {
	struct device dev;
};

static void wcn_child_container_release(struct device *dev)
{
	struct wcn_child_container *container;

	container = container_of(dev, struct wcn_child_container, dev);
	of_node_put(dev->of_node);
	kfree(container);
}

static struct device *wcn_child_container_create(struct wcn_core *core,
						 const char *name)
{
	struct wcn_child_container *container;
	struct device_node *np;
	int ret;

	np = of_get_child_by_name(core->dev->of_node, name);
	if (!np)
		return NULL;

	if (!of_device_is_available(np)) {
		of_node_put(np);
		return NULL;
	}

	container = kzalloc_obj(*container);
	if (!container) {
		of_node_put(np);
		return ERR_PTR(-ENOMEM);
	}

	device_initialize(&container->dev);
	container->dev.parent = core->dev;
	container->dev.release = wcn_child_container_release;
	container->dev.of_node = np;

	ret = dev_set_name(&container->dev, "%s:%s", dev_name(core->dev), name);
	if (ret) {
		put_device(&container->dev);
		return ERR_PTR(ret);
	}

	ret = device_add(&container->dev);
	if (ret) {
		put_device(&container->dev);
		return ERR_PTR(ret);
	}

	return &container->dev;
}

static struct platform_device *wcn_child_create(struct wcn_core *core,
						struct device *parent,
						const char *node_name,
						unsigned int id)
{
	const struct wcn_child_info *info = &core->chip->children[id];
	struct platform_device *pdev;
	struct wcn_child_pdata pdata = {
		.core = core,
		.data = &info->data,
	};
	struct device_node *np;
	int ret;

	if (!info->name)
		return NULL;

	if (node_name)
		np = of_get_child_by_name(parent->of_node, node_name);
	else
		np = of_node_get(parent->of_node);
	if (!np)
		return NULL;

	if (!of_device_is_available(np)) {
		of_node_put(np);
		return NULL;
	}

	pdev = platform_device_alloc(info->name, id);
	if (!pdev) {
		of_node_put(np);
		return ERR_PTR(-ENOMEM);
	}

	pdev->dev.parent = parent;
	pdev->dev.of_node = np;

	ret = platform_device_add_data(pdev, &pdata, sizeof(pdata));
	if (ret) {
		platform_device_put(pdev);
		return ERR_PTR(ret);
	}

	ret = platform_device_add(pdev);
	if (ret) {
		platform_device_put(pdev);
		return ERR_PTR(ret);
	}

	return pdev;
}

int wcn_children_create(struct wcn_core *core)
{
	int ret;

	core->btwf_dev = wcn_child_container_create(core, "btwf");
	if (IS_ERR(core->btwf_dev)) {
		ret = PTR_ERR(core->btwf_dev);
		core->btwf_dev = NULL;
		return ret;
	}
	if (!core->btwf_dev)
		return -ENOENT;

	core->child_pdevs[WCN_CHILD_WLAN] =
		wcn_child_create(core, core->btwf_dev,
				 core->chip->children[WCN_CHILD_WLAN].name,
				 WCN_CHILD_WLAN);
	if (IS_ERR(core->child_pdevs[WCN_CHILD_WLAN])) {
		ret = PTR_ERR(core->child_pdevs[WCN_CHILD_WLAN]);
		core->child_pdevs[WCN_CHILD_WLAN] = NULL;
		goto err;
	}

	core->child_pdevs[WCN_CHILD_BT] =
		wcn_child_create(core, core->btwf_dev,
				 core->chip->children[WCN_CHILD_BT].name,
				 WCN_CHILD_BT);
	if (IS_ERR(core->child_pdevs[WCN_CHILD_BT])) {
		ret = PTR_ERR(core->child_pdevs[WCN_CHILD_BT]);
		core->child_pdevs[WCN_CHILD_BT] = NULL;
		goto err;
	}

	core->child_pdevs[WCN_CHILD_FM] =
		wcn_child_create(core, core->btwf_dev,
				 core->chip->children[WCN_CHILD_FM].name,
				 WCN_CHILD_FM);
	if (IS_ERR(core->child_pdevs[WCN_CHILD_FM])) {
		ret = PTR_ERR(core->child_pdevs[WCN_CHILD_FM]);
		core->child_pdevs[WCN_CHILD_FM] = NULL;
		goto err;
	}

	core->gnss_dev = wcn_child_container_create(core, "gnss");
	if (IS_ERR(core->gnss_dev)) {
		ret = PTR_ERR(core->gnss_dev);
		core->gnss_dev = NULL;
		goto err;
	}

	if (core->gnss_dev) {
		core->child_pdevs[WCN_CHILD_GNSS] =
			wcn_child_create(core, core->gnss_dev, NULL,
					 WCN_CHILD_GNSS);
		if (IS_ERR(core->child_pdevs[WCN_CHILD_GNSS])) {
			ret = PTR_ERR(core->child_pdevs[WCN_CHILD_GNSS]);
			core->child_pdevs[WCN_CHILD_GNSS] = NULL;
			goto err;
		}
	}

	return 0;

err:
	wcn_children_destroy(core);
	return ret;
}

void wcn_children_destroy(struct wcn_core *core)
{
	int i;

	for (i = 0; i < WCN_CHILD_MAX; i++) {
		if (!core->child_pdevs[i])
			continue;

		platform_device_unregister(core->child_pdevs[i]);
		core->child_pdevs[i] = NULL;
	}

	if (core->btwf_dev) {
		device_unregister(core->btwf_dev);
		core->btwf_dev = NULL;
	}

	if (core->gnss_dev) {
		device_unregister(core->gnss_dev);
		core->gnss_dev = NULL;
	}
}
