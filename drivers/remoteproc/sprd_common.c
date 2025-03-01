// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc remoteproc helpers
 *
 * Copyright (C) 2024 Otto Pflüger
 */

#include <linux/rpmsg/sprd_sipc.h>

#include "sprd_common.h"

#define to_sipc_subdev(_sd) container_of(_sd, struct sprd_sipc_subdev, subdev)

static int sipc_subdev_prepare(struct rproc_subdev *rpsubdev)
{
	struct sprd_sipc_subdev *sd = to_sipc_subdev(rpsubdev);

	sd->sipc = sprd_sipc_register(sd->dev, sd->node);

	return PTR_ERR_OR_ZERO(sd->sipc);
}

static void sipc_subdev_stop(struct rproc_subdev *rpsubdev, bool crashed)
{
	struct sprd_sipc_subdev *sd = to_sipc_subdev(rpsubdev);

	sprd_sipc_unregister(sd->sipc);
	sd->sipc = NULL;
}

static void sipc_subdev_unprepare(struct rproc_subdev *rpsubdev)
{
	struct sprd_sipc_subdev *sd = to_sipc_subdev(rpsubdev);

	/* unregister if stop wasn't called due to a start failure */
	if (sd->sipc) {
		sprd_sipc_unregister(sd->sipc);
		sd->sipc = NULL;
	}
}

static void remove_sipc_subdev(void *data)
{
	struct sprd_sipc_subdev *sd = data;

	rproc_remove_subdev(sd->rproc, &sd->subdev);
	of_node_put(sd->node);
}

int devm_sprd_rproc_add_sipc_subdev(struct rproc *rproc,
				    struct device_node *np,
				    struct sprd_sipc_subdev *sd)
{
	struct device *dev = &rproc->dev;

	sd->node = of_get_child_by_name(np, "sipc");
	if (!sd->node)
		return 0;

	sd->dev = dev;
	sd->rproc = rproc;
	sd->subdev.prepare = sipc_subdev_prepare;
	sd->subdev.unprepare = sipc_subdev_unprepare;
	sd->subdev.stop = sipc_subdev_stop;

	rproc_add_subdev(rproc, &sd->subdev);

	return devm_add_action_or_reset(rproc->dev.parent, remove_sipc_subdev, sd);
}
EXPORT_SYMBOL_GPL(devm_sprd_rproc_add_sipc_subdev);

MODULE_DESCRIPTION("Unisoc remoteproc helper driver");
MODULE_LICENSE("GPL");
