// SPDX-License-Identifier: GPL-2.0-only
/*
 * Unisoc UMS512 hardware CPU DVFS.
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 * Register sequences based on Copyright (C) 2020 Unisoc, Inc.
 */

#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/regmap.h>

#include <linux/soc/sprd/cpufreq-v1.h>

#define UMS512_CLUSTER_COUNT	2
#define UMS512_BIN_COUNT	5
#define UMS512_MAX_OPPS		9
#define UMS512_MAX_GRADES	8

static const char * const ums512_bin_names[UMS512_BIN_COUNT] = {
	NULL, "ff", "tt", "ss", "od",
};

static const unsigned int ums512_opp_count[UMS512_CLUSTER_COUNT] = { 9, 5 };

static const unsigned long ums512_freq_hz[UMS512_CLUSTER_COUNT][UMS512_MAX_OPPS] = {
	{
		614400000UL, 768000000UL, 962000000UL,
		1144000000UL, 1228800000UL, 1482000000UL,
		1536000000UL, 1716000000UL, 1820000000UL,
	},
	{
		1228800000UL, 1482000000UL, 1536000000UL,
		1742000000UL, 1820000000UL,
	},
};

struct ums512_index_map {
	u32 little[11];
	u32 big[7];
	u32 scu[10];
	u32 periph[6];
	u32 gic[6];
};

#define UMS512_HOST_INDEX(sel, div, volt, scu, peri, gic, mpll, gic_shift) \
	((sel) | ((div) << 2) | ((scu) << 5) | ((peri) << 9) | \
	 ((mpll) << 12) | ((volt) << 15) | ((gic) << (gic_shift)))
#define UMS512_LITTLE_INDEX(sel, div, volt, scu, peri, gic, mpll) \
	UMS512_HOST_INDEX(sel, div, volt, scu, peri, gic, mpll, 18)
#define UMS512_BIG_INDEX(sel, div, volt, scu, peri, gic, mpll) \
	UMS512_HOST_INDEX(sel, div, volt, scu, peri, gic, mpll, 21)
#define UMS512_SCU_INDEX(sel, div, volt, ace, mpll) \
	((sel) | ((div) << 2) | ((ace) << 5) | ((mpll) << 8) | ((volt) << 11))
#define UMS512_BUS_INDEX(sel, div, volt) \
	((sel) | ((div) << 2) | ((volt) << 5))

/*
 * T610 hardware selectors and votes, indexed by the cached eFuse bin.
 * Voltage indices select the higher of the stock cold/65 C requirements for
 * every consumer, including the SCU, peripheral and GIC votes. The OPP table
 * must supply the matching named voltage profile. Bin zero uses stock defaults.
 * Unused FF/TT peripheral tail rows repeat the preceding initialized row.
 */
static const struct ums512_index_map ums512_index_maps[UMS512_BIN_COUNT] = {
	/* Unprogrammed/default. */
	[0] = {
		.little = {
			UMS512_LITTLE_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_LITTLE_INDEX(1, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(2, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 0, 2, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 1, 3, 1, 1, 1),
			UMS512_LITTLE_INDEX(1, 0, 2, 4, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 3, 5, 4, 4, 2),
			UMS512_LITTLE_INDEX(2, 0, 4, 6, 4, 4, 0),
			UMS512_LITTLE_INDEX(3, 0, 5, 7, 4, 5, 3),
			UMS512_LITTLE_INDEX(3, 0, 6, 8, 4, 5, 4),
			UMS512_LITTLE_INDEX(3, 0, 6, 8, 4, 5, 4),
		},
		.big = {
			UMS512_BIG_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_BIG_INDEX(1, 0, 0, 3, 1, 1, 0),
			UMS512_BIG_INDEX(3, 0, 1, 5, 4, 4, 0),
			UMS512_BIG_INDEX(2, 0, 2, 6, 4, 4, 0),
			UMS512_BIG_INDEX(3, 0, 3, 7, 4, 5, 1),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
		},
		.scu = {
			UMS512_SCU_INDEX(0, 0, 0, 0, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 1),
			UMS512_SCU_INDEX(1, 0, 1, 1, 0),
			UMS512_SCU_INDEX(3, 0, 2, 1, 2),
			UMS512_SCU_INDEX(3, 0, 3, 1, 3),
			UMS512_SCU_INDEX(3, 0, 4, 1, 4),
			UMS512_SCU_INDEX(3, 0, 5, 1, 5),
			UMS512_SCU_INDEX(3, 0, 6, 1, 6),
			UMS512_SCU_INDEX(3, 0, 6, 1, 6),
		},
		.periph = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 1),
			UMS512_BUS_INDEX(3, 1, 1),
			UMS512_BUS_INDEX(2, 0, 1),
			UMS512_BUS_INDEX(3, 0, 5),
		},
		.gic = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 1),
			UMS512_BUS_INDEX(3, 1, 1),
			UMS512_BUS_INDEX(2, 0, 1),
			UMS512_BUS_INDEX(3, 0, 1),
		},
	},
	/* FF. */
	[1] = {
		.little = {
			UMS512_LITTLE_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_LITTLE_INDEX(1, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(2, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 0, 2, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 0, 3, 1, 1, 1),
			UMS512_LITTLE_INDEX(1, 0, 0, 4, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 1, 5, 4, 4, 2),
			UMS512_LITTLE_INDEX(2, 0, 2, 6, 4, 4, 0),
			UMS512_LITTLE_INDEX(3, 0, 3, 7, 4, 5, 3),
			UMS512_LITTLE_INDEX(3, 0, 4, 8, 4, 5, 4),
			UMS512_LITTLE_INDEX(3, 0, 4, 8, 4, 5, 4),
		},
		.big = {
			UMS512_BIG_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_BIG_INDEX(1, 0, 0, 3, 1, 1, 0),
			UMS512_BIG_INDEX(3, 0, 1, 5, 4, 4, 0),
			UMS512_BIG_INDEX(2, 0, 2, 6, 4, 4, 0),
			UMS512_BIG_INDEX(3, 0, 3, 7, 4, 5, 1),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
		},
		.scu = {
			UMS512_SCU_INDEX(0, 0, 0, 0, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 1),
			UMS512_SCU_INDEX(1, 0, 0, 1, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 2),
			UMS512_SCU_INDEX(3, 0, 1, 1, 3),
			UMS512_SCU_INDEX(3, 0, 2, 1, 4),
			UMS512_SCU_INDEX(3, 0, 3, 1, 5),
			UMS512_SCU_INDEX(3, 0, 4, 1, 6),
			UMS512_SCU_INDEX(3, 0, 4, 1, 6),
		},
		.periph = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 1),
			UMS512_BUS_INDEX(3, 1, 1),
			UMS512_BUS_INDEX(2, 0, 1),
			UMS512_BUS_INDEX(2, 0, 1),
		},
		.gic = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 1),
			UMS512_BUS_INDEX(3, 1, 1),
			UMS512_BUS_INDEX(2, 0, 1),
			UMS512_BUS_INDEX(3, 0, 1),
		},
	},
	/* TT. */
	[2] = {
		.little = {
			UMS512_LITTLE_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_LITTLE_INDEX(1, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(2, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 0, 2, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 0, 3, 1, 1, 1),
			UMS512_LITTLE_INDEX(1, 0, 1, 4, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 2, 5, 4, 4, 2),
			UMS512_LITTLE_INDEX(2, 0, 3, 6, 4, 4, 0),
			UMS512_LITTLE_INDEX(3, 0, 4, 7, 4, 5, 3),
			UMS512_LITTLE_INDEX(3, 0, 5, 8, 4, 5, 4),
			UMS512_LITTLE_INDEX(3, 0, 5, 8, 4, 5, 4),
		},
		.big = {
			UMS512_BIG_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_BIG_INDEX(1, 0, 0, 3, 1, 1, 0),
			UMS512_BIG_INDEX(3, 0, 1, 5, 4, 4, 0),
			UMS512_BIG_INDEX(2, 0, 2, 6, 4, 4, 0),
			UMS512_BIG_INDEX(3, 0, 3, 7, 4, 5, 1),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
		},
		.scu = {
			UMS512_SCU_INDEX(0, 0, 0, 0, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 1),
			UMS512_SCU_INDEX(1, 0, 0, 1, 0),
			UMS512_SCU_INDEX(3, 0, 1, 1, 2),
			UMS512_SCU_INDEX(3, 0, 2, 1, 3),
			UMS512_SCU_INDEX(3, 0, 3, 1, 4),
			UMS512_SCU_INDEX(3, 0, 4, 1, 5),
			UMS512_SCU_INDEX(3, 0, 5, 1, 6),
			UMS512_SCU_INDEX(3, 0, 5, 1, 6),
		},
		.periph = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 2),
			UMS512_BUS_INDEX(3, 1, 2),
			UMS512_BUS_INDEX(2, 0, 2),
			UMS512_BUS_INDEX(2, 0, 2),
		},
		.gic = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 2),
			UMS512_BUS_INDEX(3, 1, 2),
			UMS512_BUS_INDEX(2, 0, 2),
			UMS512_BUS_INDEX(3, 0, 2),
		},
	},
	/* SS. */
	[3] = {
		.little = {
			UMS512_LITTLE_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_LITTLE_INDEX(1, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(2, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 0, 2, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 1, 3, 1, 1, 1),
			UMS512_LITTLE_INDEX(1, 0, 2, 4, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 3, 5, 4, 4, 2),
			UMS512_LITTLE_INDEX(2, 0, 4, 6, 4, 4, 0),
			UMS512_LITTLE_INDEX(3, 0, 5, 7, 4, 5, 3),
			UMS512_LITTLE_INDEX(3, 0, 6, 8, 4, 5, 4),
			UMS512_LITTLE_INDEX(3, 0, 6, 8, 4, 5, 4),
		},
		.big = {
			UMS512_BIG_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_BIG_INDEX(1, 0, 0, 3, 1, 1, 0),
			UMS512_BIG_INDEX(3, 0, 1, 5, 4, 4, 0),
			UMS512_BIG_INDEX(2, 0, 2, 6, 4, 4, 0),
			UMS512_BIG_INDEX(3, 0, 3, 7, 4, 5, 1),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
		},
		.scu = {
			UMS512_SCU_INDEX(0, 0, 0, 0, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 1),
			UMS512_SCU_INDEX(1, 0, 1, 1, 0),
			UMS512_SCU_INDEX(3, 0, 2, 1, 2),
			UMS512_SCU_INDEX(3, 0, 3, 1, 3),
			UMS512_SCU_INDEX(3, 0, 4, 1, 4),
			UMS512_SCU_INDEX(3, 0, 5, 1, 5),
			UMS512_SCU_INDEX(3, 0, 6, 1, 6),
			UMS512_SCU_INDEX(3, 0, 6, 1, 6),
		},
		.periph = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 3),
			UMS512_BUS_INDEX(3, 1, 3),
			UMS512_BUS_INDEX(2, 0, 3),
			UMS512_BUS_INDEX(3, 0, 5),
		},
		.gic = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 3),
			UMS512_BUS_INDEX(3, 1, 3),
			UMS512_BUS_INDEX(2, 0, 3),
			UMS512_BUS_INDEX(3, 0, 3),
		},
	},
	/* OD. */
	[4] = {
		.little = {
			UMS512_LITTLE_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_LITTLE_INDEX(1, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(2, 1, 0, 1, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 0, 2, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 1, 3, 1, 1, 1),
			UMS512_LITTLE_INDEX(1, 0, 2, 4, 1, 1, 0),
			UMS512_LITTLE_INDEX(3, 0, 3, 5, 4, 4, 2),
			UMS512_LITTLE_INDEX(2, 0, 4, 6, 4, 4, 0),
			UMS512_LITTLE_INDEX(3, 0, 5, 7, 4, 5, 3),
			UMS512_LITTLE_INDEX(3, 0, 6, 8, 4, 5, 4),
			UMS512_LITTLE_INDEX(3, 0, 6, 8, 4, 5, 4),
		},
		.big = {
			UMS512_BIG_INDEX(0, 0, 0, 0, 0, 0, 0),
			UMS512_BIG_INDEX(1, 0, 0, 3, 1, 1, 0),
			UMS512_BIG_INDEX(3, 0, 1, 5, 4, 4, 0),
			UMS512_BIG_INDEX(2, 0, 2, 6, 4, 4, 0),
			UMS512_BIG_INDEX(3, 0, 3, 7, 4, 5, 1),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
			UMS512_BIG_INDEX(3, 0, 4, 8, 4, 5, 2),
		},
		.scu = {
			UMS512_SCU_INDEX(0, 0, 0, 0, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 0),
			UMS512_SCU_INDEX(3, 0, 0, 1, 1),
			UMS512_SCU_INDEX(1, 0, 1, 1, 0),
			UMS512_SCU_INDEX(3, 0, 2, 1, 2),
			UMS512_SCU_INDEX(3, 0, 3, 1, 3),
			UMS512_SCU_INDEX(3, 0, 4, 1, 4),
			UMS512_SCU_INDEX(3, 0, 5, 1, 5),
			UMS512_SCU_INDEX(3, 0, 6, 1, 6),
			UMS512_SCU_INDEX(3, 0, 6, 1, 6),
		},
		.periph = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 1),
			UMS512_BUS_INDEX(3, 1, 1),
			UMS512_BUS_INDEX(2, 0, 1),
			UMS512_BUS_INDEX(3, 0, 5),
		},
		.gic = {
			UMS512_BUS_INDEX(0, 0, 0),
			UMS512_BUS_INDEX(1, 0, 0),
			UMS512_BUS_INDEX(2, 1, 1),
			UMS512_BUS_INDEX(3, 1, 1),
			UMS512_BUS_INDEX(2, 0, 1),
			UMS512_BUS_INDEX(3, 0, 1),
		},
	},
};

#undef UMS512_BUS_INDEX
#undef UMS512_SCU_INDEX
#undef UMS512_BIG_INDEX
#undef UMS512_LITTLE_INDEX
#undef UMS512_HOST_INDEX

#define UMS512_SETTLE_US		200
#define UMS512_PMIC_STEP_UV	3125
#define UMS512_PMIC_SLEW_UV_US	9375
#define UMS512_PMIC_MARGIN_US	20
#define UMS512_SAVED_REGS	128
#define UMS512_WORK_INDEX_MASK	GENMASK(3, 0)
#define UMS512_IDLE_DISABLE	GENMASK(11, 0)

enum ums512_block {
	UMS512_APCPU,
	UMS512_TOP,
	UMS512_G2,
	UMS512_G3,
	UMS512_PMU,
	UMS512_BLOCK_COUNT,
};

struct ums512_saved_reg {
	u32 reg, mask, value;
	enum ums512_block block;
};

struct ums512_dvfs {
	struct ums512_saved_reg saved[UMS512_SAVED_REGS];
	unsigned int saved_count;
	u32 delays[UMS512_CLUSTER_COUNT][UMS512_MAX_GRADES - 1];
};

static const u32 ums512_work_regs[] = { 0x214, 0x224, 0x22c };
static const u32 ums512_low_indices[] = { 1, 1, 3 };

/* Save only the fields owned by this driver, once, before changing them. */
static int ums512_update(struct sprd_cpufreq_v1 *dvfs, enum ums512_block block,
			 u32 reg, u32 mask, u32 value)
{
	struct ums512_dvfs *priv = dvfs->priv;
	struct ums512_saved_reg *saved;
	unsigned int i;
	int ret;

	for (i = 0; i < priv->saved_count; i++) {
		saved = &priv->saved[i];
		if (saved->block == block && saved->reg == reg)
			break;
	}
	if (i == priv->saved_count) {
		if (i == ARRAY_SIZE(priv->saved))
			return -E2BIG;
		saved = &priv->saved[i];
		ret = regmap_read(dvfs->maps[block], reg, &saved->value);
		if (ret)
			return ret;
		saved->block = block;
		saved->reg = reg;
		priv->saved_count++;
	}
	saved->mask |= mask;

	return regmap_update_bits(dvfs->maps[block], reg, mask, value);
}

static bool ums512_is_work_reg(const struct ums512_saved_reg *saved)
{
	return saved->block == UMS512_APCPU &&
		(saved->reg == 0x214 || saved->reg == 0x224 || saved->reg == 0x22c);
}

static bool ums512_is_source_reg(const struct ums512_saved_reg *saved)
{
	return (saved->block == UMS512_G2 && saved->reg == 0x44) ||
		(saved->block == UMS512_G3 &&
		 (saved->reg == 0x98 || saved->reg == 0xdc));
}

static void ums512_restore(struct sprd_cpufreq_v1 *dvfs)
{
	struct ums512_dvfs *priv = dvfs->priv;
	struct ums512_saved_reg *saved;
	int i, ret, error = 0;

	if (!priv->saved_count)
		return;

	/* Neither CPU should consume high grades while the old banks return. */
	for (i = 0; i < ARRAY_SIZE(ums512_work_regs); i++) {
		ret = regmap_update_bits(dvfs->maps[UMS512_APCPU],
					 ums512_work_regs[i], UMS512_WORK_INDEX_MASK,
					 ums512_low_indices[i]);
		if (ret)
			error = ret;
	}
	fsleep(UMS512_SETTLE_US);
	if (error)
		goto out;
	for (i = priv->saved_count - 1; i >= 0; i--) {
		saved = &priv->saved[i];
		if (ums512_is_work_reg(saved) || ums512_is_source_reg(saved))
			continue;
		ret = regmap_update_bits(dvfs->maps[saved->block], saved->reg,
					 saved->mask, saved->value);
		if (ret)
			error = ret;
	}
	if (error)
		goto out;
	for (i = priv->saved_count - 1; i >= 0; i--) {
		saved = &priv->saved[i];
		if (!ums512_is_work_reg(saved))
			continue;
		ret = regmap_update_bits(dvfs->maps[saved->block], saved->reg,
					 saved->mask, saved->value);
		if (ret)
			error = ret;
	}
	fsleep(UMS512_SETTLE_US);
	if (error)
		goto out;
	for (i = priv->saved_count - 1; i >= 0; i--) {
		saved = &priv->saved[i];
		if (!ums512_is_source_reg(saved))
			continue;
		ret = regmap_update_bits(dvfs->maps[saved->block], saved->reg,
					 saved->mask, saved->value);
		if (ret)
			error = ret;
	}
	fsleep(UMS512_SETTLE_US);
out:
	if (error)
		dev_err(dvfs->dev, "failed to restore firmware DVFS state: %d\n", error);
}

static int ums512_check_map(const u32 *map, unsigned int count,
			    unsigned int shift, unsigned int grades)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (((map[i] >> shift) & 7) >= grades)
			return -EINVAL;
	return 0;
}

static int ums512_prepare_timings(struct sprd_cpufreq_v1 *dvfs,
				  struct sprd_cpufreq_v1_cluster *cluster)
{
	struct ums512_dvfs *priv = dvfs->priv;
	u32 step, maximum;
	u64 delay;
	unsigned int i, j;

	for (i = 0; i + 1 < cluster->grade_count; i++) {
		maximum = 0;
		for (j = 0; j + i + 1 < cluster->grade_count; j++) {
			step = cluster->grades[j + i + 1] - cluster->grades[j];
			maximum = max(maximum, step);
		}
		delay = DIV_ROUND_UP(maximum, UMS512_PMIC_SLEW_UV_US) +
			UMS512_PMIC_MARGIN_US;
		delay = DIV_ROUND_UP_ULL(delay * dvfs->clock_rate, USEC_PER_SEC);
		if (delay > U16_MAX)
			return -ERANGE;
		priv->delays[cluster->id][i] = delay;
	}
	return 0;
}

static int ums512_validate_maps(struct sprd_cpufreq_v1 *dvfs)
{
	const struct ums512_index_map *little = &ums512_index_maps[dvfs->clusters[0].bin];
	const struct ums512_index_map *big = &ums512_index_maps[dvfs->clusters[1].bin];
	unsigned int grades = dvfs->clusters[0].grade_count;
	unsigned int i;
	int ret;

	if (!dvfs->clock_rate || dvfs->clock_rate > 1000000000UL)
		return -EINVAL;

	for (i = 0; i < UMS512_CLUSTER_COUNT; i++) {
		ret = ums512_prepare_timings(dvfs, &dvfs->clusters[i]);
		if (ret)
			return ret;
	}

	ret = ums512_check_map(little->little, ARRAY_SIZE(little->little), 15, grades);
	if (!ret)
		ret = ums512_check_map(little->scu, ARRAY_SIZE(little->scu), 11, grades);
	if (!ret)
		ret = ums512_check_map(little->periph, ARRAY_SIZE(little->periph), 5, grades);
	if (!ret)
		ret = ums512_check_map(little->gic, ARRAY_SIZE(little->gic), 5, grades);
	if (!ret)
		ret = ums512_check_map(big->big, ARRAY_SIZE(big->big), 15,
				       dvfs->clusters[1].grade_count);
	return ret;
}

static int ums512_write_map(struct sprd_cpufreq_v1 *dvfs, u32 reg,
			    const u32 *map, unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = ums512_update(dvfs, UMS512_APCPU, reg + i * 4, U32_MAX, map[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int ums512_program_voltages(struct sprd_cpufreq_v1 *dvfs,
				   struct sprd_cpufreq_v1_cluster *cluster)
{
	struct ums512_dvfs *priv = dvfs->priv;
	u32 reg, shift, cycles;
	unsigned int i;
	int ret;

	for (i = 0; i < cluster->grade_count; i++) {
		reg = (cluster->id ? 0x100 : 0xf4) + (i / 3) * 4;
		shift = (i % 3) * 9;
		ret = ums512_update(dvfs, UMS512_TOP, reg, 0x1ff << shift,
				    (cluster->grades[i] / UMS512_PMIC_STEP_UV) << shift);
		if (ret)
			return ret;
	}
	for (i = 0; i + 1 < cluster->grade_count; i++) {
		cycles = priv->delays[cluster->id][i];
		shift = (i % 2) * 16;
		reg = i == 6 ? (cluster->id ? 0x118 : 0x110) :
			(cluster->id ? 0x84 : 0x58) - (i / 2) * 4;
		ret = ums512_update(dvfs, UMS512_TOP, reg, 0xffffU << shift, cycles << shift);
		if (ret)
			return ret;
		reg = i == 6 ? (cluster->id ? 0x11c : 0x114) :
			(cluster->id ? 0x90 : 0x64) - (i / 2) * 4;
		ret = ums512_update(dvfs, UMS512_TOP, reg, 0xffffU << shift, cycles << shift);
		if (ret)
			return ret;
	}

	shift = cluster->id ? 0 : 3;
	return ums512_update(dvfs, UMS512_TOP, 0x16c, 7 << shift,
			      (cluster->grade_count - 1) << shift);
}

static int ums512_program_plls(struct sprd_cpufreq_v1 *dvfs)
{
	/* Additional MPLL banks specified by the UMS512 hardware BSP. */
	static const struct {
		enum ums512_block block;
		u16 reg, n;
		u8 icp;
	} banks[] = {
		{ UMS512_G3, 0x8c, 0x4d, 5 }, { UMS512_G3, 0x90, 0x4d, 5 },
		{ UMS512_G3, 0x94, 0x4d, 5 }, { UMS512_G2, 0x2c, 0x46, 4 },
		{ UMS512_G2, 0x30, 0x48, 4 }, { UMS512_G2, 0x34, 0x4d, 5 },
		{ UMS512_G2, 0x38, 0x4d, 5 }, { UMS512_G2, 0x3c, 0x4d, 5 },
		{ UMS512_G2, 0x40, 0x4d, 5 }, { UMS512_G3, 0xd8, 0x36, 2 },
		{ UMS512_G3, 0xd4, 0x33, 1 }, { UMS512_G3, 0xd0, 0x2f, 1 },
		{ UMS512_G3, 0xcc, 0x2b, 0 }, { UMS512_G3, 0xc8, 0x27, 0 },
	};
	unsigned int i;
	int ret;

	ret = ums512_update(dvfs, UMS512_G3, 0x98, 0xd0, 0);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_G2, 0x44, 0xd0, 0);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_G3, 0xdc, 0x68, 0);
	if (ret)
		return ret;
	fsleep(50);
	for (i = 0; i < ARRAY_SIZE(banks); i++) {
		ret = ums512_update(dvfs, banks[i].block, banks[i].reg,
				    GENMASK(14, 0), (banks[i].n << 4) | banks[i].icp);
		if (ret)
			return ret;
	}
	for (i = 0; i < 3; i++) {
		ret = ums512_update(dvfs, UMS512_APCPU, 0x244 + i * 0x14, 3, 3);
		if (ret)
			return ret;
	}
	return 0;
}

static int ums512_hardware_init(struct sprd_cpufreq_v1 *dvfs)
{
	struct ums512_dvfs *priv = dvfs->priv;
	const struct ums512_index_map *little = &ums512_index_maps[dvfs->clusters[0].bin];
	const struct ums512_index_map *big = &ums512_index_maps[dvfs->clusters[1].bin];
	struct ums512_saved_reg *saved;
	unsigned int i;
	int ret;

	/* Rollback must know all three original requests before staging any. */
	for (i = 0; i < ARRAY_SIZE(ums512_work_regs); i++) {
		saved = &priv->saved[i];
		ret = regmap_read(dvfs->maps[UMS512_APCPU], ums512_work_regs[i],
				  &saved->value);
		if (ret)
			return ret;
		saved->block = UMS512_APCPU;
		saved->reg = ums512_work_regs[i];
		saved->mask = UMS512_WORK_INDEX_MASK;
	}
	priv->saved_count = ARRAY_SIZE(ums512_work_regs);
	for (i = 0; i < ARRAY_SIZE(ums512_work_regs); i++) {
		ret = ums512_update(dvfs, UMS512_APCPU, ums512_work_regs[i],
				    UMS512_WORK_INDEX_MASK, ums512_low_indices[i]);
		if (ret)
			return ret;
	}
	fsleep(UMS512_SETTLE_US);
	ret = ums512_program_plls(dvfs);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_TOP, 0x120, BIT(0), 0);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_PMU, 0x4fc, 3, 3);
	if (ret)
		return ret;
	for (i = 0; i < UMS512_CLUSTER_COUNT; i++) {
		ret = ums512_program_voltages(dvfs, &dvfs->clusters[i]);
		if (ret)
			return ret;
	}
	ret = ums512_write_map(dvfs, 0x60, little->little, ARRAY_SIZE(little->little));
	if (!ret)
		ret = ums512_write_map(dvfs, 0xa0, big->big, ARRAY_SIZE(big->big));
	if (!ret)
		ret = ums512_write_map(dvfs, 0x180, little->scu, ARRAY_SIZE(little->scu));
	if (!ret)
		ret = ums512_write_map(dvfs, 0x1f4, little->periph, ARRAY_SIZE(little->periph));
	if (!ret)
		ret = ums512_write_map(dvfs, 0x280, little->gic, ARRAY_SIZE(little->gic));
	if (!ret)
		ret = ums512_update(dvfs, UMS512_APCPU, 0x218, UMS512_WORK_INDEX_MASK, 1);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_APCPU, 0x228, UMS512_WORK_INDEX_MASK, 1);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_APCPU, 0x1c, UMS512_IDLE_DISABLE,
				    UMS512_IDLE_DISABLE);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_TOP, 0x68, BIT(20), 0);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_TOP, 0x94, BIT(0), 0);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_APCPU, 0x18, 7, 7);
	if (!ret)
		ret = ums512_update(dvfs, UMS512_TOP, 0x150, BIT(6) | BIT(7), 0);

	return ret;
}

static int ums512_set_target(struct sprd_cpufreq_v1_cluster *cluster, unsigned int index)
{
	struct sprd_cpufreq_v1 *dvfs = cluster->parent;
	u32 reg = ums512_work_regs[cluster->id];
	u32 value;

	if (index >= ums512_opp_count[cluster->id])
		return -EINVAL;
	value = readl_relaxed(dvfs->base + reg);
	value = (value & ~UMS512_WORK_INDEX_MASK) | (index + 1);
	writel(value, dvfs->base + reg);
	/* Flush the request before starting the PMIC/PLL settling interval. */
	readl(dvfs->base + reg);
	fsleep(UMS512_SETTLE_US);

	return 0;
}

static unsigned int ums512_get_frequency(struct sprd_cpufreq_v1_cluster *cluster)
{
	u32 index;

	index = readl_relaxed(cluster->parent->base + ums512_work_regs[cluster->id]) &
		UMS512_WORK_INDEX_MASK;
	if (!index)
		return 26000;
	/* The hardware has one duplicate top row after the usable OPPs. */
	if (index > ums512_opp_count[cluster->id] + 1)
		return 0;
	index = min(index, ums512_opp_count[cluster->id]);
	return cluster->table[index - 1].frequency;
}

static int ums512_resume(struct sprd_cpufreq_v1 *dvfs)
{
	/* Firmware changes the idle votes across system sleep on this SoC. */
	return regmap_update_bits(dvfs->maps[UMS512_APCPU], 0x1c,
				  UMS512_IDLE_DISABLE, UMS512_IDLE_DISABLE);
}

static const struct sprd_cpufreq_v1_cluster_data ums512_clusters[] = {
	{
		.freq_hz = ums512_freq_hz[0],
		.num_opps = 9,
		.max_grades = UMS512_MAX_GRADES,
		.voltage_step_uv = UMS512_PMIC_STEP_UV,
		.voltage_selector_mask = 0x1ff,
	},
	{
		.freq_hz = ums512_freq_hz[1],
		.num_opps = 5,
		.max_grades = 6,
		.voltage_step_uv = UMS512_PMIC_STEP_UV,
		.voltage_selector_mask = 0x1ff,
	},
};

static const char * const ums512_syscons[] = {
	[UMS512_APCPU] = NULL,
	[UMS512_TOP] = "sprd,top-dvfs",
	[UMS512_G2] = "sprd,anlg-phy-g2",
	[UMS512_G3] = "sprd,anlg-phy-g3",
	[UMS512_PMU] = "sprd,pmu",
};

static const struct regmap_config ums512_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x2fc,
	.fast_io = true,
};

static const struct sprd_cpufreq_v1_ops ums512_ops = {
	.validate = ums512_validate_maps,
	.init = ums512_hardware_init,
	.restore = ums512_restore,
	.set_target = ums512_set_target,
	.get_frequency = ums512_get_frequency,
	.resume = ums512_resume,
};

const struct sprd_cpufreq_v1_soc_data ums512_cpufreq_data = {
	.clusters = ums512_clusters,
	.num_clusters = ARRAY_SIZE(ums512_clusters),
	.bin_names = ums512_bin_names,
	.num_bins = ARRAY_SIZE(ums512_bin_names),
	.syscons = ums512_syscons,
	.num_maps = ARRAY_SIZE(ums512_syscons),
	.regmap_config = &ums512_regmap_config,
	.priv_size = sizeof(struct ums512_dvfs),
	.settle_us = UMS512_SETTLE_US,
	.transition_delay_us = 500,
	.ops = &ums512_ops,
};
