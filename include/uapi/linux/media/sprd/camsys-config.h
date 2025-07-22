/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (C) 2025 Otto Pflüger
 */

#ifndef _UAPI_MEDIA_SPRD_CAMSYS_CONFIG_H_
#define _UAPI_MEDIA_SPRD_CAMSYS_CONFIG_H_

#include <linux/types.h>

#include <linux/media/v4l2-isp.h>

#define SPRD_CAMSYS_AE_MAX_BLOCKS_X	64
#define SPRD_CAMSYS_AE_MAX_BLOCKS_Y	64
#define SPRD_CAMSYS_AE_MAX_BLOCKS	(SPRD_CAMSYS_AE_MAX_BLOCKS_X * \
					 SPRD_CAMSYS_AE_MAX_BLOCKS_Y)

struct sprd_camsys_ae_chan_hist {
	__u32 low_sum;
	__u32 mid_sum;
	__u32 high_sum;
	__u16 low_cnt;
	__u16 high_cnt;
} __attribute__((packed));

struct sprd_camsys_ae_block_stats {
	struct sprd_camsys_ae_chan_hist g;
	struct sprd_camsys_ae_chan_hist r;
	struct sprd_camsys_ae_chan_hist b;
} __attribute__((packed));

struct sprd_camsys_ae_stats {
	struct sprd_camsys_ae_block_stats blk[SPRD_CAMSYS_AE_MAX_BLOCKS];
} __attribute__((aligned(8)));

#define SPRD_CAMSYS_AF_MAX_BLOCKS_X	20
#define SPRD_CAMSYS_AF_MAX_BLOCKS_Y	15
#define SPRD_CAMSYS_AF_MAX_BLOCKS	(SPRD_CAMSYS_AF_MAX_BLOCKS_X * \
					 SPRD_CAMSYS_AF_MAX_BLOCKS_Y)

struct sprd_camsys_af_block_stats {
	__u32 contrast_1;
	__u32 contrast_2;
	__u32 pixel_sum;
	__u32 reserved;
} __attribute__((packed));

struct sprd_camsys_af_stats {
	struct sprd_camsys_af_block_stats blk[SPRD_CAMSYS_AF_MAX_BLOCKS];
} __attribute__((aligned(8)));

struct sprd_dcam_statistics {
	struct sprd_camsys_ae_stats ae;
	struct sprd_camsys_af_stats af;
};

struct sprd_camsys_ae_config {
	struct v4l2_isp_params_block_header header;
	__u16 offset_x;
	__u16 offset_y;
	__u16 blk_num_x;
	__u16 blk_num_y;
	__u16 blk_width;
	__u16 blk_height;
	__u16 r_low;
	__u16 r_high;
	__u16 g_low;
	__u16 g_high;
	__u16 b_low;
	__u16 b_high;
} __attribute__((aligned(8)));

struct sprd_camsys_af_config {
	struct v4l2_isp_params_block_header header;
	__u16 offset_x;
	__u16 offset_y;
	__u16 blk_num_x;
	__u16 blk_num_y;
	__u16 blk_width;
	__u16 blk_height;
} __attribute__((aligned(8)));

struct sprd_camsys_awb_config {
	struct v4l2_isp_params_block_header header;
	__u16 gain_r;
	__u16 gain_b;
	__u16 gain_gb;
	__u16 gain_gr;
	__u16 offset_r;
	__u16 offset_b;
	__u16 offset_gr;
	__u16 offset_gb;
} __attribute__((aligned(8)));

struct sprd_camsys_blc_config {
	struct v4l2_isp_params_block_header header;
	__u16 r;
	__u16 b;
	__u16 gr;
	__u16 gb;
} __attribute__((aligned(8)));

struct sprd_camsys_ccm_config {
	struct v4l2_isp_params_block_header header;
	__u16 r_r;
	__u16 r_g;
	__u16 r_b;
	__u16 g_r;
	__u16 g_g;
	__u16 g_b;
	__u16 b_r;
	__u16 b_g;
	__u16 b_b;
} __attribute__((aligned(8)));

struct sprd_camsys_gamma_config {
	struct v4l2_isp_params_block_header header;
	__u8 r_tbl[257];
	__u8 g_tbl[257];
	__u8 b_tbl[257];
} __attribute__((aligned(8)));

#define SPRD_CAMSYS_LSC_MAX_WEIGHTS		128

struct sprd_camsys_lsc_config {
	struct v4l2_isp_params_block_header header;
	__u16 blk_width;
	__u8 blk_num_x;
	__u8 blk_num_y;
	__u16 weights[SPRD_CAMSYS_LSC_MAX_WEIGHTS][3];
} __attribute__((aligned(8)));

enum sprd_dcam_block_type {
	SPRD_DCAM_BLOCK_AE,
	SPRD_DCAM_BLOCK_AF,
	SPRD_DCAM_BLOCK_AWB,
	SPRD_DCAM_BLOCK_BLC,
	SPRD_DCAM_BLOCK_LSC,
#ifdef __KERNEL__
	SPRD_DCAM_BLOCK_NUM
#endif
};

#define SPRD_DCAM_MAX_CONFIG_SIZE			\
	(sizeof(struct sprd_camsys_ae_config) +		\
	 sizeof(struct sprd_camsys_af_config) +		\
	 sizeof(struct sprd_camsys_awb_config) +	\
	 sizeof(struct sprd_camsys_blc_config) +	\
	 sizeof(struct sprd_camsys_lsc_config))

enum sprd_isp_block_type {
	SPRD_ISP_BLOCK_CCM,
	SPRD_ISP_BLOCK_GAMMA,
#ifdef __KERNEL__
	SPRD_ISP_BLOCK_NUM
#endif
};

#define SPRD_ISP_MAX_CONFIG_SIZE			\
	(sizeof(struct sprd_camsys_ccm_config) +	\
	 sizeof(struct sprd_camsys_gamma_config))

#endif
