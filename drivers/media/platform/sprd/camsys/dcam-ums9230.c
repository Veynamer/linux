// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Unisoc camera frontend driver - UMS9230 platform
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/iopoll.h>

#include "camsys.h"
#include "dcam-regs-ums9230.h"

static void ums9230_dcam_set_path(struct sprd_dcam *dcam, u32 mbus_code,
				  u32 width, u32 height)
{
	u32 val, hwfmt, imgtype, bayer;

	switch (mbus_code) {
	default:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		hwfmt = DCAM_HWFMT_RAW_8;
		imgtype = DCAM_IMG_TYPE_RAW8;
		bayer = DCAM_BAYER_8BIT;
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		hwfmt = DCAM_HWFMT_RAW_PACK_10;
		imgtype = DCAM_IMG_TYPE_RAW10;
		bayer = DCAM_BAYER_10BIT;
		break;
	}

	switch (mbus_code) {
	default:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		bayer |= DCAM_BAYER_BGGR;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		bayer |= DCAM_BAYER_GBRG;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		bayer |= DCAM_BAYER_GRBG;
		break;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		bayer |= DCAM_BAYER_RGGB;
		break;
	}

	val = DCAM_MIPI_CAP_MODE | DCAM_MIPI_CAP_RAW;
	writel(val, dcam->regs + DCAM_MIPI_CAP_CFG);

	val = readl(dcam->regs + DCAM_BAYER_INFO_CFG);
	val &= ~(DCAM_BAYER_BITS | DCAM_BAYER_PATTERN);
	val |= bayer;
	writel(val, dcam->regs + DCAM_BAYER_INFO_CFG);

	val = FIELD_PREP(DCAM_DIM_Y, height - 1) |
	      FIELD_PREP(DCAM_DIM_X, width - 1);
	writel(val, dcam->regs + DCAM_MIPI_CAP_END);

	val = DCAM_FULL_SRC_SEL | FIELD_PREP(DCAM_FULL_HWFMT, hwfmt);
	writel(val, dcam->regs + DCAM_FULL_CFG);

	val = imgtype | FIELD_PREP(DCAM_IMAGE_MODE, 1);
	writel(val, dcam->regs + DCAM_IMAGE_CONTROL);
}

static void ums9230_dcam_config_ae(struct sprd_dcam *dcam,
				   union sprd_camsys_config_block block)
{
	struct sprd_camsys_ae_config *ae = block.ae;
	u32 val;

	if (block.header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		dcam->stats.blocks_enabled &= ~SPRD_CAMSYS_STATS_AE;
		return;
	}

	/* Protect the stats buffer from out-of-bounds access. */
	if (ae->blk_num_x > SPRD_CAMSYS_AE_MAX_BLOCKS_X ||
	    ae->blk_num_y > SPRD_CAMSYS_AE_MAX_BLOCKS_Y) {
		dev_dbg(dcam->dev,
			"not setting invalid AE config: %dx%d blocks\n",
			ae->blk_num_x, ae->blk_num_y);
		return;
	}

	dcam->stats.blocks_enabled |= SPRD_CAMSYS_STATS_AE;

	val = FIELD_PREP(DCAM_DIM_Y, ae->offset_y) |
	      FIELD_PREP(DCAM_DIM_X, ae->offset_x);
	writel(val, dcam->regs + DCAM_AEM_OFFSET);

	val = FIELD_PREP(DCAM_AEM_BLK_NUM_Y, ae->blk_num_y) |
	      FIELD_PREP(DCAM_AEM_BLK_NUM_X, ae->blk_num_x);
	writel(val, dcam->regs + DCAM_AEM_BLK_NUM);

	val = FIELD_PREP(DCAM_AEM_BLK_HEIGHT, ae->blk_height) |
	      FIELD_PREP(DCAM_AEM_BLK_WIDTH, ae->blk_width);
	writel(val, dcam->regs + DCAM_AEM_BLK_SIZE);

	val = FIELD_PREP(DCAM_AEM_RED_THR_LOW, ae->r_low) |
	      FIELD_PREP(DCAM_AEM_RED_THR_HIGH, ae->r_high);
	writel(val, dcam->regs + DCAM_AEM_RED_THR);

	val = FIELD_PREP(DCAM_AEM_GREEN_THR_LOW, ae->g_low) |
	      FIELD_PREP(DCAM_AEM_GREEN_THR_HIGH, ae->g_high);
	writel(val, dcam->regs + DCAM_AEM_GREEN_THR);

	val = FIELD_PREP(DCAM_AEM_BLUE_THR_LOW, ae->b_low) |
	      FIELD_PREP(DCAM_AEM_BLUE_THR_HIGH, ae->b_high);
	writel(val, dcam->regs + DCAM_AEM_BLUE_THR);
}

static void ums9230_dcam_config_af(struct sprd_dcam *dcam,
				   union sprd_camsys_config_block block)
{
	struct sprd_camsys_af_config *af = block.af;
	u32 val;

	if (block.header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		dcam->stats.blocks_enabled &= ~SPRD_CAMSYS_STATS_AF;
		return;
	}

	if (af->blk_num_x > SPRD_CAMSYS_AF_MAX_BLOCKS_X ||
	    af->blk_num_y > SPRD_CAMSYS_AF_MAX_BLOCKS_Y) {
		dev_dbg(dcam->dev,
			"not setting invalid AF config: %dx%d blocks\n",
			af->blk_num_x, af->blk_num_y);
		return;
	}

	dcam->stats.blocks_enabled |= SPRD_CAMSYS_STATS_AF;

	val = FIELD_PREP(DCAM_ISP_AFM_DONE_TILE_NUM_X, af->blk_num_x - 1) |
	      FIELD_PREP(DCAM_ISP_AFM_DONE_TILE_NUM_Y, af->blk_num_y - 1);
	writel(val, dcam->regs + DCAM_ISP_AFM_PARAMETERS);

	val = FIELD_PREP(DCAM_DIM_Y, af->offset_y) |
	      FIELD_PREP(DCAM_DIM_X, af->offset_x);
	writel(val, dcam->regs + DCAM_ISP_AFM_WIN_OFFSET);

	val = FIELD_PREP(DCAM_DIM_Y, af->blk_height) |
	      FIELD_PREP(DCAM_DIM_X, af->blk_width);
	writel(val, dcam->regs + DCAM_ISP_AFM_WIN_SIZE);

	val = FIELD_PREP(DCAM_DIM_Y, af->blk_num_y) |
	      FIELD_PREP(DCAM_DIM_X, af->blk_num_x);
	writel(val, dcam->regs + DCAM_ISP_AFM_WIN_NUM);
}

static void ums9230_dcam_config_awb(struct sprd_dcam *dcam,
				    union sprd_camsys_config_block block)
{
	struct sprd_camsys_awb_config *awb = block.awb;
	u32 val;

	if (block.header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		writel(DCAM_ISP_AWBC_BYPASS,
		       dcam->regs + DCAM_ISP_AWBC_GAIN0);
		return;
	}

	val = FIELD_PREP(DCAM_ISP_AWBC_GAIN_B, awb->gain_b) |
	      FIELD_PREP(DCAM_ISP_AWBC_GAIN_R, awb->gain_r);
	writel(val, dcam->regs + DCAM_ISP_AWBC_GAIN0);

	val = FIELD_PREP(DCAM_ISP_AWBC_GAIN_GB, awb->gain_gb) |
	      FIELD_PREP(DCAM_ISP_AWBC_GAIN_GR, awb->gain_gr);
	writel(val, dcam->regs + DCAM_ISP_AWBC_GAIN1);

	val = FIELD_PREP(DCAM_ISP_AWBC_OFFSET_B, awb->offset_b) |
	      FIELD_PREP(DCAM_ISP_AWBC_OFFSET_R, awb->offset_r);
	writel(val, dcam->regs + DCAM_ISP_AWBC_OFFSET0);

	val = FIELD_PREP(DCAM_ISP_AWBC_OFFSET_GB, awb->offset_gb) |
	      FIELD_PREP(DCAM_ISP_AWBC_OFFSET_GR, awb->offset_gr);
	writel(val, dcam->regs + DCAM_ISP_AWBC_OFFSET1);
}

static void ums9230_dcam_config_blc(struct sprd_dcam *dcam,
				    union sprd_camsys_config_block block)
{
	struct sprd_camsys_blc_config *blc = block.blc;
	u32 val;

	if (block.header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		writel(DCAM_BLC_BYPASS, dcam->regs + DCAM_BLC_PARA_R_B);
		return;
	}

	val = FIELD_PREP(DCAM_BLC_PARA_B, blc->b) |
	      FIELD_PREP(DCAM_BLC_PARA_R, blc->r);
	writel(val, dcam->regs + DCAM_BLC_PARA_R_B);

	val = FIELD_PREP(DCAM_BLC_PARA_GB, blc->gb) |
	      FIELD_PREP(DCAM_BLC_PARA_GR, blc->gr);
	writel(val, dcam->regs + DCAM_BLC_PARA_G);
}

static void ums9230_dcam_config_lsc(struct sprd_dcam *dcam,
				    union sprd_camsys_config_block block)
{
	u32 __iomem *weight_addr = dcam->regs + DCAM_LENS_WEIGHT_TABLE;
	struct sprd_camsys_lsc_config *lsc = block.lsc;
	unsigned int i, num_weights;
	u32 val;

	if (block.header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		writel(DCAM_LENS_BYPASS, dcam->regs + DCAM_LENS_LOAD_ENABLE);
		return;
	}

	num_weights = (lsc->blk_width / 2) + 1;
	if (num_weights > SPRD_CAMSYS_LSC_MAX_WEIGHTS) {
		dev_warn_ratelimited(dcam->dev,
				     "lsc blk_width (%d) out of range\n",
				     lsc->blk_width);
		return;
	}

	for (i = 0; i < num_weights; i++) {
		val = lsc->weights[i][0] | lsc->weights[i][1] << 16;
		writel(val, weight_addr++);
		val = lsc->weights[i][2];
		writel(val, weight_addr++);
	}

	val = lsc->blk_num_y * lsc->blk_num_x;
	writel(val, dcam->regs + DCAM_LENS_GRID_NUMBER);

	val = FIELD_PREP(DCAM_LENS_GRID_WIDTH, lsc->blk_width) |
	      FIELD_PREP(DCAM_LENS_GRID_Y_NUM, lsc->blk_num_y) |
	      FIELD_PREP(DCAM_LENS_GRID_X_NUM, lsc->blk_num_x);
	writel(val, dcam->regs + DCAM_LENS_GRID_SIZE);

	writel(lsc->blk_num_x, dcam->regs + DCAM_LENS_SLICE_CTRL1);
}

static void ums9230_dcam_check_lsc_loaded(struct sprd_dcam *dcam)
{
	u32 val = readl(dcam->regs + DCAM_LENS_LOAD_ENABLE);

	if (val & DCAM_LENS_LOAD_DONE) {
		writel(DCAM_LENS_LOAD_DONE_ACK,
		       dcam->regs + DCAM_LENS_LOAD_TRIGGER);
		sprd_lsc_done(&dcam->lsc, dcam->sequence);
		dev_dbg(dcam->dev, "lsc load done\n");
	}
}

static void ums9230_dcam_set_lsc_buf(struct sprd_dcam *dcam,
				     struct sprd_lsc_buffer *buf)
{
	u32 val;

	if (!buf)
		return;

	/*
	 * Enable internal SRAM access to allow loading the first buffer before
	 * streaming starts. This must be done after initializing the weights.
	 */
	writel(1, dcam->regs + DCAM_APB_SRAM_CTRL);

	writel(buf->addr, dcam->regs + DCAM_LENS_BASE_RADDR);

	val = DCAM_LENS_LOAD_START;
	writel(val, dcam->regs + DCAM_LENS_LOAD_TRIGGER);

	val = readl(dcam->regs + DCAM_LENS_LOAD_ENABLE);
	val ^= DCAM_LENS_LOAD_BUF_SEL;
	val &= ~DCAM_LENS_BYPASS;
	writel(val, dcam->regs + DCAM_LENS_LOAD_ENABLE);
}

static void ums9230_dcam_set_capture_buf(struct sprd_dcam *dcam,
					 struct sprd_capture_buffer *buf)
{
	u32 val;

	val = readl(dcam->regs + DCAM_FULL_CFG);
	if (buf)
		val |= DCAM_FULL_PATH_EN;
	else
		val &= ~DCAM_FULL_PATH_EN;
	writel(val, dcam->regs + DCAM_FULL_CFG);

	if (buf)
		writel(buf->addr, dcam->regs + DCAM_FULL_BASE_WADDR);
}

static void ums9230_dcam_set_stats_buf(struct sprd_dcam *dcam,
				       struct sprd_stats_buffer *buf)
{
	if (buf && (dcam->stats.blocks_enabled & SPRD_CAMSYS_STATS_AE)) {
		writel(0, dcam->regs + DCAM_AEM_FRM_CTRL0);
		writel(DCAM_AEM_FRM_SINGLE_START,
		       dcam->regs + DCAM_AEM_FRM_CTRL1);

		writel(buf->ae_addr, dcam->regs + DCAM_AEM_BASE_WADDR);
	} else {
		writel(DCAM_AEM_BYPASS, dcam->regs + DCAM_AEM_FRM_CTRL0);
	}

	if (buf && (dcam->stats.blocks_enabled & SPRD_CAMSYS_STATS_AF)) {
		writel(0, dcam->regs + DCAM_ISP_AFM_FRM_CTRL0);
		writel(DCAM_ISP_AFM_FRM_SINGLE_START,
		       dcam->regs + DCAM_ISP_AFM_FRM_CTRL1);

		writel(buf->af_addr, dcam->regs + DCAM_ISP_AFM_BASE_WADDR);
	} else {
		writel(DCAM_ISP_AFM_BYPASS, dcam->regs + DCAM_ISP_AFM_FRM_CTRL0);
	}
}

static void ums9230_dcam_start(struct sprd_dcam *dcam)
{
	u32 val;

	val = UMS9230_DCAM_IRQ_CAP_SOF |
	      UMS9230_DCAM_IRQ_DCAM_OVF |
	      UMS9230_DCAM_IRQ_CAP_LINE_ERR |
	      UMS9230_DCAM_IRQ_CAP_FRM_ERR |
	      UMS9230_DCAM_IRQ_FULL_PATH_TX_DONE |
	      UMS9230_DCAM_IRQ_AEM_TX_DONE |
	      UMS9230_DCAM_IRQ_AFM_ALL_DONE |
	      UMS9230_DCAM_IRQ_MMU_INT;
	writel(val, dcam->regs + DCAM_INT_EN);

	/* Update shadow registers now to start capturing the first frame. */
	writel(DCAM_FORCE_COPY_ALL, dcam->regs + DCAM_CONTROL);

	val = readl(dcam->regs + DCAM_MIPI_CAP_CFG);
	val |= DCAM_MIPI_CAP_EN;
	writel(val, dcam->regs + DCAM_MIPI_CAP_CFG);

	/* Enable the SRAM if it hasn't been enabled yet */
	writel(1, dcam->regs + DCAM_APB_SRAM_CTRL);
}

static void ums9230_dcam_stop(struct sprd_dcam *dcam)
{
	u32 val;
	int ret;

	writel(0, dcam->regs + DCAM_INT_EN);

	writel(0, dcam->regs + DCAM_MIPI_CAP_CFG);
	writel(0xffffffff, dcam->regs + DCAM_PATH_STOP);

	ret = readl_poll_timeout(dcam->regs + DCAM_PATH_BUSY, val, !val,
				 500, 2000000);
	if (ret)
		dev_warn(dcam->dev, "dcam%d stop timeout: %08x\n",
			 dcam->index, val);
}

static irqreturn_t ums9230_handle_irq(int irq, void *irqdata)
{
	struct sprd_dcam *dcam = irqdata;
	u32 val;

	val = readl(dcam->regs + DCAM_INT_MASK);
	writel(val, dcam->regs + DCAM_INT_CLR);

	dev_dbg(dcam->dev, "dcam%d IRQ: %08x\n", dcam->index, val);

	if (val & UMS9230_DCAM_IRQ_FULL_PATH_TX_DONE)
		sprd_capture_done(&dcam->capture, dcam->sequence - 1);

	if (val & UMS9230_DCAM_IRQ_AEM_TX_DONE)
		sprd_stats_done(&dcam->stats, dcam->sequence - 1,
				SPRD_CAMSYS_STATS_AE);

	if (val & UMS9230_DCAM_IRQ_AFM_ALL_DONE)
		sprd_stats_done(&dcam->stats, dcam->sequence - 1,
				SPRD_CAMSYS_STATS_AF);

	/*
	 * It is important to handle SOF after TX_DONE so that the buffers
	 * are always returned on time.
	 */
	if (val & UMS9230_DCAM_IRQ_CAP_SOF) {
		ums9230_dcam_check_lsc_loaded(dcam);
		sprd_dcam_sof(dcam);

		/*
		 * Tell the hardware to update the shadow registers once the
		 * next frame is ready to be processed.
		 */
		writel(DCAM_AUTO_COPY_ALL, dcam->regs + DCAM_CONTROL);
	}

	if (val & UMS9230_DCAM_IRQ_DCAM_OVF)
		dev_err(dcam->dev, "dcam%d overflow\n", dcam->index);
	if (val & UMS9230_DCAM_IRQ_CAP_LINE_ERR)
		dev_err(dcam->dev, "dcam%d line error\n", dcam->index);
	if (val & UMS9230_DCAM_IRQ_CAP_FRM_ERR)
		dev_err(dcam->dev, "dcam%d frame error\n", dcam->index);
	if (val & UMS9230_DCAM_IRQ_MMU_INT)
		dev_err(dcam->dev, "dcam%d mmu error\n", dcam->index);

	return IRQ_HANDLED;
}

const struct sprd_dcam_hw_ops ums9230_dcam_ops = {
	.config_block = {
		[SPRD_DCAM_BLOCK_AE] = ums9230_dcam_config_ae,
		[SPRD_DCAM_BLOCK_AF] = ums9230_dcam_config_af,
		[SPRD_DCAM_BLOCK_AWB] = ums9230_dcam_config_awb,
		[SPRD_DCAM_BLOCK_BLC] = ums9230_dcam_config_blc,
		[SPRD_DCAM_BLOCK_LSC] = ums9230_dcam_config_lsc,
	},

	.set_path = ums9230_dcam_set_path,
	.set_lsc_buf = ums9230_dcam_set_lsc_buf,
	.set_capture_buf = ums9230_dcam_set_capture_buf,
	.set_stats_buf = ums9230_dcam_set_stats_buf,
	.start = ums9230_dcam_start,
	.stop = ums9230_dcam_stop,
	.handle_irq = ums9230_handle_irq,
};
