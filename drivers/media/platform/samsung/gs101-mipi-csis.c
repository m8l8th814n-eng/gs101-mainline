// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs101 (Tensor) MIPI-CSIS receiver driver.
 *
 * Incremental mainline port of the CSIS v5.4 IP (gs101). Register map/sequences
 * derived from the Samsung "pablo" is-hw-csi-v5_4.* and the gs101 LWIS DT
 * (google gs101-isp.dtsi).
 *
 * Stage 3b-3-2b: resource bring-up + CSIS_VERSION + a V4L2 sub-device that
 * binds the sensor (media graph) + a vb2 video capture node (/dev/video). The
 * CSIS write-DMA register programming (raw Bayer -> the vb2 buffers) is added
 * in the following increment (3b-3-2c).
 *
 * Copyright 2026
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "gs101-mipi-csis-regs.h"

/* CSIS link register bank (per gs101-isp.dtsi reg-name "csis-link0"). */
#define CSIS_REG_VERSION	0x0000
#define CSIS_REG_CMN_CTRL	0x0004

/*
 * WDMA layout inside the csis-dma bank (0x1A4D0000), confirmed by probing the
 * live/dead (0xdeadc0de) register map on the SoC:
 *
 *   - 10 DMA contexts of 0x1000 each start at 0x1000 (ZSL0-2, STRP0-2,
 *     CSIS_DMA0-3). CSIS_DMA0 is at 0x7000, so a context base is
 *     0x7000 + dma_ch * 0x1000.
 *   - Within a context: VC channel n (CHX regs) at +n*0x100 (vc0..vc3), the
 *     ctl/common block (INT_ENABLE/INT_SRC/CMN_CTRL) at +0x400, and the input
 *     mux (CSIS link select) at +0x500.
 *
 * The sensors transmit on MIPI VC0, so capture uses channel 0 of the context.
 */
#define GS101_CSIS_DMA_CTX_BASE		0x7000	/* CSIS_DMA0 context */
#define GS101_CSIS_DMA_CTX_STRIDE	0x1000	/* per DMA context */
#define GS101_CSIS_DMA_VC_STRIDE	0x100	/* per VC channel in a context */
#define GS101_CSIS_DMA_CTL_OFFSET	0x400	/* ctl/common block in a context */
#define GS101_CSIS_DMA_MUX_OFFSET	0x500	/* input mux (link select) */

/* Capture uses virtual channel 0, MIPI CSI-2 data type RAW10 (0x2b). */
#define GS101_CSIS_VC0			0
#define GS101_CSIS_DT_RAW10		0x2b

/* Number of MIPI D/C-PHYs / CSIS links on gs101. */
#define GS101_CSIS_NUM_PHYS	8

/* CSIS sub-device pads. */
#define GS101_CSIS_PAD_SINK	0
#define GS101_CSIS_PAD_SOURCE	1
#define GS101_CSIS_PADS_NUM	2

#define GS101_CSIS_DEF_WIDTH	1920
#define GS101_CSIS_DEF_HEIGHT	1080

struct gs101_csis_buffer {
	struct vb2_v4l2_buffer	vb;
	struct list_head	list;
	dma_addr_t		addr;
};

struct gs101_csis {
	struct device		*dev;
	void __iomem		*link;		/* csis-link0 */
	void __iomem		*dma;		/* csis-dma (WDMA) */
	struct clk_bulk_data	*clks;
	int			num_clks;
	struct phy		*phys[GS101_CSIS_NUM_PHYS];
	int			num_phys;

	/* V4L2 / media */
	struct v4l2_device		v4l2_dev;
	struct media_device		mdev;
	struct v4l2_subdev		sd;
	struct media_pad		pads[GS101_CSIS_PADS_NUM];
	struct v4l2_async_notifier	notifier;
	struct v4l2_subdev		*sensor_sd;
	struct v4l2_mbus_framefmt	fmt;

	/* Video capture node */
	struct video_device		vdev;
	struct media_pad		vdev_pad;
	struct media_pipeline		pipe;
	struct vb2_queue		queue;
	struct v4l2_pix_format		pixfmt;
	struct list_head		buf_list;
	struct gs101_csis_buffer	*cur_buf;
	spinlock_t			buf_lock;	/* protects buf_list/cur_buf */
	struct mutex			lock;		/* serialises ioctls */
	int				irq;

	/*
	 * Each gs101 CSIS link is a separate DT node/driver instance sharing the
	 * one csis-dma bank. dma_ch selects this instance's WDMA context
	 * (CSIS_DMA0 for 1a440000, CSIS_DMA1 for 1a460000); its base is
	 * dma + GS101_CSIS_DMA_CTX_BASE + dma_ch * GS101_CSIS_DMA_CTX_STRIDE.
	 * link_idx is the CSIS link number (0, 2, ...) written to the context's
	 * input mux to route that link into this WDMA context. The sensors send
	 * on MIPI VC0, so capture uses channel 0 of the context.
	 */
	unsigned int			dma_ch;
	unsigned int			link_idx;

	/* MIPI CSI-2 data lanes for this instance (imx355=4, imx386=2). */
	unsigned int			lanes;

	/* Sensor link frequency (DDR clock) from the DT endpoint, for the PHY. */
	s64				link_freq;
};

static inline struct gs101_csis *sd_to_csis(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gs101_csis, sd);
}

/* ---------------- subdev ops ---------------- */

static int gs101_csis_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gs101_csis *csis = sd_to_csis(sd);

	if (!csis->sensor_sd)
		return -ENODEV;

	/* Pass-through to the sensor for now; CSIS/WDMA bring-up is next. */
	return v4l2_subdev_call(csis->sensor_sd, video, s_stream, enable);
}

static const struct v4l2_subdev_video_ops gs101_csis_video_ops = {
	.s_stream = gs101_csis_s_stream,
};

static int gs101_csis_init_state(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_state_get_format(state, GS101_CSIS_PAD_SINK);
	fmt->width = GS101_CSIS_DEF_WIDTH;
	fmt->height = GS101_CSIS_DEF_HEIGHT;
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;

	*v4l2_subdev_state_get_format(state, GS101_CSIS_PAD_SOURCE) = *fmt;

	return 0;
}

static int gs101_csis_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *sink;

	sink = v4l2_subdev_state_get_format(state, GS101_CSIS_PAD_SINK);
	if (format->pad == GS101_CSIS_PAD_SINK)
		*sink = format->format;
	else
		format->format = *sink;

	*v4l2_subdev_state_get_format(state, GS101_CSIS_PAD_SOURCE) = *sink;
	return 0;
}

static const struct v4l2_subdev_pad_ops gs101_csis_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gs101_csis_set_fmt,
};

static const struct v4l2_subdev_ops gs101_csis_subdev_ops = {
	.video	= &gs101_csis_video_ops,
	.pad	= &gs101_csis_pad_ops,
};

static const struct v4l2_subdev_internal_ops gs101_csis_internal_ops = {
	.init_state = gs101_csis_init_state,
};

/* ---------------- CSIS/WDMA hardware ---------------- */

/* Base of this instance's WDMA context within the shared csis-dma bank. */
static inline void __iomem *gs101_csis_ctx(struct gs101_csis *csis)
{
	return csis->dma + GS101_CSIS_DMA_CTX_BASE +
	       csis->dma_ch * GS101_CSIS_DMA_CTX_STRIDE;
}

static void gs101_csis_hw_set_dma_addr(struct gs101_csis *csis, dma_addr_t addr)
{
	/* Sensor sends MIPI VC0 -> channel 0 of the context. */
	void __iomem *vc = gs101_csis_ctx(csis);
	u32 seq;

	is_hw_set_reg(vc, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_ADDR1],
		      lower_32_bits(addr));
	seq = is_hw_get_reg(vc, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FCNTSEQ]);
	is_hw_set_reg(vc, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FCNTSEQ], seq | 1);
}

static void gs101_csis_hw_start(struct gs101_csis *csis, dma_addr_t addr)
{
	void __iomem *link = csis->link;
	void __iomem *ctx = gs101_csis_ctx(csis);
	void __iomem *vc0 = ctx;				/* channel 0 */
	void __iomem *ctl = ctx + GS101_CSIS_DMA_CTL_OFFSET;
	unsigned int cfg = CSIS_R_ISP_CONFIG_CH0;	/* MIPI VC0 */
	unsigned int resol = CSIS_R_ISP_RESOL_CH0;
	u32 w = csis->pixfmt.width;
	u32 h = csis->pixfmt.height;
	u32 val;

	/* Link soft reset. */
	is_hw_set_field(link, &csi_regs[CSIS_R_CSIS_CMN_CTRL],
			&csi_fields[CSIS_F_SW_RESET], 1);
	udelay(20);

	/* LANE_NUMBER = lanes - 1; enable one data-lane bit per lane. */
	is_hw_set_field(link, &csi_regs[CSIS_R_CSIS_CMN_CTRL],
			&csi_fields[CSIS_F_LANE_NUMBER], csis->lanes - 1);
	is_hw_set_field(link, &csi_regs[CSIS_R_PHY_CMN_CTRL],
			&csi_fields[CSIS_F_ENABLE_DAT],
			(1 << csis->lanes) - 1);

	/* VC input config: RAW10, single pixel mode, resolution. */
	val = is_hw_get_reg(link, &csi_regs[cfg]);
	val = is_hw_set_field_value(val, &csi_fields[CSIS_F_VIRTUAL_CHANNEL],
				    GS101_CSIS_VC0);
	val = is_hw_set_field_value(val, &csi_fields[CSIS_F_DATAFORMAT],
				    GS101_CSIS_DT_RAW10);
	val = is_hw_set_field_value(val, &csi_fields[CSIS_F_PIXEL_MODE],
				    CSIS_PIXEL_MODE_SING);
	is_hw_set_reg(link, &csi_regs[cfg], val);

	val = is_hw_get_reg(link, &csi_regs[resol]);
	val = is_hw_set_field_value(val, &csi_fields[CSIS_F_HRESOL], w);
	val = is_hw_set_field_value(val, &csi_fields[CSIS_F_VRESOL], h);
	is_hw_set_reg(link, &csi_regs[resol], val);

	/*
	 * WDMA input mux: route this CSIS link into the context (pablo writes
	 * the link index to regs_mux before configuring the DMA).
	 */
	writel(csis->link_idx, ctx + GS101_CSIS_DMA_MUX_OFFSET);

	/* WDMA VC0: 2D, RAW10 unpacked to 16-bit, resolution, stride. */
	val = is_hw_get_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FMT]);
	val = is_hw_set_field_value(val, &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DIM],
				    CSIS_REG_DMA_2D_DMA);
	val = is_hw_set_field_value(val,
				    &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DATAFORMAT],
				    CSIS_DMA_FMT_U10BIT_UNPACK_MSB_ZERO);
	is_hw_set_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FMT], val);

	val = is_hw_get_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_RESOL]);
	val = is_hw_set_field_value(val, &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_HRESOL], w);
	val = is_hw_set_field_value(val, &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_VRESOL], h);
	is_hw_set_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_RESOL], val);

	is_hw_set_field(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_STRIDE],
			&csi_dmax_chx_fields[CSIS_DMAX_CHX_F_STRIDE],
			csis->pixfmt.bytesperline);

	gs101_csis_hw_set_dma_addr(csis, addr);

	is_hw_set_field(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_CTRL],
			&csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DMA_ENABLE], 1);

	/* Enable DMA frame interrupts. */
	is_hw_set_reg(ctl, &csi_dmax_regs[CSIS_DMAX_R_INT_ENABLE],
		      CSIS_DMA_IRQ_MASK);

	/* Latch shadow registers, turn the PHY clock lane + CSI core on. */
	is_hw_set_field(link, &csi_regs[CSIS_R_CSIS_UPD_SDW],
			&csi_fields[CSIS_F_UPDATE_SHADOW], 0xF);
	is_hw_set_field(link, &csi_regs[CSIS_R_PHY_CMN_CTRL],
			&csi_fields[CSIS_F_ENABLE_CLK], 1);
	is_hw_set_field(link, &csi_regs[CSIS_R_CSIS_CMN_CTRL],
			&csi_fields[CSIS_F_CSI_EN], 1);
}

static void gs101_csis_hw_stop(struct gs101_csis *csis)
{
	void __iomem *link = csis->link;
	void __iomem *ctx = gs101_csis_ctx(csis);
	void __iomem *vc0 = ctx;				/* channel 0 */
	void __iomem *ctl = ctx + GS101_CSIS_DMA_CTL_OFFSET;

	/*
	 * Debug: FRM_CNT > 0 means the link received frames from the sensor
	 * (PHY + sensor OK); the DMA ctl INT_SRC and the channel VCNT show
	 * whether the WDMA wrote anything.
	 */
	dev_info(csis->dev,
		 "stop dump: PHY_STATUS=0x%08x FRM_CNT=0x%08x INT_SRC0=0x%08x ERR_FS=0x%08x ERR_FE=0x%08x\n",
		 is_hw_get_reg(link, &csi_regs[CSIS_R_PHY_STATUS]),
		 is_hw_get_reg(link, &csi_regs[CSIS_R_FRM_CNT_CH0]),
		 is_hw_get_reg(link, &csi_regs[CSIS_R_CSIS_INT_SRC0]),
		 is_hw_get_reg(link, &csi_regs[CSIS_R_ERR_LOST_FS]),
		 is_hw_get_reg(link, &csi_regs[CSIS_R_ERR_LOST_FE]));
	dev_info(csis->dev,
		 "stop dump: DMA INT_SRC=0x%08x CHX_CTRL=0x%08x CHX_ADDR1=0x%08x MUX=0x%08x\n",
		 is_hw_get_reg(ctl, &csi_dmax_regs[CSIS_DMAX_R_INT_SRC]),
		 is_hw_get_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_CTRL]),
		 is_hw_get_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_ADDR1]),
		 readl(ctx + GS101_CSIS_DMA_MUX_OFFSET));

	is_hw_set_field(link, &csi_regs[CSIS_R_CSIS_CMN_CTRL],
			&csi_fields[CSIS_F_CSI_EN], 0);
	is_hw_set_field(link, &csi_regs[CSIS_R_PHY_CMN_CTRL],
			&csi_fields[CSIS_F_ENABLE_CLK], 0);
	is_hw_set_field(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_CTRL],
			&csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DMA_ENABLE], 0);
	is_hw_set_reg(ctl, &csi_dmax_regs[CSIS_DMAX_R_INT_ENABLE], 0);
}

static irqreturn_t gs101_csis_irq(int irq, void *data)
{
	struct gs101_csis *csis = data;
	void __iomem *ctl = gs101_csis_ctx(csis) + GS101_CSIS_DMA_CTL_OFFSET;
	u32 src;

	src = is_hw_get_reg(ctl, &csi_dmax_regs[CSIS_DMAX_R_INT_SRC]);
	if (!src)
		return IRQ_NONE;
	is_hw_set_reg(ctl, &csi_dmax_regs[CSIS_DMAX_R_INT_SRC], src);	/* clear */

	/* Sensor on MIPI VC0 -> channel 0 frame-end. */
	if (src & (1 << (CSIS_INT_DMA_FRAME_END + GS101_CSIS_VC0))) {
		struct gs101_csis_buffer *done, *next;

		spin_lock(&csis->buf_lock);
		done = csis->cur_buf;
		next = list_first_entry_or_null(&csis->buf_list,
						struct gs101_csis_buffer, list);
		if (next) {
			list_del(&next->list);
			gs101_csis_hw_set_dma_addr(csis, next->addr);
		}
		csis->cur_buf = next;
		spin_unlock(&csis->buf_lock);

		if (done) {
			done->vb.vb2_buf.timestamp = ktime_get_ns();
			done->vb.sequence = 0;
			done->vb.field = V4L2_FIELD_NONE;
			vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
	}

	return IRQ_HANDLED;
}

/* ---------------- vb2 ops ---------------- */

static int gs101_csis_queue_setup(struct vb2_queue *q,
				  unsigned int *num_buffers,
				  unsigned int *num_planes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct gs101_csis *csis = vb2_get_drv_priv(q);
	unsigned int size = csis->pixfmt.sizeimage;

	if (*num_planes) {
		if (*num_planes != 1 || sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*num_planes = 1;
	sizes[0] = size;
	return 0;
}

static int gs101_csis_buf_prepare(struct vb2_buffer *vb)
{
	struct gs101_csis *csis = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < csis->pixfmt.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, csis->pixfmt.sizeimage);
	return 0;
}

static void gs101_csis_buf_queue(struct vb2_buffer *vb)
{
	struct gs101_csis *csis = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct gs101_csis_buffer *buf =
		container_of(vbuf, struct gs101_csis_buffer, vb);
	unsigned long flags;

	buf->addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	spin_lock_irqsave(&csis->buf_lock, flags);
	list_add_tail(&buf->list, &csis->buf_list);
	spin_unlock_irqrestore(&csis->buf_lock, flags);
}

static void gs101_csis_return_buffers(struct gs101_csis *csis,
				      enum vb2_buffer_state state)
{
	struct gs101_csis_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&csis->buf_lock, flags);
	if (csis->cur_buf) {
		vb2_buffer_done(&csis->cur_buf->vb.vb2_buf, state);
		csis->cur_buf = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &csis->buf_list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&csis->buf_lock, flags);
}

static int gs101_csis_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct gs101_csis *csis = vb2_get_drv_priv(q);
	struct gs101_csis_buffer *first;
	unsigned long flags;
	int ret;

	ret = video_device_pipeline_start(&csis->vdev, &csis->pipe);
	if (ret)
		goto err_return;

	/* Bring up the D-PHY for the active link (link0 for now). */
	if (csis->num_phys) {
		union phy_configure_opts opts = { };

		ret = phy_init(csis->phys[0]);
		if (ret)
			goto err_stop_pipe;
		ret = phy_power_on(csis->phys[0]);
		if (ret) {
			phy_exit(csis->phys[0]);
			goto err_stop_pipe;
		}
		/*
		 * DISABLED AGAIN: even with the SYSREG reset-deassert added to
		 * phy_power_on(), phy_configure() still hangs the kernel at stream
		 * time. Do NOT re-enable by guessing -- next attempt must be
		 * guided by the pstore/ramoops crash trace (which write hangs:
		 * reset reg, bias ioremap 0x1A4F1000, or a lane reg).
		 */
		opts.mipi_dphy.lanes = csis->lanes;
		opts.mipi_dphy.hs_clk_rate = csis->link_freq * 2;
		(void)opts;
	}

	/* Take the first queued buffer as the DMA target. */
	spin_lock_irqsave(&csis->buf_lock, flags);
	first = list_first_entry_or_null(&csis->buf_list,
					 struct gs101_csis_buffer, list);
	if (first) {
		list_del(&first->list);
		csis->cur_buf = first;
	}
	spin_unlock_irqrestore(&csis->buf_lock, flags);

	if (!first) {
		ret = -EINVAL;
		goto err_phy;
	}

	gs101_csis_hw_start(csis, first->addr);

	ret = v4l2_subdev_call(&csis->sd, video, s_stream, 1);
	if (ret && ret != -ENOIOCTLCMD)
		goto err_hw_stop;

	return 0;

err_hw_stop:
	gs101_csis_hw_stop(csis);
err_phy:
	if (csis->num_phys) {
		phy_power_off(csis->phys[0]);
		phy_exit(csis->phys[0]);
	}
err_stop_pipe:
	video_device_pipeline_stop(&csis->vdev);
err_return:
	gs101_csis_return_buffers(csis, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void gs101_csis_stop_streaming(struct vb2_queue *q)
{
	struct gs101_csis *csis = vb2_get_drv_priv(q);

	v4l2_subdev_call(&csis->sd, video, s_stream, 0);
	gs101_csis_hw_stop(csis);
	if (csis->num_phys) {
		phy_power_off(csis->phys[0]);
		phy_exit(csis->phys[0]);
	}
	video_device_pipeline_stop(&csis->vdev);
	gs101_csis_return_buffers(csis, VB2_BUF_STATE_ERROR);
	csis->cur_buf = NULL;
}

static const struct vb2_ops gs101_csis_vb2_ops = {
	.queue_setup		= gs101_csis_queue_setup,
	.buf_prepare		= gs101_csis_buf_prepare,
	.buf_queue		= gs101_csis_buf_queue,
	.start_streaming	= gs101_csis_start_streaming,
	.stop_streaming		= gs101_csis_stop_streaming,
};

/* ---------------- v4l2 ioctl ops ---------------- */

static void gs101_csis_set_pixfmt(struct gs101_csis *csis,
				  struct v4l2_pix_format *pf)
{
	pf->pixelformat = V4L2_PIX_FMT_SRGGB10;
	pf->field = V4L2_FIELD_NONE;
	pf->colorspace = V4L2_COLORSPACE_RAW;
	if (!pf->width)
		pf->width = GS101_CSIS_DEF_WIDTH;
	if (!pf->height)
		pf->height = GS101_CSIS_DEF_HEIGHT;
	/* RAW10 packed as 16-bit per pixel for the WDMA output. */
	pf->bytesperline = pf->width * 2;
	pf->sizeimage = pf->bytesperline * pf->height;
}

static int gs101_csis_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, "gs101-mipi-csis", sizeof(cap->driver));
	strscpy(cap->card, "gs101-mipi-csis", sizeof(cap->card));
	return 0;
}

static int gs101_csis_enum_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_SRGGB10;
	return 0;
}

static int gs101_csis_g_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct gs101_csis *csis = video_drvdata(file);

	f->fmt.pix = csis->pixfmt;
	return 0;
}

static int gs101_csis_try_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	gs101_csis_set_pixfmt(video_drvdata(file), &f->fmt.pix);
	return 0;
}

static int gs101_csis_s_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct gs101_csis *csis = video_drvdata(file);

	if (vb2_is_busy(&csis->queue))
		return -EBUSY;

	gs101_csis_set_pixfmt(csis, &f->fmt.pix);
	csis->pixfmt = f->fmt.pix;
	return 0;
}

static int gs101_csis_enum_input(struct file *file, void *priv,
				 struct v4l2_input *inp)
{
	if (inp->index)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(inp->name, "camera", sizeof(inp->name));
	return 0;
}

static int gs101_csis_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int gs101_csis_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops gs101_csis_ioctl_ops = {
	.vidioc_querycap		= gs101_csis_querycap,
	.vidioc_enum_input		= gs101_csis_enum_input,
	.vidioc_g_input			= gs101_csis_g_input,
	.vidioc_s_input			= gs101_csis_s_input,
	.vidioc_enum_fmt_vid_cap	= gs101_csis_enum_fmt,
	.vidioc_g_fmt_vid_cap		= gs101_csis_g_fmt,
	.vidioc_try_fmt_vid_cap		= gs101_csis_try_fmt,
	.vidioc_s_fmt_vid_cap		= gs101_csis_s_fmt,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations gs101_csis_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.read		= vb2_fop_read,
	.unlocked_ioctl	= video_ioctl2,
};

static int gs101_csis_register_video(struct gs101_csis *csis)
{
	struct video_device *vdev = &csis->vdev;
	struct vb2_queue *q = &csis->queue;
	int ret;

	mutex_init(&csis->lock);
	spin_lock_init(&csis->buf_lock);
	INIT_LIST_HEAD(&csis->buf_list);

	csis->pixfmt.width = GS101_CSIS_DEF_WIDTH;
	csis->pixfmt.height = GS101_CSIS_DEF_HEIGHT;
	gs101_csis_set_pixfmt(csis, &csis->pixfmt);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = csis;
	q->ops = &gs101_csis_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct gs101_csis_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &csis->lock;
	q->dev = csis->dev;
	q->min_queued_buffers = 2;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	csis->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, &csis->vdev_pad);
	if (ret)
		return ret;

	/*
	 * Include the bound sensor so the two capture nodes are distinguishable
	 * in `v4l2-ctl --list-devices` (the /dev/videoN numbering is assigned in
	 * probe order and is not stable across boots).
	 */
	snprintf(vdev->name, sizeof(vdev->name), "gs101-csis %s",
		 csis->sensor_sd ? csis->sensor_sd->name : dev_name(csis->dev));
	vdev->fops = &gs101_csis_fops;
	vdev->ioctl_ops = &gs101_csis_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &csis->lock;
	vdev->queue = q;
	vdev->v4l2_dev = &csis->v4l2_dev;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	video_set_drvdata(vdev, csis);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	/* CSIS source pad -> video sink pad. */
	ret = media_create_pad_link(&csis->sd.entity, GS101_CSIS_PAD_SOURCE,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		video_unregister_device(vdev);

	return ret;
}

/* ---------------- async notifier ---------------- */

static int gs101_csis_notify_bound(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *subdev,
				   struct v4l2_async_connection *asc)
{
	struct gs101_csis *csis =
		container_of(notifier, struct gs101_csis, notifier);

	csis->sensor_sd = subdev;
	dev_info(csis->dev, "bound sensor %s\n", subdev->name);
	return 0;
}

static int gs101_csis_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct gs101_csis *csis =
		container_of(notifier, struct gs101_csis, notifier);
	struct v4l2_subdev *sensor = csis->sensor_sd;
	int ret, source_pad;

	if (!sensor)
		return -ENODEV;

	source_pad = media_entity_get_fwnode_pad(&sensor->entity,
						 sensor->fwnode,
						 MEDIA_PAD_FL_SOURCE);
	if (source_pad < 0) {
		dev_err(csis->dev, "no source pad on sensor\n");
		return source_pad;
	}

	ret = media_create_pad_link(&sensor->entity, source_pad,
				    &csis->sd.entity, GS101_CSIS_PAD_SINK,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(csis->dev, "failed to create sensor link\n");
		return ret;
	}

	ret = gs101_csis_register_video(csis);
	if (ret)
		return ret;

	ret = v4l2_device_register_subdev_nodes(&csis->v4l2_dev);
	if (ret)
		return ret;

	return media_device_register(&csis->mdev);
}

static const struct v4l2_async_notifier_operations gs101_csis_notify_ops = {
	.bound		= gs101_csis_notify_bound,
	.complete	= gs101_csis_notify_complete,
};

static int gs101_csis_parse_dt(struct gs101_csis *csis)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *ep;
	struct v4l2_async_connection *asc;
	int ret;

	/*
	 * v4l2_async_nf_add_fwnode_remote() takes the LOCAL endpoint (our sink)
	 * and resolves the remote (sensor) itself.
	 */
	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(csis->dev),
					     GS101_CSIS_PAD_SINK, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return dev_err_probe(csis->dev, -EINVAL,
				     "no sink endpoint (sensor) in DT\n");

	/* Data-lane count for the link programming (defaults to 4). */
	if (v4l2_fwnode_endpoint_parse(ep, &vep) == 0 &&
	    vep.bus.mipi_csi2.num_data_lanes)
		csis->lanes = vep.bus.mipi_csi2.num_data_lanes;
	else
		csis->lanes = 4;

	/* Sensor link (DDR) frequency for the D-PHY config; default 360 MHz. */
	if (fwnode_property_read_u64_array(ep, "link-frequencies",
					   (u64 *)&csis->link_freq, 1))
		csis->link_freq = 360000000;

	asc = v4l2_async_nf_add_fwnode_remote(&csis->notifier, ep,
					      struct v4l2_async_connection);
	fwnode_handle_put(ep);
	if (IS_ERR(asc))
		return PTR_ERR(asc);

	ret = v4l2_async_nf_register(&csis->notifier);
	if (ret)
		dev_err(csis->dev, "failed to register async notifier\n");

	return ret;
}

/* ---------------- resources / probe ---------------- */

static int gs101_csis_get_phys(struct gs101_csis *csis)
{
	struct device *dev = csis->dev;
	int i, count;

	count = of_count_phandle_with_args(dev->of_node, "phys", "#phy-cells");
	if (count <= 0)
		return 0;
	if (count > GS101_CSIS_NUM_PHYS)
		count = GS101_CSIS_NUM_PHYS;

	for (i = 0; i < count; i++) {
		csis->phys[i] = devm_of_phy_get_by_index(dev, dev->of_node, i);
		if (IS_ERR(csis->phys[i]))
			return dev_err_probe(dev, PTR_ERR(csis->phys[i]),
					     "failed to get phy %d\n", i);
	}
	csis->num_phys = count;
	return 0;
}

static int gs101_csis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs101_csis *csis;
	struct resource *res;
	int ret;

	csis = devm_kzalloc(dev, sizeof(*csis), GFP_KERNEL);
	if (!csis)
		return -ENOMEM;

	csis->dev = dev;
	platform_set_drvdata(pdev, csis);

	/*
	 * reg[0] is this instance's CSIS link bank (link N = 0x1A440000 +
	 * N * 0x10000). Each gs101 CSIS link is separate hardware and is
	 * modelled as its own DT node (imx355 on link0, imx386 on link2), so
	 * the link is taken by index rather than by a fixed "csis-link0" name.
	 * link_idx (derived from the bank address) is written to the WDMA input
	 * mux to route this link into the DMA context.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no link reg\n");
	csis->link_idx = (res->start - 0x1A440000) / 0x10000;
	csis->link = devm_ioremap_resource(dev, res);
	if (IS_ERR(csis->link))
		return PTR_ERR(csis->link);

	/*
	 * The csis-dma (WDMA) bank is shared by all link instances: each writes
	 * through its own DMA context. Map it non-exclusively (devm_ioremap, not
	 * devm_ioremap_resource) so a second instance can map it too.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csis-dma");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no csis-dma reg\n");
	csis->dma = devm_ioremap(dev, res->start, resource_size(res));
	if (!csis->dma)
		return -ENOMEM;

	/* WDMA context for this instance (CSIS_DMA0/1); DT prop kept as -vc. */
	of_property_read_u32(dev->of_node, "google,csis-dma-vc", &csis->dma_ch);
	if (csis->dma_ch > 3)
		return dev_err_probe(dev, -EINVAL, "invalid csis-dma-ch %u\n",
				     csis->dma_ch);

	/*
	 * The csis-phy register bank (0x1A4F0000) belongs to the MIPI D/C-PHY
	 * driver (phy-exynos-mipi-gs101); the CSIS reaches the PHY through the
	 * generic PHY framework, so it must not ioremap that bank itself.
	 */
	csis->num_clks = devm_clk_bulk_get_all(dev, &csis->clks);
	if (csis->num_clks < 0)
		return dev_err_probe(dev, csis->num_clks,
				     "failed to get clocks\n");

	ret = gs101_csis_get_phys(csis);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/*
	 * Attach the dedicated CMA carveout (DT "memory-region") so the large
	 * per-frame vb2 buffers (e.g. 3280x2464x2 ~16 MB for imx355) come from
	 * a pool big enough for them, not the small fragmented shared CMA.
	 * Non-fatal: without a memory-region the device uses the default CMA.
	 */
	ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, 0);
	if (!ret)
		dev_info(dev, "using reserved CMA memory-region for buffers\n");
	else if (ret != -ENODEV)
		dev_warn(dev, "no reserved memory region (%d), using default CMA\n",
			 ret);
	else
		dev_info(dev, "no memory-region in DT, using default CMA\n");

	/*
	 * Enable the CSIS bus clock so the register banks are accessible. The
	 * gs101 CSIS power domain is left on by the bootloader (PMU CSIS status
	 * bit set), and mainline does not model/gate it, so it stays powered.
	 */
	ret = clk_bulk_prepare_enable(csis->num_clks, csis->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	dev_info(dev, "CSIS version 0x%08x\n",
		 readl(csis->link + CSIS_REG_VERSION));

	csis->irq = platform_get_irq_byname_optional(pdev, "csis-dma0");
	if (csis->irq < 0)
		csis->irq = platform_get_irq(pdev, 0);
	if (csis->irq < 0) {
		ret = csis->irq;
		clk_bulk_disable_unprepare(csis->num_clks, csis->clks);
		return ret;
	}
	ret = devm_request_irq(dev, csis->irq, gs101_csis_irq, 0,
			       "gs101-mipi-csis", csis);
	if (ret) {
		dev_err(dev, "failed to request IRQ\n");
		clk_bulk_disable_unprepare(csis->num_clks, csis->clks);
		return ret;
	}

	/* Media device. */
	csis->mdev.dev = dev;
	strscpy(csis->mdev.model, "gs101-mipi-csis", sizeof(csis->mdev.model));
	media_device_init(&csis->mdev);

	csis->v4l2_dev.mdev = &csis->mdev;
	ret = v4l2_device_register(dev, &csis->v4l2_dev);
	if (ret)
		goto err_media_cleanup;

	/* CSIS sub-device. */
	v4l2_subdev_init(&csis->sd, &gs101_csis_subdev_ops);
	csis->sd.owner = THIS_MODULE;
	csis->sd.dev = dev;
	csis->sd.internal_ops = &gs101_csis_internal_ops;
	csis->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(csis->sd.name, sizeof(csis->sd.name), "gs101-csis");
	csis->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	csis->pads[GS101_CSIS_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csis->pads[GS101_CSIS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&csis->sd.entity, GS101_CSIS_PADS_NUM,
				     csis->pads);
	if (ret)
		goto err_v4l2_unreg;

	ret = v4l2_subdev_init_finalize(&csis->sd);
	if (ret)
		goto err_entity_cleanup;

	ret = v4l2_device_register_subdev(&csis->v4l2_dev, &csis->sd);
	if (ret)
		goto err_subdev_cleanup;

	/* Async notifier for the sensor. */
	v4l2_async_nf_init(&csis->notifier, &csis->v4l2_dev);
	csis->notifier.ops = &gs101_csis_notify_ops;

	ret = gs101_csis_parse_dt(csis);
	if (ret)
		goto err_nf_cleanup;

	dev_info(dev, "gs101 MIPI-CSIS bound: %d clocks, %d phys\n",
		 csis->num_clks, csis->num_phys);

	return 0;

err_nf_cleanup:
	v4l2_async_nf_cleanup(&csis->notifier);
	v4l2_device_unregister_subdev(&csis->sd);
err_subdev_cleanup:
	v4l2_subdev_cleanup(&csis->sd);
err_entity_cleanup:
	media_entity_cleanup(&csis->sd.entity);
err_v4l2_unreg:
	v4l2_device_unregister(&csis->v4l2_dev);
err_media_cleanup:
	media_device_cleanup(&csis->mdev);
	clk_bulk_disable_unprepare(csis->num_clks, csis->clks);
	of_reserved_mem_device_release(dev);
	return ret;
}

static void gs101_csis_remove(struct platform_device *pdev)
{
	struct gs101_csis *csis = platform_get_drvdata(pdev);

	media_device_unregister(&csis->mdev);
	video_unregister_device(&csis->vdev);
	v4l2_async_nf_unregister(&csis->notifier);
	v4l2_async_nf_cleanup(&csis->notifier);
	v4l2_device_unregister_subdev(&csis->sd);
	v4l2_subdev_cleanup(&csis->sd);
	media_entity_cleanup(&csis->sd.entity);
	v4l2_device_unregister(&csis->v4l2_dev);
	media_device_cleanup(&csis->mdev);
	clk_bulk_disable_unprepare(csis->num_clks, csis->clks);
	of_reserved_mem_device_release(&pdev->dev);
}

static const struct of_device_id gs101_csis_of_match[] = {
	{ .compatible = "google,gs101-mipi-csis" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs101_csis_of_match);

static struct platform_driver gs101_csis_driver = {
	.probe	= gs101_csis_probe,
	.remove	= gs101_csis_remove,
	.driver	= {
		.name		= "gs101-mipi-csis",
		.of_match_table	= gs101_csis_of_match,
	},
};
module_platform_driver(gs101_csis_driver);

MODULE_DESCRIPTION("Google gs101 MIPI-CSIS receiver driver");
MODULE_LICENSE("GPL");
