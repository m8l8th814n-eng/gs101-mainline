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

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>
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
#define GS101_CSIS_DMA_MUX_OFFSET	0x500	/* input mux (link select) -- WRONG register, see hw_start */

/*
 * SYSREG_CSIS (0x1A420000) WDMA routing, as the stock HAL programs it
 * (liblyric csi_context.cc, bank 0xb): a "slot" register per link input
 * (0x430 + slot * 4 = csis link number), a per-WDMA-context mux (0x408 +
 * ctx * 4 = slot) and one enable bit per (link, ctx) pair in 0x488
 * (1 << (link + ctx * 8)). The link index alone is not enough for the
 * receiver to reach the DMA. We use slot = ctx.
 */
#define GS101_SYSREG_CSIS_LINK_SLOT(slot)	(0x430 + (slot) * 4)
#define GS101_SYSREG_CSIS_DMA_MUX(ctx)		(0x408 + (ctx) * 4)
#define GS101_SYSREG_CSIS_DMA_EN		0x488

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

/*
 * EBUF handling at stream start: 0 = program it like pablo (default),
 * 1 = bypass it. Runtime switch so both can be tried on one build
 * (/sys/module/gs101_mipi_csis/parameters/ebuf_bypass).
 */
static bool gs101_csis_ebuf_bypass = true;	/* 2026-09-19: only bypass captures */
module_param_named(ebuf_bypass, gs101_csis_ebuf_bypass, bool, 0644);
MODULE_PARM_DESC(ebuf_bypass, "bypass the CSIS elastic buffer (default 1; 0 = program it like pablo)");

/*
 * Bring-up knobs (2026-09-19): the stock HAL writes csis_dma0_ch0_fmt = 0x1e,
 * which does not decode in the pablo v5.4 field layout we use (DATAFORMAT
 * 6 = U10 unpack MSB zero, 2D), and pablo sets the link's PIXEL_MODE to
 * QUAD for the OTF path while we use SINGLE. Both only take effect before
 * the first frame, so they are runtime parameters rather than rebuilds.
 */
static unsigned int gs101_csis_dma_fmt = 6;
module_param_named(dma_fmt, gs101_csis_dma_fmt, uint, 0644);
MODULE_PARM_DESC(dma_fmt, "raw value for the WDMA channel FMT register (default 6)");

static unsigned int gs101_csis_pixel_mode = 2;	/* 2026-09-19: QUAD = 3/3 captures, single = link overflow */
module_param_named(pixel_mode, gs101_csis_pixel_mode, uint, 0644);
MODULE_PARM_DESC(pixel_mode, "link ISP_CONFIG PIXEL_MODE: 0 single, 1 dual, 2 quad (default 2)");

/*
 * WDMA context override (-1 = the DT google,csis-dma-vc). 2026-09-19: the
 * rear on context 1 gets frame-ends but all-zero buffers while the front on
 * context 0 captures; running the rear on context 0 tells whether it is the
 * context (its AXI port) or the link. Only one instance may stream while set.
 */
static int gs101_csis_dma_ctx;	/* 2026-09-19: rear on ctx0 captures, on ctx1 all zeros -> 0 for now */
module_param_named(dma_ctx, gs101_csis_dma_ctx, int, 0644);
MODULE_PARM_DESC(dma_ctx, "force WDMA context 0..3 for every instance (default 0; -1 = DT)");

/*
 * EBUFn_CTRL_EN raw value when the EBUF is programmed (ebuf_bypass=0).
 * pablo sets only ABORT_CTRL_EN (bit1) = 0x2; the stock HAL builds its
 * ebuf0_ctrl_en from the constant 0x2c (csi_context.cc, decomp 2026-09-19).
 */
static unsigned int gs101_csis_ebuf_ctrl_en = 0x2;
module_param_named(ebuf_ctrl_en, gs101_csis_ebuf_ctrl_en, uint, 0644);
MODULE_PARM_DESC(ebuf_ctrl_en, "raw EBUFn_CTRL_EN value (default 0x2, HAL uses 0x2c)");

/*
 * SYSREG_CSIS routing slot (EBUF channel) to use; -1 = the instance's WDMA
 * context number (front: 0, rear: 1). 2026-09-19: the rear on slot/ctx 1 got
 * no frames at all in EBUF bypass while the front on slot/ctx 0 captured, so
 * slot 0 for a single stream is a thing to try before blaming ctx1.
 */
static int gs101_csis_sysreg_slot;	/* 2026-09-19: slot 1 routed nothing, slot 0 captured the rear */
module_param_named(sysreg_slot, gs101_csis_sysreg_slot, int, 0644);
MODULE_PARM_DESC(sysreg_slot, "SYSREG_CSIS routing slot (default 0; -1 = WDMA context number)");

/* WDMA DATA_CTRL input path for channel 0: 0 = OTF (pablo default), 1 = PRL. */
static bool gs101_csis_dma_input_prl;
module_param_named(dma_input_prl, gs101_csis_dma_input_prl, bool, 0644);
MODULE_PARM_DESC(dma_input_prl, "WDMA ch0 input path 1 = parallel, 0 = OTF (default)");

struct gs101_csis {
	struct device		*dev;
	void __iomem		*link;		/* csis-link0 */
	void __iomem		*dma;		/* csis-dma (WDMA) */
	void __iomem		*ebuf;		/* csis-ebuf (elastic buffer), optional */
	struct clk_bulk_data	*clks;
	int			num_clks;
	/* ACPM CAM DVFS domain (clock-names "cam-dvfs"); stock LWIS votes 67 MHz. */
	struct clk		*cam_dvfs;
	/* SYSREG_CSIS syscon ("samsung,sysreg"): WDMA input routing. */
	struct regmap		*sysreg;
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
	spinlock_t			buf_lock;	/* protects buf_list/cur_buf/dma_armed */
	bool				dma_armed;	/* WDMA channel programmed by start_streaming */
	unsigned int			sequence;	/* frame counter for vb2 */
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
	/* WDMA context whose IRQ this instance owns (DT google,csis-dma-vc). */
	unsigned int			irq_ctx;
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
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;	/* was SGRBG10: the sensors send RGGB */
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
	/*
	 * FRAMECNT_SEQ: one bit per address slot that holds a valid buffer;
	 * the channel walks the set bits with ACTIVE_FRAMEPTR. Its reset value
	 * is all ones, so "seq | 1" (pablo's idiom on a register pablo has
	 * cleared elsewhere) left every slot enabled and the DMA cycled through
	 * ADDR2..ADDR32 = 0 -> writes to address 0 -> OTF OVERLAP + ABORTED on
	 * every frame (seen live 2026-09-18: frameptr advancing 0x1f,0,4,9 with
	 * only slot 0 programmed). We use slot 0 only.
	 */
	seq = BIT(0);
	is_hw_set_reg(vc, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FCNTSEQ], seq);
}

/*
 * Channel 0 write enable. Cleared on buffer underrun so the WDMA never keeps
 * writing into a buffer already handed back to userspace; re-armed from
 * buf_queue when a new buffer arrives.
 */
static void gs101_csis_hw_dma_enable(struct gs101_csis *csis, bool on)
{
	void __iomem *vc = gs101_csis_ctx(csis);

	is_hw_set_field(vc, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_CTRL],
			&csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DMA_ENABLE], on);
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

	/*
	 * WDMA common block (0x1A4D0000 + 0x8, CSIS_CMN_DMA_CTRL): IP_PROCESSING
	 * = 1 enables the DMA IP's Q-channel clock (pablo csi_hw_s_dma_common /
	 * csi_hw_dma_common_reset(on)). 2026-09-18: with it left at the reset
	 * value 0 the WDMA never wrote a line and its INT_SRC showed a permanent
	 * ABORT_DONE (bit 13) -> level IRQ storm (11 M interrupts) although the
	 * link had HS lock and counted frames. Shared by both link instances;
	 * left on at stop (pablo clears it only for power-off).
	 */
	/*
	 * WDMA common SW reset first (pablo csi_hw_dma_common_reset(on): set
	 * SW_RESET, poll until the block clears it, then IP_PROCESSING = 1).
	 * Without it the channel keeps its state across streams: after the
	 * first 8 frames the channel counter (ACT FRAMECNT 0x708) froze and no
	 * later stream ever latched ACTIVE_ENABLE. Done here, before the link
	 * and the sensor run, where pablo does it (never tried from userspace:
	 * the device froze after a stream stop before that test, 2026-09-18).
	 * The block is shared by both link instances, so this also resets a
	 * stream running on the other one; simultaneous front+rear capture is
	 * not supported yet.
	 */
	{
		unsigned int retry = 10;

		is_hw_set_field(csis->dma,
				&csi_cmn_dma_regs[CSIS_CMN_DMA_R_CSIS_CMN_DMA_CTRL],
				&csi_cmn_dma_fields[CSIS_CMN_DMA_F_SW_RESET], 1);
		while (--retry) {
			if (is_hw_get_field(csis->dma,
					    &csi_cmn_dma_regs[CSIS_CMN_DMA_R_CSIS_CMN_DMA_CTRL],
					    &csi_cmn_dma_fields[CSIS_CMN_DMA_F_SW_RESET]) != 1)
				break;
			udelay(10);
		}
		if (!retry)
			dev_warn(csis->dev, "WDMA common SW reset did not clear\n");
	}
	val = is_hw_get_reg(csis->dma,
			    &csi_cmn_dma_regs[CSIS_CMN_DMA_R_CSIS_CMN_DMA_CTRL]);
	val = is_hw_set_field_value(val,
				    &csi_cmn_dma_fields[CSIS_CMN_DMA_F_IP_PROCESSING], 1);
	val = is_hw_set_field_value(val,
				    &csi_cmn_dma_fields[CSIS_CMN_DMA_F_SW_RESET], 0);
	is_hw_set_reg(csis->dma, &csi_cmn_dma_regs[CSIS_CMN_DMA_R_CSIS_CMN_DMA_CTRL],
		      val);
	/*
	 * CSIS_CMN_DMA_CLK_CTRL bit0 CLKGATE_OFF: the stock HAL writes this
	 * register at WDMA init (csis_cmn_dma_clk_ctrl, value built with its
	 * set-bit helper); reset value 0 = the DMA IP gates its own clock.
	 * pablo never touches it. Added 2026-09-19 while the WDMA still never
	 * latched ACTIVE_ENABLE (OTF OVERLAP + ABORTED on every frame with the
	 * slot/sequence/routing/EBUF all in place).
	 */
	is_hw_set_field(csis->dma, &csi_cmn_dma_regs[CSIS_CMN_DMA_R_CSIS_CMN_DMA_CLK_CTRL],
			&csi_cmn_dma_fields[CSIS_CMN_DMA_F_CLKGATE_OFF], 1);

	/*
	 * EBUF (elastic buffer, 0x1A4C0000) sits between the links and the
	 * WDMA. Its bypass bit reads 0 at boot here (pablo's reset value is 1)
	 * with nothing else programmed -> the data never reached the DMA.
	 * 2026-09-18, live on link4: bypass = 1 from userspace made the WDMA
	 * see frame starts at once (OVERLAP IRQs); pablo's full EBUF setup
	 * (chid size, ctrl_en, num_of_cameras, bypass 0) did not. Bypass it.
	 */
	if (csis->ebuf && gs101_csis_ebuf_bypass) {
		is_hw_set_field(csis->ebuf, &csi_ebuf_regs[CSIS_EBUF_R_EBUF_CTRL],
				&csi_ebuf_fields[CSIS_EBUF_F_EBUF_BYPASS], 1);
	} else if (csis->ebuf) {
		/*
		 * Real EBUF setup, pablo csi_hw_s_ebuf_enable(on, ebuf_ch, mode) +
		 * csi_hw_s_cfg_ebuf(ebuf_ch, vc, w, h), ebuf_ch = our SYSREG slot
		 * = dma_ch, vc 0. The EBUF re-times the OTF stream for the WDMA
		 * (OUT_MIN_HBLANK); in bypass the WDMA saw only OTF OVERLAP on
		 * every frame and never latched ACTIVE_ENABLE (2026-09-18), so
		 * this is the default and bypass stays as the module parameter.
		 * Interrupt mask not enabled (no EBUF IRQ handler yet).
		 */
		void __iomem *eb = csis->ebuf;
		unsigned int ch = gs101_csis_sysreg_slot < 0 ?
				  csis->dma_ch : gs101_csis_sysreg_slot;
		const struct is_reg *r_ctrl_en =
			&csi_ebuf_regs[CSIS_EBUF_R_EBUF0_CTRL_EN + 6 * ch];
		const struct is_reg *r_sync =
			&csi_ebuf_regs[CSIS_EBUF_R_EBUF0_OUT_SYNC_CTRL + 6 * ch];
		const struct is_reg *r_size =
			&csi_ebuf_regs[CSIS_EBUF_R_EBUF0_CHID0_SIZE + 6 * ch];

		is_hw_set_field(eb, r_size,
				&csi_ebuf_fields[CSIS_EBUF_F_EBUFX_CHIDX_SIZE_V], h);
		is_hw_set_field(eb, r_size,
				&csi_ebuf_fields[CSIS_EBUF_F_EBUFX_CHIDX_SIZE_H], w / 4);
		is_hw_set_field(eb, &csi_ebuf_regs[CSIS_EBUF_R_EBUF_CTRL],
				&csi_ebuf_fields[CSIS_EBUF_F_EBUF_BYPASS], 0);
		is_hw_set_reg(eb, r_ctrl_en, gs101_csis_ebuf_ctrl_en);
		is_hw_set_field(eb, r_sync,
				&csi_ebuf_fields[CSIS_EBUF_F_EBUFX_OUT_MIN_HBLANK], 0x10);
#if 0	/* pablo's two field writes, replaced by the raw ebuf_ctrl_en parameter */
		is_hw_set_field(eb, r_ctrl_en,
				&csi_ebuf_fields[CSIS_EBUF_F_EBUFX_ABORT_CTRL_EN], 1);
		is_hw_set_field(eb, r_ctrl_en,
				&csi_ebuf_fields[CSIS_EBUF_F_EBUFX_ABORT_AUTO_GEN_FAKE], 0);
#endif
		is_hw_set_field(eb, &csi_ebuf_regs[CSIS_EBUF_R_EBUF_NUM_OF_CAMERAS],
				&csi_ebuf_fields[CSIS_EBUF_F_NUM_OF_CAMERAS], 1);
	}

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
				    gs101_csis_pixel_mode);	/* was CSIS_PIXEL_MODE_SING */
	is_hw_set_reg(link, &csi_regs[cfg], val);

	val = is_hw_get_reg(link, &csi_regs[resol]);
	val = is_hw_set_field_value(val, &csi_fields[CSIS_F_HRESOL], w);
	val = is_hw_set_field_value(val, &csi_fields[CSIS_F_VRESOL], h);
	is_hw_set_reg(link, &csi_regs[resol], val);

	/*
	 * WDMA input routing lives in SYSREG_CSIS (stock HAL), not in the DMA
	 * context bank: slot -> link, ctx -> slot, then the (link, ctx) enable
	 * bit. 2026-09-18: the old write below went to ctx + 0x500 (read back
	 * 0 forever) -- wrong register.
	 */
	if (csis->sysreg) {
		unsigned int slot = gs101_csis_sysreg_slot < 0 ?
				    csis->dma_ch : gs101_csis_sysreg_slot;

		regmap_write(csis->sysreg,
			     GS101_SYSREG_CSIS_LINK_SLOT(slot), csis->link_idx);
		regmap_write(csis->sysreg,
			     GS101_SYSREG_CSIS_DMA_MUX(csis->dma_ch), slot);
		regmap_update_bits(csis->sysreg, GS101_SYSREG_CSIS_DMA_EN,
				   BIT(csis->link_idx + csis->dma_ch * 8),
				   BIT(csis->link_idx + csis->dma_ch * 8));
	}
#if 0	/* pablo-style mux write into the DMA context: not this hardware's register */
	writel(csis->link_idx, ctx + GS101_CSIS_DMA_MUX_OFFSET);
#endif

	/* DMA input path for ch0 (pablo csi_hw_s_config_dma_cmn: OTF unless potf). */
	is_hw_set_field(ctl, &csi_dmax_regs[CSIS_DMAX_R_DATA_CTRL],
			&csi_dmax_fields[CSIS_DMAX_F_DMA_INPUT_PATH_CH0],
			gs101_csis_dma_input_prl ? 1 : 0);

	/* WDMA VC0: 2D, RAW10 unpacked to 16-bit, resolution, stride. */
	if (gs101_csis_dma_fmt != 6) {
		/* raw override, see the dma_fmt parameter */
		is_hw_set_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FMT],
			      gs101_csis_dma_fmt);
	} else {
		val = is_hw_get_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FMT]);
		val = is_hw_set_field_value(val,
					    &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DIM],
					    CSIS_REG_DMA_2D_DMA);
		val = is_hw_set_field_value(val,
					    &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DATAFORMAT],
					    CSIS_DMA_FMT_U10BIT_UNPACK_MSB_ZERO);
		is_hw_set_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_FMT], val);
	}

	val = is_hw_get_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_RESOL]);
	val = is_hw_set_field_value(val, &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_HRESOL], w);
	val = is_hw_set_field_value(val, &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_VRESOL], h);
	is_hw_set_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_RESOL], val);

	is_hw_set_field(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_STRIDE],
			&csi_dmax_chx_fields[CSIS_DMAX_CHX_F_STRIDE],
			csis->pixfmt.bytesperline);

	gs101_csis_hw_set_dma_addr(csis, addr);

	/*
	 * Frame pointer: the channel's ACTIVE_FRAMEPTR resets to 0x1f (slot
	 * 31) and only moves when UPDT_PTR_EN is set together with the slot
	 * (pablo csi_hw_s_frameptr). We fill slot 0 (ADDR1, FCNTSEQ bit 0), so
	 * point it there or every frame start hits an empty slot (OTF OVERLAP
	 * with nothing written, seen live 2026-09-18).
	 */
	val = is_hw_get_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_CTRL]);
	val = is_hw_set_field_value(val,
				    &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_UPDT_PTR_EN], 1);
	val = is_hw_set_field_value(val,
				    &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_UPDT_FRAMEPTR], 0);
	/*
	 * DMA_ENABLE stays 0 here. 2026-09-19: with identical settings some
	 * streams captured and some died from the first frame (OTF OVERLAP +
	 * ACTIVE_ABORTED, channel dead until the common SW reset) -- the
	 * channel was enabled before the sensor ran and took its first frame
	 * boundary wherever the sensor happened to be. pablo enables the DMA
	 * per frame from the link FRAME_START handler, i.e. always on a
	 * boundary; we enable it in start_streaming after the link has seen
	 * a FRAMEEND (gs101_csis_wait_frame_boundary).
	 */
	val = is_hw_set_field_value(val,
				    &csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DMA_ENABLE], 0);
	is_hw_set_reg(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_CTRL], val);
#if 0	/* replaced by the pointer-aware write above (2026-09-18) */
	is_hw_set_field(vc0, &csi_dmax_chx_regs[CSIS_DMAX_CHX_R_CTRL],
			&csi_dmax_chx_fields[CSIS_DMAX_CHX_F_DMA_ENABLE], 1);
#endif

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

	/* Release the EBUF channel (pablo csi_hw_s_ebuf_enable(off)). */
	if (csis->ebuf && !gs101_csis_ebuf_bypass) {
		unsigned int ch = gs101_csis_sysreg_slot < 0 ?
				  csis->dma_ch : gs101_csis_sysreg_slot;

		is_hw_set_field(csis->ebuf,
				&csi_ebuf_regs[CSIS_EBUF_R_EBUF0_CTRL_EN + 6 * ch],
				&csi_ebuf_fields[CSIS_EBUF_F_EBUFX_ABORT_CTRL_EN], 0);
		is_hw_set_field(csis->ebuf, &csi_ebuf_regs[CSIS_EBUF_R_EBUF_NUM_OF_CAMERAS],
				&csi_ebuf_fields[CSIS_EBUF_F_NUM_OF_CAMERAS], 0);
		is_hw_set_field(csis->ebuf, &csi_ebuf_regs[CSIS_EBUF_R_EBUF_CTRL],
				&csi_ebuf_fields[CSIS_EBUF_F_EBUF_BYPASS], 1);
	}

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

/*
 * Which instance currently streams on each WDMA context. The DMA IRQ lines
 * are per context and requested per instance from the DT (csis-dma%u), so
 * with the dma_ctx override the rear can run on context 0 whose IRQ lands in
 * the front instance: dispatch to the owner instead.
 */
static struct gs101_csis *gs101_csis_ctx_owner[4];

static irqreturn_t gs101_csis_irq(int irq, void *data)
{
	struct gs101_csis *csis = data;
	void __iomem *ctl;
	u32 src;

	u32 err;

	if (csis->irq_ctx < 4 && gs101_csis_ctx_owner[csis->irq_ctx] &&
	    gs101_csis_ctx_owner[csis->irq_ctx] != csis)
		csis = gs101_csis_ctx_owner[csis->irq_ctx];
	ctl = gs101_csis_ctx(csis) + GS101_CSIS_DMA_CTL_OFFSET;

	src = is_hw_get_reg(ctl, &csi_dmax_regs[CSIS_DMAX_R_INT_SRC]);
	if (!src)
		return IRQ_NONE;
	is_hw_set_reg(ctl, &csi_dmax_regs[CSIS_DMAX_R_INT_SRC], src);	/* clear */

	/*
	 * Channel-0 write errors: the frame that just completed (if any) is
	 * not trustworthy, hand it back as ERROR instead of silently DONE.
	 */
	/*
	 * ABORT_DONE (bit 13) is a state flag, not an error: it stays asserted
	 * while the DMA IP is idle/aborted (seen permanently with IP_PROCESSING
	 * = 0, see hw_start). Not an error and not a frame either.
	 */
	err = src & (BIT(CSIS_INT_DMA_FIFO_FULL) |
		     BIT(CSIS_INT_DMA_LASTDATA_ERROR) |
		     BIT(CSIS_INT_DMA_LASTADDR_ERROR) |
		     BIT(CSIS_INT_DMA_FSTART_IN_FLUSH + GS101_CSIS_VC0) |
		     BIT(CSIS_INT_DMA_OVERLAP + GS101_CSIS_VC0) |
		     BIT(CSIS_INT_DMA_FRAME_DROP + GS101_CSIS_VC0));
	if (err)
		dev_warn_ratelimited(csis->dev, "WDMA error INT_SRC=0x%08x\n", src);

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
		} else {
			/* Underrun: stop writing until buf_queue re-arms us. */
			gs101_csis_hw_dma_enable(csis, false);
		}
		csis->cur_buf = next;
		spin_unlock(&csis->buf_lock);

		if (done) {
			done->vb.vb2_buf.timestamp = ktime_get_ns();
			done->vb.sequence = csis->sequence++;
			done->vb.field = V4L2_FIELD_NONE;
			vb2_buffer_done(&done->vb.vb2_buf,
					err ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
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
	if (csis->dma_armed && !csis->cur_buf) {
		/* Recover from underrun: this buffer becomes the DMA target. */
		csis->cur_buf = buf;
		gs101_csis_hw_set_dma_addr(csis, buf->addr);
		gs101_csis_hw_dma_enable(csis, true);
	} else {
		list_add_tail(&buf->list, &csis->buf_list);
	}
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

/*
 * TEST (d) 2026-09-18: stock-exact power CYCLE of the camera domains before
 * the PHY is touched. Every other precondition is eliminated (clock state
 * identical to stock, isolation bypass verified, reset sysreg untouched like
 * stock, my SMCs irrelevant, EL3 priv_reg proxy refused, PMIC rails on) and
 * the first DCPHY bank write (0x1A4F1300 / M_BIAS 0x1A4F1000) still freezes
 * the SoC. What stock does at every camera open that we never did: genpd
 * powers the domains OFF at boot (unused) and back ON at open, running vendor
 * pmucal_local disable/enable = save CMU/SYSREG state -> tz_save SMC ->
 * CONFIGURATION bit0=0 -> wait STATUS=0, then bit0=1 -> wait STATUS=1 ->
 * tz_restore SMC -> restore saved state. Vendor DT: lwis_csi (links + PHY +
 * WDMA) lives in pd_pdp, and pd_csis is held by the two CSIS SYSMMUs, so at
 * camera open BOTH come up (csis first via the iommu device link, then pdp).
 * pd_csis alone was verified to cycle cleanly (1->0->1, all SMCs 0) and did
 * not help. Offsets: flexpmu_cal_local_gs101.h csis_xxx and pdp_xxx tables,
 * exynos-pd_el3.c, pmucal_local.c. PMU_ALIVE writes go via SMC_CMD_PRIV_REG.
 * Remove once the freeze is understood.
 */
struct gs101_pd_desc {
	const char *name;
	u32 pmu_cfg;		/* PMU_ALIVE CONFIGURATION reg (STATUS = +4) */
	u32 tz_addr;		/* need_smc: D_TZPC base + 0x204 */
	u32 cmu_base;		/* block CMU */
	u32 sysreg_base;	/* block SYSREG */
	u32 qch_first, qch_last;
	bool has_vra_mux;	/* PDP also has PLL_CON0_MUX_CLKCMU_PDP_VRA_USER @0x610 */
};

static const struct gs101_pd_desc gs101_pd_csis = {
	.name = "csis", .pmu_cfg = 0x17462400, .tz_addr = 0x1A410204,
	.cmu_base = 0x1A400000, .sysreg_base = 0x1A420000,
	.qch_first = 0x3048, .qch_last = 0x3110,	/* CSISX8_C2 .. SYSREG_CSIS */
};

static const struct gs101_pd_desc gs101_pd_pdp = {
	.name = "pdp", .pmu_cfg = 0x17462480, .tz_addr = 0x1AA10204,
	.cmu_base = 0x1AA00000, .sysreg_base = 0x1AA20000,
	.qch_first = 0x304c, .qch_last = 0x30d0,	/* D_TZPC_PDP .. VRA */
	.has_vra_mux = true,
};

#define GS101_PD_QCH_MAX	64

static void __maybe_unused gs101_csis_pd_cycle(struct gs101_csis *csis,
				const struct gs101_pd_desc *pd)
{
	void __iomem *cmu = ioremap(pd->cmu_base, 0x4000);
	void __iomem *sysreg = ioremap(pd->sysreg_base, 0x200);
	void __iomem *pmu = ioremap(pd->pmu_cfg, 0x8);
	unsigned int nqch = (pd->qch_last - pd->qch_first) / 4 + 1;
	u32 qch[GS101_PD_QCH_MAX];
	u32 busp, pll, vra = 0, opt, drcg, memclk, st;
	struct arm_smccc_res res;
	int i, t;

	if (!cmu || !sysreg || !pmu || nqch > GS101_PD_QCH_MAX) {
		dev_err(csis->dev, "pd cycle %s: ioremap failed\n", pd->name);
		goto out;
	}

	/* pmucal_rae_save_seq(<pd>_save) */
	busp = readl(cmu + 0x1800);		/* CLK_CON_DIV_DIV_CLK_<PD>_BUSP */
	pll = readl(cmu + 0x0600);		/* PLL_CON0_MUX_CLKCMU_<PD>_BUS_USER */
	if (pd->has_vra_mux)
		vra = readl(cmu + 0x0610);	/* PLL_CON0_MUX_CLKCMU_PDP_VRA_USER */
	for (i = 0; i < nqch; i++)
		qch[i] = readl(cmu + pd->qch_first + i * 4);
	opt = readl(cmu + 0x0800);		/* <PD>_CMU_<PD>_CONTROLLER_OPTION */
	drcg = readl(sysreg + 0x0104);		/* SYSREG_<PD>_BUS_COMPONENT_DRCG_EN */
	memclk = readl(sysreg + 0x0108);	/* SYSREG_<PD>_MEMCLK */
	dev_info(csis->dev, "pd cycle %s: saved busp=0x%x pll=0x%x opt=0x%x qch0=0x%x drcg=0x%x memclk=0x%x status=0x%x\n",
		 pd->name, busp, pll, opt, qch[0], drcg, memclk, readl(pmu + 0x4));

	/* exynos_pd_tz_save(need_smc) */
	arm_smccc_smc(0x82000410, 0 /*EXYNOS_GET_IN_PD_DOWN*/, pd->tz_addr,
		      2 /*RUNTIME_PM_TZPC_GROUP*/, 0, 0, 0, 0, &res);
	dev_info(csis->dev, "pd cycle %s: tz_save ret=0x%lx\n", pd->name, res.a0);

	/* <pd>_off: CONFIGURATION bit0=0, wait STATUS bit0==0 */
	arm_smccc_smc(0x82000504, pd->pmu_cfg, 2 /*PRIV_REG_OPTION_RMW*/, 0x1, 0x0,
		      0, 0, 0, &res);
	for (t = 1000; t && (readl(pmu + 0x4) & 0x1); t--)
		udelay(10);
	st = readl(pmu + 0x4);
	dev_info(csis->dev, "pd cycle %s: OFF priv ret=0x%lx status=0x%x %s\n",
		 pd->name, res.a0, st, (st & 1) ? "(TIMEOUT, still on)" : "(off)");

	/* <pd>_on: CONFIGURATION bit0=1, wait STATUS bit0==1 */
	arm_smccc_smc(0x82000504, pd->pmu_cfg, 2, 0x1, 0x1, 0, 0, 0, &res);
	for (t = 1000; t && !(readl(pmu + 0x4) & 0x1); t--)
		udelay(10);
	st = readl(pmu + 0x4);
	dev_info(csis->dev, "pd cycle %s: ON priv ret=0x%lx status=0x%x %s\n",
		 pd->name, res.a0, st, (st & 1) ? "(on)" : "(TIMEOUT, still off)");

	/* exynos_pd_tz_restore(need_smc) */
	arm_smccc_smc(0x82000410, 1 /*EXYNOS_WAKEUP_PD_DOWN*/, pd->tz_addr, 2,
		      0, 0, 0, 0, &res);
	dev_info(csis->dev, "pd cycle %s: tz_restore ret=0x%lx\n", pd->name, res.a0);

	if (!(st & 1)) {
		dev_err(csis->dev, "pd cycle %s: domain did not come back, skipping restore\n",
			pd->name);
		goto out;
	}

	/* pmucal_rae_restore_seq(<pd>_save), table order */
	writel(busp, cmu + 0x1800);
	if (pd->has_vra_mux)
		writel(vra, cmu + 0x0610);
	writel(pll, cmu + 0x0600);
	for (i = nqch - 1; i >= 0; i--)
		writel(qch[i], cmu + pd->qch_first + i * 4);
	writel(opt, cmu + 0x0800);
	writel(drcg, sysreg + 0x0104);
	writel(memclk, sysreg + 0x0108);
	dev_info(csis->dev, "pd cycle %s: restored\n", pd->name);
out:
	if (pmu)
		iounmap(pmu);
	if (sysreg)
		iounmap(sysreg);
	if (cmu)
		iounmap(cmu);
}

/*
 * Wait for a CLEAN frame on the link, then return right after its FRAMEEND
 * (vertical blanking) so the caller can enable the WDMA channel before the
 * next FRAMESTART. "Clean" = a frame during which INT_SRC0 reported no
 * packet errors (ID/CRC/ECC/overflow) and INT_SRC1 no lost FS/FE.
 *
 * 2026-09-19 history: enabling the channel before the sensor ran captured
 * 3 times and died 2 times (OTF OVERLAP + ABORTED from the first frame);
 * enabling right after any FRAMEEND died once; enabling mid-frame after a
 * FRAMESTART (pablo-style) left the channel active but never completing a
 * frame (2/2). The link's INT_SRC0 shows ID/CRC/ECC/OVER errors and lost
 * FS/FE at stream start, i.e. junk packets while the PHY settles; a DMA
 * enabled while that junk flows takes a bogus frame start and aborts.
 * Returns 0 in the blanking after a clean frame, -ETIMEDOUT otherwise.
 */
static int gs101_csis_wait_clean_frame(struct gs101_csis *csis)
{
	void __iomem *link = csis->link;
	const struct is_reg *r0 = &csi_regs[CSIS_R_CSIS_INT_SRC0];
	const struct is_reg *r1 = &csi_regs[CSIS_R_CSIS_INT_SRC1];
	u32 fs = 1U << csi_fields[CSIS_F_FRAMESTART].bit_start;
	u32 fe = 1U << csi_fields[CSIS_F_FRAMEEND].bit_start;
	u32 lost = (1U << csi_fields[CSIS_F_ERR_LOST_FS].bit_start) |
		   (1U << csi_fields[CSIS_F_ERR_LOST_FE].bit_start);
	unsigned int i, frames = 0;
	u32 s0, s1;

	writel(~0U, link + r0->sfr_offset);		/* clear everything */
	writel(~0U, link + r1->sfr_offset);
	for (i = 0; i < 600; i++) {			/* <= 600 ms */
		s1 = readl(link + r1->sfr_offset);
		if (s1 & fe) {
			s0 = readl(link + r0->sfr_offset);
			frames++;
			if (!(s0 & 0x00ffffff) && !(s1 & lost) && (s1 & fs)) {
				writel(~0U, link + r0->sfr_offset);
				writel(~0U, link + r1->sfr_offset);
				dev_info(csis->dev, "clean frame after %u frame(s)\n", frames);
				return 0;
			}
			/* dirty frame: clear and evaluate the next one */
			writel(~0U, link + r0->sfr_offset);
			writel(~0U, link + r1->sfr_offset);
		}
		usleep_range(1000, 1500);
	}
	return -ETIMEDOUT;
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

	/*
	 * Vote the ACPM CAM DVFS floor like stock LWIS does at device enable
	 * (lwis_platform_device_enable: core_clock_qos = 67000 kHz on
	 * CLOCK_FAMILY_CAM). ACPM already reports 67 MHz at boot, so this is a
	 * no-op today; it keeps the vote explicit and lets us raise it later.
	 */
	if (csis->cam_dvfs) {
		ret = clk_set_rate(csis->cam_dvfs, 67000000);
		dev_info(csis->dev, "cam dvfs: set 67 MHz ret=%d now %lu Hz\n",
			 ret, clk_get_rate(csis->cam_dvfs));
		ret = 0;
	}

	/*
	 * TEST (d'): power-cycle pd_csis AND pd_pdp like stock does at every
	 * camera open. (DIAG bail removed 2026-09-18: pd_csis alone was seen to
	 * cycle cleanly, status 1->0->1 with all SMCs returning 0, and the CSIS
	 * link read back fine afterwards -- so that cycle is real and was not
	 * the missing precondition.)
	 */
#if 0	/*
	 * TEST (e) 2026-09-18: pd cycle DISABLED. Hypothesis: the vendor Linux
	 * path never programs the DCPHY analog registers (vendor phy driver =
	 * isolation only, HAL = gnr_con0 + phy_cmn_ctrl only, per the HAL
	 * register table), so the PHY must come up with usable hardware
	 * defaults and our pd_csis OFF->ON wiped them to the zeros we read.
	 * Boot fresh, do NOT cycle, bypass isolation, read the bank.
	 */
	gs101_csis_pd_cycle(csis, &gs101_pd_csis);
	gs101_csis_pd_cycle(csis, &gs101_pd_pdp);
	dev_info(csis->dev, "pd cycle: CSIS version now 0x%08x\n",
		 readl(csis->link + CSIS_REG_VERSION));
#endif

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

#if 0	/*
		 * (TEST (e) answered: boot defaults are ZERO even without the pd
		 * cycle, so the cycle wiped nothing. Bail off again; pd cycle stays
		 * off. Next: sensor streaming + PHY configured, force the
		 * MIPI_PHY_LINK_WRAP ACLK gate to manual from userspace and watch
		 * PHY_STATUS.)
		 *
		 * DIAG 2026-09-18 (devmem2 probe) -- ANSWERED: with the PHY left
		 * powered here, the full vendor DCPHY0 register sequence replayed
		 * from userspace (phy-bank-write-probe.sh) succeeded with every
		 * value read back. The "freeze" was a stack overrun in
		 * exynos_mipi_phy_configure() (u32 info[4] indexed by SETTLE=4).
		 */
		dev_info(csis->dev, "DIAG: PHY left powered, bailing before phy_configure; run phy-bank-probe.sh\n");
		ret = -EINVAL;
		goto err_stop_pipe;
#endif

		/*
		 * DIAG: RE confirmed the DCPHY is programmed from NORMAL world
		 * (LWIS kernel readl/writel + HAL gnr_con0 via ioctl) -- no secure
		 * handoff. So the freeze is a precondition, not a TZPC wall. The
		 * PHY region (0x1A4F...) sits behind PMU isolation 0x3ebc; the link
		 * (0x1A44...) does not and IS NS-readable. Verify the isolation bit
		 * actually flipped to bypass after phy_power_on (never observable
		 * before -- the error path re-isolates). Also dump the CSIS reset
		 * sysreg. Bail before phy_configure so the device survives and
		 * dmesg is readable.
		 */
#if 0	/*
	 * DIAG ANSWERED 2026-09-18: after phy_power_on the PMU isolation reads
	 * 0x17463ebc=0x00000001 (bypass=1, isolation IS cleared) and the CSIS
	 * reset sysreg reads 0x1A420500=0x00000000 (bootloader/stock state; the
	 * vendor never writes it). So neither isolation nor reset state is the
	 * precondition. Bail removed: phy_configure() runs again. Kept for reuse.
	 */
		{
			void __iomem *iso = ioremap(0x17463ebc, 4);
			void __iomem *rst = ioremap(0x1A420500, 4);

			if (iso) {
				u32 v = readl(iso);

				dev_info(csis->dev,
					 "post power_on: iso 0x17463ebc=0x%08x bypass=%u\n",
					 v, v & 1);
				iounmap(iso);
			}
			if (rst) {
				u32 v = readl(rst);

				dev_info(csis->dev,
					 "csis reset 0x1A420500=0x%08x bit0=%u\n",
					 v, v & 1);
				iounmap(rst);
			}
		}
		ret = -EINVAL;
		goto err_phy;
#endif	/* DIAG iso/reset */

#if 0	/*
	 * TEST (b): all of my secure SMCs (pd-csis power via SMC_CMD_PRIV_REG +
	 * exynos_pd_tz_restore) and the CMU-ungate no-op are disabled, to check
	 * whether the bootloader already leaves the CSIS SFR window (incl. the
	 * DCPHY at 0x1A4F1300) EL1-accessible. If phy_configure() still hard-
	 * freezes with this off -> the D_TZPC wall is set by bl31 regardless of
	 * our SMCs (so the fix must route the DCPHY writes through EL3). If it
	 * works -> our tz_restore was re-securing a region the bootloader left
	 * open.
	 */
		/*
		 * Power up the CSIS/D-PHY power domain exactly the way the vendor
		 * PMU-CAL does it (Build 3). cal_pd_control() -> pmucal_local_enable
		 * -> pmucal_rae is DIRECT MMIO (writel/readl), NOT the ACPM mailbox
		 * (ACPM in cal-if is only for DVFS). The vendor csis_on[] sequence
		 * (flexpmu_cal_local_gs101) is exactly:
		 *   write CSIS_CONFIGURATION (0x17462400) bit0 = 1
		 *   wait  CSIS_STATUS        (0x17462404) bit0 == 1
		 * followed by exynos_pd_tz_restore(0x1A410204) (an EL3 SMC that
		 * restores the domain's S2MPU/TZPC config). Done here as ONE atomic
		 * power-on, with bit0 only (not the generic exynos5433-pd 0xf) and
		 * the SMC immediately after, right before the D-PHY SFR access. The
		 * genpd/pd_csis approach is dropped (it wrote 0xf and decoupled the
		 * SMC in time, which left the bank inaccessible -> hard freeze).
		 */
		{
			void __iomem *pmu = ioremap(0x17462400, 0x8);
			struct arm_smccc_res res;
			int t = 1000;
			u32 v;

			if (pmu) {
				/*
				 * CSIS_CONFIGURATION (0x17462400) is a SECURE PMU_ALIVE
				 * register (PMU_ALIVE_BASE_ADDR=0x17460000). The vendor
				 * pmucal writes it via SMC_CMD_PRIV_REG (0x82000504), NOT
				 * writel -- a direct EL1 write is silently dropped, which is
				 * why the earlier writel() never actually powered the domain
				 * (the "on" we saw was the bootloader's leftover state). Read
				 * is direct; RMW bit0; write through the secure SMC.
				 */
				v = (readl(pmu) & ~0x1) | 0x1;
				arm_smccc_smc(0x82000504, 0x17462400, 1 /*WRITE*/, v,
					      0, 0, 0, 0, &res);
				while (t-- && !(readl(pmu + 0x4) & 0x1))
					udelay(10);
				dev_info(csis->dev,
					 "csis pd on: priv ret=0x%lx status=0x%08x\n",
					 res.a0, readl(pmu + 0x4));
				iounmap(pmu);
			}

			/* S2MPU/TZPC restore in EL3 (bl31), same as vendor cal-if. */
			arm_smccc_smc(0x82000410, 1, 0x1A410204, 2,
				      0, 0, 0, 0, &res);
			dev_info(csis->dev, "csis tz_restore smc ret=0x%lx\n",
				 res.a0);
		}

		/*
		 * pd-csis is now powered; ungate the CMU_CSIS leaf clocks
		 * (D-PHY link wrap + APB/WDMA) before the first D-PHY SFR access
		 * in phy_configure(), or the SFR bus stalls -> EL3 reset.
		 */
		gs101_csis_cmu_ungate(csis);
#endif	/* TEST (b) */

		/* Configure the D-PHY analog/settle for reception (VC0, RAW10). */
		opts.mipi_dphy.lanes = csis->lanes;
		opts.mipi_dphy.hs_clk_rate = csis->link_freq * 2;
		ret = phy_configure(csis->phys[0], &opts);
		if (ret) {
			phy_power_off(csis->phys[0]);
			phy_exit(csis->phys[0]);
			goto err_stop_pipe;
		}
		dev_info(csis->dev, "phy up (lanes=%u hs_clk=%llu)\n",
			 csis->lanes, (u64)csis->link_freq * 2);
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

	if (gs101_csis_dma_ctx >= 0 && gs101_csis_dma_ctx <= 3 &&
	    csis->dma_ch != (unsigned int)gs101_csis_dma_ctx) {
		dev_info(csis->dev, "dma_ctx override: context %u -> %d\n",
			 csis->dma_ch, gs101_csis_dma_ctx);
		csis->dma_ch = gs101_csis_dma_ctx;
	}
	gs101_csis_hw_start(csis, first->addr);
	spin_lock_irqsave(&csis->buf_lock, flags);
	csis->sequence = 0;
	csis->dma_armed = true;
	if (csis->dma_ch < 4)
		gs101_csis_ctx_owner[csis->dma_ch] = csis;
	spin_unlock_irqrestore(&csis->buf_lock, flags);
	dev_info(csis->dev, "hw_start done, enabling sensor stream\n");

	ret = v4l2_subdev_call(&csis->sd, video, s_stream, 1);
	if (ret && ret != -ENOIOCTLCMD)
		goto err_hw_stop;

	/* Enable the WDMA channel in the blanking after a clean frame (see hw_start). */
	ret = gs101_csis_wait_clean_frame(csis);
	if (ret)
		dev_warn(csis->dev, "no clean link frame within 600 ms, enabling WDMA anyway\n");
	gs101_csis_hw_dma_enable(csis, true);

	return 0;

err_hw_stop:
	spin_lock_irqsave(&csis->buf_lock, flags);
	csis->dma_armed = false;
	spin_unlock_irqrestore(&csis->buf_lock, flags);
	if (csis->dma_ch < 4 && gs101_csis_ctx_owner[csis->dma_ch] == csis)
		gs101_csis_ctx_owner[csis->dma_ch] = NULL;
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

	unsigned long flags;

	v4l2_subdev_call(&csis->sd, video, s_stream, 0);
	spin_lock_irqsave(&csis->buf_lock, flags);
	csis->dma_armed = false;
	spin_unlock_irqrestore(&csis->buf_lock, flags);
	if (csis->dma_ch < 4 && gs101_csis_ctx_owner[csis->dma_ch] == csis)
		gs101_csis_ctx_owner[csis->dma_ch] = NULL;
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

/*
 * libcamera (simple pipeline, IO_MC) enumerates frame sizes per pixel
 * format / media bus code. The WDMA takes any size the sensor sends, so
 * report a stepwise range rather than a list.
 */
static int gs101_csis_enum_framesizes(struct file *file, void *priv,
				      struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;
	if (fsize->pixel_format != V4L2_PIX_FMT_SRGGB10)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = 16;
	fsize->stepwise.max_width = 8192;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = 16;
	fsize->stepwise.max_height = 8192;
	fsize->stepwise.step_height = 2;
	return 0;
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
	/*
	 * V4L2_CAP_IO_MC: libcamera's simple pipeline enumerates the video
	 * node's formats filtered by the media bus code it set on the subdev
	 * chain ("Media bus code filtering not supported" otherwise, 2026-09-19).
	 * The WDMA writes whatever Bayer order the sensor sends, 10 bit in 16;
	 * the only pixel format we expose is SRGGB10 (imx355/imx386 order).
	 */
	if (f->mbus_code && f->mbus_code != MEDIA_BUS_FMT_SRGGB10_1X10)
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
	.vidioc_enum_framesizes		= gs101_csis_enum_framesizes,
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
			    V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;
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
	char irq_name[16];
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

	/* csis-ebuf (shared like csis-dma, mapped non-exclusively); optional. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csis-ebuf");
	if (res) {
		csis->ebuf = devm_ioremap(dev, res->start, resource_size(res));
		if (!csis->ebuf)
			return -ENOMEM;
	} else {
		dev_warn(dev, "no csis-ebuf reg: EBUF bypass not programmed\n");
	}

	/* WDMA context for this instance (CSIS_DMA0/1); DT prop kept as -vc. */
	of_property_read_u32(dev->of_node, "google,csis-dma-vc", &csis->dma_ch);
	if (csis->dma_ch > 3)
		return dev_err_probe(dev, -EINVAL, "invalid csis-dma-ch %u\n",
				     csis->dma_ch);
	csis->irq_ctx = csis->dma_ch;

	/*
	 * The csis-phy register bank (0x1A4F0000) belongs to the MIPI D/C-PHY
	 * driver (phy-exynos-mipi-gs101); the CSIS reaches the PHY through the
	 * generic PHY framework, so it must not ioremap that bank itself.
	 */
	csis->num_clks = devm_clk_bulk_get_all(dev, &csis->clks);
	if (csis->num_clks < 0)
		return dev_err_probe(dev, csis->num_clks,
				     "failed to get clocks\n");
	{
		int i;

		for (i = 0; i < csis->num_clks; i++)
			if (csis->clks[i].id && !strcmp(csis->clks[i].id, "cam-dvfs"))
				csis->cam_dvfs = csis->clks[i].clk;
	}

	/*
	 * SYSREG_CSIS for the WDMA routing (see hw_start). Optional so the old
	 * DT still probes, but without it no frame reaches the DMA.
	 */
	csis->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "samsung,sysreg");
	if (IS_ERR(csis->sysreg)) {
		dev_warn(dev, "no samsung,sysreg syscon (%ld): WDMA routing not programmed\n",
			 PTR_ERR(csis->sysreg));
		csis->sysreg = NULL;
	}

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

	snprintf(irq_name, sizeof(irq_name), "csis-dma%u", csis->dma_ch);
	csis->irq = platform_get_irq_byname_optional(pdev, irq_name);
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
