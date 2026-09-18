// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

#define IMX386_REG_MODE_SELECT		0x0100
#define IMX386_MODE_STANDBY		0x00
#define IMX386_MODE_STREAMING		0x01

/* Chip ID */
#define IMX386_REG_CHIP_ID		0x0016
#define IMX386_CHIP_ID			0x0386

/* V_TIMING internal */
#define IMX386_REG_FLL			0x0340
#define IMX386_FLL_MAX			0xffff

/* Exposure control */
#define IMX386_REG_EXPOSURE		0x0202
#define IMX386_EXPOSURE_MIN		1
#define IMX386_EXPOSURE_STEP		1
#define IMX386_EXPOSURE_DEFAULT		0x0282

/* Analog gain control */
#define IMX386_REG_ANALOG_GAIN		0x0204
#define IMX386_ANA_GAIN_MIN		0
#define IMX386_ANA_GAIN_MAX		960
#define IMX386_ANA_GAIN_STEP		1
#define IMX386_ANA_GAIN_DEFAULT		0

/* Digital gain control */
#define IMX386_REG_DPGA_USE_GLOBAL_GAIN	0x3070
#define IMX386_REG_DIG_GAIN_GLOBAL	0x020e
#define IMX386_DGTL_GAIN_MIN		256
#define IMX386_DGTL_GAIN_MAX		4095
#define IMX386_DGTL_GAIN_STEP		1
#define IMX386_DGTL_GAIN_DEFAULT	256

/* Test Pattern Control */
#define IMX386_REG_TEST_PATTERN		0x0600
#define IMX386_TEST_PATTERN_DISABLED		0
#define IMX386_TEST_PATTERN_SOLID_COLOR		1
#define IMX386_TEST_PATTERN_COLOR_BARS		2
#define IMX386_TEST_PATTERN_GRAY_COLOR_BARS	3
#define IMX386_TEST_PATTERN_PN9			4

/* Flip Control */
#define IMX386_REG_ORIENTATION		0x0101

/* default link frequency and external clock */
#define IMX386_LINK_FREQ_DEFAULT	360000000LL
#define IMX386_EXT_CLK			19200000
/*
 * The register tables below are written for a 24 MHz INCK (EXCK_FRQ 0x1800,
 * VT PLL 24/3*56, OP PLL 24/12*114), which is also what the vendor
 * gs101-oriole-camera.dtsi requests for this sensor (clock-rates 24000000).
 * gs101 cannot divide a shared PLL to exactly 24 MHz; CIS_CLK0 from shared2/2
 * (~400 MHz / 17) gives ~23.5 MHz, which the sensor tolerates. Accept the
 * external clock within 5% of 24 MHz (and the raw 19.2 MHz of the original
 * platform).
 */
#define IMX386_EXT_CLK_24MHZ		24000000
#define IMX386_EXT_CLK_TOLERANCE_PCT	5
#define IMX386_LINK_FREQ_INDEX		0

/* number of data lanes */
#define IMX386_DATA_LANES		2

struct imx386_reg {
	u16 address;
	u8 val;
};

struct imx386_reg_list {
	u32 num_of_regs;
	const struct imx386_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx386_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;

	/* V-timing */
	u32 fll_def;
	u32 fll_min;

	/* H-timing */
	u32 llp;

	/* index of link frequency */
	u32 link_freq_index;

	/* Default register values */
	struct imx386_reg_list reg_list;
};

struct imx386_hwcfg {
	unsigned long link_freq_bitmap;
};

struct imx386 {
	struct device *dev;
	struct clk *clk;

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	/* Current mode */
	const struct imx386_mode *cur_mode;

	struct imx386_hwcfg *hwcfg;

	/*
	 * Mutex for serialized access:
	 * Protect sensor set pad format and start/stop streaming safely.
	 * Protect access to sensor v4l2 controls.
	 */
	struct mutex mutex;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
};

static const struct regulator_bulk_data imx386_supplies[] = {
	{ .supply = "avdd" },
	{ .supply = "dvdd" },
	{ .supply = "dovdd" },
};

static const struct imx386_reg imx386_global_regs[] = {
	/*
	 * Software reset (0x0103 = 0x01) is issued explicitly in
	 * imx386_start_streaming() with a recovery delay; keeping it here would
	 * reset the sensor again in the middle of the settings burst. The
	 * sensor NAKs I2C for a short while after reset, so the register that
	 * follows must not be written immediately (else -EIO).
	 */
	/* { 0x0103, 0x01 }, */
	{ 0x0136, 0x18 },
	{ 0x0137, 0x00 },
	{ 0x3a7d, 0x00 },
	{ 0x3a7e, 0x02 },
	{ 0x3a7f, 0x05 },
	{ 0x3100, 0x00 },
	{ 0x3101, 0x40 },
	{ 0x3102, 0x00 },
	{ 0x3103, 0x10 },
	{ 0x3104, 0x01 },
	{ 0x3105, 0xe8 },
	{ 0x3106, 0x01 },
	{ 0x3107, 0xf0 },
	{ 0x3150, 0x04 },
	{ 0x3151, 0x03 },
	{ 0x3152, 0x02 },
	{ 0x3153, 0x01 },
	{ 0x5a86, 0x00 },
	{ 0x5a87, 0x82 },
	{ 0x5d1a, 0x00 },
	{ 0x5d95, 0x02 },
	{ 0x5e1b, 0x00 },
	{ 0x5f5a, 0x00 },
	{ 0x5f5b, 0x04 },
	{ 0x682c, 0x31 },
	{ 0x6831, 0x31 },
	{ 0x6835, 0x0e },
	{ 0x6836, 0x31 },
	{ 0x6838, 0x30 },
	{ 0x683a, 0x06 },
	{ 0x683b, 0x33 },
	{ 0x683d, 0x30 },
	{ 0x6842, 0x31 },
	{ 0x6844, 0x31 },
	{ 0x6847, 0x31 },
	{ 0x6849, 0x31 },
	{ 0x684d, 0x0e },
	{ 0x684e, 0x32 },
	{ 0x6850, 0x31 },
	{ 0x6852, 0x06 },
	{ 0x6853, 0x33 },
	{ 0x6855, 0x31 },
	{ 0x685a, 0x32 },
	{ 0x685c, 0x33 },
	{ 0x685f, 0x31 },
	{ 0x6861, 0x33 },
	{ 0x6865, 0x0d },
	{ 0x6866, 0x33 },
	{ 0x6868, 0x31 },
	{ 0x686b, 0x34 },
	{ 0x686d, 0x31 },
	{ 0x6872, 0x32 },
	{ 0x6877, 0x33 },
	{ 0x7ff0, 0x01 },
	{ 0x7ff4, 0x08 },
	{ 0x7ff5, 0x3c },
	{ 0x7ffa, 0x01 },
	{ 0x7ffd, 0x00 },
	{ 0x831e, 0x00 },
	{ 0x831f, 0x00 },
	{ 0x9301, 0xbd },
	{ 0x9b94, 0x03 },
	{ 0x9b95, 0x00 },
	{ 0x9b96, 0x08 },
	{ 0x9b97, 0x00 },
	{ 0x9b98, 0x0a },
	{ 0x9b99, 0x00 },
	{ 0x9ba7, 0x18 },
	{ 0x9ba8, 0x18 },
	{ 0x9d04, 0x08 },
	{ 0x9d50, 0x8c },
	{ 0x9d51, 0x64 },
	{ 0x9d52, 0x50 },
	{ 0x9e31, 0x04 },
	{ 0x9e32, 0x04 },
	{ 0x9e33, 0x04 },
	{ 0x9e34, 0x04 },
	{ 0xa200, 0x00 },
	{ 0xa201, 0x0a },
	{ 0xa202, 0x00 },
	{ 0xa203, 0x0a },
	{ 0xa204, 0x00 },
	{ 0xa205, 0x0a },
	{ 0xa206, 0x01 },
	{ 0xa207, 0xc0 },
	{ 0xa208, 0x00 },
	{ 0xa209, 0xc0 },
	{ 0xa20c, 0x00 },
	{ 0xa20d, 0x0a },
	{ 0xa20e, 0x00 },
	{ 0xa20f, 0x0a },
	{ 0xa210, 0x00 },
	{ 0xa211, 0x0a },
	{ 0xa212, 0x01 },
	{ 0xa213, 0xc0 },
	{ 0xa214, 0x00 },
	{ 0xa215, 0xc0 },
	{ 0xa300, 0x00 },
	{ 0xa301, 0x0a },
	{ 0xa302, 0x00 },
	{ 0xa303, 0x0a },
	{ 0xa304, 0x00 },
	{ 0xa305, 0x0a },
	{ 0xa306, 0x01 },
	{ 0xa307, 0xc0 },
	{ 0xa308, 0x00 },
	{ 0xa309, 0xc0 },
	{ 0xa30c, 0x00 },
	{ 0xa30d, 0x0a },
	{ 0xa30e, 0x00 },
	{ 0xa30f, 0x0a },
	{ 0xa310, 0x00 },
	{ 0xa311, 0x0a },
	{ 0xa312, 0x01 },
	{ 0xa313, 0xc0 },
	{ 0xa314, 0x00 },
	{ 0xa315, 0xc0 },
	{ 0xbc19, 0x01 },
	{ 0xbc1c, 0x0a },
	{ 0x0101, 0x00 },
	{ 0x0101, 0x03 },
	{ 0x300b, 0x01 },
};

static const struct imx386_reg_list imx386_global_setting = {
	.num_of_regs = ARRAY_SIZE(imx386_global_regs),
	.regs = imx386_global_regs,
};

/* 1920x1080 @ 60fps, 2-lane, 10-bit RAW (ported from vendor imx386) */
static const struct imx386_reg mode_1920x1080_regs[] = {
	{ 0x0112, 0x0a },
	{ 0x0113, 0x0a },
	{ 0x0301, 0x06 },
	{ 0x0303, 0x02 },
	{ 0x0305, 0x03 },
	{ 0x0306, 0x00 },
	{ 0x0307, 0x38 },
	{ 0x0309, 0x0a },
	{ 0x030b, 0x01 },
	{ 0x030d, 0x0c },
	{ 0x030e, 0x01 },
	{ 0x030f, 0x72 },
	{ 0x0310, 0x01 },
	{ 0x0342, 0x08 },
	{ 0x0343, 0xd0 },
	{ 0x0340, 0x04 },
	{ 0x0341, 0x80 },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x60 },
	{ 0x0346, 0x01 },
	{ 0x0347, 0xac },
	{ 0x0348, 0x0f },
	{ 0x0349, 0x5f },
	{ 0x034a, 0x0a },
	{ 0x034b, 0x1b },
	{ 0x0385, 0x01 },
	{ 0x0387, 0x01 },
	{ 0x0900, 0x01 },
	{ 0x0901, 0x22 },
	{ 0x300d, 0x00 },
	{ 0x302e, 0x00 },
	{ 0x0401, 0x00 },
	{ 0x0404, 0x00 },
	{ 0x0405, 0x10 },
	{ 0x040c, 0x07 },
	{ 0x040d, 0x80 },
	{ 0x040e, 0x04 },
	{ 0x040f, 0x38 },
	{ 0x034c, 0x07 },
	{ 0x034d, 0x80 },
	{ 0x034e, 0x04 },
	{ 0x034f, 0x38 },
	{ 0x0114, 0x01 },
	{ 0x0408, 0x00 },
	{ 0x0409, 0x00 },
	{ 0x040a, 0x00 },
	{ 0x040b, 0x00 },
	{ 0x0902, 0x02 },
	{ 0x3030, 0x00 },
	{ 0x3031, 0x01 },
	{ 0x3032, 0x00 },
	{ 0x3047, 0x01 },
	{ 0x3049, 0x00 },
	{ 0x30e6, 0x00 },
	{ 0x30e7, 0x00 },
	{ 0x4e25, 0x80 },
	{ 0x663a, 0x01 },
	{ 0x9311, 0x3f },
	{ 0xa0cd, 0x0a },
	{ 0xa0ce, 0x0a },
	{ 0xa0cf, 0x0a },
	{ 0x0202, 0x04 },
	{ 0x0203, 0x36 },
	{ 0x020e, 0x04 },
	{ 0x020f, 0x00 },
};

static const char * const imx386_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

static const s64 link_freq_menu_items[] = {
	IMX386_LINK_FREQ_DEFAULT,
};

/* Mode configs */
static const struct imx386_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.fll_def = 1152,
		.fll_min = 1152,
		.llp = 2256,
		.link_freq_index = IMX386_LINK_FREQ_INDEX,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
	},
};

static inline struct imx386 *to_imx386(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx386, sd);
}

/* Get bayer order based on flip setting. */
static u32 imx386_get_format_code(struct imx386 *imx386)
{
	/*
	 * Only one bayer order is supported.
	 * It depends on the flip settings.
	 */
	u32 code;
	static const u32 codes[2][2] = {
		{ MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SGRBG10_1X10, },
		{ MEDIA_BUS_FMT_SGBRG10_1X10, MEDIA_BUS_FMT_SBGGR10_1X10, },
	};

	lockdep_assert_held(&imx386->mutex);
	code = codes[imx386->vflip->val][imx386->hflip->val];

	return code;
}

/* Read registers up to 4 at a time */
static int imx386_read_reg(struct imx386 *imx386, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx386->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = { 0 };
	int ret;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 4 at a time */
static int imx386_write_reg(struct imx386 *imx386, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx386->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx386_write_regs(struct imx386 *imx386,
			     const struct imx386_reg *regs, u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = imx386_write_reg(imx386, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(imx386->dev,
					    "write reg 0x%4.4x return err %d",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Open sub-device */
static int imx386_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx386 *imx386 = to_imx386(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, 0);

	mutex_lock(&imx386->mutex);

	/* Initialize try_fmt */
	try_fmt->width = imx386->cur_mode->width;
	try_fmt->height = imx386->cur_mode->height;
	try_fmt->code = imx386_get_format_code(imx386);
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx386->mutex);

	return 0;
}

static int imx386_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx386 *imx386 = container_of(ctrl->handler,
					     struct imx386, ctrl_handler);
	s64 max;
	int ret;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx386->cur_mode->height + ctrl->val - 10;
		__v4l2_ctrl_modify_range(imx386->exposure,
					 imx386->exposure->minimum,
					 max, imx386->exposure->step, max);
		break;
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (!pm_runtime_get_if_in_use(imx386->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		/* Analog gain = 1024/(1024 - ctrl->val) times */
		ret = imx386_write_reg(imx386, IMX386_REG_ANALOG_GAIN, 2,
				       ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx386_write_reg(imx386, IMX386_REG_DIG_GAIN_GLOBAL, 2,
				       ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx386_write_reg(imx386, IMX386_REG_EXPOSURE, 2,
				       ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		/* Update FLL that meets expected vertical blanking */
		ret = imx386_write_reg(imx386, IMX386_REG_FLL, 2,
				       imx386->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx386_write_reg(imx386, IMX386_REG_TEST_PATTERN,
				       2, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx386_write_reg(imx386, IMX386_REG_ORIENTATION, 1,
				       imx386->hflip->val |
				       imx386->vflip->val << 1);
		break;
	default:
		ret = -EINVAL;
		dev_info(imx386->dev, "ctrl(id:0x%x,val:0x%x) is not handled",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(imx386->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx386_ctrl_ops = {
	.s_ctrl = imx386_set_ctrl,
};

static int imx386_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx386 *imx386 = to_imx386(sd);

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&imx386->mutex);
	code->code = imx386_get_format_code(imx386);
	mutex_unlock(&imx386->mutex);

	return 0;
}

static int imx386_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx386 *imx386 = to_imx386(sd);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	mutex_lock(&imx386->mutex);
	if (fse->code != imx386_get_format_code(imx386)) {
		mutex_unlock(&imx386->mutex);
		return -EINVAL;
	}
	mutex_unlock(&imx386->mutex);

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx386_update_pad_format(struct imx386 *imx386,
				     const struct imx386_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = imx386_get_format_code(imx386);
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx386_do_get_pad_format(struct imx386 *imx386,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		imx386_update_pad_format(imx386, imx386->cur_mode, fmt);
	}

	return 0;
}

static int imx386_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx386 *imx386 = to_imx386(sd);
	int ret;

	mutex_lock(&imx386->mutex);
	ret = imx386_do_get_pad_format(imx386, sd_state, fmt);
	mutex_unlock(&imx386->mutex);

	return ret;
}

static int
imx386_set_pad_format(struct v4l2_subdev *sd,
		      struct v4l2_subdev_state *sd_state,
		      struct v4l2_subdev_format *fmt)
{
	struct imx386 *imx386 = to_imx386(sd);
	const struct imx386_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	s32 vblank_def;
	s32 vblank_min;
	s64 h_blank;
	u64 pixel_rate;
	u32 height;

	mutex_lock(&imx386->mutex);

	/*
	 * Only one bayer order is supported.
	 * It depends on the flip settings.
	 */
	fmt->format.code = imx386_get_format_code(imx386);

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	imx386_update_pad_format(imx386, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		imx386->cur_mode = mode;
		pixel_rate = IMX386_LINK_FREQ_DEFAULT * 2 * IMX386_DATA_LANES;
		do_div(pixel_rate, 10);
		__v4l2_ctrl_s_ctrl_int64(imx386->pixel_rate, pixel_rate);
		/* Update limits and set FPS to default */
		height = imx386->cur_mode->height;
		vblank_def = imx386->cur_mode->fll_def - height;
		vblank_min = imx386->cur_mode->fll_min - height;
		height = IMX386_FLL_MAX - height;
		__v4l2_ctrl_modify_range(imx386->vblank, vblank_min, height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(imx386->vblank, vblank_def);
		h_blank = mode->llp - imx386->cur_mode->width;
		/*
		 * Currently hblank is not changeable.
		 * So FPS control is done only by vblank.
		 */
		__v4l2_ctrl_modify_range(imx386->hblank, h_blank,
					 h_blank, 1, h_blank);
	}

	mutex_unlock(&imx386->mutex);

	return 0;
}

/* Start streaming */
static int imx386_start_streaming(struct imx386 *imx386)
{
	const struct imx386_reg_list *reg_list;
	int ret;

	/*
	 * Software reset first, then wait for the sensor to recover. Sony IMX
	 * sensors NAK I2C for a short window after a 0x0103 reset, so writing
	 * the first setting immediately returns -EIO.
	 */
	ret = imx386_write_reg(imx386, 0x0103, 1, 0x01);
	if (ret) {
		dev_err(imx386->dev, "failed to soft reset sensor");
		return ret;
	}
	usleep_range(12000, 13000);

	/* Global Setting */
	reg_list = &imx386_global_setting;
	ret = imx386_write_regs(imx386, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(imx386->dev, "failed to set global settings");
		return ret;
	}

	/* Apply default values of current mode */
	reg_list = &imx386->cur_mode->reg_list;
	ret = imx386_write_regs(imx386, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(imx386->dev, "failed to set mode");
		return ret;
	}

	/* set digital gain control to all color mode */
	ret = imx386_write_reg(imx386, IMX386_REG_DPGA_USE_GLOBAL_GAIN, 1, 1);
	if (ret)
		return ret;

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx386->sd.ctrl_handler);
	if (ret)
		return ret;

	return imx386_write_reg(imx386, IMX386_REG_MODE_SELECT,
				1, IMX386_MODE_STREAMING);
}

/* Stop streaming */
static int imx386_stop_streaming(struct imx386 *imx386)
{
	return imx386_write_reg(imx386, IMX386_REG_MODE_SELECT,
				1, IMX386_MODE_STANDBY);
}

static int imx386_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx386 *imx386 = to_imx386(sd);
	int ret = 0;

	mutex_lock(&imx386->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(imx386->dev);
		if (ret < 0)
			goto err_unlock;

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx386_start_streaming(imx386);
		if (ret)
			goto err_rpm_put;
	} else {
		imx386_stop_streaming(imx386);
		pm_runtime_put(imx386->dev);
	}

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx386->vflip, enable);
	__v4l2_ctrl_grab(imx386->hflip, enable);

	mutex_unlock(&imx386->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(imx386->dev);
err_unlock:
	mutex_unlock(&imx386->mutex);

	return ret;
}

/* Verify chip ID */
static int imx386_identify_module(struct imx386 *imx386)
{
	int ret;
	u32 val;

	ret = imx386_read_reg(imx386, IMX386_REG_CHIP_ID, 2, &val);
	if (ret)
		return ret;

	if (val != IMX386_CHIP_ID) {
		dev_err(imx386->dev, "chip id mismatch: %x!=%x",
			IMX386_CHIP_ID, val);
		return -EIO;
	}
	return 0;
}

static const struct v4l2_subdev_core_ops imx386_subdev_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx386_video_ops = {
	.s_stream = imx386_set_stream,
};

static const struct v4l2_subdev_pad_ops imx386_pad_ops = {
	.enum_mbus_code = imx386_enum_mbus_code,
	.get_fmt = imx386_get_pad_format,
	.set_fmt = imx386_set_pad_format,
	.enum_frame_size = imx386_enum_frame_size,
};

static const struct v4l2_subdev_ops imx386_subdev_ops = {
	.core = &imx386_subdev_core_ops,
	.video = &imx386_video_ops,
	.pad = &imx386_pad_ops,
};

static const struct media_entity_operations imx386_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops imx386_internal_ops = {
	.open = imx386_open,
};

static int imx386_power_off(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx386 *imx386 = to_imx386(sd);

	/*
	 * Mirror imx386_power_on() / the vendor power-down order: assert reset,
	 * then stop the master clock, then drop the regulators, with the same
	 * settling delays between steps.
	 */
	gpiod_set_value_cansleep(imx386->reset_gpio, 1);
	usleep_range(1000, 2000);

	clk_disable_unprepare(imx386->clk);
	usleep_range(1000, 2000);

	regulator_bulk_disable(ARRAY_SIZE(imx386_supplies), imx386->supplies);
	usleep_range(1000, 2000);

	return 0;
}

static int imx386_power_on(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx386 *imx386 = to_imx386(sd);
	int ret;

	/*
	 * Follow the vendor (LWIS) power-up order used on the Pixel 6 module:
	 * regulators first, then the master clock, then release reset, with
	 * settling delays in between. The mainline default enabled the clock
	 * before the regulators, which leaves this sensor unresponsive on I2C.
	 */
	ret = regulator_bulk_enable(ARRAY_SIZE(imx386_supplies),
				    imx386->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulators");

	usleep_range(1000, 2000);

	ret = clk_prepare_enable(imx386->clk);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enable clock");
		goto error_disable_regulators;
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(imx386->reset_gpio, 0);
	usleep_range(10000, 11000);

	return 0;

error_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(imx386_supplies), imx386->supplies);
	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx386_pm_ops, imx386_power_off,
				 imx386_power_on, NULL);

/* Initialize control handlers */
static int imx386_init_controls(struct imx386 *imx386)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max;
	s64 vblank_def;
	s64 vblank_min;
	s64 hblank;
	u64 pixel_rate;
	const struct imx386_mode *mode;
	u32 max;
	int ret;

	ctrl_hdlr = &imx386->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &imx386->mutex;
	max = ARRAY_SIZE(link_freq_menu_items) - 1;
	imx386->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx386_ctrl_ops,
						   V4L2_CID_LINK_FREQ, max, 0,
						   link_freq_menu_items);
	if (imx386->link_freq)
		imx386->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample */
	pixel_rate = IMX386_LINK_FREQ_DEFAULT * 2 * 4;
	do_div(pixel_rate, 10);
	/* By default, PIXEL_RATE is read only */
	imx386->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, pixel_rate,
					       pixel_rate, 1, pixel_rate);

	/* Initialize vblank/hblank/exposure parameters based on current mode */
	mode = imx386->cur_mode;
	vblank_def = mode->fll_def - mode->height;
	vblank_min = mode->fll_min - mode->height;
	imx386->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_min,
					   IMX386_FLL_MAX - mode->height,
					   1, vblank_def);

	hblank = mode->llp - mode->width;
	imx386->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (imx386->hblank)
		imx386->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* fll >= exposure time + adjust parameter (default value is 10) */
	exposure_max = mode->fll_def - 10;
	imx386->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX386_EXPOSURE_MIN, exposure_max,
					     IMX386_EXPOSURE_STEP,
					     IMX386_EXPOSURE_DEFAULT);

	imx386->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx386->hflip)
		imx386->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx386->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx386->vflip)
		imx386->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX386_ANA_GAIN_MIN, IMX386_ANA_GAIN_MAX,
			  IMX386_ANA_GAIN_STEP, IMX386_ANA_GAIN_DEFAULT);

	/* Digital gain */
	v4l2_ctrl_new_std(ctrl_hdlr, &imx386_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX386_DGTL_GAIN_MIN, IMX386_DGTL_GAIN_MAX,
			  IMX386_DGTL_GAIN_STEP, IMX386_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx386_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx386_test_pattern_menu) - 1,
				     0, 0, imx386_test_pattern_menu);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx386->dev, "control init failed: %d", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(imx386->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx386_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx386->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static struct imx386_hwcfg *imx386_get_hwcfg(struct device *dev)
{
	struct imx386_hwcfg *cfg;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	int ret;

	if (!fwnode)
		return NULL;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return NULL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	if (ret)
		goto out_err;

	cfg = devm_kzalloc(dev, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		goto out_err;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX386_DATA_LANES)
		goto out_err;

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       link_freq_menu_items,
				       ARRAY_SIZE(link_freq_menu_items),
				       &cfg->link_freq_bitmap);
	if (ret)
		goto out_err;

	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(ep);
	return cfg;

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(ep);
	return NULL;
}

static int imx386_probe(struct i2c_client *client)
{
	struct imx386 *imx386;
	unsigned long freq;
	int ret;

	imx386 = devm_kzalloc(&client->dev, sizeof(*imx386), GFP_KERNEL);
	if (!imx386)
		return -ENOMEM;

	imx386->dev = &client->dev;

	mutex_init(&imx386->mutex);

	imx386->clk = devm_v4l2_sensor_clk_get(imx386->dev, NULL);
	if (IS_ERR(imx386->clk))
		return dev_err_probe(imx386->dev, PTR_ERR(imx386->clk),
				     "failed to get clock\n");

	freq = clk_get_rate(imx386->clk);
	{
		unsigned long err = freq > IMX386_EXT_CLK_24MHZ ?
				    freq - IMX386_EXT_CLK_24MHZ :
				    IMX386_EXT_CLK_24MHZ - freq;

		if (freq != IMX386_EXT_CLK &&
		    err * 100 > IMX386_EXT_CLK_24MHZ * IMX386_EXT_CLK_TOLERANCE_PCT)
			return dev_err_probe(imx386->dev, -EINVAL,
					     "external clock %lu is not supported\n",
					     freq);
	}
	dev_info(imx386->dev, "external clock %lu Hz (tables are for 24 MHz)\n", freq);

	ret = devm_regulator_bulk_get_const(imx386->dev,
					    ARRAY_SIZE(imx386_supplies),
					    imx386_supplies,
					    &imx386->supplies);
	if (ret) {
		dev_err_probe(imx386->dev, ret, "could not get regulators");
		goto error_probe;
	}

	imx386->reset_gpio = devm_gpiod_get_optional(imx386->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx386->reset_gpio)) {
		ret = dev_err_probe(imx386->dev, PTR_ERR(imx386->reset_gpio),
				    "failed to get gpios");
		goto error_probe;
	}

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx386->sd, client, &imx386_subdev_ops);

	imx386->hwcfg = imx386_get_hwcfg(imx386->dev);
	if (!imx386->hwcfg) {
		dev_err(imx386->dev, "failed to get hwcfg");
		ret = -ENODEV;
		goto error_probe;
	}

	ret = imx386_power_on(imx386->dev);
	if (ret)
		goto error_probe;

	/* Check module identity */
	ret = imx386_identify_module(imx386);
	if (ret) {
		dev_err(imx386->dev, "failed to find sensor: %d", ret);
		goto error_power_off;
	}

	/* Set default mode to max resolution */
	imx386->cur_mode = &supported_modes[0];

	ret = imx386_init_controls(imx386);
	if (ret) {
		dev_err(imx386->dev, "failed to init controls: %d", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	imx386->sd.internal_ops = &imx386_internal_ops;
	imx386->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;
	imx386->sd.entity.ops = &imx386_subdev_entity_ops;
	imx386->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx386->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx386->sd.entity, 1, &imx386->pad);
	if (ret) {
		dev_err(imx386->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(imx386->dev);
	pm_runtime_enable(imx386->dev);
	pm_runtime_idle(imx386->dev);

	ret = v4l2_async_register_subdev_sensor(&imx386->sd);
	if (ret < 0)
		goto error_media_entity_runtime_pm;

	return 0;

error_media_entity_runtime_pm:
	pm_runtime_disable(imx386->dev);
	pm_runtime_set_suspended(imx386->dev);
	media_entity_cleanup(&imx386->sd.entity);

error_handler_free:
	v4l2_ctrl_handler_free(imx386->sd.ctrl_handler);

error_power_off:
	imx386_power_off(imx386->dev);

error_probe:
	mutex_destroy(&imx386->mutex);

	return ret;
}

static void imx386_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx386 *imx386 = to_imx386(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(imx386->dev);

	if (!pm_runtime_status_suspended(imx386->dev)) {
		imx386_power_off(imx386->dev);
		pm_runtime_set_suspended(imx386->dev);
	}

	mutex_destroy(&imx386->mutex);
}

static const struct acpi_device_id imx386_acpi_ids[] __maybe_unused = {
	{ "SONY355A" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, imx386_acpi_ids);

static const struct of_device_id imx386_match_table[] = {
	{ .compatible = "sony,imx386", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx386_match_table);

static struct i2c_driver imx386_i2c_driver = {
	.driver = {
		.name = "imx386",
		.acpi_match_table = ACPI_PTR(imx386_acpi_ids),
		.of_match_table = imx386_match_table,
		.pm = &imx386_pm_ops,
	},
	.probe = imx386_probe,
	.remove = imx386_remove,
};
module_i2c_driver(imx386_i2c_driver);

MODULE_AUTHOR("Qiu, Tianshu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Rapolu, Chiranjeevi");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Yang, Hyungwoo");
MODULE_DESCRIPTION("Sony imx386 sensor driver");
MODULE_LICENSE("GPL v2");
