// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs101 (Tensor) MIPI-CSIS receiver driver.
 *
 * Incremental mainline port of the CSIS v5.2 IP (same as Samsung Exynos2100).
 * Register map/sequences derived from the Exynos2100 "pablo" is-hw-csi-v5_2.*
 * and the gs101 LWIS DT (google gs101-isp.dtsi).
 *
 * Stage 3b-2: resource bring-up + a V4L2 sub-device with an async notifier that
 * binds the camera sensor, forming the media graph (sensor -> csis). The CSIS
 * write-DMA (raw Bayer -> DRAM) capture video node + streaming come next.
 *
 * Copyright 2026
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

/* CSIS link register bank (per gs101-isp.dtsi reg-name "csis-link0"). */
#define CSIS_REG_VERSION	0x0000
#define CSIS_REG_CMN_CTRL	0x0004

/* Number of MIPI D/C-PHYs / CSIS links on gs101. */
#define GS101_CSIS_NUM_PHYS	8

/* CSIS sub-device pads. */
#define GS101_CSIS_PAD_SINK	0
#define GS101_CSIS_PAD_SOURCE	1
#define GS101_CSIS_PADS_NUM	2

#define GS101_CSIS_DEF_WIDTH	1920
#define GS101_CSIS_DEF_HEIGHT	1080

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

	/* Sensor drives the format; keep sink == source (pass-through). */
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
	int ret;
	u32 source_pad;

	if (!sensor)
		return -ENODEV;

	ret = media_entity_get_fwnode_pad(&sensor->entity,
					  sensor->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(csis->dev, "no source pad on sensor\n");
		return ret;
	}
	source_pad = ret;

	ret = media_create_pad_link(&sensor->entity, source_pad,
				    &csis->sd.entity, GS101_CSIS_PAD_SINK,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(csis->dev, "failed to create sensor link\n");
		return ret;
	}

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
	int ret;

	csis = devm_kzalloc(dev, sizeof(*csis), GFP_KERNEL);
	if (!csis)
		return -ENOMEM;

	csis->dev = dev;
	platform_set_drvdata(pdev, csis);

	csis->link = devm_platform_ioremap_resource_byname(pdev, "csis-link0");
	if (IS_ERR(csis->link))
		return PTR_ERR(csis->link);

	csis->dma = devm_platform_ioremap_resource_byname(pdev, "csis-dma");
	if (IS_ERR(csis->dma))
		return PTR_ERR(csis->dma);

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
	return ret;
}

static void gs101_csis_remove(struct platform_device *pdev)
{
	struct gs101_csis *csis = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&csis->notifier);
	v4l2_async_nf_cleanup(&csis->notifier);
	media_device_unregister(&csis->mdev);
	v4l2_device_unregister_subdev(&csis->sd);
	v4l2_subdev_cleanup(&csis->sd);
	media_entity_cleanup(&csis->sd.entity);
	v4l2_device_unregister(&csis->v4l2_dev);
	media_device_cleanup(&csis->mdev);
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
