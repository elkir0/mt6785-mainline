// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6785 SENINF (MIPI CSI-2 sensor interface) V4L2 subdev driver.
 *
 * The MT6785 SENINF is the ISP7-generation "phy_2_0" IP: MIPI RX analog
 * PHY (0x11c80000, 0x4000 per CSI port) feeding per-port CSI-2 receivers,
 * a TOP crossbar (muxes) and a CAM_MUX stage whose output 2 goes to
 * CAMSV0.  This driver programs the register sequence validated on the
 * Lenovo Yoga Tab 11 by the experiment modules (drivers/staging/yt11-cam)
 * and tools/camera/ov-cap.py: every write below mirrors a line of
 * ov-cap.py's setup_phy_seninf(), in the same order, for the CSI port,
 * lane count, HS-settle and frame size taken from the DT endpoint and the
 * active subdev format.
 *
 * Media graph:
 *   sensor -> seninf sink pad N (= CSI port N) -> seninf source pad 3
 *           -> camsv0 subdev -> /dev/videoN
 *
 * The source pad always routes through SENINF mux 2 / CAM_MUX 2 because
 * CAMSV0 is hardwired to cam_mux 2 on this SoC.
 *
 * Structure modeled on the BayLibre MT8365 ISP3.0 driver (mtk_seninf.c,
 * drivers/media/platform/mediatek in the "Add Mediatek ISP3.0" v8 series)
 * with the MT6785 register layout from the begonia bring-up notes.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include "mt6785-cam.h"

/* ---- Register map (0x1a040000), from ov-cap.py / begonia NOTES.md ---- */

/* Per-SENINF-instance blocks, stride 0x1000, i = 0..4. */
#define SENINF_CTRL(i)			(0x200 + 0x1000 * (i))
#define SENINF_TG(i)			(0x600 + 0x1000 * (i))
#define SENINF_CSI2(i)			(0xa00 + 0x1000 * (i))
#define SENINF_MUX(i)			(0xd00 + 0x1000 * (i))
/* Shared TOP / CAM_MUX blocks. */
#define SENINF_TOP_MUX_CTRL(mux)	((mux) < 4 ? 0x10 : 0x14)
#define SENINF_TOP_MUX_SHIFT(mux)	(8 * ((mux) % 4))
#define SENINF_TOP_PHY_CTRL(port)	(0x60 + 4 * (port))
#define SENINF_CAMMUX			0x400

/* MIPI RX analog window (0x11c80000): per-port ANA-A/B, DPHY, CPHY. */
#define ANA_A(port)			(0x4000 * (port))
#define ANA_B(port)			(0x4000 * (port) + 0x1000)
#define ANA_DPHY(port)			(0x4000 * (port) + 0x2000)
#define ANA_CPHY(port)			(0x4000 * (port) + 0x3000)

/*
 * HAL SeninfDrvImp::getCSIInfo(): CSI port -> SENINF instance.
 * Ports 0, 1, 2 exist on the MT6785 package.
 */
static const unsigned int csi_port_to_seninf[MT6785_CAM_CSI_PORTS] = { 0, 2, 4 };

/* DPHY HS-settle, from ov-cap.py --settle default (validated value). */
#define MT6785_SENINF_SETTLE		15

static unsigned int hs_settle = MT6785_SENINF_SETTLE;
module_param(hs_settle, uint, 0644);
MODULE_PARM_DESC(hs_settle,
		 "DPHY HS-settle value (bits [15:8] of DPHY +0x20..0x2c), default 15");

#define MT6785_SENINF_AUTOSUSPEND_MS	2000

/**
 * struct mt6785_seninf - SENINF device state
 * @dev: platform device
 * @base: mapped SENINF registers (0x1a040000)
 * @ana: mapped MIPI RX analog registers (0x11c80000)
 * @clks: bulk clocks (cam, top-seninf muxes, APMIXED mipic0/1 26 MHz)
 * @num_clks: number of entries in @clks
 * @media_dev: registered media controller device
 * @v4l2_dev: V4L2 device owning the pipeline
 * @sd: this driver's V4L2 subdev
 * @pads: sink pads 0..2 (CSI ports) + source pad 3
 * @notifier: async notifier waiting for the sensor subdev
 * @csi_port: CSI port (0..2) of the bound sensor, or -1 when none
 * @seninf_id: SENINF instance (0..4) the port maps to
 * @lanes: number of CSI-2 data lanes of the bound sensor
 * @settle: DPHY HS-settle value (module parameter override or default)
 * @source_sd: bound sensor subdev
 * @source_pad: sensor's source pad feeding our sink
 * @streaming: s_stream state of this subdev
 */
struct mt6785_seninf {
	struct device *dev;
	void __iomem *base;
	void __iomem *ana;

	struct clk_bulk_data *clks;
	unsigned int num_clks;

	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev sd;
	struct media_pad pads[MT6785_CAM_NUM_PADS];
	struct v4l2_async_notifier notifier;

	int csi_port;
	unsigned int seninf_id;
	unsigned int lanes;
	unsigned int settle;

	struct v4l2_subdev *source_sd;
	struct media_pad *source_pad;

	bool streaming;
};

static struct mt6785_seninf *sd_to_seninf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt6785_seninf, sd);
}

static inline u32 seninf_rd(struct mt6785_seninf *s, u32 off)
{
	return readl(s->base + off);
}

static inline void seninf_wr(struct mt6785_seninf *s, u32 off, u32 val)
{
	writel(val, s->base + off);
}

static inline void seninf_rmw(struct mt6785_seninf *s, u32 off,
			      u32 clr, u32 set)
{
	u32 v = seninf_rd(s, off);

	seninf_wr(s, off, (v & ~clr) | set);
}

static inline u32 ana_rd(struct mt6785_seninf *s, u32 off)
{
	return readl(s->ana + off);
}

static inline void ana_wr(struct mt6785_seninf *s, u32 off, u32 val)
{
	writel(val, s->ana + off);
}

static inline void ana_rmw(struct mt6785_seninf *s, u32 off, u32 clr, u32 set)
{
	u32 v = ana_rd(s, off);

	ana_wr(s, off, (v & ~clr) | set);
}

static inline void ana_field(struct mt6785_seninf *s, u32 off,
			     u32 mask, u32 val)
{
	ana_rmw(s, off, mask, val & mask);
}

#define FIELD_MASK(hi, lo)		GENMASK(hi, lo)
#define FIELD_VAL(hi, lo, val)		(((val) << (lo)) & GENMASK(hi, lo))

/* --------------------------------------------------------------------------
 * Hardware configuration: the validated ov-cap.py sequence
 * (tools/camera/ov-cap.py, setup_phy_seninf(); line references below).
 */

/*
 * ov-cap.py phyA_init(), lines 246-265: bias and termination of the two
 * analog lanes blocks of the CSI port.
 */
static void seninf_phy_init(struct mt6785_seninf *s, int port)
{
	unsigned int aoff;
	int i;

	for (aoff = ANA_A(port); aoff <= ANA_B(port); aoff += 0x1000) {
		/* field(ana, a + 0x4, hi, lo, val) for the five field groups */
		ana_field(s, aoff + 0x4, FIELD_MASK(2, 0), FIELD_VAL(2, 0, 4));
		ana_field(s, aoff + 0x4, FIELD_MASK(6, 4), FIELD_VAL(6, 4, 4));
		ana_field(s, aoff + 0x4, FIELD_MASK(10, 8), FIELD_VAL(10, 8, 4));
		ana_field(s, aoff + 0x4, FIELD_MASK(14, 12), FIELD_VAL(14, 12, 4));
		ana_field(s, aoff + 0x4, FIELD_MASK(19, 16), FIELD_VAL(19, 16, 8));

		for (i = 0x18; i <= 0x20; i += 4) {
			ana_field(s, aoff + i, FIELD_MASK(5, 4), FIELD_VAL(5, 4, 1));
			ana_field(s, aoff + i, FIELD_MASK(21, 20), FIELD_VAL(21, 20, 1));
			ana_field(s, aoff + i, FIELD_MASK(7, 6), FIELD_VAL(7, 6, 1));
			ana_field(s, aoff + i, FIELD_MASK(23, 22), FIELD_VAL(23, 22, 1));
			ana_rmw(s, aoff + i, 0, (1 << 2) | (1 << 18));
			ana_field(s, aoff + i, FIELD_MASK(11, 8), FIELD_VAL(11, 8, 3));
			ana_field(s, aoff + i, FIELD_MASK(27, 24), FIELD_VAL(27, 24, 3));
		}

		ana_field(s, aoff + 0x8, FIELD_MASK(4, 0), FIELD_VAL(4, 0, 0x10));
		ana_field(s, aoff + 0x8, FIELD_MASK(12, 8), FIELD_VAL(12, 8, 0x10));
		ana_field(s, aoff + 0x8, FIELD_MASK(20, 16), FIELD_VAL(20, 16, 0x10));
		ana_field(s, aoff + 0x8, FIELD_MASK(28, 24), FIELD_VAL(28, 24, 0x10));
		ana_field(s, aoff + 0xc, FIELD_MASK(4, 0), FIELD_VAL(4, 0, 0x10));
		ana_field(s, aoff + 0xc, FIELD_MASK(12, 8), FIELD_VAL(12, 8, 0x10));

		ana_field(s, aoff + 0x30, FIELD_MASK(4, 0), FIELD_VAL(4, 0, 8));
		ana_field(s, aoff + 0x30, FIELD_MASK(12, 8), FIELD_VAL(12, 8, 8));
		ana_field(s, aoff + 0x30, FIELD_MASK(20, 16), FIELD_VAL(20, 16, 8));
		ana_field(s, aoff + 0x3c, FIELD_MASK(4, 0), FIELD_VAL(4, 0, 8));
		ana_field(s, aoff + 0x3c, FIELD_MASK(12, 8), FIELD_VAL(12, 8, 8));
		ana_field(s, aoff + 0x3c, FIELD_MASK(20, 16), FIELD_VAL(20, 16, 8));

		ana_rmw(s, aoff + 0x28, 0, 1);
		ana_rmw(s, aoff + 0x34, 0, 1);
		ana_field(s, aoff + 0x30, FIELD_MASK(28, 24), FIELD_VAL(28, 24, 1));
		ana_field(s, aoff + 0x3c, FIELD_MASK(28, 24), FIELD_VAL(28, 24, 1));
	}
}

/*
 * ov-cap.py dphy_init(), lines 268-271: per-lane HS-settle (bits [15:8]),
 * [23:16] = 0x20 and bit 29 on DPHY +0x20..+0x2c.
 */
static void seninf_dphy_init(struct mt6785_seninf *s, int port)
{
	unsigned int dphy = ANA_DPHY(port);
	int off;

	for (off = 0x20; off <= 0x2c; off += 4) {
		ana_field(s, dphy + off, FIELD_MASK(23, 16), FIELD_VAL(23, 16, 0x20));
		ana_field(s, dphy + off, FIELD_MASK(15, 8), FIELD_VAL(15, 8, s->settle));
		ana_rmw(s, dphy + off, 0, 1 << 29);
	}
}

/*
 * ov-cap.py lines 277-284: CSI2 receiver of the SENINF instance.
 * CSI2_EN = (1 << lanes) - 1, IRQ enable and packet counters on.
 */
static void seninf_csi2_setup(struct mt6785_seninf *s)
{
	unsigned int csi2 = SENINF_CSI2(s->seninf_id);
	u32 csi_en = (1 << s->lanes) - 1;

	seninf_rmw(s, csi2 + 0x10, 0, 0x100);		/* RESYNC_MERGE_CTRL */
	seninf_rmw(s, csi2 + 0x8, 0x7ff, 0);		/* HDR_MODE */
	seninf_rmw(s, csi2 + 0x4, BIT(0), 0);		/* CPHY_SEL = D-PHY */
	seninf_rmw(s, csi2 + 0xc0, 0, 0x80000000);	/* IRQ_EN */
	seninf_wr(s, csi2 + 0x0, csi_en);		/* CSI2_EN lane mask */
	seninf_wr(s, csi2 + 0xe0, 0xffffffff);		/* DBG_CTRL counters */
}

/*
 * ov-cap.py lines 289-301: 4D1C clock on A.L2, D-PHY lane mux and
 * CK_DATA_MUX_EN.
 */
static void seninf_phy_setting(struct mt6785_seninf *s, int port)
{
	unsigned int a = ANA_A(port), b = ANA_B(port);
	unsigned int dphy = ANA_DPHY(port);

	ana_rmw(s, a, BIT(1), 0);
	ana_rmw(s, b, BIT(1), 0);
	ana_rmw(s, a, 0x20 | 0x100, 0x800);
	ana_rmw(s, b, 0x20 | 0x100 | 0x800, 0);
	ana_rmw(s, a, 0, 0x40 | 0x200 | 0x1000);
	ana_rmw(s, b, 0, 0x40 | 0x200 | 0x1000);

	ana_field(s, dphy + 0x4, FIELD_MASK(18, 16), FIELD_VAL(18, 16, 0));
	ana_field(s, dphy + 0x4, FIELD_MASK(10, 8), FIELD_VAL(10, 8, 1));
	ana_field(s, dphy + 0x4, FIELD_MASK(2, 0), FIELD_VAL(2, 0, 2));
	ana_field(s, dphy + 0x4, FIELD_MASK(14, 12), FIELD_VAL(14, 12, 3));
	ana_field(s, dphy + 0x4, FIELD_MASK(22, 20), FIELD_VAL(22, 20, 4));
	ana_rmw(s, dphy, 0x2, 0x100 | 0x200 | 0x400 | 0x800 | 0x1);
	ana_rmw(s, dphy + 0x4, 0, 0x80000000);		/* CK_DATA_MUX_EN */
}

/*
 * ov-cap.py lines 303-316: analog power-on; waits for the calibration
 * status bits (ANA +0x48 & 0x29 == 0x29).  The APMIXED mipic0/1 26 MHz
 * gates must be on (handled through the clock framework).
 */
static int seninf_phy_power_on(struct mt6785_seninf *s, int port)
{
	unsigned int aoff;
	int ret, a;

	for (aoff = ANA_A(port); aoff <= ANA_B(port); aoff += 0x1000) {
		for (a = 0x18; a <= 0x20; a += 4)
			ana_rmw(s, aoff + a, 1 | 0x10000, 0);
		ana_rmw(s, aoff, 4 | 8, 0);
		usleep_range(300, 400);
		ana_rmw(s, aoff, 0, 8);
		usleep_range(100, 200);
		ana_rmw(s, aoff, 0, 4);
		usleep_range(100, 200);
		for (a = 0x18; a <= 0x20; a += 4)
			ana_rmw(s, aoff + a, 0, 1 | 0x10000);
		usleep_range(2000, 2500);
	}

	/* ov-cap.py prints ANA +0x48 per lane block; poll both. */
	for (aoff = ANA_A(port); aoff <= ANA_B(port); aoff += 0x1000) {
		u32 status;

		ret = readl_poll_timeout(s->ana + aoff + 0x48, status,
					 (status & 0x29) == 0x29, 1000, 50000);
		if (ret) {
			dev_err(s->dev,
				"CSI port %d ANA +0x%04x calibration timeout (0x%08x); are the APMIXED mipic 26 MHz clocks on?\n",
				port, aoff + 0x48, status);
			return ret;
		}
	}

	return 0;
}

/*
 * ov-cap.py lines 318-336: SENINF enable, TOP crossbar (mux 2 ->
 * CAMSV0), MUX control and CAM_MUX enables.
 */
static void seninf_routing_setup(struct mt6785_seninf *s)
{
	unsigned int ctrl = SENINF_CTRL(s->seninf_id);
	unsigned int mux = SENINF_MUX(MT6785_CAM_MUX);
	u32 top_reg = SENINF_TOP_MUX_CTRL(MT6785_CAM_MUX);
	u32 top_shift = SENINF_TOP_MUX_SHIFT(MT6785_CAM_MUX);

	/* seninf_setting: CSI2_CTRL and SENINF_CTRL enables */
	seninf_rmw(s, ctrl + 0x10, 0, 1);
	seninf_rmw(s, ctrl + 0x0, 0, 1);

	/* TOP_MUX_CTRL nibble: route this SENINF into mux 2 */
	seninf_rmw(s, top_reg, 0xf << top_shift, s->seninf_id << top_shift);

	/* mux2: MIPI source (8), 1-pixel mode, polarities 0, reset, enable */
	seninf_rmw(s, mux + 0x4, 0x30f, 0x8);
	seninf_rmw(s, mux + 0x8, 0x30000, 0);
	seninf_rmw(s, mux + 0x0, 0, 0x6);
	seninf_rmw(s, mux + 0x0, 0x6, 0);
	seninf_rmw(s, mux + 0x0, 0, 1);
	seninf_wr(s, mux + 0xa8, 1);			/* FRAME_SIZE_MON_CTRL */

	/* CAM_MUX enables (ov-cap.py line 336, cam_mux 2 of 0..4) */
	seninf_rmw(s, SENINF_CAMMUX + 0x10, 0, 0x1f);
	seninf_rmw(s, SENINF_CAMMUX + 0x18, 0, 0x1);
}

/* ov-cap.py line 274: setSeninfCsiMipi, CPHY top +0x18 |= 0x10. */
static void seninf_csi_mipi(struct mt6785_seninf *s, int port)
{
	ana_rmw(s, ANA_CPHY(port) + 0x18, 0, 0x10);
}

/*
 * Full stream-on sequence (same order as ov-cap.py setup_phy_seninf):
 * phyA_init -> dphy_init -> setSeninfCsiMipi -> csi2_setting ->
 * TOP_PHY_CTRL -> phyA_setting -> dphy_setting (inside phy_setting) ->
 * phyA_power_on -> seninf_setting/routing.  Must run with clocks and
 * the CAM power domain on (runtime PM get).
 */
static int seninf_hw_stream_on(struct mt6785_seninf *s,
			       const struct v4l2_mbus_framefmt *fmt)
{
	int port = s->csi_port;
	int ret;

	if (port < 0 || port >= MT6785_CAM_CSI_PORTS) {
		dev_err(s->dev, "no sensor bound to a CSI port\n");
		return -ENODEV;
	}

	/* TOP_PHY_CTRL_CSIp: D-PHY mode (ov-cap.py line 287). */
	seninf_rmw(s, SENINF_TOP_PHY_CTRL(port), 0x2, 0x1);

	seninf_phy_init(s, port);
	seninf_dphy_init(s, port);
	seninf_csi_mipi(s, port);
	seninf_csi2_setup(s);
	seninf_phy_setting(s, port);
	ret = seninf_phy_power_on(s, port);
	if (ret)
		return ret;

	seninf_routing_setup(s);

	dev_dbg(s->dev, "stream on: CSI port %d -> SENINF%u, %u lanes, %ux%u\n",
		port, s->seninf_id, s->lanes, fmt->width, fmt->height);

	return 0;
}

/* Reverse of the above: disable MUX, CSI2, SENINF, then the analog PHY. */
static void seninf_hw_stream_off(struct mt6785_seninf *s)
{
	unsigned int ctrl = SENINF_CTRL(s->seninf_id);
	unsigned int csi2 = SENINF_CSI2(s->seninf_id);
	unsigned int mux = SENINF_MUX(MT6785_CAM_MUX);
	unsigned int aoff;

	if (s->csi_port < 0)
		return;

	/* Stop the mux first so CAMSV sees no further pixels. */
	seninf_rmw(s, mux + 0x0, 0x6, 0x6);
	seninf_rmw(s, mux + 0x0, 0x7, 0);
	seninf_rmw(s, mux + 0xa8, 1, 0);

	seninf_wr(s, csi2 + 0x0, 0);			/* CSI2_EN off */
	seninf_rmw(s, ctrl + 0x10, 1, 0);		/* CSI2_CTRL off */
	seninf_rmw(s, ctrl + 0x0, 1, 0);		/* SENINF off */

	/* PHY power-down (ov-cap.py phyA_power_on with bits cleared). */
	for (aoff = ANA_A(s->csi_port); aoff <= ANA_B(s->csi_port);
	     aoff += 0x1000) {
		ana_rmw(s, aoff, 0, 4 | 8);
		ana_rmw(s, aoff, 4 | 8, 0);
	}
	/* D-PHY mode off for the port. */
	seninf_rmw(s, SENINF_TOP_PHY_CTRL(s->csi_port), 0x3, 0x2);
}

/* --------------------------------------------------------------------------
 * V4L2 subdev operations
 */

/*
 * Default format on every pad: the validated capture geometry
 * (640x480 SBGGR10, the ov8856 24 MHz mode), so a fresh boot has a
 * coherent pipeline and link validation passes without userspace
 * setting every pad by hand.
 */
static const struct v4l2_mbus_framefmt seninf_default_fmt = {
	.code = MEDIA_BUS_FMT_SBGGR10_1X10,
	.width = 640,
	.height = 480,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

/* Bayer media bus codes passed through unchanged (subset of what the
 * ov8856/s5k4h7 sensors emit). */
static const u32 seninf_bus_codes[] = {
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
};

static int seninf_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(seninf_bus_codes))
		return -EINVAL;

	code->code = seninf_bus_codes[code->index];

	return 0;
}

static struct v4l2_mbus_framefmt *
seninf_get_ffmt(struct v4l2_subdev_state *state, unsigned int pad)
{
	return v4l2_subdev_state_get_format(state, pad, 0);
}

static int seninf_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(seninf_bus_codes); i++)
		if (seninf_bus_codes[i] == fmt->format.code)
			break;
	if (i == ARRAY_SIZE(seninf_bus_codes))
		fmt->format.code = seninf_default_fmt.code;

	fmt->format.field = V4L2_FIELD_NONE;

	*seninf_get_ffmt(state, fmt->pad) = fmt->format;

	/* Propagate to the source pad: the SENINF is pass-through. */
	if (fmt->pad != MT6785_CAM_PAD_SRC) {
		struct v4l2_mbus_framefmt *src;

		src = seninf_get_ffmt(state, MT6785_CAM_PAD_SRC);
		src->width = fmt->format.width;
		src->height = fmt->format.height;
		src->code = fmt->format.code;
	}

	return 0;
}

static int seninf_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	unsigned int pad;

	for (pad = 0; pad < MT6785_CAM_NUM_PADS; pad++)
		*seninf_get_ffmt(state, pad) = seninf_default_fmt;

	return 0;
}

static int seninf_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt6785_seninf *s = sd_to_seninf(sd);
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	fmt = seninf_get_ffmt(state, MT6785_CAM_PAD_SRC);

	if (enable) {
		if (s->streaming)
			goto out;

		ret = pm_runtime_resume_and_get(s->dev);
		if (ret < 0)
			goto out;

		/*
		 * Program the receiver before the sensor starts, matching
		 * ov-cap.py (SENINF + PHY first, then sensor stream on,
		 * then CAMSV0 armed by the CAMSV driver's start_streaming).
		 */
		ret = seninf_hw_stream_on(s, fmt);
		if (ret) {
			pm_runtime_mark_last_busy(s->dev);
			pm_runtime_put_autosuspend(s->dev);
			goto out;
		}

		ret = v4l2_subdev_call(s->source_sd, video, s_stream, 1);
		if (ret) {
			seninf_hw_stream_off(s);
			pm_runtime_mark_last_busy(s->dev);
			pm_runtime_put_autosuspend(s->dev);
			goto out;
		}

		s->streaming = true;
	} else {
		if (!s->streaming)
			goto out;

		v4l2_subdev_call(s->source_sd, video, s_stream, 0);
		seninf_hw_stream_off(s);
		s->streaming = false;
		pm_runtime_mark_last_busy(s->dev);
		pm_runtime_put_autosuspend(s->dev);
	}

out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

/*
 * Wrap the default link validation so a mismatch is a dev_err with the
 * pads and both formats named (the core only dev_dbg()s it).
 */
static int seninf_link_validate(struct v4l2_subdev *sd,
				struct media_link *link,
				struct v4l2_subdev_format *source_fmt,
				struct v4l2_subdev_format *sink_fmt)
{
	int ret;

	ret = v4l2_subdev_link_validate_default(sd, link, source_fmt,
						sink_fmt);
	if (ret)
		dev_err(sd->dev,
			"link %s:%u -> %s:%u format mismatch: source %ux%u code 0x%x, sink %ux%u code 0x%x\n",
			link->source->entity->name, link->source->index,
			link->sink->entity->name, link->sink->index,
			source_fmt->format.width, source_fmt->format.height,
			source_fmt->format.code,
			sink_fmt->format.width, sink_fmt->format.height,
			sink_fmt->format.code);

	return ret;
}

static const struct v4l2_subdev_pad_ops seninf_pad_ops = {
	.enum_mbus_code = seninf_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = seninf_set_fmt,
	.link_validate = seninf_link_validate,
};

static const struct v4l2_subdev_video_ops seninf_video_ops = {
	.s_stream = seninf_s_stream,
};

static const struct v4l2_subdev_ops seninf_subdev_ops = {
	.video = &seninf_video_ops,
	.pad = &seninf_pad_ops,
};

static const struct v4l2_subdev_internal_ops seninf_internal_ops = {
	.init_state = seninf_init_state,
};

static const struct media_entity_operations seninf_media_ops = {
	/* DT port number == sink pad number (0..2 CSI ports). */
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_validate = v4l2_subdev_link_validate,
};

/* --------------------------------------------------------------------------
 * Async notifier: bind the sensor subdev of whichever CSI port's DT
 * endpoint points at this SENINF.
 */

struct seninf_async_subdev {
	struct v4l2_async_connection asc;
	struct mt6785_seninf *seninf;
	unsigned int port;
};

static int seninf_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_connection *asc)
{
	struct seninf_async_subdev *sasd =
		container_of(asc, struct seninf_async_subdev, asc);
	struct mt6785_seninf *s = sasd->seninf;
	struct media_pad *source_pad;
	int ret;

	dev_info(s->dev, "%s bound on port %u\n",
		 sd->entity.name, sasd->port);

	if (sasd->port < MT6785_CAM_CSI_PORTS) {
		/* Sensor -> sink pad of its CSI port. */
		ret = v4l2_create_fwnode_links_to_pad(sd, &s->pads[sasd->port],
						      MEDIA_LNK_FL_IMMUTABLE |
						      MEDIA_LNK_FL_ENABLED);
		if (ret) {
			dev_err(s->dev, "failed to create link to sensor: %d\n",
				ret);
			return ret;
		}

		source_pad = media_pad_remote_pad_unique(&s->pads[sasd->port]);
		if (IS_ERR(source_pad)) {
			dev_err(s->dev, "no unique sensor pad on port %u\n",
				sasd->port);
			return PTR_ERR(source_pad);
		}

		s->source_sd = sd;
		s->source_pad = source_pad;
		s->csi_port = sasd->port;
		s->seninf_id = csi_port_to_seninf[sasd->port];
	} else {
		/*
		 * CAMSV <- our source pad (port 3).  The DT already pairs
		 * the endpoints, so create the link directly:
		 * v4l2_create_fwnode_links_to_pad(&s->sd, ...) would be a
		 * silent no-op here — it iterates src_sd->fwnode, which is
		 * only assigned for async-registered subdevs
		 * (__v4l2_async_register_subdev), and the SENINF registers
		 * itself directly under its v4l2_device.
		 */
		ret = media_create_pad_link(&s->sd.entity, MT6785_CAM_PAD_SRC,
					    &sd->entity, 0,
					    MEDIA_LNK_FL_IMMUTABLE |
					    MEDIA_LNK_FL_ENABLED);
		if (ret) {
			dev_err(s->dev, "failed to link %s:%u -> %s:0: %d\n",
				s->sd.entity.name, MT6785_CAM_PAD_SRC,
				sd->entity.name, ret);
			return ret;
		}
	}

	return 0;
}

static int seninf_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mt6785_seninf *s =
		container_of(notifier, struct mt6785_seninf, notifier);
	int ret;

	if (s->csi_port < 0)
		dev_warn(s->dev,
			 "pipeline complete without a bound sensor (no /dev/video streaming possible)\n");

	ret = v4l2_device_register_subdev_nodes(&s->v4l2_dev);
	if (ret)
		dev_err(s->dev, "failed to register subdev nodes: %d\n", ret);

	return ret;
}

static const struct v4l2_async_notifier_operations seninf_async_ops = {
	.bound = seninf_notifier_bound,
	.complete = seninf_notifier_complete,
};

static int seninf_parse_endpoint(struct mt6785_seninf *s, unsigned int port)
{
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct seninf_async_subdev *sasd;
	struct fwnode_handle *ep, *remote;
	bool is_sink = port < MT6785_CAM_CSI_PORTS;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(s->dev), port, 0, 0);
	if (!ep)
		return 0;

	remote = fwnode_graph_get_remote_endpoint(ep);
	if (!remote) {
		dev_err(s->dev, "port %u endpoint has no remote\n", port);
		fwnode_handle_put(ep);
		return -EINVAL;
	}

	if (is_sink) {
		/*
		 * data-lanes lives on the sensor's endpoint; parse the
		 * remote side for the lane count, our own endpoint only
		 * carries the link.
		 */
		ret = v4l2_fwnode_endpoint_parse(remote, &vep);
		if (ret) {
			dev_err(s->dev,
				"failed to parse port %u remote endpoint: %d\n",
				port, ret);
			goto out;
		}

		s->lanes = vep.bus.mipi_csi2.num_data_lanes;
		if (s->lanes != 1 && s->lanes != 2 && s->lanes != 4) {
			dev_err(s->dev, "port %u: invalid data lane count %u\n",
				port, s->lanes);
			ret = -EINVAL;
			goto out;
		}
	}

	sasd = v4l2_async_nf_add_fwnode(&s->notifier, remote,
					struct seninf_async_subdev);
	if (IS_ERR(sasd)) {
		ret = PTR_ERR(sasd);
		goto out;
	}

	sasd->seninf = s;
	sasd->port = port;

	dev_dbg(s->dev, "port %u: remote endpoint added%s\n", port,
		is_sink ? "" : " (source)");

	ret = 0;
out:
	fwnode_handle_put(ep);
	fwnode_handle_put(remote);
	return ret;
}

/* --------------------------------------------------------------------------
 * Probe / remove
 */

static const char *const seninf_clk_names[] = {
	"cam-main", "cam-gals", "cam-tg", "cam-seninf",
	"top-seninf0", "top-seninf1", "top-seninf2",
	"mipic0", "mipic1",
};

static int seninf_runtime_resume(struct device *dev)
{
	struct mt6785_seninf *s = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(s->num_clks, s->clks);
}

static int seninf_runtime_suspend(struct device *dev)
{
	struct mt6785_seninf *s = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(s->num_clks, s->clks);

	return 0;
}

static const struct dev_pm_ops seninf_pm_ops = {
	SET_RUNTIME_PM_OPS(seninf_runtime_suspend, seninf_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static int seninf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6785_seninf *s;
	unsigned int i;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	s->csi_port = -1;
	s->settle = hs_settle;
	dev_set_drvdata(dev, s);

	s->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->base))
		return PTR_ERR(s->base);

	s->ana = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(s->ana))
		return PTR_ERR(s->ana);

	s->num_clks = ARRAY_SIZE(seninf_clk_names);
	s->clks = devm_kcalloc(dev, s->num_clks, sizeof(*s->clks), GFP_KERNEL);
	if (!s->clks)
		return -ENOMEM;
	for (i = 0; i < s->num_clks; i++)
		s->clks[i].id = seninf_clk_names[i];

	ret = devm_clk_bulk_get(dev, s->num_clks, s->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	/* Media controller + V4L2 device. */
	s->media_dev.dev = dev;
	strscpy(s->media_dev.model, "mtk-mt6785-cam", sizeof(s->media_dev.model));
	media_device_init(&s->media_dev);

	s->v4l2_dev.mdev = &s->media_dev;
	ret = v4l2_device_register(dev, &s->v4l2_dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register V4L2 device\n");
		goto err_media;
	}

	/* Subdev with one source pad per CAMSV sink... here a single one. */
	for (i = 0; i < MT6785_CAM_CSI_PORTS; i++)
		s->pads[i].flags = MEDIA_PAD_FL_SINK;
	s->pads[MT6785_CAM_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	v4l2_subdev_init(&s->sd, &seninf_subdev_ops);
	s->sd.internal_ops = &seninf_internal_ops;
	s->sd.dev = dev;
	/*
	 * Set fwnode explicitly: the async registration path would do it,
	 * but this subdev registers directly under its own v4l2_device.
	 */
	s->sd.fwnode = dev_fwnode(dev);
	s->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	s->sd.entity.ops = &seninf_media_ops;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(s->sd.name, "mtk-mt6785-seninf", sizeof(s->sd.name));
	v4l2_set_subdevdata(&s->sd, s);

	ret = media_entity_pads_init(&s->sd.entity, MT6785_CAM_NUM_PADS, s->pads);
	if (ret)
		goto err_v4l2;

	ret = v4l2_device_register_subdev(&s->v4l2_dev, &s->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_v4l2;
	}

	ret = v4l2_subdev_init_finalize(&s->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev state\n");
		goto err_subdev;
	}

	/* Async notifier: sensor endpoints on ports 0..2, CAMSV on port 3. */
	v4l2_async_nf_init(&s->notifier, &s->v4l2_dev);
	for (i = 0; i < MT6785_CAM_NUM_PADS; i++) {
		ret = seninf_parse_endpoint(s, i);
		if (ret) {
			dev_err_probe(dev, ret,
				      "failed to parse DT endpoint of port %u\n",
				      i);
			goto err_notifier;
		}
	}
	s->notifier.ops = &seninf_async_ops;

	ret = v4l2_async_nf_register(&s->notifier);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register async notifier\n");
		goto err_notifier;
	}

	ret = media_device_register(&s->media_dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register media device\n");
		goto err_nf;
	}

	pm_runtime_set_autosuspend_delay(dev, MT6785_SENINF_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	devm_pm_runtime_enable(dev);
	/* Clocks stay off until the pipeline streams. */
	pm_runtime_set_suspended(dev);

	dev_info(dev, "MT6785 SENINF registered (CSI ports 0-2 -> mux 2 -> CAMSV0)\n");

	return 0;

err_nf:
	v4l2_async_nf_unregister(&s->notifier);
err_notifier:
	v4l2_async_nf_cleanup(&s->notifier);
	v4l2_subdev_cleanup(&s->sd);
err_subdev:
	v4l2_device_unregister_subdev(&s->sd);
err_v4l2:
	v4l2_device_unregister(&s->v4l2_dev);
err_media:
	media_device_cleanup(&s->media_dev);
	return ret;
}

static void seninf_remove(struct platform_device *pdev)
{
	struct mt6785_seninf *s = dev_get_drvdata(&pdev->dev);

	media_device_unregister(&s->media_dev);
	v4l2_async_nf_unregister(&s->notifier);
	v4l2_async_nf_cleanup(&s->notifier);
	v4l2_device_unregister_subdev(&s->sd);
	v4l2_subdev_cleanup(&s->sd);
	v4l2_device_unregister(&s->v4l2_dev);
	media_device_cleanup(&s->media_dev);
}

static const struct of_device_id seninf_of_match[] = {
	{ .compatible = "mediatek,mt6785-seninf" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, seninf_of_match);

static struct platform_driver mt6785_seninf_driver = {
	.probe = seninf_probe,
	.remove = seninf_remove,
	.driver = {
		.name = "mtk-mt6785-seninf",
		.pm = &seninf_pm_ops,
		.of_match_table = seninf_of_match,
	},
};

module_platform_driver(mt6785_seninf_driver);

MODULE_DESCRIPTION("MediaTek MT6785 SENINF CSI-2 receiver driver");
MODULE_LICENSE("GPL");
