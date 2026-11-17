// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K4H7 image sensor driver
 *
 * Copyright (c) 2026 Lenovomarchy project
 *
 * V4L2 subdev driver for the Samsung S5K4H7 (S5K4H7YX family, chip ID 0x487b
 * at registers 0x0000/0x0001) 8Mpixel RAW10 MIPI CSI-2 sensor.  There is no
 * upstream driver; the register sequences are taken from MediaTek GPL vendor
 * drivers (URLs cited next to each table) for a 24 MHz master clock and
 * 4-lane 700 Mbps D-PHY link, which is also the configuration of the Lenovo
 * YT-J706F tablet this driver was written for.
 *
 * The driver structure follows drivers/media/i2c/og0ve1b.c and the register
 * map conventions of drivers/media/i2c/s5kjn1.c (CCS-style layout: streaming
 * 0x0100, orientation 0x0101, exposure 0x0202, analogue gain 0x0204,
 * frame length 0x0340, line length 0x0342, output size 0x034c/0x034e).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Chip identification: 16-bit registers 0x0000..0x0001 on this sensor */
#define S5K4H7_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K4H7_CHIP_ID			0x487b

#define S5K4H7_REG_CTRL_MODE		CCI_REG8(0x0100)
#define S5K4H7_MODE_STANDBY		0x00
#define S5K4H7_MODE_STREAMING		BIT(0)

/* 0x0101[0] horizontal mirror, 0x0101[1] vertical flip */
#define S5K4H7_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K4H7_HFLIP			BIT(0)
#define S5K4H7_VFLIP			BIT(1)

/* Coarse integration time in lines */
#define S5K4H7_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K4H7_EXPOSURE_MIN		2
#define S5K4H7_EXPOSURE_STEP		1

/*
 * Analogue gain, 0x0204/0x0205, linear in 1/32 steps:
 * real gain = code / 32 (32 = 1x).  The Lenovo vendor driver exposes a
 * gain table from 1024 to 16384 in steps of 32, i.e. code 32..512, gain
 * 1x..16x, and sets the default to 4x (code 0x80 = BASEGAIN * 4 / 32).
 */
#define S5K4H7_REG_AGAIN		CCI_REG16(0x0204)
#define S5K4H7_AGAIN_MIN		32
#define S5K4H7_AGAIN_MAX		512
#define S5K4H7_AGAIN_STEP		1
#define S5K4H7_AGAIN_DEFAULT		128

/* Frame and line length, in lines / pixel clocks */
#define S5K4H7_REG_VTS			CCI_REG16(0x0340)
#define S5K4H7_VTS_MAX			0xffff
#define S5K4H7_REG_HTS			CCI_REG16(0x0342)

/* Embedded test pattern generator, 0x0600 low byte selects the pattern */
#define S5K4H7_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define S5K4H7_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define S5K4H7_DATA_LANES		4
#define S5K4H7_BITS_PER_SAMPLE		10

/*
 * Pixel array geometry, in crop-register coordinates (0x0344/0x0345
 * hold the first column, 0x0346/0x0347 the first row).
 *
 * Derived from the register tables above:
 *   - the capture table addresses X 8..3271 and Y 8..2455
 *     (0x0344..0x034b), a 3264x2448 window at (8, 8): the active area;
 *   - the 16:9 window (top 314) and the 640x480 window (left 360,
 *     top 272) sit exactly centred in it (8 + (2448 - 1836) / 2 = 314,
 *     8 + (3264 - 2560) / 2 = 360, 8 + (2448 - 1920) / 2 = 272),
 *     which confirms the offsets.
 *
 * Assumed: the native array is the active area plus the 8 boundary
 * pixels the (8, 8) start implies, symmetric on all four sides
 * (3280x2464).  The vendor tables never address pixels outside the
 * active area, so a wider boundary margin is not observable from them.
 */
#define S5K4H7_NATIVE_WIDTH		3280U
#define S5K4H7_NATIVE_HEIGHT		2464U
#define S5K4H7_PIXEL_ARRAY_LEFT		8U
#define S5K4H7_PIXEL_ARRAY_TOP		8U
#define S5K4H7_PIXEL_ARRAY_WIDTH	3264U
#define S5K4H7_PIXEL_ARRAY_HEIGHT	2448U

/* All vendor modes use pclk 280 MHz: link 700 Mbps x 4 lanes / 10 bits */
#define S5K4H7_LINK_FREQ_700MHZ		(700 * HZ_PER_MHZ)
#define S5K4H7_PIXEL_RATE		(S5K4H7_LINK_FREQ_700MHZ * 2 * \
					 S5K4H7_DATA_LANES / \
					 S5K4H7_BITS_PER_SAMPLE)

#define to_s5k4h7(_sd)			container_of(_sd, struct s5k4h7, sd)

static const s64 s5k4h7_link_freq_menu[] = {
	S5K4H7_LINK_FREQ_700MHZ,
};

/*
 * Register tables below are copied verbatim from MediaTek GPL vendor
 * drivers for the S5K4H7 (chip ID 0x487b), which configure the sensor
 * for a 24 MHz master clock and 4-lane 700 Mbps RAW10 MIPI output
 * (pclk 280 MHz, HTS 3688, VTS 2530 in the full-framerate modes):
 *
 * Lenovo Tab P12 (mt8792) kernel modules, s5k4h7_mipi_raw:
 * https://github.com/lenovo-mt8792-devs/android_kernel_lenovo_mt8792-modules
 *   drivers/media/platform/mtkcam/imgsensor/src-v4l2/common/s5k4h7_mipi_raw/
 *   s5k4h7_Sensor_setting.h (init, preview, capture, normal_video)
 *
 * Xiaomi Merlin (mt6768) kernel, s5k4h7yx_mipi_raw (same register set,
 * tables re-checked against this second source):
 * https://github.com/LineageOS/android_kernel_xiaomi_mt6768
 *   drivers/misc/mediatek/imgsensor/src/mt6768/camera_project/merlin/
 *   s5k4h7yx_mipi_raw/s5k4h7yxmipi_Sensor.c (hs_video 640x480 table)
 */

/*
 * Sensor initialisation, run once after power-up with streaming off.
 * Source: Lenovo mt8792 s5k4h7_Sensor_setting.h addr_data_pair_init[]
 * (identical to the sensor_init() function of the other vendor drivers,
 * down to the trailing 0x30cf digital-gain-disable write).
 */
static const struct cci_reg_sequence s5k4h7_init_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },	/* streaming off */
	{ CCI_REG8(0x0b05), 0x01 },
	{ CCI_REG8(0x3074), 0x06 },
	{ CCI_REG8(0x3075), 0x2f },
	{ CCI_REG8(0x308a), 0x20 },
	{ CCI_REG8(0x308b), 0x08 },
	{ CCI_REG8(0x308c), 0x0b },
	{ CCI_REG8(0x3081), 0x07 },
	{ CCI_REG8(0x307b), 0x85 },
	{ CCI_REG8(0x307a), 0x0a },
	{ CCI_REG8(0x3079), 0x0a },
	{ CCI_REG8(0x306e), 0x71 },
	{ CCI_REG8(0x306f), 0x28 },
	{ CCI_REG8(0x301f), 0x20 },
	{ CCI_REG8(0x306b), 0x9a },
	{ CCI_REG8(0x3091), 0x1f },
	{ CCI_REG8(0x30c4), 0x06 },
	{ CCI_REG8(0x3200), 0x09 },
	{ CCI_REG8(0x306a), 0x79 },
	{ CCI_REG8(0x30b0), 0xff },
	{ CCI_REG8(0x306d), 0x08 },
	{ CCI_REG8(0x3080), 0x00 },
	{ CCI_REG8(0x3929), 0x3f },
	{ CCI_REG8(0x3084), 0x16 },
	{ CCI_REG8(0x3070), 0x0f },
	{ CCI_REG8(0x3b45), 0x01 },
	{ CCI_REG8(0x30c2), 0x05 },
	{ CCI_REG8(0x3069), 0x87 },
	{ CCI_REG8(0x3924), 0x7f },
	{ CCI_REG8(0x3925), 0xfd },
	{ CCI_REG8(0x3c08), 0xff },
	{ CCI_REG8(0x3c09), 0xff },
	{ CCI_REG8(0x3c31), 0xff },
	{ CCI_REG8(0x3c32), 0xff },
	{ CCI_REG8(0x30cf), 0x00 },	/* digital gain off */
};

/*
 * 3264x2448 full resolution, 30 fps, no binning (0x0387 = 0x01).
 * Source: Lenovo mt8792 s5k4h7_Sensor_setting.h addr_data_pair_capture[].
 */
static const struct cci_reg_sequence s5k4h7_3264x2448_30fps_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },	/* streaming off */
	{ CCI_REG8(0x0136), 0x18 },	/* MCLK 24 MHz */
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },	/* PLL */
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xaf },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x0301), 0x04 },
	{ CCI_REG8(0x0820), 0x02 },	/* MIPI 700 Mbps */
	{ CCI_REG8(0x0821), 0xbc },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	{ CCI_REG8(0x0112), 0x0a },	/* RAW10, YUV interleaved off */
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },	/* 4 data lanes */
	{ CCI_REG8(0x3906), 0x04 },
	{ CCI_REG8(0x0344), 0x00 },	/* crop 0x0008..0x0cc7 x 0x0008..0x0997 */
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0c },
	{ CCI_REG8(0x0349), 0xc7 },
	{ CCI_REG8(0x034a), 0x09 },
	{ CCI_REG8(0x034b), 0x97 },
	{ CCI_REG8(0x034c), 0x0c },	/* output 3264 */
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x09 },	/* output 2448 */
	{ CCI_REG8(0x034f), 0x90 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },	/* no binning */
	{ CCI_REG8(0x0101), 0x00 },	/* orientation normal */
	{ CCI_REG8(0x0340), 0x09 },	/* VTS 2530 */
	{ CCI_REG8(0x0341), 0xe2 },
	{ CCI_REG8(0x0342), 0x0e },	/* HTS 3688 */
	{ CCI_REG8(0x0343), 0x68 },
	{ CCI_REG8(0x0200), 0x0d },	/* exposure limit 0x0dd8 lines */
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x02 },	/* exposure 0x0208 lines */
	{ CCI_REG8(0x0203), 0x08 },
	{ CCI_REG8(0x3400), 0x01 },	/* LSC off */
};

/*
 * 1632x1224 2x2 binned, 30 fps (0x0387 = 0x03, 0x0900/0x0901 = 0x01/0x22).
 * Source: Lenovo mt8792 s5k4h7_Sensor_setting.h addr_data_pair_preview[].
 */
static const struct cci_reg_sequence s5k4h7_1632x1224_30fps_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xaf },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x0301), 0x04 },
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0xbc },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x3906), 0x00 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0c },
	{ CCI_REG8(0x0349), 0xc7 },
	{ CCI_REG8(0x034a), 0x09 },
	{ CCI_REG8(0x034b), 0x97 },
	{ CCI_REG8(0x034c), 0x06 },	/* output 1632 */
	{ CCI_REG8(0x034d), 0x60 },
	{ CCI_REG8(0x034e), 0x04 },	/* output 1224 */
	{ CCI_REG8(0x034f), 0xc8 },
	{ CCI_REG8(0x0900), 0x01 },	/* binned mode */
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x03 },	/* 2x2 binning */
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xe2 },
	{ CCI_REG8(0x0342), 0x0e },
	{ CCI_REG8(0x0343), 0x68 },
	{ CCI_REG8(0x0200), 0x0d },
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x08 },
};

/*
 * 3264x1836 16:9 crop, 30 fps (same PLL and timings as capture).
 * Source: Lenovo mt8792 s5k4h7_Sensor_setting.h
 * addr_data_pair_normal_video[].
 */
static const struct cci_reg_sequence s5k4h7_3264x1836_30fps_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xaf },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x0301), 0x04 },
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0xbc },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x3906), 0x04 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x01 },	/* vertical crop start 0x013a */
	{ CCI_REG8(0x0347), 0x3a },
	{ CCI_REG8(0x0348), 0x0c },
	{ CCI_REG8(0x0349), 0xc7 },
	{ CCI_REG8(0x034a), 0x08 },	/* vertical crop end 0x0865 */
	{ CCI_REG8(0x034b), 0x65 },
	{ CCI_REG8(0x034c), 0x0c },	/* output 3264 */
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x07 },	/* output 1836 */
	{ CCI_REG8(0x034f), 0x2c },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xe2 },
	{ CCI_REG8(0x0342), 0x0e },
	{ CCI_REG8(0x0343), 0x68 },
	{ CCI_REG8(0x0200), 0x0d },
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x08 },
	{ CCI_REG8(0x3400), 0x01 },
};

/*
 * 640x480 4x4 binned, 120 fps.  The Lenovo tree carries no such mode; this
 * table is the S5K4H7YX (same silicon family, same PLL registers 0x030f =
 * 0xaf, 0x0821 = 0xbc = 700 Mbps) hs_video sequence of the Xiaomi Merlin
 * mt6768 vendor driver, re-checked register by register.  VTS 0x0278.
 * Source: LineageOS android_kernel_xiaomi_mt6768, merlin s5k4h7yx_mipi_raw,
 * hs_video_setting().
 */
static const struct cci_reg_sequence s5k4h7_640x480_120fps_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xaf },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x0301), 0x04 },
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0xbc },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x3906), 0x00 },
	{ CCI_REG8(0x0344), 0x01 },	/* crop 0x0168..0x0b67 x 0x0110..0x088f */
	{ CCI_REG8(0x0345), 0x68 },
	{ CCI_REG8(0x0346), 0x01 },
	{ CCI_REG8(0x0347), 0x10 },
	{ CCI_REG8(0x0348), 0x0b },
	{ CCI_REG8(0x0349), 0x67 },
	{ CCI_REG8(0x034a), 0x08 },
	{ CCI_REG8(0x034b), 0x8f },
	{ CCI_REG8(0x034c), 0x02 },	/* output 640 */
	{ CCI_REG8(0x034d), 0x80 },
	{ CCI_REG8(0x034e), 0x01 },	/* output 480 */
	{ CCI_REG8(0x034f), 0xe0 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x44 },	/* 4x4 binning */
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x07 },	/* 4x4 binning */
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0340), 0x02 },	/* VTS 0x0278 */
	{ CCI_REG8(0x0341), 0x78 },
	{ CCI_REG8(0x0342), 0x0e },	/* HTS 3688 */
	{ CCI_REG8(0x0343), 0x68 },
	{ CCI_REG8(0x0200), 0x0d },
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x08 },
	{ CCI_REG8(0x3400), 0x01 },
};

struct s5k4h7_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k4h7_mode {
	u32 width;			/* Frame width in pixels */
	u32 height;			/* Frame height in pixels */
	u32 hts;			/* Horizontal timing size, pixels */
	u32 vts;			/* Default vertical timing size, lines */
	u32 exposure_default;		/* Default exposure, lines */
	u32 exposure_margin;		/* Margin between VTS and exposure */

	/* Analogue crop rectangle, crop-register coordinates */
	struct v4l2_rect crop;

	const struct s5k4h7_reg_list reg_list;
};

/*
 * All modes run the same 280 MHz pixel clock, HTS 3688.  Crop
 * rectangles are read from each table's 0x0344..0x034b window registers
 * (start inclusive, end inclusive): full active area for 3264x2448 and
 * the 2x2-binned 1632x1224, 16:9 window for 3264x1836, 2560x1920
 * 4x4-binned window for 640x480.
 */
static const struct s5k4h7_mode s5k4h7_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.hts = 3688,
		.vts = 2530,
		.exposure_default = 0x208,
		.exposure_margin = 6,
		.crop = {
			.left = 8,
			.top = 8,
			.width = 3264,
			.height = 2448,
		},
		.reg_list = {
			.regs = s5k4h7_3264x2448_30fps_regs,
			.num_regs = ARRAY_SIZE(s5k4h7_3264x2448_30fps_regs),
		},
	},
	{
		.width = 3264,
		.height = 1836,
		.hts = 3688,
		.vts = 2530,
		.exposure_default = 0x208,
		.exposure_margin = 6,
		.crop = {
			.left = 8,
			.top = 314,
			.width = 3264,
			.height = 1836,
		},
		.reg_list = {
			.regs = s5k4h7_3264x1836_30fps_regs,
			.num_regs = ARRAY_SIZE(s5k4h7_3264x1836_30fps_regs),
		},
	},
	{
		.width = 1632,
		.height = 1224,
		.hts = 3688,
		.vts = 2530,
		.exposure_default = 0x208,
		.exposure_margin = 6,
		.crop = {
			.left = 8,
			.top = 8,
			.width = 3264,
			.height = 2448,
		},
		.reg_list = {
			.regs = s5k4h7_1632x1224_30fps_regs,
			.num_regs = ARRAY_SIZE(s5k4h7_1632x1224_30fps_regs),
		},
	},
	{
		.width = 640,
		.height = 480,
		.hts = 3688,
		.vts = 0x278,
		.exposure_default = 0x208,
		.exposure_margin = 6,
		.crop = {
			.left = 360,
			.top = 272,
			.width = 2560,
			.height = 1920,
		},
		.reg_list = {
			.regs = s5k4h7_640x480_120fps_regs,
			.num_regs = ARRAY_SIZE(s5k4h7_640x480_120fps_regs),
		},
	},
};

/*
 * Test pattern values for 0x0600, from the vendor drivers: 0x0001 is the
 * colour-bar mode used by the Lenovo and Realme drivers, 0x0000 disables
 * the generator.  Higher values follow the Samsung convention seen in
 * other S5K drivers (s5kjn1 0x0600 menu); only the colour bar was ever
 * exercised by the vendor code.
 */
static const char * const s5k4h7_test_pattern_menu[] = {
	"Disabled",
	"Color bars",
	"Solid color",
	"PN9",
};

static const char * const s5k4h7_supply_names[] = {
	"avdd",		/* Analog power, 2.8 V */
	"dovdd",	/* Digital I/O power, 1.8 V */
	"dvdd",		/* Digital core power, 1.05-1.2 V */
};

#define S5K4H7_NUM_SUPPLIES	ARRAY_SIZE(s5k4h7_supply_names)

struct s5k4h7 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K4H7_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	const struct s5k4h7_mode *mode;
};

/* Bayer order changes with the flips, like on s5kjn1 */
static const u32 s5k4h7_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,	MEDIA_BUS_FMT_SGBRG10_1X10,
};

static u32 s5k4h7_get_format_code(struct s5k4h7 *s5k4h7)
{
	unsigned int i;

	i = (s5k4h7->vflip->val ? 2 : 0) | (s5k4h7->hflip->val ? 1 : 0);

	return s5k4h7_mbus_formats[i];
}

static int s5k4h7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7 *s5k4h7 = container_of(ctrl->handler, struct s5k4h7,
					     ctrl_handler);
	const struct s5k4h7_mode *mode = s5k4h7->mode;
	s64 exposure_max;
	int ret;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->height + ctrl->val - mode->exposure_margin;
		__v4l2_ctrl_modify_range(s5k4h7->exposure,
					 s5k4h7->exposure->minimum,
					 exposure_max,
					 s5k4h7->exposure->step,
					 s5k4h7->exposure->default_value);
		break;
	}

	/* V4L2 controls are applied when the sensor is powered up */
	if (!pm_runtime_get_if_active(s5k4h7->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_AGAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_ORIENTATION,
				(s5k4h7->vflip->val ? S5K4H7_VFLIP : 0) |
				(s5k4h7->hflip->val ? S5K4H7_HFLIP : 0), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k4h7->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7_ctrl_ops = {
	.s_ctrl = s5k4h7_set_ctrl,
};

static int s5k4h7_init_controls(struct s5k4h7 *s5k4h7)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k4h7->ctrl_handler;
	const struct s5k4h7_mode *mode = s5k4h7->mode;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, h_blank, v_blank;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 10);

	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5k4h7_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(s5k4h7_link_freq_menu) - 1,
				      0, s5k4h7_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  0, S5K4H7_PIXEL_RATE, 1, S5K4H7_PIXEL_RATE);

	h_blank = mode->hts - mode->width;
	s5k4h7->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					   V4L2_CID_HBLANK, h_blank, h_blank,
					   1, h_blank);
	if (s5k4h7->hblank)
		s5k4h7->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v_blank = mode->vts - mode->height;
	s5k4h7->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					   V4L2_CID_VBLANK, v_blank,
					   S5K4H7_VTS_MAX - mode->height, 1,
					   v_blank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K4H7_AGAIN_MIN, S5K4H7_AGAIN_MAX,
			  S5K4H7_AGAIN_STEP, S5K4H7_AGAIN_DEFAULT);

	exposure_max = mode->vts - mode->exposure_margin;
	s5k4h7->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K4H7_EXPOSURE_MIN,
					     exposure_max,
					     S5K4H7_EXPOSURE_STEP,
					     mode->exposure_default);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k4h7_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k4h7_test_pattern_menu) - 1,
				     0, 0, s5k4h7_test_pattern_menu);

	s5k4h7->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k4h7->hflip)
		s5k4h7->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k4h7->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k4h7->vflip)
		s5k4h7->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	ret = v4l2_fwnode_device_parse(s5k4h7->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k4h7_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	s5k4h7->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void s5k4h7_update_pad_format(struct s5k4h7 *s5k4h7,
				     const struct s5k4h7_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k4h7_get_format_code(s5k4h7);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k4h7_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5k4h7_mode *mode;
	struct v4l2_rect *crop;

	mode = v4l2_find_nearest_size(s5k4h7_modes,
				      ARRAY_SIZE(s5k4h7_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5k4h7_update_pad_format(s5k4h7, mode, &fmt->format);
	crop = v4l2_subdev_state_get_crop(state, 0);

	/* Format code can change with the flip controls */
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5k4h7->mode == mode)
		goto set_format;

	/* Update limits and set FPS and exposure to default values */
	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5k4h7->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5k4h7->vblank, vblank,
				 S5K4H7_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k4h7->vblank, vblank);

	exposure_max = mode->vts - mode->exposure_margin;
	__v4l2_ctrl_modify_range(s5k4h7->exposure, S5K4H7_EXPOSURE_MIN,
				 exposure_max, S5K4H7_EXPOSURE_STEP,
				 mode->exposure_default);
	__v4l2_ctrl_s_ctrl(s5k4h7->exposure, mode->exposure_default);

	if (s5k4h7->sd.ctrl_handler->error)
		return s5k4h7->sd.ctrl_handler->error;

	s5k4h7->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*crop = mode->crop;

	return 0;
}

static int s5k4h7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	/* Media bus code index is constant, but code formats are not */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k4h7_get_format_code(s5k4h7);

	return 0;
}

static int s5k4h7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->pad != 0)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = S5K4H7_PIXEL_ARRAY_TOP;
		sel->r.left = S5K4H7_PIXEL_ARRAY_LEFT;
		sel->r.width = S5K4H7_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5K4H7_PIXEL_ARRAY_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = S5K4H7_NATIVE_WIDTH;
		sel->r.height = S5K4H7_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k4h7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k4h7_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10 &&
	    fse->code != MEDIA_BUS_FMT_SRGGB10_1X10 &&
	    fse->code != MEDIA_BUS_FMT_SBGGR10_1X10 &&
	    fse->code != MEDIA_BUS_FMT_SGBRG10_1X10)
		return -EINVAL;

	fse->min_width = s5k4h7_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4h7_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k4h7_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	const struct s5k4h7_reg_list *reg_list = &s5k4h7->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5k4h7->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(s5k4h7->regmap, s5k4h7_init_regs,
				  ARRAY_SIZE(s5k4h7_init_regs), NULL);
	if (ret) {
		dev_err(s5k4h7->dev, "failed to write init settings: %d\n", ret);
		goto error;
	}

	ret = cci_multi_reg_write(s5k4h7->regmap, reg_list->regs,
				  reg_list->num_regs, NULL);
	if (ret) {
		dev_err(s5k4h7->dev, "failed to set mode: %d\n", ret);
		goto error;
	}

	ret = __v4l2_ctrl_handler_setup(s5k4h7->sd.ctrl_handler);
	if (ret)
		goto error;

	/*
	 * Wait for the PLLs to lock before enabling the interface, like the
	 * vendor drivers' stream-on paths (they poll 0x0005 for the frame
	 * counter to stop first; a fixed delay is safe for a first bring-up).
	 */
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_CTRL_MODE,
			S5K4H7_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(s5k4h7->dev, "failed to start streaming: %d\n", ret);
		goto error;
	}

	return 0;

error:
	pm_runtime_put_autosuspend(s5k4h7->dev);

	return ret;
}

static int s5k4h7_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	int ret;

	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_CTRL_MODE,
			S5K4H7_MODE_STANDBY, NULL);
	if (ret)
		dev_err(s5k4h7->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5k4h7->dev);

	return ret;
}

static int s5k4h7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = MEDIA_BUS_FMT_SGRBG10_1X10,
			.width = s5k4h7_modes[0].width,
			.height = s5k4h7_modes[0].height,
		},
	};

	s5k4h7_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k4h7_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k4h7_pad_ops = {
	.set_fmt = s5k4h7_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.enum_mbus_code = s5k4h7_enum_mbus_code,
	.enum_frame_size = s5k4h7_enum_frame_size,
	.get_selection = s5k4h7_get_selection,
	.enable_streams = s5k4h7_enable_streams,
	.disable_streams = s5k4h7_disable_streams,
};

static const struct v4l2_subdev_ops s5k4h7_subdev_ops = {
	.video = &s5k4h7_video_ops,
	.pad = &s5k4h7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k4h7_internal_ops = {
	.init_state = s5k4h7_init_state,
};

static const struct media_entity_operations s5k4h7_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k4h7_identify_sensor(struct s5k4h7 *s5k4h7)
{
	u64 val;
	int ret;

	ret = cci_read(s5k4h7->regmap, S5K4H7_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5k4h7->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5K4H7_CHIP_ID) {
		dev_err(s5k4h7->dev, "chip id mismatch: %x != %llx\n",
			S5K4H7_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int s5k4h7_check_hwcfg(struct s5k4h7 *s5k4h7)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k4h7->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K4H7_DATA_LANES) {
		dev_err(s5k4h7->dev, "invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k4h7->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k4h7_link_freq_menu,
				       ARRAY_SIZE(s5k4h7_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k4h7_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	int ret;

	/*
	 * Power sequence of the vendor drivers (Lenovo mt8792 pw_seq):
	 * MCLK and core/I/O rails first, reset released last.  The reset
	 * GPIO is active low in the gpiod sense (stock DT: high = run).
	 */
	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	ret = regulator_bulk_enable(S5K4H7_NUM_SUPPLIES, s5k4h7->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(s5k4h7->mclk);
	if (ret)
		goto disable_regulators;

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 0);
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	return 0;

disable_regulators:
	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	regulator_bulk_disable(S5K4H7_NUM_SUPPLIES, s5k4h7->supplies);

	return ret;
}

static int s5k4h7_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	clk_disable_unprepare(s5k4h7->mclk);
	regulator_bulk_disable(S5K4H7_NUM_SUPPLIES, s5k4h7->supplies);

	return 0;
}

static int s5k4h7_probe(struct i2c_client *client)
{
	struct s5k4h7 *s5k4h7;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5k4h7 = devm_kzalloc(&client->dev, sizeof(*s5k4h7), GFP_KERNEL);
	if (!s5k4h7)
		return -ENOMEM;

	s5k4h7->dev = &client->dev;

	v4l2_i2c_subdev_init(&s5k4h7->sd, client, &s5k4h7_subdev_ops);

	s5k4h7->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k4h7->regmap))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->regmap),
				     "failed to init CCI\n");

	s5k4h7->mclk = devm_v4l2_sensor_clk_get(s5k4h7->dev, NULL);
	if (IS_ERR(s5k4h7->mclk))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5k4h7->mclk);
	if (freq != S5K4H7_MCLK_FREQ_24MHZ)
		return dev_err_probe(s5k4h7->dev, -EINVAL,
				     "MCLK frequency %lu is not supported\n",
				     freq);

	ret = s5k4h7_check_hwcfg(s5k4h7);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret,
				     "failed to check HW configuration\n");

	/* Active low: asserted (1) keeps the sensor in reset */
	s5k4h7->reset_gpio = devm_gpiod_get_optional(s5k4h7->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k4h7->reset_gpio))
		return dev_err_probe(s5k4h7->dev,
				     PTR_ERR(s5k4h7->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < S5K4H7_NUM_SUPPLIES; i++)
		s5k4h7->supplies[i].supply = s5k4h7_supply_names[i];

	ret = devm_regulator_bulk_get(s5k4h7->dev, S5K4H7_NUM_SUPPLIES,
				      s5k4h7->supplies);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret,
				     "failed to get supply regulators\n");

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5k4h7_power_on(s5k4h7->dev);
	if (ret)
		return ret;

	ret = s5k4h7_identify_sensor(s5k4h7);
	if (ret) {
		dev_err_probe(s5k4h7->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	s5k4h7->mode = &s5k4h7_modes[0];
	ret = s5k4h7_init_controls(s5k4h7);
	if (ret) {
		dev_err_probe(s5k4h7->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5k4h7->sd.state_lock = s5k4h7->ctrl_handler.lock;
	s5k4h7->sd.internal_ops = &s5k4h7_internal_ops;
	s5k4h7->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k4h7->sd.entity.ops = &s5k4h7_subdev_entity_ops;
	s5k4h7->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k4h7->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k4h7->sd.entity, 1, &s5k4h7->pad);
	if (ret) {
		dev_err_probe(s5k4h7->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5k4h7->sd);
	if (ret < 0) {
		dev_err_probe(s5k4h7->dev, ret,
			      "failed to finalize subdev init\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5k4h7->dev);
	pm_runtime_enable(s5k4h7->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k4h7->sd);
	if (ret < 0) {
		dev_err_probe(s5k4h7->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_autosuspend_delay(s5k4h7->dev, 1000);
	pm_runtime_use_autosuspend(s5k4h7->dev);
	pm_runtime_idle(s5k4h7->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5k4h7->sd);
	pm_runtime_disable(s5k4h7->dev);
	pm_runtime_set_suspended(s5k4h7->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5k4h7->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5k4h7->sd.ctrl_handler);

power_off:
	s5k4h7_power_off(s5k4h7->dev);

	return ret;
}

static void s5k4h7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k4h7->dev);

	if (!pm_runtime_status_suspended(s5k4h7->dev)) {
		s5k4h7_power_off(s5k4h7->dev);
		pm_runtime_set_suspended(s5k4h7->dev);
	}
}

static const struct dev_pm_ops s5k4h7_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4h7_power_off, s5k4h7_power_on, NULL)
};

static const struct of_device_id s5k4h7_of_match[] = {
	{ .compatible = "samsung,s5k4h7" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4h7_of_match);

static struct i2c_driver s5k4h7_i2c_driver = {
	.driver = {
		.name = "s5k4h7",
		.pm = &s5k4h7_pm_ops,
		.of_match_table = s5k4h7_of_match,
	},
	.probe = s5k4h7_probe,
	.remove = s5k4h7_remove,
};

module_i2c_driver(s5k4h7_i2c_driver);

MODULE_AUTHOR("Lenovomarchy project");
MODULE_DESCRIPTION("Samsung S5K4H7 image sensor driver");
MODULE_LICENSE("GPL");
