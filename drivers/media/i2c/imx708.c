// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX708 cameras.
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2020 Raspberry Pi Ltd
 *
 * V0.0X01.0X00 first version.
 * TODO: Implement Rockchip HDR functionality
 * TODO: Implement EEPROM read/write
 * TODO: Implement extraction of CSI DPHY parameters
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <linux/rk-camera-module.h>
#include <linux/version.h>

#define DRIVER_VERSION KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN V4L2_CID_GAIN
#endif

#ifndef V4L2_CID_NOTIFY_GAINS
#define V4L2_CID_NOTIFY_GAINS			(V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 9)
#endif

#define IMX708_NAME "imx708"
#define OF_CAMERA_HDR_MODE "rockchip,camera-hdr-mode"

#define IMX708_LANES 2

/*
 * Parameter to adjust Quad Bayer re-mosaic broken line correction
 * strength, used in full-resolution mode only. Set zero to disable.
 */
static int qbc_adjust = 2;
module_param(qbc_adjust, int, 0644);
MODULE_PARM_DESC(qbc_adjust, "Quad Bayer broken line correction strength [0,2-5]");

#define IMX708_REG_VALUE_08BIT		1
#define IMX708_REG_VALUE_16BIT		2

/* Chip ID */
#define IMX708_REG_CHIP_ID		0x0016
#define IMX708_CHIP_ID			0x0708

#define IMX708_REG_MODE_SELECT		0x0100
#define IMX708_MODE_STANDBY		0x00
#define IMX708_MODE_STREAMING		0x01

#define IMX708_REG_ORIENTATION		0x101

#define IMX708_INCLK_FREQ		24000000

/* Default initial pixel rate, will get updated for each mode. */
#define IMX708_INITIAL_PIXEL_RATE	590000000

/* V_TIMING internal */
#define IMX708_REG_FRAME_LENGTH		0x0340
#define IMX708_FRAME_LENGTH_MAX		0xffff

/* Long exposure multiplier */
#define IMX708_LONG_EXP_SHIFT_MAX	7
#define IMX708_LONG_EXP_SHIFT_REG	0x3100

/* Exposure control */
#define IMX708_REG_EXPOSURE		0x0202
#define IMX708_EXPOSURE_OFFSET		48
#define IMX708_EXPOSURE_DEFAULT		0x640
#define IMX708_EXPOSURE_STEP		1
#define IMX708_EXPOSURE_MIN		1
#define IMX708_EXPOSURE_MAX		(IMX708_FRAME_LENGTH_MAX - \
					 IMX708_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX708_REG_ANALOG_GAIN		0x0204
#define IMX708_ANA_GAIN_MIN		112
#define IMX708_ANA_GAIN_MAX		960
#define IMX708_ANA_GAIN_STEP		1
#define IMX708_ANA_GAIN_DEFAULT	   IMX708_ANA_GAIN_MIN

/* Digital gain control */
#define IMX708_REG_DIGITAL_GAIN		0x020e
#define IMX708_DGTL_GAIN_MIN		0x0100
#define IMX708_DGTL_GAIN_MAX		0xffff
#define IMX708_DGTL_GAIN_DEFAULT	0x0100
#define IMX708_DGTL_GAIN_STEP		1

/* Colour balance controls */
#define IMX708_REG_COLOUR_BALANCE_RED   0x0b90
#define IMX708_REG_COLOUR_BALANCE_BLUE	0x0b92
#define IMX708_COLOUR_BALANCE_MIN	0x01
#define IMX708_COLOUR_BALANCE_MAX	0xffff
#define IMX708_COLOUR_BALANCE_STEP	0x01
#define IMX708_COLOUR_BALANCE_DEFAULT	0x100

/* Test Pattern Control */
#define IMX708_REG_TEST_PATTERN		0x0600
#define IMX708_TEST_PATTERN_DISABLE	0
#define IMX708_TEST_PATTERN_SOLID_COLOR	1
#define IMX708_TEST_PATTERN_COLOR_BARS	2
#define IMX708_TEST_PATTERN_GREY_COLOR	3
#define IMX708_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX708_REG_TEST_PATTERN_R	0x0602
#define IMX708_REG_TEST_PATTERN_GR	0x0604
#define IMX708_REG_TEST_PATTERN_B	0x0606
#define IMX708_REG_TEST_PATTERN_GB	0x0608
#define IMX708_TEST_PATTERN_COLOUR_MIN	0
#define IMX708_TEST_PATTERN_COLOUR_MAX	0x0fff
#define IMX708_TEST_PATTERN_COLOUR_STEP	1

#define IMX708_REG_BASE_SPC_GAINS_L	0x7b10
#define IMX708_REG_BASE_SPC_GAINS_R	0x7c00

/* HDR exposure ratio (long:med == med:short) */
#define IMX708_HDR_EXPOSURE_RATIO       4
#define IMX708_REG_MID_EXPOSURE		0x3116
#define IMX708_REG_SHT_EXPOSURE		0x0224
#define IMX708_REG_MID_ANALOG_GAIN	0x3118
#define IMX708_REG_SHT_ANALOG_GAIN	0x0216

/* QBC Re-mosaic broken line correction registers */
#define IMX708_LPF_INTENSITY_EN		0xC428
#define IMX708_LPF_INTENSITY_ENABLED	0x00
#define IMX708_LPF_INTENSITY_DISABLED	0x01
#define IMX708_LPF_INTENSITY		0xC429

/* IMX708 native and active pixel array size. */
#define IMX708_NATIVE_WIDTH		4640U
#define IMX708_NATIVE_HEIGHT		2658U
#define IMX708_PIXEL_ARRAY_LEFT		16U
#define IMX708_PIXEL_ARRAY_TOP		24U
#define IMX708_PIXEL_ARRAY_WIDTH	4608U
#define IMX708_PIXEL_ARRAY_HEIGHT	2592U

struct imx708_reg {
	u16 address;
	u8 val;
};

struct imx708_reg_list {
	unsigned int num_of_regs;
	const struct imx708_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx708_mode {
	u32 bus_fmt;

	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	struct v4l2_fract max_fps;

	/* H-timing in pixels */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	unsigned int vblank_min;

	/* Default framerate. */
	unsigned int vblank_default;

	/* Default register values */
	struct imx708_reg_list reg_list;

	/* Not all modes have the same pixel rate. */
	u64 pixel_rate;

	/* Not all modes have the same minimum exposure. */
	u32 exposure_lines_min;

	/* Not all modes have the same exposure lines step. */
	u32 exposure_lines_step;

	/* Rockchip HDR mode */
	u32 rk_hdr_mode;

	/* VC numbers for each pad */
	u32 vc[PAD_MAX];

	/* Bit depth */
	u32 bpp;

	/* Quad Bayer Re-mosaic flag */
	bool remosaic;
};

/* Default PDAF pixel correction gains */
static const u8 pdaf_gains[2][9] = {
	{ 0x4c, 0x4c, 0x4c, 0x46, 0x3e, 0x38, 0x35, 0x35, 0x35 },
	{ 0x35, 0x35, 0x35, 0x38, 0x3e, 0x46, 0x4c, 0x4c, 0x4c }
};

/* Link frequency setup */
enum {
	IMX708_LINK_FREQ_450MHZ,
	IMX708_LINK_FREQ_447MHZ,
	IMX708_LINK_FREQ_453MHZ,
};

static const s64 link_freqs[] = {
	[IMX708_LINK_FREQ_450MHZ] = 450000000,
	[IMX708_LINK_FREQ_447MHZ] = 447000000,
	[IMX708_LINK_FREQ_453MHZ] = 453000000,
};

/* 450MHz is the nominal "default" link frequency */
static const struct imx708_reg link_450Mhz_regs[] = {
	{0x030E, 0x01},
	{0x030F, 0x2c},
};

static const struct imx708_reg link_447Mhz_regs[] = {
	{0x030E, 0x01},
	{0x030F, 0x2a},
};

static const struct imx708_reg link_453Mhz_regs[] = {
	{0x030E, 0x01},
	{0x030F, 0x2e},
};

static const struct imx708_reg_list link_freq_regs[] = {
	[IMX708_LINK_FREQ_450MHZ] = {
		.regs = link_450Mhz_regs,
		.num_of_regs = ARRAY_SIZE(link_450Mhz_regs)
	},
	[IMX708_LINK_FREQ_447MHZ] = {
		.regs = link_447Mhz_regs,
		.num_of_regs = ARRAY_SIZE(link_447Mhz_regs)
	},
	[IMX708_LINK_FREQ_453MHZ] = {
		.regs = link_453Mhz_regs,
		.num_of_regs = ARRAY_SIZE(link_453Mhz_regs)
	},
};

static const struct imx708_reg mode_common_regs[] = {
	{0x0100, 0x00},
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x33F0, 0x02},
	{0x33F1, 0x05},
	{0x3062, 0x00},
	{0x3063, 0x12},
	{0x3068, 0x00},
	{0x3069, 0x12},
	{0x306A, 0x00},
	{0x306B, 0x30},
	{0x3076, 0x00},
	{0x3077, 0x30},
	{0x3078, 0x00},
	{0x3079, 0x30},
	{0x5E54, 0x0C},
	{0x6E44, 0x00},
	{0xB0B6, 0x01},
	{0xE829, 0x00},
	{0xF001, 0x08},
	{0xF003, 0x08},
	{0xF00D, 0x10},
	{0xF00F, 0x10},
	{0xF031, 0x08},
	{0xF033, 0x08},
	{0xF03D, 0x10},
	{0xF03F, 0x10},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x01},
	{0x0B8E, 0x01},
	{0x0B8F, 0x00},
	{0x0B94, 0x01},
	{0x0B95, 0x00},
	{0x3400, 0x01},
	{0x3478, 0x01},
	{0x3479, 0x1c},
	{0x3091, 0x01},
	{0x3092, 0x00},
	{0x3419, 0x00},
	{0xBCF1, 0x02},
	{0x3094, 0x01},
	{0x3095, 0x01},
	{0x3362, 0x00},
	{0x3363, 0x00},
	{0x3364, 0x00},
	{0x3365, 0x00},
	{0x0138, 0x01},
};

/* 10-bit. */
static const struct imx708_reg mode_4608x2592_regs[] = {
	{0x0342, 0x3D},
	{0x0343, 0x20},
	{0x0340, 0x0A},
	{0x0341, 0x59},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x11},
	{0x0349, 0xFF},
	{0x034A, 0X0A},
	{0x034B, 0x1F},
	{0x0220, 0x62},
	{0x0222, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x0A},
	{0x3200, 0x01},
	{0x3201, 0x01},
	{0x32D5, 0x01},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x00},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x12},
	{0x040D, 0x00},
	{0x040E, 0x0A},
	{0x040F, 0x20},
	{0x034C, 0x12},
	{0x034D, 0x00},
	{0x034E, 0x0A},
	{0x034F, 0x20},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0x7C},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x64},
	{0x3CA4, 0x00},
	{0x3CA5, 0x00},
	{0x3CA6, 0x00},
	{0x3CA7, 0x00},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x08},
	{0x3CBA, 0x00},
	{0x3CBB, 0x00},
	{0x3CBC, 0x00},
	{0x3CBD, 0x3C},
	{0x3CBE, 0x00},
	{0x3CBF, 0x00},
	{0x0202, 0x0A},
	{0x0203, 0x29},
	{0x0224, 0x01},
	{0x0225, 0xF4},
	{0x3116, 0x01},
	{0x3117, 0xF4},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x00},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x01},
	{0x341f, 0x20},
	{0x3420, 0x00},
	{0x3421, 0xd8},
	{0x3366, 0x00},
	{0x3367, 0x00},
	{0x3368, 0x00},
	{0x3369, 0x00},
};


static const struct imx708_reg mode_2x2binned_regs[] = {
	{0x0342, 0x1E},
	{0x0343, 0x90},
	{0x0340, 0x05},
	{0x0341, 0x38},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x11},
	{0x0349, 0xFF},
	{0x034A, 0X0A},
	{0x034B, 0x1F},
	{0x0220, 0x62},
	{0x0222, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x08},
	{0x3200, 0x41},
	{0x3201, 0x41},
	{0x32D5, 0x00},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x00},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x09},
	{0x040D, 0x00},
	{0x040E, 0x05},
	{0x040F, 0x10},
	{0x034C, 0x09},
	{0x034D, 0x00},
	{0x034E, 0x05},
	{0x034F, 0x10},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0x7A},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x3C},
	{0x3CA4, 0x00},
	{0x3CA5, 0x3C},
	{0x3CA6, 0x00},
	{0x3CA7, 0x00},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x1C},
	{0x3CBA, 0x00},
	{0x3CBB, 0x08},
	{0x3CBC, 0x00},
	{0x3CBD, 0x1E},
	{0x3CBE, 0x00},
	{0x3CBF, 0x0A},
	{0x0202, 0x05},
	{0x0203, 0x08},
	{0x0224, 0x01},
	{0x0225, 0xF4},
	{0x3116, 0x01},
	{0x3117, 0xF4},
	{0x0204, 0x00},
	{0x0205, 0x70},
	{0x0216, 0x00},
	{0x0217, 0x70},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x70},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x00},
	{0x341f, 0x90},
	{0x3420, 0x00},
	{0x3421, 0x6c},
	{0x3366, 0x00},
	{0x3367, 0x00},
	{0x3368, 0x00},
	{0x3369, 0x00},
};

static const struct imx708_reg mode_2x2binned_720p_regs[] = {
	{0x0342, 0x14},
	{0x0343, 0x60},
	{0x0340, 0x04},
	{0x0341, 0xB6},
	{0x0344, 0x03},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xB0},
	{0x0348, 0x0E},
	{0x0349, 0xFF},
	{0x034A, 0x08},
	{0x034B, 0x6F},
	{0x0220, 0x62},
	{0x0222, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x08},
	{0x3200, 0x41},
	{0x3201, 0x41},
	{0x32D5, 0x00},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x01},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x06},
	{0x040D, 0x00},
	{0x040E, 0x03},
	{0x040F, 0x60},
	{0x034C, 0x06},
	{0x034D, 0x00},
	{0x034E, 0x03},
	{0x034F, 0x60},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0x76},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x3C},
	{0x3CA4, 0x01},
	{0x3CA5, 0x5E},
	{0x3CA6, 0x00},
	{0x3CA7, 0x00},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x0C},
	{0x3CBA, 0x00},
	{0x3CBB, 0x04},
	{0x3CBC, 0x00},
	{0x3CBD, 0x1E},
	{0x3CBE, 0x00},
	{0x3CBF, 0x05},
	{0x0202, 0x04},
	{0x0203, 0x86},
	{0x0224, 0x01},
	{0x0225, 0xF4},
	{0x3116, 0x01},
	{0x3117, 0xF4},
	{0x0204, 0x00},
	{0x0205, 0x70},
	{0x0216, 0x00},
	{0x0217, 0x70},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x70},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x00},
	{0x341f, 0x60},
	{0x3420, 0x00},
	{0x3421, 0x48},
	{0x3366, 0x00},
	{0x3367, 0x00},
	{0x3368, 0x00},
	{0x3369, 0x00},
};

static const struct imx708_reg mode_hdr_regs[] = {
	{0x0342, 0x14},
	{0x0343, 0x60},
	{0x0340, 0x0A},
	{0x0341, 0x5B},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x11},
	{0x0349, 0xFF},
	{0x034A, 0X0A},
	{0x034B, 0x1F},
	{0x0220, 0x01},
	{0x0222, IMX708_HDR_EXPOSURE_RATIO},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x0A},
	{0x3200, 0x01},
	{0x3201, 0x01},
	{0x32D5, 0x00},
	{0x32D6, 0x00},
	{0x32DB, 0x01},
	{0x32DF, 0x00},
	{0x350C, 0x00},
	{0x350D, 0x00},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x09},
	{0x040D, 0x00},
	{0x040E, 0x05},
	{0x040F, 0x10},
	{0x034C, 0x09},
	{0x034D, 0x00},
	{0x034E, 0x05},
	{0x034F, 0x10},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xA2},
	{0x030B, 0x02},
	{0x030D, 0x04},
	{0x0310, 0x01},
	{0x3CA0, 0x00},
	{0x3CA1, 0x00},
	{0x3CA4, 0x00},
	{0x3CA5, 0x00},
	{0x3CA6, 0x00},
	{0x3CA7, 0x28},
	{0x3CAA, 0x00},
	{0x3CAB, 0x00},
	{0x3CB8, 0x00},
	{0x3CB9, 0x30},
	{0x3CBA, 0x00},
	{0x3CBB, 0x00},
	{0x3CBC, 0x00},
	{0x3CBD, 0x32},
	{0x3CBE, 0x00},
	{0x3CBF, 0x00},
	{0x0202, 0x0A},
	{0x0203, 0x2B},
	{0x0224, 0x0A},
	{0x0225, 0x2B},
	{0x3116, 0x0A},
	{0x3117, 0x2B},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x3118, 0x00},
	{0x3119, 0x00},
	{0x311A, 0x01},
	{0x311B, 0x00},
	{0x341a, 0x00},
	{0x341b, 0x00},
	{0x341c, 0x00},
	{0x341d, 0x00},
	{0x341e, 0x00},
	{0x341f, 0x90},
	{0x3420, 0x00},
	{0x3421, 0x6c},
	{0x3360, 0x01},
	{0x3361, 0x01},
	{0x3366, 0x09},
	{0x3367, 0x00},
	{0x3368, 0x05},
	{0x3369, 0x10},
};

/* Mode configs. Keep separate lists for when HDR is enabled or not. */
static const struct imx708_mode supported_modes_10bit_no_hdr[] = {
	/* {
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		
		.width = 4608,
		.height = 2592,
		.max_fps = {
			.numerator = 10000,
			.denominator = 140000,
		},
		.line_length_pix = 0x3d20,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT,
			.top = IMX708_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 58,
		.vblank_default = 58,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4608x2592_regs),
			.regs = mode_4608x2592_regs,
		},
		.pixel_rate = 595200000,
		.exposure_lines_min = 8,
		.exposure_lines_step = 1,
		.rk_hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.bpp = 10,
		.remosaic = true
	}, 
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 660000,
		},
		.line_length_pix = 0x1e90,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT,
			.top = IMX708_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 40,
		.vblank_default = 1198,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2x2binned_regs),
			.regs = mode_2x2binned_regs,
		},
		.pixel_rate = 585600000,
		.exposure_lines_min = 4,
		.exposure_lines_step = 2,
		.rk_hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.bpp = 10,
		.remosaic = false
	},*/
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		/* 2x2 binned and cropped for 720p. */
		.width = 1536,
		.height = 864,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1200000,
		},
		.line_length_pix = 0x1460,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT + 768,
			.top = IMX708_PIXEL_ARRAY_TOP + 432,
			.width = 3072,
			.height = 1728,
		},
		.vblank_min = 40,
		.vblank_default = 2755,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2x2binned_720p_regs),
			.regs = mode_2x2binned_720p_regs,
		},
		.pixel_rate = 566400000,
		.exposure_lines_min = 4,
		.exposure_lines_step = 2,
		.rk_hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.bpp = 10,
		.remosaic = false
	},
};

static const struct imx708_mode supported_modes_10bit_hdr[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		/* There's only one HDR mode, which is 2x2 downscaled */
		.width = 2304,
		.height = 1296,
		.max_fps = {
			.numerator = 10000,
			.denominator = 310000,
		},
		.line_length_pix = 0x1460,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT,
			.top = IMX708_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 3673,
		.vblank_default = 3673,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_hdr_regs),
			.regs = mode_hdr_regs,
		},
		.pixel_rate = 777600000,
		.exposure_lines_min = 8 * IMX708_HDR_EXPOSURE_RATIO * IMX708_HDR_EXPOSURE_RATIO,
		.exposure_lines_step = 2 * IMX708_HDR_EXPOSURE_RATIO * IMX708_HDR_EXPOSURE_RATIO,
		.rk_hdr_mode = HDR_X3,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_2,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_2,//S->csi wr2
		.bpp = 10,
		.remosaic = false
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		/* There's only one HDR mode, which is 2x2 downscaled */
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 310000,
		},
		.line_length_pix = 0x1460,
		.crop = {
			.left = IMX708_PIXEL_ARRAY_LEFT,
			.top = IMX708_PIXEL_ARRAY_TOP,
			.width = 4608,
			.height = 2592,
		},
		.vblank_min = 3673,
		.vblank_default = 3673,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_hdr_regs),
			.regs = mode_hdr_regs,
		},
		.pixel_rate = 777600000,
		.exposure_lines_min = 8 * IMX708_HDR_EXPOSURE_RATIO * IMX708_HDR_EXPOSURE_RATIO,
		.exposure_lines_step = 2 * IMX708_HDR_EXPOSURE_RATIO * IMX708_HDR_EXPOSURE_RATIO,
		.rk_hdr_mode = HDR_X3,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_2,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_2,//S->csi wr2
		.bpp = 10,
		.remosaic = false
	}
};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char * const imx708_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx708_test_pattern_val[] = {
	IMX708_TEST_PATTERN_DISABLE,
	IMX708_TEST_PATTERN_COLOR_BARS,
	IMX708_TEST_PATTERN_SOLID_COLOR,
	IMX708_TEST_PATTERN_GREY_COLOR,
	IMX708_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx708_supply_name[] = {
	/* Supplies can be enabled in any order */
	"vana1",  /* Analog1 (2.8V) supply */
	"vana2",  /* Analog2 (1.8V) supply */
	"vdig",  /* Digital Core (1.1V) supply */
	"vddl",  /* IF (1.8V) supply */
};

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define IMX708_XCLR_MIN_DELAY_US	8000
#define IMX708_XCLR_DELAY_RANGE_US	1000

struct imx708 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;

	struct v4l2_mbus_framefmt fmt;

	struct clk *inclk;
	u32 inclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx708_supply_name)];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hdr_mode;
	struct v4l2_ctrl *link_freq;
	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};

	/* Current mode */
	const struct imx708_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
	bool power_on;

	/* Rockchip module */
	u32 module_index;
	u32 cfg_num;
	const char *module_facing;
	const char *module_name;
	const char *len_name;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK */
	unsigned int long_exp_shift;

	unsigned int link_freq_idx;
};

static inline struct imx708 *to_imx708(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx708, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx708_mode **mode_list,
				  unsigned int *num_modes,
				  bool hdr_enable)
{
	switch (code) {
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		if (hdr_enable) {
			*mode_list = supported_modes_10bit_hdr;
			*num_modes = ARRAY_SIZE(supported_modes_10bit_hdr);
		} else {
			*mode_list = supported_modes_10bit_no_hdr;
			*num_modes = ARRAY_SIZE(supported_modes_10bit_no_hdr);
		}
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

static int imx708_read_reg(struct imx708 *imx708, u16 reg, u32 len, u32 *val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    struct i2c_msg msgs[2];
    u8 addr_buf[2] = { reg >> 8, reg & 0xff };
    u8 data_buf[4] = { 0, };
    int ret;

    dev_info(&client->dev, "Reading register 0x%04x (len: %u)\n", reg, len);

    if (len > 4) {
        dev_err(&client->dev, "Register read length %u too large\n", len);
        return -EINVAL;
    }

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = ARRAY_SIZE(addr_buf);
    msgs[0].buf = addr_buf;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = &data_buf[4 - len];

    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret != ARRAY_SIZE(msgs)) {
        dev_err(&client->dev, "Register read failed: %d\n", ret);
        return -EIO;
    }

    *val = get_unaligned_be32(data_buf);
    dev_info(&client->dev, "Register 0x%04x read complete, value: 0x%x\n", reg, *val);

    return 0;
}
static int imx708_write_reg(struct imx708 *imx708, u16 reg, u32 len, u32 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    u8 buf[6];

    dev_info(&client->dev, "Writing register 0x%04x: 0x%x (len: %u)\n", reg, val, len);

    if (len > 4) {
        dev_err(&client->dev, "Register write length %u too large\n", len);
        return -EINVAL;
    }

    put_unaligned_be16(reg, buf);
    put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
    if (i2c_master_send(client, buf, len + 2) != len + 2) {
        dev_err(&client->dev, "Register write failed\n");
        return -EIO;
    }

    dev_info(&client->dev, "Register write complete\n");
    return 0;
}
static int imx708_write_regs(struct imx708 *imx708,
                            const struct imx708_reg *regs, u32 len)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    unsigned int i;
    int ret;

    dev_info(&client->dev, "Writing register list (count: %u)\n", len);

    for (i = 0; i < len; i++) {
        ret = imx708_write_reg(imx708, regs[i].address, 1, regs[i].val);
        if (ret) {
            dev_err(&client->dev, "Failed to write reg 0x%4.4x. error = %d\n",
                    regs[i].address, ret);
            return ret;
        }
    }

    dev_info(&client->dev, "Register list write completed successfully\n");
    return 0;
}

static void imx708_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	pr_info("=== imx708_reset_colorspace called===\n");
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx708_update_image_pad_format(struct imx708 *imx708,
					   const struct imx708_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	dev_info(&client->dev, "update_image_pad_format\n");
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx708_reset_colorspace(&fmt->format);
}

/* Get bayer order based on flip setting. */
static u32 imx708_get_format_code(struct imx708 *imx708)
{
	unsigned int i;

	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	dev_info(&client->dev, "get_format_code\n");

	lockdep_assert_held(&imx708->mutex);

	i = (imx708->vflip->val ? 2 : 0) |
	    (imx708->hflip->val ? 1 : 0);

	return codes[i];
}



static void imx708_set_default_format(struct imx708 *imx708)
{
	struct v4l2_mbus_framefmt *fmt = &imx708->fmt;

	pr_info("=== imx708_set_default_format start===\n");

	/* Set default mode to max resolution */
	imx708->mode = &supported_modes_10bit_no_hdr[0];

	/* fmt->code not set as it will always be computed based on flips */
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = imx708->mode->width;
	fmt->height = imx708->mode->height;
	fmt->field = V4L2_FIELD_NONE;

	pr_info("=== imx708_set_default_format end===\n");
}

static int imx708_check_hwcfg(struct device *dev, struct imx708 *imx708)
{
    struct fwnode_handle *endpoint;
    struct fwnode_handle *remote;
    struct v4l2_fwnode_endpoint ep_cfg = {
        .bus_type = V4L2_MBUS_CSI2_DPHY
    };
    struct v4l2_fwnode_endpoint remote_ep = {
        .bus_type = V4L2_MBUS_CSI2_DPHY
    };
    int ret = -EINVAL;
    int i;

    dev_info(dev, "Checking hardware config...\n");

    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!endpoint) {
        dev_err(dev, "Endpoint node not found\n");
        return -EINVAL;
    }

    if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
        dev_err(dev, "Could not parse endpoint\n");
        goto error_out;
    }

    /* Add more debug info */
    dev_info(dev, "Endpoint info:");
    dev_info(dev, "  bus_type: %d", ep_cfg.bus_type);
    dev_info(dev, "  data lanes: %d", ep_cfg.bus.mipi_csi2.num_data_lanes);
    dev_info(dev, "  clock lane: %d", ep_cfg.bus.mipi_csi2.clock_lane);
    dev_info(dev, "  link frequencies count: %d", ep_cfg.nr_of_link_frequencies);
    
    /* Dump remote endpoint info if available */
    remote = fwnode_graph_get_remote_endpoint(endpoint);
    if (remote) {
        dev_info(dev, "Found remote endpoint");
        if (!v4l2_fwnode_endpoint_parse(remote, &remote_ep)) {
            dev_info(dev, "Remote endpoint info:");
            dev_info(dev, "  bus_type: %d", remote_ep.bus_type);
            dev_info(dev, "  data lanes: %d", remote_ep.bus.mipi_csi2.num_data_lanes);
            dev_info(dev, "  clock lane: %d", remote_ep.bus.mipi_csi2.clock_lane);
        }
    } else {
        dev_err(dev, "No remote endpoint found!");
    }

    /* Check the number of MIPI CSI2 data lanes */
    if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
        dev_err(dev, "Only 2 data lanes supported (found %d)\n", 
                ep_cfg.bus.mipi_csi2.num_data_lanes);
        goto error_out;
    }

    /* Check the link frequency set in device tree */
    if (!ep_cfg.nr_of_link_frequencies) {
        dev_err(dev, "Link-frequency property not found in DT\n");
        goto error_out;
    }

    dev_info(dev, "Link frequency from DT: %lld\n", ep_cfg.link_frequencies[0]);

    for (i = 0; i < ARRAY_SIZE(link_freqs); i++) {
        if (link_freqs[i] == ep_cfg.link_frequencies[0]) {
            imx708->link_freq_idx = i;
            dev_info(dev, "Found matching link frequency at index %d\n", i);
            break;
        }
    }

    if (i == ARRAY_SIZE(link_freqs)) {
        dev_err(dev, "Link frequency not supported: %lld\n",
                ep_cfg.link_frequencies[0]);
        ret = -EINVAL;
        goto error_out;
    }

    ret = 0;

error_out:
    v4l2_fwnode_endpoint_free(&ep_cfg);
    fwnode_handle_put(endpoint);

    return ret;
}

static int imx708_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct imx708 *imx708 = to_imx708(sd);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct v4l2_mbus_framefmt *try_fmt_img =
        v4l2_subdev_get_try_format(sd, fh->pad, 0);
    struct v4l2_rect *try_crop;




    dev_info(&client->dev, "=== imx708_open called ===\n");
    dev_info(&client->dev, "Opening subdev\n");

    mutex_lock(&imx708->mutex);

    /* Initialize try_fmt for the image pad */
    dev_info(&client->dev, "HDR mode: %d\n", imx708->hdr_mode->val);
    
    if (imx708->hdr_mode->val) {
        try_fmt_img->width = supported_modes_10bit_hdr[0].width;
        try_fmt_img->height = supported_modes_10bit_hdr[0].height;
        dev_info(&client->dev, "Using HDR mode resolution: %dx%d\n",
                try_fmt_img->width, try_fmt_img->height);
    } else {
        try_fmt_img->width = supported_modes_10bit_no_hdr[0].width;
        try_fmt_img->height = supported_modes_10bit_no_hdr[0].height;
        dev_info(&client->dev, "Using non-HDR mode resolution: %dx%d\n",
                try_fmt_img->width, try_fmt_img->height);
    }
    
    try_fmt_img->code = imx708_get_format_code(imx708);
    try_fmt_img->field = V4L2_FIELD_NONE;
    dev_info(&client->dev, "Format code set to: 0x%x\n", try_fmt_img->code);

    /* Initialize try_crop */
    try_crop = v4l2_subdev_get_try_crop(sd, fh->pad, 0);
    /* try_crop->left = IMX708_PIXEL_ARRAY_LEFT;
    try_crop->top = IMX708_PIXEL_ARRAY_TOP;
    try_crop->width = IMX708_PIXEL_ARRAY_WIDTH;
    try_crop->height = IMX708_PIXEL_ARRAY_HEIGHT; */
	try_crop->left = supported_modes_10bit_no_hdr[0].crop.left;
    try_crop->top = supported_modes_10bit_no_hdr[0].crop.top;
    try_crop->width = supported_modes_10bit_no_hdr[0].crop.width;
    try_crop->height = supported_modes_10bit_no_hdr[0].crop.height;
    dev_info(&client->dev, "Crop rectangle set to: (%d,%d)/%dx%d\n",
             try_crop->left, try_crop->top,
             try_crop->width, try_crop->height);

    mutex_unlock(&imx708->mutex);

    dev_info(&client->dev, "Subdev opened successfully\n");
    dev_info(&client->dev, "=== imx708_open completed ===\n");
    return 0;
}


static int imx708_set_exposure(struct imx708 *imx708, unsigned int val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    dev_info(&client->dev, "Setting exposure to %u\n", val);

    val = max(val, imx708->mode->exposure_lines_min);
    val -= val % imx708->mode->exposure_lines_step;

    dev_info(&client->dev, "Adjusted exposure value: %u\n", val);

    return imx708_write_reg(imx708, IMX708_REG_EXPOSURE,
                           IMX708_REG_VALUE_16BIT,
                           val >> imx708->long_exp_shift);
}

static void imx708_adjust_exposure_range(struct imx708 *imx708,
                                       struct v4l2_ctrl *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    int exposure_max, exposure_def;

    dev_info(&client->dev, "Adjusting exposure range\n");

    exposure_max = imx708->mode->height + imx708->vblank->val -
                  IMX708_EXPOSURE_OFFSET;
    exposure_def = min(exposure_max, imx708->exposure->val);

    dev_info(&client->dev, "New exposure - max: %d, default: %d\n",
             exposure_max, exposure_def);

    __v4l2_ctrl_modify_range(imx708->exposure, imx708->exposure->minimum,
                            exposure_max, imx708->exposure->step,
                            exposure_def);
}

static int imx708_set_analogue_gain(struct imx708 *imx708, unsigned int val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    dev_info(&client->dev, "Setting analog gain to %u\n", val);

    return imx708_write_reg(imx708, IMX708_REG_ANALOG_GAIN,
                           IMX708_REG_VALUE_16BIT, val);
}

static int imx708_set_frame_length(struct imx708 *imx708, unsigned int val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    int ret;

    dev_info(&client->dev, "Setting frame length to %u\n", val);

    imx708->long_exp_shift = 0;

    while (val > IMX708_FRAME_LENGTH_MAX) {
        imx708->long_exp_shift++;
        val >>= 1;
        dev_info(&client->dev, "Adjusted frame length: %u, shift: %u\n",
                 val, imx708->long_exp_shift);
    }

    ret = imx708_write_reg(imx708, IMX708_REG_FRAME_LENGTH,
                          IMX708_REG_VALUE_16BIT, val);
    if (ret) {
        dev_err(&client->dev, "Failed to set frame length\n");
        return ret;
    }

    return imx708_write_reg(imx708, IMX708_LONG_EXP_SHIFT_REG,
                           IMX708_REG_VALUE_08BIT, imx708->long_exp_shift);
}

static void imx708_set_framing_limits(struct imx708 *imx708)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    const struct imx708_mode *mode = imx708->mode;
    unsigned int hblank;

    dev_info(&client->dev, "Setting framing limits\n");

    __v4l2_ctrl_modify_range(imx708->pixel_rate,
                            mode->pixel_rate, mode->pixel_rate,
                            1, mode->pixel_rate);

    /* Update limits and set FPS to default */
    __v4l2_ctrl_modify_range(imx708->vblank, mode->vblank_min,
                            ((1 << IMX708_LONG_EXP_SHIFT_MAX) *
                            IMX708_FRAME_LENGTH_MAX) - mode->height,
                            1, mode->vblank_default);

    hblank = mode->line_length_pix - mode->width;
    __v4l2_ctrl_modify_range(imx708->hblank, hblank, hblank, 1, hblank);

    dev_info(&client->dev, "Framing limits set - vblank min: %d, hblank: %u\n",
             mode->vblank_min, hblank);
}


static int imx708_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct imx708 *imx708 =
        container_of(ctrl->handler, struct imx708, ctrl_handler);
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    const struct imx708_mode *mode_list;
    unsigned int code, num_modes;
    int ret = 0;
    bool hdr;




    dev_info(&client->dev, "=== imx708_set_ctrl called, ctrl id: 0x%x ===\n", 
              ctrl->id);

    switch (ctrl->id) {
    case V4L2_CID_VBLANK:
        dev_info(&client->dev, "Handling VBLANK control\n");
        imx708_adjust_exposure_range(imx708, ctrl);
        break;

    case V4L2_CID_WIDE_DYNAMIC_RANGE:
        dev_info(&client->dev, "Handling WIDE_DYNAMIC_RANGE control\n");
        hdr = imx708->mode->rk_hdr_mode != NO_HDR;
        if (imx708->mode && hdr != ctrl->val) {
            dev_info(&client->dev, "Updating HDR mode from %d to %d\n", 
                     hdr, ctrl->val);
            code = imx708_get_format_code(imx708);
            get_mode_table(code, &mode_list, &num_modes, ctrl->val);
            imx708->mode = v4l2_find_nearest_size(mode_list,
                                                 num_modes,
                                                 width, height,
                                                 imx708->mode->width,
                                                 imx708->mode->height);
            imx708_set_framing_limits(imx708);
        }
        break;
    }

    dev_info(&client->dev, "Checking power status\n");
    if (pm_runtime_get_if_in_use(&client->dev) == 0) {
        dev_info(&client->dev, "Device not powered, skipping control set\n");
        return 0;
    }

    switch (ctrl->id) {
    case V4L2_CID_ANALOGUE_GAIN:
        dev_info(&client->dev, "Setting analog gain: %d\n", ctrl->val);
        imx708_set_analogue_gain(imx708, ctrl->val);
        break;
    case V4L2_CID_EXPOSURE:
        dev_info(&client->dev, "Setting exposure: %d\n", ctrl->val);
        ret = imx708_set_exposure(imx708, ctrl->val);
        break;
    case V4L2_CID_DIGITAL_GAIN:
        dev_info(&client->dev, "Setting digital gain: %d\n", ctrl->val);
        ret = imx708_write_reg(imx708, IMX708_REG_DIGITAL_GAIN,
                              IMX708_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_TEST_PATTERN:
        dev_info(&client->dev, "Setting test pattern: %d\n", ctrl->val);
        ret = imx708_write_reg(imx708, IMX708_REG_TEST_PATTERN,
                              IMX708_REG_VALUE_16BIT,
                              imx708_test_pattern_val[ctrl->val]);
        break;
    case V4L2_CID_TEST_PATTERN_RED:
        dev_info(&client->dev, "Setting test pattern red: %d\n", ctrl->val);
        ret = imx708_write_reg(imx708, IMX708_REG_TEST_PATTERN_R,
                              IMX708_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_TEST_PATTERN_GREENR:
        dev_info(&client->dev, "Setting test pattern green-r: %d\n", ctrl->val);
        ret = imx708_write_reg(imx708, IMX708_REG_TEST_PATTERN_GR,
                              IMX708_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_TEST_PATTERN_BLUE:
        dev_info(&client->dev, "Setting test pattern blue: %d\n", ctrl->val);
        ret = imx708_write_reg(imx708, IMX708_REG_TEST_PATTERN_B,
                              IMX708_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_TEST_PATTERN_GREENB:
        dev_info(&client->dev, "Setting test pattern green-b: %d\n", ctrl->val);
        ret = imx708_write_reg(imx708, IMX708_REG_TEST_PATTERN_GB,
                              IMX708_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_HFLIP:
    case V4L2_CID_VFLIP:
        dev_info(&client->dev, "Setting flip h:%d v:%d\n", 
                 imx708->hflip->val, imx708->vflip->val);
        ret = imx708_write_reg(imx708, IMX708_REG_ORIENTATION, 1,
                              imx708->hflip->val |
                              imx708->vflip->val << 1);
        break;
    case V4L2_CID_VBLANK:
        dev_info(&client->dev, "Setting frame length for vblank: %d\n", ctrl->val);
        ret = imx708_set_frame_length(imx708,
                                     imx708->mode->height + ctrl->val);
        break;
    case V4L2_CID_WIDE_DYNAMIC_RANGE:
        dev_info(&client->dev, "HDR mode already handled\n");
        break;
    default:
        dev_info(&client->dev,
                 "ctrl(id:0x%x,val:0x%x) is not handled\n",
                 ctrl->id, ctrl->val);
        ret = -EINVAL;
        break;
    }

    pm_runtime_put(&client->dev);


    dev_info(&client->dev, "=== imx708_set_ctrl completed, ret: %d ===\n", 
              ret);

    return ret;
}

static const struct v4l2_ctrl_ops imx708_ctrl_ops = {
	.s_ctrl = imx708_set_ctrl,
};

static int imx708_g_frame_interval(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_frame_interval *fi)
{
    struct imx708 *imx708 = to_imx708(sd);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    const struct imx708_mode *mode = imx708->mode;



    dev_info(&client->dev, "=== imx708_g_frame_interval called ===\n");

    fi->interval = mode->max_fps;
    dev_info(&client->dev, "Frame interval set to %d/%d\n", 
             fi->interval.numerator, fi->interval.denominator);


    dev_info(&client->dev, "=== imx708_g_frame_interval completed ===\n");

    return 0;
}

static int imx708_enum_mbus_code(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_mbus_code_enum *code)
{
    struct imx708 *imx708 = to_imx708(sd);
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);

    dev_info(&client->dev, "Enumerating mbus codes: index %d\n", code->index);

    if (code->index >= (ARRAY_SIZE(codes) / 4)) {
        dev_err(&client->dev, "Invalid mbus code index: %d\n", code->index);
        return -EINVAL;
    }

    code->code = imx708_get_format_code(imx708);
    dev_info(&client->dev, "Returning mbus code: 0x%x\n", code->code);

    return 0;
}

static int imx708_enum_frame_size(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_pad_config *cfg,
                                 struct v4l2_subdev_frame_size_enum *fse)
{
    struct imx708 *imx708 = to_imx708(sd);
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    const struct imx708_mode *mode_list;
    unsigned int num_modes;

    dev_info(&client->dev, "Enumerating frame sizes: index %d, code 0x%x\n",
             fse->index, fse->code);

    get_mode_table(fse->code, &mode_list, &num_modes,
                   imx708->hdr_mode->val);

    if (fse->index >= num_modes) {
        dev_err(&client->dev, "Invalid frame size index: %d\n", fse->index);
        return -EINVAL;
    }

    if (fse->code != imx708_get_format_code(imx708)) {
        dev_err(&client->dev, "Invalid format code: 0x%x\n", fse->code);
        return -EINVAL;
    }

    fse->min_width = mode_list[fse->index].width;
    fse->max_width = fse->min_width;
    fse->min_height = mode_list[fse->index].height;
    fse->max_height = fse->min_height;

    dev_info(&client->dev, "Frame size %dx%d\n",
             fse->min_width, fse->min_height);

    return 0;
}





static int imx708_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	
	
	struct imx708 *imx708 = to_imx708(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	dev_info(&client->dev, "get_pad_format\n");

	mutex_lock(&imx708->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx708->sd, cfg, fmt->pad);
		/* update the code which could change due to vflip or hflip */
		try_fmt->code = imx708_get_format_code(imx708);
		fmt->format = *try_fmt;
	} else {
		imx708_update_image_pad_format(imx708, imx708->mode, fmt);
		fmt->format.code = imx708_get_format_code(imx708);
	}

	mutex_unlock(&imx708->mutex);
	return 0;
}

static int imx708_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx708 *imx708 = to_imx708(sd);
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx708_mode *mode;
	const struct imx708_mode *mode_list;
	unsigned int num_modes;

	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	dev_info(&client->dev, "set_pad_format\n");

	mutex_lock(&imx708->mutex);

	/* Bayer order varies with flips */
	fmt->format.code = imx708_get_format_code(imx708);

	get_mode_table(fmt->format.code, &mode_list, &num_modes,
			   imx708->hdr_mode->val);

	mode = v4l2_find_nearest_size(mode_list,
					  num_modes,
					  width, height,
					  fmt->format.width,
					  fmt->format.height);
	imx708_update_image_pad_format(imx708, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg,
							  fmt->pad);
		*framefmt = fmt->format;
	} else {
		imx708->mode = mode;
		imx708_set_framing_limits(imx708);
	}

	mutex_unlock(&imx708->mutex);

	return 0;
}



static const struct v4l2_rect *
__imx708_get_pad_crop(struct imx708 *imx708, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{

	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	dev_info(&client->dev, "get_pad_crop\n");

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx708->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx708->mode->crop;
	}

	return NULL;
}

static int imx708_get_selection(struct v4l2_subdev *sd,
                               struct v4l2_subdev_pad_config *cfg,
                               struct v4l2_subdev_selection *sel)
{
    struct imx708 *imx708 = to_imx708(sd);
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);

    dev_info(&client->dev, "Getting selection type %d\n", sel->target);

    switch (sel->target) {
    case V4L2_SEL_TGT_CROP: {
        mutex_lock(&imx708->mutex);
        sel->r = *__imx708_get_pad_crop(imx708, cfg, sel->pad,
                                       sel->which);
        mutex_unlock(&imx708->mutex);
        dev_info(&client->dev, "Crop rectangle: (%d,%d)/%dx%d\n",
                 sel->r.left, sel->r.top, sel->r.width, sel->r.height);
        return 0;
    }

    case V4L2_SEL_TGT_NATIVE_SIZE:
        sel->r.left = 0;
        sel->r.top = 0;
        sel->r.width = IMX708_NATIVE_WIDTH;
        sel->r.height = IMX708_NATIVE_HEIGHT;
        dev_info(&client->dev, "Native size: %dx%d\n",
                 sel->r.width, sel->r.height);
        return 0;

    case V4L2_SEL_TGT_CROP_DEFAULT:
    case V4L2_SEL_TGT_CROP_BOUNDS:
        sel->r.left = IMX708_PIXEL_ARRAY_LEFT;
        sel->r.top = IMX708_PIXEL_ARRAY_TOP;
        sel->r.width = IMX708_PIXEL_ARRAY_WIDTH;
        sel->r.height = IMX708_PIXEL_ARRAY_HEIGHT;
        dev_info(&client->dev, "Crop bounds: (%d,%d)/%dx%d\n",
                 sel->r.left, sel->r.top, sel->r.width, sel->r.height);
        return 0;
    }

    dev_err(&client->dev, "Invalid selection target: %d\n", sel->target);
    return -EINVAL;
}

/* Start streaming */
static int imx708_start_streaming(struct imx708 *imx708)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    const struct imx708_reg_list *reg_list, *freq_regs;
    int i, ret;
    u32 val;

    dev_info(&client->dev, "Starting streaming\n");

    if (!imx708->common_regs_written) {
        dev_info(&client->dev, "Writing common registers\n");
        ret = imx708_write_regs(imx708, mode_common_regs,
                               ARRAY_SIZE(mode_common_regs));
        if (ret) {
            dev_err(&client->dev, "Failed to set common settings\n");
            return ret;
        }

        ret = imx708_read_reg(imx708, IMX708_REG_BASE_SPC_GAINS_L,
                             IMX708_REG_VALUE_08BIT, &val);
        if (ret == 0 && val == 0x40) {
            dev_info(&client->dev, "Setting PDAF gains\n");
            for (i = 0; i < 54 && ret == 0; i++) {
                ret = imx708_write_reg(imx708,
                                     IMX708_REG_BASE_SPC_GAINS_L + i,
                                     IMX708_REG_VALUE_08BIT,
                                     pdaf_gains[0][i % 9]);
            }
            for (i = 0; i < 54 && ret == 0; i++) {
                ret = imx708_write_reg(imx708,
                                     IMX708_REG_BASE_SPC_GAINS_R + i,
                                     IMX708_REG_VALUE_08BIT,
                                     pdaf_gains[1][i % 9]);
            }
        }
        if (ret) {
            dev_err(&client->dev, "Failed to set PDAF gains\n");
            return ret;
        }

        imx708->common_regs_written = true;
    }

    dev_info(&client->dev, "Applying mode-specific settings\n");
    reg_list = &imx708->mode->reg_list;
    ret = imx708_write_regs(imx708, reg_list->regs, reg_list->num_of_regs);
    if (ret) {
        dev_err(&client->dev, "Failed to set mode\n");
        return ret;
    }

    dev_info(&client->dev, "Setting link frequency\n");
    freq_regs = &link_freq_regs[imx708->link_freq_idx];
    ret = imx708_write_regs(imx708, freq_regs->regs,
                           freq_regs->num_of_regs);
    if (ret) {
        dev_err(&client->dev, "Failed to set link frequency\n");
        return ret;
    }

    /* Handle Quad Bayer re-mosaic settings */
    if (imx708->mode->remosaic && qbc_adjust > 0) {
        dev_info(&client->dev, "Configuring Quad Bayer settings\n");
        imx708_write_reg(imx708, IMX708_LPF_INTENSITY,
                        IMX708_REG_VALUE_08BIT, qbc_adjust);
        imx708_write_reg(imx708, IMX708_LPF_INTENSITY_EN,
                        IMX708_REG_VALUE_08BIT,
                        IMX708_LPF_INTENSITY_ENABLED);
    } else {
        imx708_write_reg(imx708, IMX708_LPF_INTENSITY_EN,
                        IMX708_REG_VALUE_08BIT,
                        IMX708_LPF_INTENSITY_DISABLED);
    }

    dev_info(&client->dev, "Setting up controls\n");
    ret = __v4l2_ctrl_handler_setup(imx708->sd.ctrl_handler);
    if (ret) {
        dev_err(&client->dev, "Failed to setup controls\n");
        return ret;
    }

    dev_info(&client->dev, "Starting sensor output\n");
    return imx708_write_reg(imx708, IMX708_REG_MODE_SELECT,
                           IMX708_REG_VALUE_08BIT, IMX708_MODE_STREAMING);
}

static void imx708_stop_streaming(struct imx708 *imx708)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    int ret;

    dev_info(&client->dev, "Stopping streaming\n");

    ret = imx708_write_reg(imx708, IMX708_REG_MODE_SELECT,
                          IMX708_REG_VALUE_08BIT, IMX708_MODE_STANDBY);
    if (ret)
        dev_err(&client->dev, "Failed to stop streaming\n");
    else
        dev_info(&client->dev, "Streaming stopped successfully\n");
}

static int imx708_set_stream(struct v4l2_subdev *sd, int enable)
{
    struct imx708 *imx708 = to_imx708(sd);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;

    dev_info(&client->dev, "%s streaming\n", enable ? "Starting" : "Stopping");

    mutex_lock(&imx708->mutex);
    if (imx708->streaming == enable) {
        mutex_unlock(&imx708->mutex);
        dev_info(&client->dev, "Streaming state already %s\n",
                 enable ? "enabled" : "disabled");
        return 0;
    }

    if (enable) {
        ret = pm_runtime_get_sync(&client->dev);
        if (ret < 0) {
            pm_runtime_put_noidle(&client->dev);
            mutex_unlock(&imx708->mutex);
            dev_err(&client->dev, "Failed to get runtime PM\n");
            return ret;
        }

        ret = imx708_start_streaming(imx708);
        if (ret) {
            dev_err(&client->dev, "Failed to start streaming\n");
            pm_runtime_put(&client->dev);
            mutex_unlock(&imx708->mutex);
            return ret;
        }
    } else {
        imx708_stop_streaming(imx708);
        pm_runtime_put(&client->dev);
    }

    imx708->streaming = enable;

    /* vflip/hflip and hdr mode cannot change during streaming */
    __v4l2_ctrl_grab(imx708->vflip, enable);
    __v4l2_ctrl_grab(imx708->hflip, enable);
    __v4l2_ctrl_grab(imx708->hdr_mode, enable);

    mutex_unlock(&imx708->mutex);

    dev_info(&client->dev, "Stream %s completed\n",
             enable ? "start" : "stop");
    return ret;
}
static int imx708_power_on(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx708 *imx708 = to_imx708(sd);
    int ret;

    dev_info(dev, "Powering on\n");

    ret = regulator_bulk_enable(ARRAY_SIZE(imx708_supply_name),
                               imx708->supplies);
    if (ret) {
        dev_err(dev, "Failed to enable regulators\n");
        return ret;
    }

    ret = clk_prepare_enable(imx708->inclk);
    if (ret) {
        dev_err(dev, "Failed to enable clock\n");
        goto reg_off;
    }

    gpiod_set_value_cansleep(imx708->reset_gpio, 1);
    usleep_range(IMX708_XCLR_MIN_DELAY_US,
                 IMX708_XCLR_MIN_DELAY_US + IMX708_XCLR_DELAY_RANGE_US);

    dev_info(dev, "Power on sequence completed\n");
    return 0;

reg_off:
    regulator_bulk_disable(ARRAY_SIZE(imx708_supply_name),
                          imx708->supplies);
    dev_err(dev, "Power on failed\n");
    return ret;
}

static int imx708_power_off(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx708 *imx708 = to_imx708(sd);

    dev_info(dev, "Powering off\n");

    gpiod_set_value_cansleep(imx708->reset_gpio, 0);
    regulator_bulk_disable(ARRAY_SIZE(imx708_supply_name),
                          imx708->supplies);
    clk_disable_unprepare(imx708->inclk);

    imx708->common_regs_written = false;

    dev_info(dev, "Power off completed\n");
    return 0;
}


static int __maybe_unused imx708_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx708 *imx708 = to_imx708(sd);

	dev_info(dev, "suspend\n");

	if (imx708->streaming)
		imx708_stop_streaming(imx708);

	return 0;
}

static int __maybe_unused imx708_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx708 *imx708 = to_imx708(sd);
	int ret;

	dev_info(dev, "resume\n");

	if (imx708->streaming) {
		ret = imx708_start_streaming(imx708);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx708_stop_streaming(imx708);
	imx708->streaming = 0;
	return ret;
}

static int imx708_get_regulators(struct imx708 *imx708)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	unsigned int i;

	dev_info(&client->dev, "Powering off\n");

	for (i = 0; i < ARRAY_SIZE(imx708_supply_name); i++)
		imx708->supplies[i].supply = imx708_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       ARRAY_SIZE(imx708_supply_name),
				       imx708->supplies);
}

/* Verify chip ID */
static int imx708_identify_module(struct imx708 *imx708)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    int ret;
    u32 val;

    dev_info(&client->dev, "Identifying module\n");

    ret = imx708_read_reg(imx708, IMX708_REG_CHIP_ID,
                         IMX708_REG_VALUE_16BIT, &val);
    if (ret) {
        dev_err(&client->dev, "Failed to read chip ID %x, error %d\n",
                IMX708_CHIP_ID, ret);
        return ret;
    }

    if (val != IMX708_CHIP_ID) {
        dev_err(&client->dev, "Chip ID mismatch: %x!=%x\n",
                IMX708_CHIP_ID, val);
        return -EIO;
    }

    ret = imx708_read_reg(imx708, 0x0000, IMX708_REG_VALUE_16BIT, &val);
    if (!ret) {
        dev_info(&client->dev, "Camera module ID 0x%04x\n", val);
        snprintf(imx708->sd.name, sizeof(imx708->sd.name), "imx708%s%s",
                 val & 0x02 ? "_wide" : "",
                 val & 0x80 ? "_noir" : "");
    }

    dev_info(&client->dev, "Module identification completed\n");
    return 0;
}

static int imx708_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx708 *imx708 = to_imx708(sd);
	struct i2c_client *client = imx708->client;
	int ret = 0;

	dev_info(&client->dev, "s_power\n");

	mutex_lock(&imx708->mutex);

	if (imx708->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		imx708->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx708->power_on = false;
	}
unlock_and_return:
	mutex_unlock(&imx708->mutex);

	return ret;
}

static int imx708_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct imx708 *imx708 = to_imx708(sd);
	const struct imx708_mode *mode = imx708->mode;
	u32 val = 0;

	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    dev_info(&client->dev, "g_mbus_config\n");

	val = 1 << (IMX708_LANES - 1) |
	      V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->rk_hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->rk_hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void imx708_get_module_inf(struct imx708 *imx708,
                                 struct rkmodule_inf *inf)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    dev_info(&client->dev, "Getting module info\n");

    memset(inf, 0, sizeof(*inf));
    strlcpy(inf->base.sensor, IMX708_NAME, sizeof(inf->base.sensor));
    strlcpy(inf->base.module, imx708->module_name,
            sizeof(inf->base.module));
    strlcpy(inf->base.lens, imx708->len_name, sizeof(inf->base.lens));

    dev_info(&client->dev, "Module info retrieved\n");
}

static int imx708_get_channel_info(struct imx708 *imx708,
				   struct rkmodule_channel_info *ch_info)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    dev_info(&client->dev, "get_channel_info\n");
	
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;

	ch_info->vc = imx708->mode->vc[ch_info->index];
	ch_info->width = imx708->mode->width;
	ch_info->height = imx708->mode->height;
	ch_info->bus_fmt = imx708->mode->bus_fmt;

	return 0;
}

static long imx708_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx708 *imx708 = to_imx708(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	//struct rkmodule_csi_dphy_param *dphy_param;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
    dev_info(&client->dev, "imx708_ioctl\n");

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx708_get_module_inf(imx708, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		if (imx708->mode->rk_hdr_mode == NO_HDR)
			hdr->esp.mode = HDR_NORMAL_VC;
		else
			hdr->esp.mode = HDR_ID_CODE;
		hdr->hdr_mode = imx708->mode->rk_hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = imx708->mode->width;
		h = imx708->mode->height;
		for (i = 0; i < imx708->cfg_num; i++) {
			if (w == supported_modes_10bit_hdr[i].width &&
			    h == supported_modes_10bit_hdr[i].height &&
			    supported_modes_10bit_hdr[i].rk_hdr_mode == hdr->hdr_mode) {
				imx708->mode = &supported_modes_10bit_hdr[i];
				break;
			}
		}
		if (i == imx708->cfg_num) {
			dev_err(&imx708->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			imx708_set_framing_limits(imx708);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream)
			ret = imx708_write_reg(imx708,
					       IMX708_REG_MODE_SELECT,
					       IMX708_REG_VALUE_08BIT,
					       IMX708_MODE_STREAMING);
		else
			ret = imx708_write_reg(imx708,
					       IMX708_REG_MODE_SELECT,
					       IMX708_REG_VALUE_08BIT,
					       IMX708_MODE_STANDBY);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = imx708_get_channel_info(imx708, ch_info);
		break;

	// case RKMODULE_GET_CSI_DPHY_PARAM:
	//	if (imx708->mode->hdr_mode == HDR_X2) {
	//		dphy_param = (struct rkmodule_csi_dphy_param *)arg;
	//		if (dphy_param->vendor == dcphy_param.vendor)
	//			*dphy_param = dcphy_param;
	//		dev_info(&imx708->client->dev,
	//			 "get sensor dphy param\n");
	//	} else
	//		ret = -EINVAL;
	//	break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long imx708_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd,
				  unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	long ret;
	u32 stream = 0;
	// struct rkmodule_csi_dphy_param *dphy_param;

	pr_info("=== imx708_compat_ioctl32 called===\n");

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx708_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree(inf);
				return -EFAULT;
			}
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -EFAULT;
		}
		ret = imx708_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx708_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr))) {
				kfree(hdr);
				return -EFAULT;
			}
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdr, up, sizeof(*hdr))) {
			kfree(hdr);
			return -EFAULT;
		}
		ret = imx708_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
		// case RKMODULE_GET_CSI_DPHY_PARAM:
		//	dphy_param = kzalloc(sizeof(*dphy_param), GFP_KERNEL);
		//	if (!dphy_param) {
		//		ret = -ENOMEM;
		//		return ret;
		//	}

		//	ret = imx708_ioctl(sd, cmd, dphy_param);
		//	if (!ret) {
		//		ret = copy_to_user(up, dphy_param, sizeof(*dphy_param));
		//		if (ret)
		//			ret = -EFAULT;
		//	}
		//	kfree(dphy_param);
		//	break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = imx708_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;

		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx708_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

static int
imx708_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx708 *imx708 = to_imx708(sd);
	const struct imx708_mode *mode_list;
	unsigned int num_modes, code;

	pr_info("=== imx708_enum_frame_interval called===\n");

	code = imx708_get_format_code(imx708);
	get_mode_table(code, &mode_list, &num_modes,
		       imx708->hdr_mode->val);

	if (fie->index >= num_modes)
		return -EINVAL;

	fie->code = mode_list[fie->index].bus_fmt;
	fie->width = mode_list[fie->index].width;
	fie->height = mode_list[fie->index].height;
	fie->interval = mode_list[fie->index].max_fps;

	return 0;
}

static const struct v4l2_subdev_core_ops imx708_core_ops = {
	.s_power = imx708_s_power,
	.ioctl = imx708_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx708_compat_ioctl32,
#endif
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx708_video_ops = {
	.s_stream = imx708_set_stream,
	.g_frame_interval = imx708_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx708_pad_ops = {
	.enum_mbus_code = imx708_enum_mbus_code,
	.enum_frame_size = imx708_enum_frame_size,
	.enum_frame_interval = imx708_enum_frame_interval,
	.get_fmt = imx708_get_pad_format,
	.set_fmt = imx708_set_pad_format,
	.get_selection = imx708_get_selection,
	.set_mbus_config = imx708_g_mbus_config,
};

static const struct v4l2_subdev_ops imx708_subdev_ops = {
	.core = &imx708_core_ops,
	.video = &imx708_video_ops,
	.pad = &imx708_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx708_internal_ops = {
	.open = imx708_open,
};

static const struct v4l2_ctrl_config imx708_notify_gains_ctrl = {
	.ops = &imx708_ctrl_ops,
	.id = V4L2_CID_NOTIFY_GAINS,
	.type = V4L2_CTRL_TYPE_U32,
	.min = IMX708_COLOUR_BALANCE_MIN,
	.max = IMX708_COLOUR_BALANCE_MAX,
	.step = IMX708_COLOUR_BALANCE_STEP,
	.def = IMX708_COLOUR_BALANCE_DEFAULT,
	.dims = { 4 },
	.elem_size = sizeof(u32),
};

/* Initialize control handlers */
static int imx708_init_controls(struct imx708 *imx708)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx708->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	unsigned int i;
	int ret;

	dev_info(&client->dev, "Initializing controls\n");

	ctrl_hdlr = &imx708->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret) {
		dev_err(&client->dev, "Failed to initialize ctrl handler (%d)\n", ret);
		return ret;
	}

	mutex_init(&imx708->mutex);
	ctrl_hdlr->lock = &imx708->mutex;

	/* By default, PIXEL_RATE is read only */
	imx708->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX708_INITIAL_PIXEL_RATE,
					       IMX708_INITIAL_PIXEL_RATE, 1,
					       IMX708_INITIAL_PIXEL_RATE);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create pixel rate control (%d)\n", ret);
		goto error;
	}

	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx708_ctrl_ops,
				      V4L2_CID_LINK_FREQ, 0, 0,
				      &link_freqs[imx708->link_freq_idx]);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create link frequency control (%d)\n", ret);
		goto error;
	}
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx708_set_framing_limits() call below.
	 */
	imx708->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create vblank control (%d)\n", ret);
		goto error;
	}

	imx708->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create hblank control (%d)\n", ret);
		goto error;
	}

	imx708->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX708_EXPOSURE_MIN,
					     IMX708_EXPOSURE_MAX,
					     IMX708_EXPOSURE_STEP,
					     IMX708_EXPOSURE_DEFAULT);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create exposure control (%d)\n", ret);
		goto error;
	}

	v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX708_ANA_GAIN_MIN, IMX708_ANA_GAIN_MAX,
			  IMX708_ANA_GAIN_STEP, IMX708_ANA_GAIN_DEFAULT);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create analog gain control (%d)\n", ret);
		goto error;
	}

	v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX708_DGTL_GAIN_MIN, IMX708_DGTL_GAIN_MAX,
			  IMX708_DGTL_GAIN_STEP, IMX708_DGTL_GAIN_DEFAULT);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create digital gain control (%d)\n", ret);
		goto error;
	}

	imx708->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create hflip control (%d)\n", ret);
		goto error;
	}

	imx708->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create vflip control (%d)\n", ret);
		goto error;
	}

	v4l2_ctrl_cluster(2, &imx708->hflip);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx708_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx708_test_pattern_menu) - 1,
				     0, 0, imx708_test_pattern_menu);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create test pattern menu (%d)\n", ret);
		goto error;
	}

	for (i = 0; i < 4; i++) {
		v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX708_TEST_PATTERN_COLOUR_MIN,
				  IMX708_TEST_PATTERN_COLOUR_MAX,
				  IMX708_TEST_PATTERN_COLOUR_STEP,
				  IMX708_TEST_PATTERN_COLOUR_MAX);
		if (ctrl_hdlr->error) {
			ret = ctrl_hdlr->error;
			dev_err(&client->dev, "Failed to create test pattern control %d (%d)\n", i, ret);
			goto error;
		}
	}

	//v4l2_ctrl_new_custom(ctrl_hdlr, &imx708_notify_gains_ctrl, NULL);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create notify gains control (%d)\n", ret);
		goto error;
	}

	imx708->hdr_mode = v4l2_ctrl_new_std(ctrl_hdlr, &imx708_ctrl_ops,
					     V4L2_CID_WIDE_DYNAMIC_RANGE,
					     0, 1, 1, 0);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create HDR mode control (%d)\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret) {
		dev_err(&client->dev, "Failed to parse device properties (%d)\n", ret);
		goto error;
	}

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx708_ctrl_ops, &props);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "Failed to create fwnode properties control (%d)\n", ret);
		goto error;
	}

	imx708->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	imx708->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx708->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx708->hdr_mode->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx708->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx708_set_framing_limits(imx708);

	dev_info(&client->dev, "Controls initialized successfully\n");
	return 0;

error:
	dev_err(&client->dev, "Cleaning up after control init failure\n");
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx708->mutex);

	return ret;
}

static void imx708_free_controls(struct imx708 *imx708)
{
	pr_info("=== imx708_free_controls called===\n");
	v4l2_ctrl_handler_free(imx708->sd.ctrl_handler);
	mutex_destroy(&imx708->mutex);
}



static int imx708_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct device_node *node = dev->of_node;
    struct imx708 *imx708;
    char facing[2];
    int ret;
    u32 i, hdr_mode = 0;

	struct v4l2_fwnode_endpoint vep = {
        .bus_type = V4L2_MBUS_CSI2_DPHY
    };
    struct fwnode_handle *ep;
    struct fwnode_handle *fwnode;

    dev_info(dev, "Probing imx708...\n");
	pr_info("=== imx708_probe called===\n");

    imx708 = devm_kzalloc(&client->dev, sizeof(*imx708), GFP_KERNEL);
    if (!imx708) {
        dev_err(dev, "Failed to allocate memory for imx708\n");
        return -ENOMEM;
    }

    dev_info(dev, "Reading device tree properties\n");
    ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
                              &imx708->module_index);
    ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
                                  &imx708->module_facing);
    ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
                                  &imx708->module_name);
    ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
                                  &imx708->len_name);
    if (ret) {
        dev_err(dev, "Failed to read DT properties\n");
        return -EINVAL;
    }
    dev_info(dev, "DT: module_index=%d, facing=%s, module_name=%s, lens=%s\n",
            imx708->module_index, imx708->module_facing,
            imx708->module_name, imx708->len_name);

    ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
    if (ret) {
        hdr_mode = NO_HDR;
        dev_warn(dev, "Get hdr mode failed! no hdr default\n");
    }
    dev_info(dev, "HDR mode: %d\n", hdr_mode);

    imx708->client = client;
    dev_info(dev, "Initializing v4l2 i2c subdev\n");
    v4l2_i2c_subdev_init(&imx708->sd, client, &imx708_subdev_ops);

    dev_info(dev, "Setting default format\n");
    imx708_set_default_format(imx708);

    if (hdr_mode != NO_HDR) {
        dev_info(dev, "Configuring HDR mode\n");
        imx708->cfg_num = ARRAY_SIZE(supported_modes_10bit_hdr);
        dev_info(dev, "Number of HDR configurations: %d\n", imx708->cfg_num);

        for (i = 0; i < imx708->cfg_num; i++) {
            if (hdr_mode == supported_modes_10bit_hdr[i].rk_hdr_mode) {
                imx708->mode = &supported_modes_10bit_hdr[i];
                dev_info(dev, "Found matching HDR mode at index %d\n", i);
                break;
            }
        }

        if (i >= imx708->cfg_num)
            dev_warn(dev, "Get hdr mode failed! no hdr config\n");
    } else {
        imx708->cfg_num = ARRAY_SIZE(supported_modes_10bit_no_hdr);
        dev_info(dev, "Using non-HDR mode, configurations: %d\n", imx708->cfg_num);
    }

    dev_info(dev, "Checking hardware configuration\n");
    ret = imx708_check_hwcfg(dev, imx708);
    if (ret) {
        dev_err(dev, "Failed hardware configuration check\n");
        return -EINVAL;
    }

	fwnode = dev_fwnode(dev);
    if (!fwnode) {
        dev_err(dev, "No device-tree node found\n");
        return -EINVAL;
    }

    dev_info(dev, "Looking for endpoints...\n");
    ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
    if (!ep) {
        dev_err(dev, "No endpoint found in device tree\n");
        return -EINVAL;
    }
    dev_info(dev, "Found endpoint\n");

    ret = v4l2_fwnode_endpoint_parse(ep, &vep);
    if (ret) {
        dev_err(dev, "Failed to parse endpoint: %d\n", ret);
        fwnode_handle_put(ep);
        return ret;
    }

    dev_info(dev, "Endpoint config: %d data lanes\n",
             vep.bus.mipi_csi2.num_data_lanes);

    fwnode_handle_put(ep);

    dev_info(dev, "Getting system clock (inclk)\n");
    imx708->inclk = devm_clk_get(dev, "inclk");
    if (IS_ERR(imx708->inclk)) {
        ret = PTR_ERR(imx708->inclk);
        dev_err(dev, "Failed to get inclk: %d\n", ret);
        return ret;
    }

    imx708->inclk_freq = clk_get_rate(imx708->inclk);
    dev_info(dev, "Clock frequency: %d Hz\n", imx708->inclk_freq);
    if (imx708->inclk_freq != IMX708_INCLK_FREQ) {
        dev_err(dev, "inclk frequency not supported: %d Hz\n",
                imx708->inclk_freq);
        return -EINVAL;
    }

    dev_info(dev, "Getting regulators\n");
    ret = imx708_get_regulators(imx708);
    if (ret) {
        dev_err(dev, "Failed to get regulators: %d\n", ret);
        return ret;
    }

    dev_info(dev, "Getting reset GPIO\n");
    imx708->reset_gpio = devm_gpiod_get_optional(dev, "reset",
                                                GPIOD_OUT_HIGH);
    if (IS_ERR(imx708->reset_gpio)) {
        ret = PTR_ERR(imx708->reset_gpio);
        dev_err(dev, "Failed to get reset GPIO: %d\n", ret);
        return ret;
    }

    dev_info(dev, "Powering on device\n");
    ret = imx708_power_on(dev);
    if (ret) {
        dev_err(dev, "Failed to power on device: %d\n", ret);
        return ret;
    }

    dev_info(dev, "Identifying module\n");
    ret = imx708_identify_module(imx708);
    if (ret) {
        dev_err(dev, "Failed to identify module: %d\n", ret);
        goto error_power_off;
    }

    dev_info(dev, "Initializing runtime PM\n");
    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
    pm_runtime_idle(dev);

    dev_info(dev, "Initializing controls\n");
    ret = imx708_init_controls(imx708);
    if (ret) {
        dev_err(dev, "Failed to init controls: %d\n", ret);
        goto error_pm_runtime;
    }

    dev_info(dev, "Initializing subdev\n");
    imx708->sd.internal_ops = &imx708_internal_ops;
    imx708->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
                       V4L2_SUBDEV_FL_HAS_EVENTS;
    imx708->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

    dev_info(dev, "Initializing media entity pads\n");
    imx708->pad.flags = MEDIA_PAD_FL_SOURCE;
	
    ret = media_entity_pads_init(&imx708->sd.entity, 1, &imx708->pad);
    if (ret) {
        dev_err(dev, "Failed to init entity pads: %d\n", ret);
        goto error_handler_free;
    }

	dev_info(dev, "Setting up media entity: %s\n", imx708->sd.entity.name);
	dev_info(dev, "Number of pads: %d\n", imx708->sd.entity.num_pads);

    memset(facing, 0, sizeof(facing));
    if (strcmp(imx708->module_facing, "back") == 0)
        facing[0] = 'b';
    else
        facing[0] = 'f';

    snprintf(imx708->sd.name, sizeof(imx708->sd.name), "m%02d_%s_%s %s",
             imx708->module_index, facing,
             IMX708_NAME, dev_name(imx708->sd.dev));
    dev_info(dev, "Device name: %s\n", imx708->sd.name);


    

	dev_info(dev, "Starting async subdev registration\n");
	dev_info(dev, "Subdev entity at %p\n", &imx708->sd.entity);
	dev_info(dev, "Entity function: 0x%x\n", imx708->sd.entity.function);
	dev_info(dev, "Entity name: %s\n", imx708->sd.entity.name);
	dev_info(dev, "Entity flags: 0x%lx\n", imx708->sd.entity.flags);
	dev_info(dev, "Subdev flags: 0x%x\n", imx708->sd.flags);  // Corrected to %x for u32



    ret = v4l2_async_register_subdev_sensor_common(&imx708->sd);
    if (ret < 0) {
        dev_err(dev, "Failed to register sensor subdev: %d\n", ret);
        goto error_media_entity;
    }

	dev_info(dev, "Successfully registered IMX708 sensor: name '%s', dev '%s'\n",
             imx708->sd.entity.name, dev_name(&client->dev));

    dev_info(dev, "Probe completed successfully\n");
    return 0;

error_media_entity:
    dev_info(dev, "Cleaning up media entity\n");
    media_entity_cleanup(&imx708->sd.entity);

error_handler_free:
    dev_info(dev, "Freeing controls\n");
    imx708_free_controls(imx708);

error_pm_runtime:
    dev_info(dev, "Disabling runtime PM\n");
    pm_runtime_disable(&client->dev);
    pm_runtime_set_suspended(&client->dev);

error_power_off:
    dev_info(dev, "Powering off device\n");
    imx708_power_off(&client->dev);

    return ret;
}

static int imx708_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx708 *imx708 = to_imx708(sd);
	
	/* Log device info before unregistering */
	dev_info(&client->dev, "Removing IMX708 device: name '%s', function type %u\n",
		 sd->entity.name, sd->entity.function);
	if (sd->dev)
		dev_info(&client->dev, "Associated device name: %s\n",
			 dev_name(sd->dev));

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx708_free_controls(imx708);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx708_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id imx708_dt_ids[] = {
	{ .compatible = "sony,imx708" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx708_dt_ids);

static const struct i2c_device_id imx708_match_id[] = {
	{ "sony,imx708", 0 },
	{},
};

static const struct dev_pm_ops imx708_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx708_suspend, imx708_resume)
	SET_RUNTIME_PM_OPS(imx708_power_off, imx708_power_on, NULL)
};

static struct i2c_driver imx708_i2c_driver = {
	.driver = {
		.name = "imx708",
		.of_match_table	= of_match_ptr(imx708_dt_ids),
		.pm = &imx708_pm_ops,
	},
	.probe = &imx708_probe,
	.remove = &imx708_remove,
	.id_table = imx708_match_id,
};

module_i2c_driver(imx708_i2c_driver);

MODULE_AUTHOR("UtsavBalar1231 <utsavbalar1231@gmail.com>");
MODULE_AUTHOR("David Plowman <david.plowman@raspberrypi.com>");
MODULE_DESCRIPTION("Sony IMX708 sensor driver");
MODULE_LICENSE("GPL v2");