/*
 * Driver for IMX219 CMOS Image Sensor from Sony
 *
 * Copyright (C) 2014, Andrew Chew <achew@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * V0.0X01.0X01 add enum_frame_interval function.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/compat.h>
#include <linux/v4l2-controls.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x1)

/* IMX219 supported geometry */
#define IMX219_TABLE_END		0xffff
#define IMX219_ANALOGUE_GAIN_MULTIPLIER	256
#define IMX219_ANALOGUE_GAIN_MIN	(1 * IMX219_ANALOGUE_GAIN_MULTIPLIER)
#define IMX219_ANALOGUE_GAIN_MAX	(11 * IMX219_ANALOGUE_GAIN_MULTIPLIER)
#define IMX219_ANALOGUE_GAIN_DEFAULT	(2 * IMX219_ANALOGUE_GAIN_MULTIPLIER)

/* In dB*256 */
#define IMX219_DIGITAL_GAIN_MIN		256
#define IMX219_DIGITAL_GAIN_MAX		43663
#define IMX219_DIGITAL_GAIN_DEFAULT	256

#define IMX219_DIGITAL_EXPOSURE_MIN	0
#define IMX219_DIGITAL_EXPOSURE_MAX	4095
#define IMX219_DIGITAL_EXPOSURE_DEFAULT	1575

#define IMX219_EXP_LINES_MARGIN	4
#define IMX219_FRAME_LENGTH_MARGIN	32
#define IMX219_VTS_MAX    0xffff

#define V4L2_CID_SENSOR_TEMP (V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 0x1001)

#define PAD0 0
#define PAD_MAX 1

#define IMX219_NAME			"imx219"

static const s64 link_freq_menu_items[] = {
	456000000,
};

struct imx219_reg {
	u16 addr;
	u8 val;
};

struct imx219_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	const struct imx219_reg *reg_list;
};

/* MCLK:24MHz  3280x2464  21.2fps   MIPI LANE2 */
static const struct imx219_reg imx219_init_tab_3280_2464_21fps[] = {
    {0x0114, 0x01},     /* CSI_LANE_MODE[1:0] */
    {0x30eb, 0x05},     /* Access Code for address over 0x3000 */
    {0x30eb, 0x0c},     /* Access Code for address over 0x3000 */
    {0x300a, 0xff},     /* Access Code for address over 0x3000 */
    {0x300b, 0xff},     /* Access Code for address over 0x3000 */
    {0x30eb, 0x05},     /* Access Code for address over 0x3000 */
    {0x30eb, 0x09},     /* Access Code for address over 0x3000 */
    {0x0128, 0x00},     /* DPHY_CNTRL */
    {0x012a, 0x18},     /* EXCK_FREQ[15:8] */
    {0x012b, 0x00},     /* EXCK_FREQ[7:0] */
	{0x0160, 0x09},     /* FRM_LENGTH_A[15:8] */
	{0x0161, 0xC4},     /* FRM_LENGTH_A[7:0] */
    {0x0164, 0x00},     /* X_ADD_STA_A[11:8]: X address start */
    {0x0165, 0x00},     /* X_ADD_STA_A[7:0]: X address start */
    {0x0166, 0x0c},     /* X_ADD_END_A[11:8]: X address end */
    {0x0167, 0xcf},     /* X_ADD_END_A[7:0]: X address end */
    {0x0168, 0x00},     /* Y_ADD_STA_A[11:8]: Y address start */
    {0x0169, 0x00},     /* Y_ADD_STA_A[7:0]: Y address start */
    {0x016a, 0x09},     /* Y_ADD_END_A[11:8]: Y address end */
    {0x016b, 0x9f},     /* Y_ADD_END_A[7:0]: Y address end */
    {0x016c, 0x0c},     /* x_output_size[11:8]: Output image width */
    {0x016d, 0xd0},     /* x_output_size[7:0]: Output image width */
    {0x016e, 0x09},     /* y_output_size[11:8]: Output image height */
    {0x016f, 0xa0},     /* y_output_size[7:0]: Output image height */
    {0x0170, 0x01},     /* X_ODD_INC_A[2:0]: X increment odd */
    {0x0171, 0x01},     /* Y_ODD_INC_A[2:0]: Y increment odd */
    {0x0174, 0x00},     /* BINNING_MODE_H_A: Horizontal binning mode */
    {0x0175, 0x00},     /* BINNING_MODE_V_A: Vertical binning mode */
    {0x0301, 0x05},     /* VTPXCK_DIV: Video timing pixel clock divider */
    {0x0303, 0x01},     /* VTSYCK_DIV: Video timing system clock divider */
    {0x0304, 0x03},     /* PREPLLCK_VT_DIV[3:0]: Pre-PLL clock Video timing divider */
    {0x0305, 0x03},     /* PREPLLCK_OP_DIV[3:0]: Pre-PLL clock Output timing divider */
    {0x0306, 0x00},     /* PLL_VT_MPY[10:8]: PLL Video timing multiplier */
    {0x0307, 0x39},     /* PLL_VT_MPY[7:0]: PLL Video timing multiplier */
    {0x030b, 0x01},     /* OPSYCK_DIV: Output system clock divider */
    {0x030c, 0x00},     /* PLL_OP_MPY[10:8]: PLL Output multiplier */
    {0x030d, 0x72},     /* PLL_OP_MPY[7:0]: PLL Output multiplier */
    {0x0624, 0x0c},     /* TP_WINDOW_WIDTH[11:8]: Test pattern window width */
    {0x0625, 0xd0},     /* TP_WINDOW_WIDTH[7:0]: Test pattern window width */
    {0x0626, 0x09},     /* TP_WINDOW_HEIGHT[11:8]: Test pattern window height */
    {0x0627, 0xa0},     /* TP_WINDOW_HEIGHT[7:0]: Test pattern window height */
    {0x455e, 0x00},     /* CIS Tuning */
    {0x471e, 0x4b},     /* CIS Tuning */
    {0x4767, 0x0f},     /* CIS Tuning */
    {0x4750, 0x14},     /* CIS Tuning */
    {0x4540, 0x00},     /* Manufacturing specific register */
    {0x47b4, 0x14},     /* CIS Tuning */
    {0x4713, 0x30},     /* Manufacturing specific register */
    {0x478b, 0x10},     /* Manufacturing specific register */
    {0x478f, 0x10},     /* Manufacturing specific register */
    {0x4793, 0x10},     /* Manufacturing specific register */
    {0x4797, 0x0e},     /* Manufacturing specific register */
    {0x479b, 0x0e},     /* Manufacturing specific register */
    {0x0162, 0x0d},     /* LINE_LENGTH_A[15:8]: Line length PCK */
    {0x0163, 0x78},     /* LINE_LENGTH_A[7:0]: Line length PCK */
    {0x0172, 0x00},     /* IMG_ORIENTATION_A: Image orientation */
    {IMX219_TABLE_END, 0x00},
};

/* MCLK:24MHz  1920x1080  30fps   MIPI LANE2 */
static const struct imx219_reg imx219_init_tab_1920_1080_30fps[] = {
   {0x30EB, 0x05},     /* Access Code for address over 0x3000 */
   {0x30EB, 0x0C},     /* Access Code for address over 0x3000 */
   {0x300A, 0xFF},     /* Access Code for address over 0x3000 */
   {0x300B, 0xFF},     /* Access Code for address over 0x3000 */
   {0x30EB, 0x05},     /* Access Code for address over 0x3000 */
   {0x30EB, 0x09},     /* Access Code for address over 0x3000 */
   {0x0114, 0x01},     /* CSI_LANE_MODE[1:0]: 2 Lane mode */
   {0x0128, 0x00},     /* DPHY_CTRL: MIPI Global timing - Auto mode */
   {0x012A, 0x18},     /* EXCK_FREQ[15:8]: Input clock frequency 24MHz */
   {0x012B, 0x00},     /* EXCK_FREQ[7:0] */
   {0x0160, 0x06},     /* FRM_LENGTH_A[15:8]: Frame length lines */
   {0x0161, 0xE6},     /* FRM_LENGTH_A[7:0] */
   {0x0162, 0x0D},     /* LINE_LENGTH_A[15:8]: Line length PCK */
   {0x0163, 0x78},     /* LINE_LENGTH_A[7:0] */
   {0x0164, 0x02},     /* X_ADD_STA_A[11:8]: X address start */
   {0x0165, 0xA8},     /* X_ADD_STA_A[7:0] */
   {0x0166, 0x0A},     /* X_ADD_END_A[11:8]: X address end */
   {0x0167, 0x27},     /* X_ADD_END_A[7:0] */
   {0x0168, 0x02},     /* Y_ADD_STA_A[11:8]: Y address start */
   {0x0169, 0xB4},     /* Y_ADD_STA_A[7:0] */
   {0x016A, 0x06},     /* Y_ADD_END_A[11:8]: Y address end */
   {0x016B, 0xEB},     /* Y_ADD_END_A[7:0] */
   {0x016C, 0x07},     /* x_output_size[11:8]: Output image width */
   {0x016D, 0x80},     /* x_output_size[7:0] */
   {0x016E, 0x04},     /* y_output_size[11:8]: Output image height */
   {0x016F, 0x38},     /* y_output_size[7:0] */
   {0x0170, 0x01},     /* X_ODD_INC_A: X increment odd */
   {0x0171, 0x01},     /* Y_ODD_INC_A: Y increment odd */
   {0x0174, 0x00},     /* BINNING_MODE_H_A: Horizontal binning mode */
   {0x0175, 0x00},     /* BINNING_MODE_V_A: Vertical binning mode */
   {0x018C, 0x0A},     /* CSI_DATA_FORMAT_A[15:8] */
   {0x018D, 0x0A},     /* CSI_DATA_FORMAT_A[7:0] */
   {0x0301, 0x05},     /* VTPXCK_DIV: Video timing pixel clock divider */
   {0x0303, 0x01},     /* VTSYCK_DIV: Video timing system clock divider */
   {0x0304, 0x03},     /* PREPLLCK_VT_DIV: Pre-PLL clock Video timing divider */
   {0x0305, 0x03},     /* PREPLLCK_OP_DIV: Pre-PLL clock Output timing divider */
   {0x0306, 0x00},     /* PLL_VT_MPY[10:8]: PLL Video timing multiplier */
   {0x0307, 0x39},     /* PLL_VT_MPY[7:0] */
   {0x0309, 0x0A},     /* OPPXCK_DIV: Output pixel clock divider */
   {0x030B, 0x01},     /* OPSYCK_DIV: Output system clock divider */
   {0x030C, 0x00},     /* PLL_OP_MPY[10:8]: PLL Output multiplier */
   {0x030D, 0x72},     /* PLL_OP_MPY[7:0] */
   {0x455E, 0x00},     /* CIS Tuning */
   {0x471E, 0x4B},     /* CIS Tuning */
   {0x4767, 0x0F},     /* CIS Tuning */
   {0x4750, 0x14},     /* CIS Tuning */
   {0x4540, 0x00},     /* Manufacturing specific register */
   {0x47B4, 0x14},     /* CIS Tuning */
   {IMX219_TABLE_END, 0x00}
};

/* MCLK:24MHz  1640x1232  30fps  MIPI LANE2  2x binning */
static const struct imx219_reg imx219_init_tab_1640_1232_30fps[] = {
   {0x0100, 0x00},     /* mode_select: Standby */
   {0x30eb, 0x05},     /* Access Code for address over 0x3000 */
   {0x30eb, 0x0c},     /* Access Code for address over 0x3000 */
   {0x300a, 0xff},     /* Access Code for address over 0x3000 */
   {0x300b, 0xff},     /* Access Code for address over 0x3000 */
   {0x30eb, 0x05},     /* Access Code for address over 0x3000 */
   {0x30eb, 0x09},     /* Access Code for address over 0x3000 */
   {0x0114, 0x01},     /* CSI_LANE_MODE[1:0]: 2 Lane mode */
   {0x0128, 0x00},     /* DPHY_CTRL: MIPI Global timing - Auto mode */
   {0x012a, 0x18},     /* EXCK_FREQ[15:8]: Input clock frequency 24MHz */
   {0x012b, 0x00},     /* EXCK_FREQ[7:0] */
   {0x0160, 0x06},     /* FRM_LENGTH_A[15:8]: Frame length lines */
   {0x0161, 0xE6},     /* FRM_LENGTH_A[7:0] */
   {0x0162, 0x0D},     /* LINE_LENGTH_A[15:8]: Line length PCK */
   {0x0163, 0x78},     /* LINE_LENGTH_A[7:0] */
   {0x0164, 0x00},     /* X_ADD_STA_A[11:8]: X address start */
   {0x0165, 0x00},     /* X_ADD_STA_A[7:0] */
   {0x0166, 0x0c},     /* X_ADD_END_A[11:8]: X address end */
   {0x0167, 0xcf},     /* X_ADD_END_A[7:0] */
   {0x0168, 0x00},     /* Y_ADD_STA_A[11:8]: Y address start */
   {0x0169, 0x00},     /* Y_ADD_STA_A[7:0] */
   {0x016a, 0x09},     /* Y_ADD_END_A[11:8]: Y address end */
   {0x016b, 0x9f},     /* Y_ADD_END_A[7:0] */
   {0x016c, 0x06},     /* x_output_size[11:8]: Output image width */
   {0x016d, 0x68},     /* x_output_size[7:0] */
   {0x016e, 0x04},     /* y_output_size[11:8]: Output image height */
   {0x016f, 0xd0},     /* y_output_size[7:0] */
   {0x0170, 0x01},     /* X_ODD_INC_A: X increment odd */
   {0x0171, 0x01},     /* Y_ODD_INC_A: Y increment odd */
   {0x0174, 0x01},     /* BINNING_MODE_H_A: Horizontal binning mode - 2x digital binning */
   {0x0175, 0x01},     /* BINNING_MODE_V_A: Vertical binning mode - 2x digital binning */
   {0x0176, 0x01},     /* BINNING_CAL_MODE_H_A: sum mode */
   {0x0177, 0x01},     /* BINNING_CAL_MODE_V_A: sum mode */
   {0x0301, 0x05},     /* VTPXCK_DIV: Video timing pixel clock divider */
   {0x0303, 0x01},     /* VTSYCK_DIV: Video timing system clock divider */
   {0x0304, 0x03},     /* PREPLLCK_VT_DIV: Pre-PLL clock Video timing divider */
   {0x0305, 0x03},     /* PREPLLCK_OP_DIV: Pre-PLL clock Output timing divider */
   {0x0306, 0x00},     /* PLL_VT_MPY[10:8]: PLL Video timing multiplier */
   {0x0307, 0x39},     /* PLL_VT_MPY[7:0] */
   {0x030b, 0x01},     /* OPSYCK_DIV: Output system clock divider */
   {0x030c, 0x00},     /* PLL_OP_MPY[10:8]: PLL Output multiplier */
   {0x030d, 0x72},     /* PLL_OP_MPY[7:0] */
   {0x0624, 0x06},     /* TP_WINDOW_WIDTH[11:8]: Test pattern window width */
   {0x0625, 0x68},     /* TP_WINDOW_WIDTH[7:0] */
   {0x0626, 0x04},     /* TP_WINDOW_HEIGHT[11:8]: Test pattern window height */
   {0x0627, 0xd0},     /* TP_WINDOW_HEIGHT[7:0] */
   {0x455e, 0x00},     /* CIS Tuning */
   {0x471e, 0x4b},     /* CIS Tuning */
   {0x4767, 0x0f},     /* CIS Tuning */
   {0x4750, 0x14},     /* CIS Tuning */
   {0x4540, 0x00},     /* Manufacturing specific register */
   {0x47b4, 0x14},     /* CIS Tuning */
   {0x4713, 0x30},     /* Manufacturing specific register */
   {0x478b, 0x10},     /* Manufacturing specific register */
   {0x478f, 0x10},     /* Manufacturing specific register */
   {0x4793, 0x10},     /* Manufacturing specific register */
   {0x4797, 0x0e},     /* Manufacturing specific register */
   {0x479b, 0x0e},     /* Manufacturing specific register */
   {0x0162, 0x0d},     /* LINE_LENGTH_A[15:8]: Line length PCK */
   {0x0163, 0x78},     /* LINE_LENGTH_A[7:0] */
   {IMX219_TABLE_END, 0x00}
};

/* MCLK:24MHz  640x480  30fps   MIPI LANE2  2x binning */
static const struct imx219_reg imx219_init_tab_640_480_30fps[] = {
   {0x0100, 0x00},     /* mode_select: Standby */
   {0x30eb, 0x05},     /* Access Code for address over 0x3000 */
   {0x30eb, 0x0c},     /* Access Code for address over 0x3000 */
   {0x300a, 0xff},     /* Access Code for address over 0x3000 */
   {0x300b, 0xff},     /* Access Code for address over 0x3000 */
   {0x30eb, 0x05},     /* Access Code for address over 0x3000 */
   {0x30eb, 0x09},     /* Access Code for address over 0x3000 */
   {0x0114, 0x01},     /* CSI_LANE_MODE[1:0]: 2 Lane mode */
   {0x0128, 0x00},     /* DPHY_CTRL: MIPI Global timing - Auto mode */
   {0x012a, 0x18},     /* EXCK_FREQ[15:8]: Input clock frequency 24MHz */
   {0x012b, 0x00},     /* EXCK_FREQ[7:0] */
   {0x0160, 0x06},     /* FRM_LENGTH_A[15:8]: Frame length lines */
   {0x0161, 0xE6},     /* FRM_LENGTH_A[7:0] */
   {0x0162, 0x0d},     /* LINE_LENGTH_A[15:8]: Line length PCK */
   {0x0163, 0x78},     /* LINE_LENGTH_A[7:0] */
   {0x0164, 0x03},     /* X_ADD_STA_A[11:8]: X address start */
   {0x0165, 0xe8},     /* X_ADD_STA_A[7:0] */
   {0x0166, 0x08},     /* X_ADD_END_A[11:8]: X address end */
   {0x0167, 0xe7},     /* X_ADD_END_A[7:0] */
   {0x0168, 0x02},     /* Y_ADD_STA_A[11:8]: Y address start */
   {0x0169, 0xf0},     /* Y_ADD_STA_A[7:0] */
   {0x016a, 0x06},     /* Y_ADD_END_A[11:8]: Y address end */
   {0x016b, 0xaf},     /* Y_ADD_END_A[7:0] */
   {0x016c, 0x02},     /* x_output_size[11:8]: Output image width */
   {0x016d, 0x80},     /* x_output_size[7:0] */
   {0x016e, 0x01},     /* y_output_size[11:8]: Output image height */
   {0x016f, 0xe0},     /* y_output_size[7:0] */
   {0x0170, 0x01},     /* X_ODD_INC_A: X increment odd */
   {0x0171, 0x01},     /* Y_ODD_INC_A: Y increment odd */
   {0x0174, 0x03},     /* BINNING_MODE_H_A: Horizontal binning mode x2 */
   {0x0175, 0x03},     /* BINNING_MODE_V_A: Vertical binning mode x2 */
   {0x0301, 0x05},     /* VTPXCK_DIV: Video timing pixel clock divider */
   {0x0303, 0x01},     /* VTSYCK_DIV: Video timing system clock divider */
   {0x0304, 0x03},     /* PREPLLCK_VT_DIV: Pre-PLL clock Video timing divider */
   {0x0305, 0x03},     /* PREPLLCK_OP_DIV: Pre-PLL clock Output timing divider */
   {0x0306, 0x00},     /* PLL_VT_MPY[10:8]: PLL Video timing multiplier */
   {0x0307, 0x39},     /* PLL_VT_MPY[7:0] */
   {0x030b, 0x01},     /* OPSYCK_DIV: Output system clock divider */
   {0x030c, 0x00},     /* PLL_OP_MPY[10:8]: PLL Output multiplier */
   {0x030d, 0x72},     /* PLL_OP_MPY[7:0] */
   {0x0624, 0x02},     /* TP_WINDOW_WIDTH[11:8]: Test pattern window width */
   {0x0625, 0x80},     /* TP_WINDOW_WIDTH[7:0] */
   {0x0626, 0x01},     /* TP_WINDOW_HEIGHT[11:8]: Test pattern window height */
   {0x0627, 0xe0},     /* TP_WINDOW_HEIGHT[7:0] */
   {0x455e, 0x00},     /* CIS Tuning */
   {0x471e, 0x4b},     /* CIS Tuning */
   {0x4767, 0x0f},     /* CIS Tuning */
   {0x4750, 0x14},     /* CIS Tuning */
   {0x4540, 0x00},     /* Manufacturing specific register */
   {0x47b4, 0x14},     /* CIS Tuning */
   {0x4713, 0x30},     /* Manufacturing specific register */
   {0x478b, 0x10},     /* Manufacturing specific register */
   {0x478f, 0x10},     /* Manufacturing specific register */
   {0x4793, 0x10},     /* Manufacturing specific register */
   {0x4797, 0x0e},     /* Manufacturing specific register */
   {0x479b, 0x0e},     /* Manufacturing specific register */
   {IMX219_TABLE_END, 0x00}
};

static const struct imx219_reg start[] = {
	{0x0100, 0x01},		/* mode select streaming on */
	{IMX219_TABLE_END, 0x00}
};

static const struct imx219_reg stop[] = {
	{0x0100, 0x00},		/* mode select streaming off */
	{IMX219_TABLE_END, 0x00}
};

enum {
	TEST_PATTERN_DISABLED,
	TEST_PATTERN_SOLID_BLACK,
	TEST_PATTERN_SOLID_WHITE,
	TEST_PATTERN_SOLID_RED,
	TEST_PATTERN_SOLID_GREEN,
	TEST_PATTERN_SOLID_BLUE,
	TEST_PATTERN_COLOR_BAR,
	TEST_PATTERN_FADE_TO_GREY_COLOR_BAR,
	TEST_PATTERN_PN9,
	TEST_PATTERN_16_SPLIT_COLOR_BAR,
	TEST_PATTERN_16_SPLIT_INVERTED_COLOR_BAR,
	TEST_PATTERN_COLUMN_COUNTER,
	TEST_PATTERN_INVERTED_COLUMN_COUNTER,
	TEST_PATTERN_PN31,
	TEST_PATTERN_MAX
};

static const char *const tp_qmenu[] = {
	"Disabled",
	"Solid Black",
	"Solid White",
	"Solid Red",
	"Solid Green",
	"Solid Blue",
	"Color Bar",
	"Fade to Grey Color Bar",
	"PN9",
	"16 Split Color Bar",
	"16 Split Inverted Color Bar",
	"Column Counter",
	"Inverted Column Counter",
	"PN31",
};

#define SIZEOF_I2C_TRANSBUF 32

struct imx219 {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct clk *clk;
	struct v4l2_rect crop_rect;
	int hflip;
	int vflip;
	struct v4l2_fract frame_interval;  // Store requested frame interval
    u8 frame_interval_modified;       // Flag to track if user changed interval
	u8 streaming;
	u8 analogue_gain;
	u16 digital_gain;	/* bits 11:0 */
	u16 exposure_time;
	u16 test_pattern;
	u16 test_pattern_solid_color_r;
	u16 test_pattern_solid_color_gr;
	u16 test_pattern_solid_color_b;
	u16 test_pattern_solid_color_gb;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
	const struct imx219_mode *cur_mode;
	u32 cfg_num;
	u16 cur_vts;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	struct v4l2_ctrl *temp_ctrl;
};

static const struct imx219_mode supported_modes[] = {
	{
		.width = 3280,
		.height = 2464,
		.max_fps = {
			.numerator = 10000,
			.denominator = 210000,
		},
		.hts_def = 0x0d78 - IMX219_EXP_LINES_MARGIN,
		.vts_def = 0x09c4,
		.reg_list = imx219_init_tab_3280_2464_21fps,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.hts_def = 0x0d78 - IMX219_EXP_LINES_MARGIN,
		.vts_def = 0x06E6,
		.reg_list = imx219_init_tab_1920_1080_30fps,
	},
	{
		.width = 1640,
		.height = 1232,
		.max_fps = {
				.numerator = 10000,
				.denominator = 300000,
		},
		.hts_def = 0x0d78 - IMX219_EXP_LINES_MARGIN,
		.vts_def = 0x06e3,
		.reg_list = imx219_init_tab_1640_1232_30fps,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = {
				.numerator = 10000,
				.denominator = 300000,
		},
		.hts_def = 0x0d78 - IMX219_EXP_LINES_MARGIN,
		.vts_def = 0x06e3,
		.reg_list = imx219_init_tab_640_480_30fps,
	},

};

static struct imx219 *to_imx219(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct imx219, subdev);
}

static int imx219_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int imx219_s_frame_interval(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_frame_interval *fi);

static int reg_write(struct i2c_client *client, const u16 addr, const u8 data)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 tx[3];
	int ret;

	dev_dbg(&client->dev, "%s(): writing addr 0x%04x = 0x%02x\n",
		__func__, addr, data);

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 3;
	msg.flags = 0;
	tx[0] = addr >> 8;
	tx[1] = addr & 0xff;
	tx[2] = data;
	ret = i2c_transfer(adap, &msg, 1);
	mdelay(2);

	if (ret != 1)
		dev_err(&client->dev, "i2c write failed at 0x%04x\n", addr);

	return ret == 1 ? 0 : -EIO;
}

static int reg_read(struct i2c_client *client, const u16 addr)
{
	u8 buf[2] = {addr >> 8, addr & 0xff};
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		}, {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = buf,
		},
	};

	dev_dbg(&client->dev, "%s(): reading from addr 0x%04x\n", __func__, addr);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "i2c read failed at 0x%04x\n", addr);
		return ret;
	}

	dev_dbg(&client->dev, "read value: 0x%02x\n", buf[0]);
	return buf[0];
}

static int reg_write_table(struct i2c_client *client,
			   const struct imx219_reg table[])
{
	const struct imx219_reg *reg;
	int ret;

	dev_dbg(&client->dev, "%s(): writing register table\n", __func__);

	for (reg = table; reg->addr != IMX219_TABLE_END; reg++) {
		ret = reg_write(client, reg->addr, reg->val);
		if (ret < 0) {
			dev_err(&client->dev, "register write failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int imx219_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	u8 reg = 0x00;
	int ret;
	u32 frame_length;

	dev_dbg(&client->dev, "%s(): enable streaming: %d\n", __func__, enable);

	if (!enable) {
		dev_dbg(&client->dev, "stopping stream\n");
		priv->streaming = enable;
		priv->frame_interval_modified = 0;
		return reg_write_table(client, stop);
	}



	ret = reg_write_table(client, priv->cur_mode->reg_list);
	if (ret)
		return ret;

	/* Handle flip/mirror */
	if (priv->hflip)
		reg |= 0x1;
	if (priv->vflip)
		reg |= 0x2;

	dev_dbg(&client->dev, "setting flip configuration: 0x%02x\n", reg);
	ret = reg_write(client, 0x0172, reg);
	if (ret) {
		dev_err(&client->dev, "error setting flip: %d\n", ret);
		return ret;
	}

    /* Handle test pattern */
    if (priv->test_pattern) {
        dev_dbg(&client->dev, "setting test pattern mode: 0x%04x\n",
            priv->test_pattern);
        ret = reg_write(client, 0x0600, priv->test_pattern >> 8);
        ret |= reg_write(client, 0x0601, priv->test_pattern & 0xff);
        if (ret) {
            dev_err(&client->dev, "error setting test pattern: %d\n", ret);
            return ret;
        }
    } else {
        dev_dbg(&client->dev, "disabling test pattern\n");
        ret = reg_write(client, 0x0600, 0x00);
        ret |= reg_write(client, 0x0601, 0x00);
        if (ret) {
            dev_err(&client->dev, "error disabling test pattern: %d\n", ret);
            return ret;
        }
    }

	/* If user set custom frame interval, apply it now */
    if (priv->frame_interval_modified) {

        frame_length = (182400000 / 3448) * 
                         priv->frame_interval.numerator / 
                         priv->frame_interval.denominator;

		

        frame_length = clamp_t(u32, frame_length,
                               priv->cur_mode->height + IMX219_FRAME_LENGTH_MARGIN,
                               IMX219_VTS_MAX);

        ret = reg_write(client, 0x0160, (frame_length >> 8) & 0xff);
        ret |= reg_write(client, 0x0161, frame_length & 0xff);
        if (ret)
            return ret;
        
		priv->cur_vts = frame_length;
    } else {
		priv->cur_vts = priv->cur_mode->vts_def - IMX219_EXP_LINES_MARGIN;
	}

	priv->streaming = enable;

	dev_dbg(&client->dev, "starting sensor output\n");
	return reg_write_table(client, start);
}

static int imx219_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);

	dev_dbg(&client->dev, "%s(): setting power: %d\n", __func__, on);

	if (on) {
		dev_dbg(&client->dev, "enabling clock\n");
		clk_prepare_enable(priv->clk);
	} else {
		dev_dbg(&client->dev, "disabling clock\n");
		clk_disable_unprepare(priv->clk);
	}

	return 0;
}

/* V4L2 ctrl operations */
static int imx219_s_ctrl_test_pattern(struct v4l2_ctrl *ctrl)
{
	struct imx219 *priv =
	    container_of(ctrl->handler, struct imx219, ctrl_handler);

	switch (ctrl->val) {
	case TEST_PATTERN_DISABLED:
		priv->test_pattern = 0x0000;
		break;
	case TEST_PATTERN_SOLID_BLACK:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_WHITE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0fff;
		priv->test_pattern_solid_color_gr = 0x0fff;
		priv->test_pattern_solid_color_b = 0x0fff;
		priv->test_pattern_solid_color_gb = 0x0fff;
		break;
	case TEST_PATTERN_SOLID_RED:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0fff;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_GREEN:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0fff;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0fff;
		break;
	case TEST_PATTERN_SOLID_BLUE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0fff;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_COLOR_BAR:
		priv->test_pattern = 0x0002;
		break;
	case TEST_PATTERN_FADE_TO_GREY_COLOR_BAR:
		priv->test_pattern = 0x0003;
		break;
	case TEST_PATTERN_PN9:
		priv->test_pattern = 0x0004;
		break;
	case TEST_PATTERN_16_SPLIT_COLOR_BAR:
		priv->test_pattern = 0x0005;
		break;
	case TEST_PATTERN_16_SPLIT_INVERTED_COLOR_BAR:
		priv->test_pattern = 0x0006;
		break;
	case TEST_PATTERN_COLUMN_COUNTER:
		priv->test_pattern = 0x0007;
		break;
	case TEST_PATTERN_INVERTED_COLUMN_COUNTER:
		priv->test_pattern = 0x0008;
		break;
	case TEST_PATTERN_PN31:
		priv->test_pattern = 0x0009;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imx219_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode = priv->cur_mode;

	dev_dbg(&client->dev, "%s()\n", __func__);

	fi->interval = mode->max_fps;

	dev_dbg(&client->dev, "frame interval: %d/%d\n",
		fi->interval.numerator, fi->interval.denominator);

	return 0;
}

static int imx219_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx219 *priv =
	    container_of(ctrl->handler, struct imx219, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	u8 reg;
	int ret;
	u16 gain = 256;
	u16 a_gain = 256;
	u16 d_gain = 1;

	dev_dbg(&client->dev, "%s(): ctrl id: 0x%x val: %d\n",
		__func__, ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		dev_dbg(&client->dev, "setting horizontal flip: %d\n", ctrl->val);
		priv->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		dev_dbg(&client->dev, "setting vertical flip: %d\n", ctrl->val);
		priv->vflip = ctrl->val;
		break;

	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_GAIN:
		dev_dbg(&client->dev, "setting gain: %d\n", ctrl->val);
		/*
		 * hal transfer (gain * 256)  to kernel
		 * than divide into analog gain & digital gain in kernel
		 */

		gain = ctrl->val;
		if (gain < 256)
			gain = 256;
		if (gain > 43663)
			gain = 43663;
		if (gain >= 256 && gain <= 2728) {
			a_gain = gain;
			d_gain = 1 * 256;
		} else {
			a_gain = 2728;
			d_gain = (gain * 256) / a_gain;
		}

		/*
		 * Analog gain, reg range[0, 232], gain value[1, 10.66]
		 * reg = 256 - 256 / again
		 * a_gain here is 256 multify
		 * so the reg = 256 - 256 * 256 / a_gain
		 */
		priv->analogue_gain = (256 - (256 * 256) / a_gain);
		if (a_gain < 256)
			priv->analogue_gain = 0;
		if (priv->analogue_gain > 232)
			priv->analogue_gain = 232;

		/*
		 * Digital gain, reg range[256, 4095], gain rage[1, 16]
		 * reg = dgain * 256
		 */
		priv->digital_gain = d_gain;
		if (priv->digital_gain < 256)
			priv->digital_gain = 256;
		if (priv->digital_gain > 4095)
			priv->digital_gain = 4095;

		/*
		 * for bank A and bank B switch
		 * exposure time , gain, vts must change at the same time
		 * so the exposure & gain can reflect at the same frame
		 */

		dev_dbg(&client->dev, "calculated gains - analog: %d, digital: %d\n",
			priv->analogue_gain, priv->digital_gain);

		ret = reg_write(client, 0x0157, priv->analogue_gain);
		ret |= reg_write(client, 0x0158, priv->digital_gain >> 8);
		ret |= reg_write(client, 0x0159, priv->digital_gain & 0xff);

		return ret;

	case V4L2_CID_EXPOSURE:
		priv->exposure_time = ctrl->val;

		dev_dbg(&client->dev, "setting exposure: %d\n", ctrl->val);

		ret = reg_write(client, 0x015a, priv->exposure_time >> 8);
		ret |= reg_write(client, 0x015b, priv->exposure_time & 0xff);
		return ret;

	case V4L2_CID_TEST_PATTERN:
		dev_dbg(&client->dev, "setting test pattern: %d\n", ctrl->val);
		return imx219_s_ctrl_test_pattern(ctrl);

	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "setting vblank: %d\n", ctrl->val);
		if (ctrl->val < priv->cur_mode->vts_def)
			ctrl->val = priv->cur_mode->vts_def;
		if ((ctrl->val - IMX219_EXP_LINES_MARGIN) != priv->cur_vts)
			priv->cur_vts = ctrl->val - IMX219_EXP_LINES_MARGIN;
		ret = reg_write(client, 0x0160, ((priv->cur_vts >> 8) & 0xff));
		ret |= reg_write(client, 0x0161, (priv->cur_vts & 0xff));
		return ret;

	default:
		dev_warn(&client->dev, "unhandled ctrl id: 0x%x val: %d\n",
			 ctrl->id, ctrl->val);
		return -EINVAL;
	}
	/* If enabled, apply settings immediately */
	reg = reg_read(client, 0x0100);
	if ((reg & 0x1f) == 0x01)
		imx219_s_stream(&priv->subdev, 1);

	return 0;
}

static int imx219_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s(): index: %d\n", __func__, code->index);

	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	
	dev_dbg(&client->dev, "returning mbus code: 0x%x\n", code->code);
	return 0;
}

static int imx219_get_reso_dist(const struct imx219_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx219_mode *imx219_find_best_fit(
					struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = imx219_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx219_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode;
	s64 h_blank, v_blank, pixel_rate;
	u32 fps = 0;

	dev_dbg(&client->dev, "%s(): width: %d height: %d\n",
		__func__, fmt->format.width, fmt->format.height);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		dev_dbg(&client->dev, "format try only\n");
		return 0;
	}

	mode = imx219_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	priv->cur_mode = mode;

	dev_dbg(&client->dev, "selected mode: %dx%d\n",
		mode->width, mode->height);

	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(priv->hblank, h_blank,
				h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(priv->vblank, v_blank,
				v_blank, 
				1, v_blank);
	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
		mode->max_fps.numerator);
	pixel_rate = mode->vts_def * mode->hts_def * fps;
	__v4l2_ctrl_modify_range(priv->pixel_rate, pixel_rate,
				pixel_rate, 1, pixel_rate);

	
	
	/* reset crop window */
	 /* For 2x2 analog binned mode, use full sensor area */
    if (mode->width == 1640 && mode->height == 1232) {
        priv->crop_rect.left = 1000;
        priv->crop_rect.top = 752;
        priv->crop_rect.width = 3280;
        priv->crop_rect.height = 2464;
	} else if (mode->width == 640 && mode->height == 480) {
		priv->crop_rect.left = 0;
        priv->crop_rect.top = 0;
        priv->crop_rect.width = 1280;
        priv->crop_rect.height = 960;
    } else {
		priv->crop_rect.left = 1640 - (mode->width / 2);
		if (priv->crop_rect.left < 0)
			priv->crop_rect.left = 0;
		priv->crop_rect.top = 1232 - (mode->height / 2);
		if (priv->crop_rect.top < 0)
			priv->crop_rect.top = 0;
		priv->crop_rect.width = mode->width;
		priv->crop_rect.height = mode->height;
	}
	dev_dbg(&client->dev, "crop window: left=%d top=%d width=%d height=%d\n",
		priv->crop_rect.left, priv->crop_rect.top,
		priv->crop_rect.width, priv->crop_rect.height);

	return 0;
}


static int imx219_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode = priv->cur_mode;

	dev_dbg(&client->dev, "%s()\n", __func__);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		dev_dbg(&client->dev, "format try only\n");
		return 0;
	}

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;

	dev_dbg(&client->dev, "current mode: %dx%d\n",
		mode->width, mode->height);

	return 0;
}

static void imx219_get_module_inf(struct imx219 *imx219,
				  struct rkmodule_inf *inf)
{

	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX219_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx219->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx219->len_name, sizeof(inf->base.lens));
}

static int imx219_get_channel_info(struct imx219 *imx219,
                                  struct rkmodule_channel_info *ch_info)
{
    
    if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
        return -EINVAL;
        
    ch_info->vc = 0;  /* IMX219 only has one channel */
    ch_info->width = imx219->cur_mode->width;
    ch_info->height = imx219->cur_mode->height;
    ch_info->bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10;
    
    return 0;
}

static long imx219_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *imx219 = to_imx219(client);
	struct rkmodule_channel_info *ch_info;
    long ret = 0;

	dev_dbg(&client->dev, "%s(): cmd: 0x%x\n", __func__, cmd);

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		dev_dbg(&client->dev, "getting module info\n");
		imx219_get_module_inf(imx219, (struct rkmodule_inf *)arg);
		break;
	case VIDIOC_S_PARM: {
		struct v4l2_streamparm *parm = arg;
		struct v4l2_subdev_frame_interval fi;

		// Validate input
		if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;

		// Call the existing s_frame_interval function
		fi.interval.numerator = parm->parm.capture.timeperframe.numerator;
		fi.interval.denominator = parm->parm.capture.timeperframe.denominator;

		return imx219_s_frame_interval(sd, &fi);
	}
	case RKMODULE_GET_CHANNEL_INFO:
        ch_info = (struct rkmodule_channel_info *)arg;
        ret = imx219_get_channel_info(imx219, ch_info);
        break;
	default:
		dev_warn(&client->dev, "unhandled ioctl cmd: 0x%x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx219_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	dev_dbg(&client->dev, "%s(): cmd: 0x%x\n", __func__, cmd);

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			dev_err(&client->dev, "failed to allocate memory\n");
			ret = -ENOMEM;
			return ret;
		}

		ret = imx219_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				dev_err(&client->dev, "failed to copy to user\n");
		}
		kfree(inf);
		break;

	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			dev_err(&client->dev, "failed to allocate memory\n");
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = imx219_ioctl(sd, cmd, cfg);
		else
			dev_err(&client->dev, "failed to copy from user\n");
		kfree(cfg);
		break;

	default:
		dev_warn(&client->dev, "unhandled compat ioctl cmd: 0x%x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int imx219_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);

	dev_dbg(&client->dev, "%s(): index: %d\n", __func__, fie->index);

	if (fie->index >= priv->cfg_num)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;

	dev_dbg(&client->dev, "frame interval [%d]: %dx%d @ %d/%d fps\n",
		fie->index, fie->width, fie->height,
		fie->interval.denominator, fie->interval.numerator);

	return 0;
}

static int imx219_s_frame_interval(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_frame_interval *fi)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct imx219 *priv = to_imx219(client);
    const struct imx219_mode *mode = priv->cur_mode;
    u32 frame_length;
    int ret;

    if (!fi->interval.numerator || !fi->interval.denominator)
        return -EINVAL;


    /* Calculate frame length for desired fps */
    frame_length = (182400000 / 3448) * fi->interval.numerator / fi->interval.denominator;

    /* Clamp frame_length between min/max values */
    frame_length = clamp_t(u32, frame_length,
                        mode->height + IMX219_FRAME_LENGTH_MARGIN,
                        IMX219_VTS_MAX);

    /* Store the frame interval for later use */
    priv->frame_interval = fi->interval;
    priv->frame_interval_modified = 1;

    /* Only update registers if streaming */
    if (priv->streaming) {
        ret = reg_write(client, 0x0160, (frame_length >> 8) & 0xff);
        ret |= reg_write(client, 0x0161, frame_length & 0xff);
        if (ret)
            return ret;
        
        priv->cur_vts = frame_length;
    }

    dev_dbg(&client->dev, "Set framerate to %d/%d fps\n",
             fi->interval.denominator, fi->interval.numerator);

    return 0;
}

/* Various V4L2 operations tables */
static struct v4l2_subdev_video_ops imx219_subdev_video_ops = {
	.s_stream = imx219_s_stream,
	.g_frame_interval = imx219_g_frame_interval,
	.s_frame_interval = imx219_s_frame_interval,
};

static struct v4l2_subdev_core_ops imx219_subdev_core_ops = {
	.s_power = imx219_s_power,
	.ioctl = imx219_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx219_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_pad_ops imx219_subdev_pad_ops = {
	.enum_mbus_code = imx219_enum_mbus_code,
	.enum_frame_interval = imx219_enum_frame_interval,
	.set_fmt = imx219_set_fmt,
	.get_fmt = imx219_get_fmt,
};

static struct v4l2_subdev_ops imx219_subdev_ops = {
	.core = &imx219_subdev_core_ops,
	.video = &imx219_subdev_video_ops,
	.pad = &imx219_subdev_pad_ops,
};

static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
	.s_ctrl = imx219_s_ctrl,
	.g_volatile_ctrl = imx219_g_volatile_ctrl,
};

static int imx219_video_probe(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	u16 model_id;
	u32 lot_id;
	u16 chip_id;
	int ret;

	dev_dbg(&client->dev, "%s()\n", __func__);

	ret = imx219_s_power(subdev, 1);
	if (ret < 0) {
		dev_err(&client->dev, "failed to power on\n");
		return ret;
	}

	/* Check and show model, lot, and chip ID. */
	ret = reg_read(client, 0x0000);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read model ID high byte\n");
		goto done;
	}
	model_id = ret << 8;

	ret = reg_read(client, 0x0001);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read model ID low byte\n");
		goto done;
	}
	model_id |= ret;

	ret = reg_read(client, 0x0004);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read lot ID high byte\n");
		goto done;
	}
	lot_id = ret << 16;

	ret = reg_read(client, 0x0005);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read lot ID mid byte\n");
		goto done;
	}
	lot_id |= ret << 8;

	ret = reg_read(client, 0x0006);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read lot ID low byte\n");
		goto done;
	}
	lot_id |= ret;

	ret = reg_read(client, 0x000D);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read chip ID high byte\n");
		goto done;
	}
	chip_id = ret << 8;

	ret = reg_read(client, 0x000E);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read chip ID low byte\n");
		goto done;
	}
	chip_id |= ret;

	if (model_id != 0x0219) {
		dev_err(&client->dev, "model ID: %x not supported!\n",
			model_id);
		ret = -ENODEV;
		goto done;
	}

	dev_info(&client->dev,
		 "model ID: 0x%04x, lot ID: 0x%06x, chip ID: 0x%04x\n",
		 model_id, lot_id, chip_id);

	ret = 0;

done:
	imx219_s_power(subdev, 0);
	return ret;
}

static int imx219_ctrls_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode = priv->cur_mode;
	s64 pixel_rate, h_blank, v_blank;
	int ret;
	u32 fps = 0;

	static const struct v4l2_ctrl_config sensor_temp_cfg = {
		.ops = &imx219_ctrl_ops,
		.id = V4L2_CID_SENSOR_TEMP,
		.name = "Sensor Temperature",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.min = -10,
		.max = 95,
		.step = 1,
		.def = 25,
	};

	dev_dbg(&client->dev, "%s()\n", __func__);

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 10);

	dev_dbg(&client->dev, "initializing controls\n");

	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	/* exposure */
	dev_dbg(&client->dev, "setting up exposure controls\n");
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN,
			  IMX219_ANALOGUE_GAIN_MIN,
			  IMX219_ANALOGUE_GAIN_MAX,
			  1, IMX219_ANALOGUE_GAIN_DEFAULT);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_GAIN,
			  IMX219_DIGITAL_GAIN_MIN,
			  IMX219_DIGITAL_GAIN_MAX, 1,
			  IMX219_DIGITAL_GAIN_DEFAULT);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_EXPOSURE,
			  IMX219_DIGITAL_EXPOSURE_MIN,
			  IMX219_DIGITAL_EXPOSURE_MAX, 1,
			  IMX219_DIGITAL_EXPOSURE_DEFAULT);

	/* blank */
	dev_info(&client->dev, "setting up blank controls\n");
	h_blank = mode->hts_def - mode->width;
	priv->hblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_HBLANK,
			  h_blank, h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	priv->vblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_VBLANK,
			  v_blank, v_blank, 1, v_blank);

	/* freq */
	dev_dbg(&client->dev, "setting up frequency controls\n");
	v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, link_freq_menu_items);
	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
		mode->max_fps.numerator);
	pixel_rate = mode->vts_def * mode->hts_def * fps;
	priv->pixel_rate = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, pixel_rate, 1, pixel_rate);

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &imx219_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(tp_qmenu) - 1, 0, 0, tp_qmenu);

	/* temperature */
	dev_dbg(&client->dev, "adding temperature reading\n");
    

	priv->temp_ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler, &sensor_temp_cfg, NULL);
    

	priv->subdev.ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "control handler error: %d\n",
			priv->ctrl_handler.error);
		ret = priv->ctrl_handler.error;
		goto error;
	}

	ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (ret < 0) {
		dev_err(&client->dev, "control handler setup failed: %d\n", ret);
		goto error;
	}

	dev_dbg(&client->dev, "controls initialized successfully\n");
	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}

static int imx219_read_temp(struct i2c_client *client, s32 *temp)
{
        int ret;
        u8 val;

        /* Check if temperature sensor is enabled */
        ret = reg_read(client, 0x0140);
        if (ret < 0) {
                dev_err(&client->dev, "Failed to read temperature register: %d\n", ret);
                return ret;
        }

        /* Enable sensor if not already enabled */
        if (!(ret & 0x80)) {
                dev_dbg(&client->dev, "Enabling temperature sensor\n");
                ret = reg_write(client, 0x0140, 0x80);
                if (ret < 0) {
                        dev_err(&client->dev, "Failed to enable temperature sensor: %d\n", ret);
                        return ret;
                }

                /* Wait a bit for temperature measurement */
                usleep_range(1000, 2000);

                /* Read temperature value again after enabling */
                ret = reg_read(client, 0x0140);
                if (ret < 0) {
                        dev_err(&client->dev, "Failed to read temperature register: %d\n", ret);
                        return ret;
                }
        }

        /* Get just the temperature value bits [6:0] */
        val = ret & 0x7F;
        
        /* Convert to degrees Celsius using formula from datasheet:
           -10°C = 0 (0x00)
           95°C = 128 (0x80)
           Linear scale between these points */
        *temp = ((val * 105) / 128) - 10;
        
        dev_dbg(&client->dev, "Raw temp reg: 0x%02x, masked: 0x%02x, temp: %d°C\n", 
                ret, val, *temp);

        return 0;
}

static int imx219_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
        struct imx219 *priv = container_of(ctrl->handler,
                                         struct imx219, ctrl_handler);
        struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
        s32 temp;
        int ret;
        u8 streaming;

        switch (ctrl->id) {
        case V4L2_CID_SENSOR_TEMP:
                /* Check if sensor is streaming */
                streaming = reg_read(client, 0x0100);
                dev_dbg(&client->dev, "Sensor streaming state: 0x%02x\n", streaming);
                
                if ((streaming & 0x01) == 0) {
                        dev_dbg(&client->dev, "Sensor not streaming, temperature may be invalid\n");
                }

                ret = imx219_read_temp(client, &temp);
                if (ret)
                        return ret;
                ctrl->val = temp;
                break;
        default:
                return -EINVAL;
        }

        return 0;
}

static int imx219_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct imx219 *priv;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	dev_dbg(dev, "%s()\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "i2c functionality check failed\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct imx219), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "reading module information\n");
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &priv->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &priv->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &priv->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &priv->len_name);
	if (ret) {
		dev_err(dev, "failed to get module information\n");
		return -EINVAL;
	}

	priv->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "could not get clock\n");
		return -EPROBE_DEFER;
	}

	/* Set default mode to highest resolution */
	priv->cur_mode = &supported_modes[0];
	priv->cfg_num = ARRAY_SIZE(supported_modes);

	dev_dbg(dev, "initializing default mode: %dx%d\n",
		priv->cur_mode->width, priv->cur_mode->height);

	priv->crop_rect.width = priv->cur_mode->width;
	priv->crop_rect.height = priv->cur_mode->height;

	priv->frame_interval_modified = false;
    priv->frame_interval.numerator = 1;
    priv->frame_interval.denominator = 30;  // Default 30fps

	v4l2_i2c_subdev_init(&priv->subdev, client, &imx219_subdev_ops);
	ret = imx219_ctrls_init(&priv->subdev);
	if (ret < 0) {
		dev_err(dev, "failed to init controls\n");
		return ret;
	}

	ret = imx219_video_probe(client);
	if (ret < 0) {
		dev_err(dev, "failed to probe sensor\n");
		goto err_free_handler;
	}

	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&priv->subdev.entity, 1, &priv->pad);
	if (ret < 0) {
		dev_err(dev, "failed to init entity pads\n");
		goto err_free_handler;
	}

	sd = &priv->subdev;
	memset(facing, 0, sizeof(facing));
	if (strcmp(priv->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 priv->module_index, facing,
		 IMX219_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor subdevice\n");
		goto err_clean_entity;
	}

	dev_dbg(dev, "probe successful\n");
	return 0;

err_clean_entity:
	media_entity_cleanup(&priv->subdev.entity);
err_free_handler:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}

static int imx219_remove(struct i2c_client *client)
{
	struct imx219 *priv = to_imx219(client);

	dev_dbg(&client->dev, "%s()\n", __func__);

	v4l2_async_unregister_subdev(&priv->subdev);
	media_entity_cleanup(&priv->subdev.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static const struct i2c_device_id imx219_id[] = {
	{"imx219", 0},
	{}
};

static const struct of_device_id imx219_of_match[] = {
	{ .compatible = "sony,imx219" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx219_of_match);

MODULE_DEVICE_TABLE(i2c, imx219_id);
static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(imx219_of_match),
		.name = IMX219_NAME,
	},
	.probe = imx219_probe,
	.remove = imx219_remove,
	.id_table = imx219_id,
};

module_i2c_driver(imx219_i2c_driver);
MODULE_DESCRIPTION("Sony IMX219 Camera driver");
MODULE_AUTHOR("Guennadi Liakhovetski <g.liakhovetski@gmx.de>");
MODULE_LICENSE("GPL v2");
