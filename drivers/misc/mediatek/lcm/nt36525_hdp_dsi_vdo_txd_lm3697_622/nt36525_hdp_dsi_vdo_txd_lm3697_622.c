/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif

#endif
#endif

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))



#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define lcm_vddi_setting(cmd) \
			lcm_util.lcm_vddi_setting(cmd)
#define lcm_reset_setting(cmd) \
			lcm_util.lcm_reset_setting(cmd)
#define lcm_enp_setting(cmd) \
			lcm_util.lcm_enp_setting(cmd)
#define lcm_enn_setting(cmd) \
			lcm_util.lcm_enn_setting(cmd)
#define lcm_bkg_setting(cmd) \
			lcm_util.lcm_bkg_setting(cmd)
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_hs_test(enter) \
		lcm_util.dsi_set_hs(enter)


#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

/*****************************************************************************
 * Define
 *****************************************************************************/
extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;
extern unsigned int lcm_ldo_vision;
extern unsigned int phone_shutdown_state;

#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH		(720)
#define FRAME_HEIGHT		(1520)
#define LCM_PHYSICAL_WIDTH		(67608)
#define LCM_PHYSICAL_HEIGHT		(142728)


#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif
#endif

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

static struct LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
extern unsigned int rf_hw_id;
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
extern unsigned int product_hw_id;
static unsigned int dimming_enable;
extern void lcm_bias_set_avdd_n_avee(int value);
extern void lcm_bias_set_control(int level);
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
};

static struct LCM_setting_table init_setting[] = {
	{0xFF, 01, {0x20} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x01, 01, {0x55} },
	{0x03, 01, {0x55} },
	{0x05, 01, {0xB1} },
	{0x07, 01, {0x73} },
	{0x08, 01, {0xCB} },
	{0x0E, 01, {0x91} },
	{0x0F, 01, {0x69} },
	{0x69, 01, {0xA9} },
	{0x94, 01, {0x40} },      /* add asymmetric gamma voltage */
	{0x95, 01, {0xE5} },
	{0x96, 01, {0x19} },
	{0xFF, 01, {0x24} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x93, 01, {0x08} },
	{0x94, 01, {0x06} },
	{0x01, 01, {0x00} },
	{0x02, 01, {0x00} },
	{0x03, 01, {0x08} },
	{0x04, 01, {0x18} },
	{0x05, 01, {0x19} },
	{0x06, 01, {0x0C} },
	{0x07, 01, {0x0D} },
	{0x08, 01, {0x0E} },
	{0x09, 01, {0x0F} },
	{0x0A, 01, {0x10} },
	{0x0B, 01, {0x11} },
	{0x0C, 01, {0x04} },
	{0x17, 01, {0x00} },
	{0x18, 01, {0x00} },
	{0x19, 01, {0x08} },
	{0x1A, 01, {0x18} },
	{0x1B, 01, {0x19} },
	{0x1C, 01, {0x0C} },
	{0x1D, 01, {0x0D} },
	{0x1E, 01, {0x0E} },
	{0x1F, 01, {0x0F} },
	{0x20, 01, {0x10} },
	{0x21, 01, {0x11} },
	{0x22, 01, {0x04} },
	{0x2F, 01, {0x04} },
	{0x33, 01, {0x32} },
	{0x37, 01, {0x44} },
	{0x39, 01, {0x00} },
	{0x3A, 01, {0x10} },
	{0x3B, 01, {0x8F} },
	{0x3D, 01, {0x51} },
	{0x3F, 01, {0x41} },
	{0x40, 01, {0x00} },
	{0x43, 01, {0x41} },
	{0x44, 01, {0x06} },
	{0x47, 01, {0x44} },
	{0x49, 01, {0x00} },
	{0x4A, 01, {0x10} },
	{0x4B, 01, {0x8F} },
	{0x4C, 01, {0x51} },
	{0x4D, 01, {0x12} },
	{0x4E, 01, {0x34} },
	{0x4F, 01, {0x56} },
	{0x51, 01, {0x65} },
	{0x52, 01, {0x43} },
	{0x53, 01, {0x21} },
	{0x55, 01, {0x43} },
	{0x56, 01, {0x46} },
	{0x5A, 01, {0x10} },
	{0x5B, 01, {0x8F} },
	{0x5E, 01, {0x06} },
	{0x5F, 01, {0x00} },
	{0x92, 01, {0xB7} },
	{0x76, 01, {0x00} },
	{0x77, 01, {0x00} },
	{0x78, 01, {0x00} },
	{0x79, 01, {0x00} },
	{0x7A, 01, {0x20} },
	{0x7B, 01, {0x3C} },
	{0x7C, 01, {0x00} },
	{0x7D, 01, {0x00} },
	{0x7E, 01, {0x08} },
	{0x87, 01, {0x02} },
	{0xAB, 01, {0x00} },
	{0xAC, 01, {0x00} },
	{0xAD, 01, {0x00} },
	{0xAF, 01, {0x04} },
	{0xB0, 01, {0x05} },
	{0xB1, 01, {0xB2} },
	{0x60, 01, {0x80} },
	{0x61, 01, {0x7C} },
	{0x64, 01, {0x11} },
	{0xFF, 01, {0x25} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x05, 01, {0x04} },
	{0x07, 01, {0x00} },
	{0x08, 01, {0x01} },
	{0x0A, 01, {0x82} },
	{0x0B, 01, {0x46} },
	{0x0C, 01, {0x01} },
	{0x17, 01, {0x82} },
	{0x18, 01, {0x06} },
	{0x19, 01, {0x0F} },
	{0xFF, 01, {0x26} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x00, 01, {0xA0} },
	{0xFF, 01, {0x27} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x96, 01, {0x03} },
	{0x97, 01, {0x15} },
	{0x98, 01, {0x11} },
	{0x99, 01, {0x3F} },
	{0x9B, 01, {0x3F} },
	{0x9D, 01, {0x25} },
	{0x9E, 01, {0x55} },
	{0x9F, 01, {0x05} },
	{0xAD, 01, {0x25} },
	{0xAE, 01, {0x55} },
	{0xAF, 01, {0x05} },
	{0xB3, 01, {0xC6} },
	{0xC0, 01, {0x18} },
	{0xC1, 01, {0xE4} },
	{0xC2, 01, {0x00} },
	{0xC3, 01, {0x00} },
	{0xC4, 01, {0xE4} },
	{0xC5, 01, {0x00} },
	{0xC6, 01, {0x00} },
	{0xC7, 01, {0x03} },
	{0xFF, 1, {0x23} },
	{REGFLAG_DELAY, 5, {} },
	{0xFB, 1, {0x01} },
	{0x12, 1, {0xAB} },
	{0x15, 1, {0xF5} },
	{0x16, 1, {0x0B} },

	//page selection cmd start
	{0xFF, 01, {0x20} },
	{0xFB, 01, {0x01} },
	//page selection cmd end
	//R(+) MCR cmd
	{0xB0, 16, {0x00, 0x17, 0x00, 0x26, 0x00, 0x40, 0x00, 0x56, 0x00, 0x6A, 0x00, 0x7C, 0x00, 0x8C, 0x00, 0x9B} },
	{0xB1, 16, {0x00, 0xA8, 0x00, 0xD7, 0x00, 0xFC, 0x01, 0x37, 0x01, 0x65, 0x01, 0xAC, 0x01, 0xE7, 0x01, 0xE8} },
	{0xB2, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x83, 0x02, 0xBC, 0x02, 0xE2, 0x03, 0x0D, 0x03, 0x24, 0x03, 0x36} },
	{0xB3, 12, {0x03, 0x49, 0x03, 0x5E, 0x03, 0x6F, 0x03, 0x90, 0x03, 0xA6, 0x03, 0xFF} },
	//G(+) MCR
	{0xB4, 16, {0x00, 0x20, 0x00, 0x2E, 0x00, 0x46, 0x00, 0x5B, 0x00, 0x6F, 0x00, 0x81, 0x00, 0x91, 0x00, 0xA0} },
	{0xB5, 16, {0x00, 0xAD, 0x00, 0xDB, 0x01, 0x00, 0x01, 0x3A, 0x01, 0x67, 0x01, 0xAD, 0x01, 0xE7, 0x01, 0xE9} },
	{0xB6, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x84, 0x02, 0xBD, 0x02, 0xE2, 0x03, 0x0C, 0x03, 0x24, 0x03, 0x36} },
	{0xB7, 12, {0x03, 0x49, 0x03, 0x5C, 0x03, 0x6D, 0x03, 0x8F, 0x03, 0xCF, 0x03, 0xFF} },
	//B(+) MCR
	{0xB8, 16, {0x00, 0x01, 0x00, 0x13, 0x00, 0x32, 0x00, 0x4B, 0x00, 0x61, 0x00, 0x75, 0x00, 0x86, 0x00, 0x96} },
	{0xB9, 16, {0x00, 0xA4, 0x00, 0xD4, 0x00, 0xFA, 0x01, 0x35, 0x01, 0x63, 0x01, 0xAA, 0x01, 0xE5, 0x01, 0xE6} },
	{0xBA, 16, {0x02, 0x1C, 0x02, 0x5B, 0x02, 0x83, 0x02, 0xBE, 0x02, 0xE5, 0x03, 0x1F, 0x03, 0x33, 0x03, 0x4C} },
	{0xBB, 12, {0x03, 0x71, 0x03, 0xE2, 0x03, 0xE3, 0x03, 0xE4, 0x03, 0xEE, 0x03, 0xFF} },
	//page selection cmd start
	{0xFF, 1, {0x21} },
	{0xFB, 1, {0x01} },
	//page selection cmd end
	//R(-) MCR cmd
	{0xB0, 16, {0x00, 0x17, 0x00, 0x26, 0x00, 0x40, 0x00, 0x56, 0x00, 0x6A, 0x00, 0x7C, 0x00, 0x8C, 0x00, 0x9B} },
	{0xB1, 16, {0x00, 0xA8, 0x00, 0xD7, 0x00, 0xFC, 0x01, 0x37, 0x01, 0x65, 0x01, 0xAC, 0x01, 0xE7, 0x01, 0xE8} },
	{0xB2, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x83, 0x02, 0xBC, 0x02, 0xE2, 0x03, 0x0D, 0x03, 0x24, 0x03, 0x36} },
	{0xB3, 12, {0x03, 0x49, 0x03, 0x5E, 0x03, 0x6F, 0x03, 0x90, 0x03, 0xA6, 0x03, 0xFF} },
	//G(-) MCR
	{0xB4, 16, {0x00, 0x20, 0x00, 0x2E, 0x00, 0x46, 0x00, 0x5B, 0x00, 0x6F, 0x00, 0x81, 0x00, 0x91, 0x00, 0xA0} },
	{0xB5, 16, {0x00, 0xAD, 0x00, 0xDB, 0x01, 0x00, 0x01, 0x3A, 0x01, 0x67, 0x01, 0xAD, 0x01, 0xE7, 0x01, 0xE9} },
	{0xB6, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x84, 0x02, 0xBD, 0x02, 0xE2, 0x03, 0x0C, 0x03, 0x24, 0x03, 0x36} },
	{0xB7, 12, {0x03, 0x49, 0x03, 0x5C, 0x03, 0x6D, 0x03, 0x8F, 0x03, 0xCF, 0x03, 0xFF} },
	//B(-) MCR
	{0xB8, 16, {0x00, 0x01, 0x00, 0x13, 0x00, 0x32, 0x00, 0x4B, 0x00, 0x61, 0x00, 0x75, 0x00, 0x86, 0x00, 0x96} },
	{0xB9, 16, {0x00, 0xA4, 0x00, 0xD4, 0x00, 0xFA, 0x01, 0x35, 0x01, 0x63, 0x01, 0xAA, 0x01, 0xE5, 0x01, 0xE6} },
	{0xBA, 16, {0x02, 0x1C, 0x02, 0x5B, 0x02, 0x83, 0x02, 0xBE, 0x02, 0xE5, 0x03, 0x1F, 0x03, 0x33, 0x03, 0x4C} },
	{0xBB, 12, {0x03, 0x71, 0x03, 0xE2, 0x03, 0xE3, 0x03, 0xE4, 0x03, 0xEE, 0x03, 0xFF} },
	// open tp term pump
	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x9E, 1, {0x00} },
	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0x13, 1, {0x00} },
	{0x15, 1, {0xB4} },
	{0x1F, 1, {0x55} },
	{0x26, 1, {0x0F} },
	//LVD setting
	{0xFF, 1, {0X20} },
	{0xFB, 1, {0x01} },
	{0x31, 1, {0x57} },
	{0x32, 1, {0x48} },
	{0xFF, 1, {0XF0} },
	{0xFB, 1, {0x01} },
	{0xCF, 1, {0x22} },
	{0xFF, 1, {0x10} },
	{REGFLAG_DELAY, 10, {} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x24} },
	{0xBA, 1, {0x02} },
	{0x29, 0, {} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 80, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
static struct LCM_setting_table init_setting_12bit[] = {
	{0xFF, 01, {0x20} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x01, 01, {0x55} },
	{0x03, 01, {0x55} },
	{0x05, 01, {0xB1} },
	{0x07, 01, {0x73} },
	{0x08, 01, {0xCB} },
	{0x0E, 01, {0x91} },
	{0x0F, 01, {0x69} },
	{0x69, 01, {0xA9} },
	{0x94, 01, {0x40} },      /* add asymmetric gamma voltage */
	{0x95, 01, {0xE5} },
	{0x96, 01, {0x19} },
	{0xFF, 01, {0x24} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x93, 01, {0x08} },
	{0x94, 01, {0x06} },
	{0x01, 01, {0x00} },
	{0x02, 01, {0x00} },
	{0x03, 01, {0x08} },
	{0x04, 01, {0x18} },
	{0x05, 01, {0x19} },
	{0x06, 01, {0x0C} },
	{0x07, 01, {0x0D} },
	{0x08, 01, {0x0E} },
	{0x09, 01, {0x0F} },
	{0x0A, 01, {0x10} },
	{0x0B, 01, {0x11} },
	{0x0C, 01, {0x04} },
	{0x17, 01, {0x00} },
	{0x18, 01, {0x00} },
	{0x19, 01, {0x08} },
	{0x1A, 01, {0x18} },
	{0x1B, 01, {0x19} },
	{0x1C, 01, {0x0C} },
	{0x1D, 01, {0x0D} },
	{0x1E, 01, {0x0E} },
	{0x1F, 01, {0x0F} },
	{0x20, 01, {0x10} },
	{0x21, 01, {0x11} },
	{0x22, 01, {0x04} },
	{0x2F, 01, {0x04} },
	{0x33, 01, {0x32} },
	{0x37, 01, {0x44} },
	{0x39, 01, {0x00} },
	{0x3A, 01, {0x10} },
	{0x3B, 01, {0x8F} },
	{0x3D, 01, {0x51} },
	{0x3F, 01, {0x41} },
	{0x40, 01, {0x00} },
	{0x43, 01, {0x41} },
	{0x44, 01, {0x06} },
	{0x47, 01, {0x44} },
	{0x49, 01, {0x00} },
	{0x4A, 01, {0x10} },
	{0x4B, 01, {0x8F} },
	{0x4C, 01, {0x51} },
	{0x4D, 01, {0x12} },
	{0x4E, 01, {0x34} },
	{0x4F, 01, {0x56} },
	{0x51, 01, {0x65} },
	{0x52, 01, {0x43} },
	{0x53, 01, {0x21} },
	{0x55, 01, {0x43} },
	{0x56, 01, {0x46} },
	{0x5A, 01, {0x10} },
	{0x5B, 01, {0x8F} },
	{0x5E, 01, {0x06} },
	{0x5F, 01, {0x00} },
	{0x92, 01, {0xB7} },
	{0x76, 01, {0x00} },
	{0x77, 01, {0x00} },
	{0x78, 01, {0x00} },
	{0x79, 01, {0x00} },
	{0x7A, 01, {0x20} },
	{0x7B, 01, {0x3C} },
	{0x7C, 01, {0x00} },
	{0x7D, 01, {0x00} },
	{0x7E, 01, {0x08} },
	{0x87, 01, {0x02} },
	{0xAB, 01, {0x00} },
	{0xAC, 01, {0x00} },
	{0xAD, 01, {0x00} },
	{0xAF, 01, {0x04} },
	{0xB0, 01, {0x05} },
	{0xB1, 01, {0xB2} },
	{0x60, 01, {0x80} },
	{0x61, 01, {0x7C} },
	{0x64, 01, {0x11} },
	{0xFF, 01, {0x25} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x05, 01, {0x04} },
	{0x07, 01, {0x00} },
	{0x08, 01, {0x01} },
	{0x0A, 01, {0x82} },
	{0x0B, 01, {0x46} },
	{0x0C, 01, {0x01} },
	{0x17, 01, {0x82} },
	{0x18, 01, {0x06} },
	{0x19, 01, {0x0F} },
	{0xFF, 01, {0x26} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x00, 01, {0xA0} },
	{0xFF, 01, {0x27} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 01, {0x01} },
	{0x96, 01, {0x03} },
	{0x97, 01, {0x15} },
	{0x98, 01, {0x11} },
	{0x99, 01, {0x3F} },
	{0x9B, 01, {0x3F} },
	{0x9D, 01, {0x25} },
	{0x9E, 01, {0x55} },
	{0x9F, 01, {0x05} },
	{0xAD, 01, {0x25} },
	{0xAE, 01, {0x55} },
	{0xAF, 01, {0x05} },
	{0xB3, 01, {0xC6} },
	{0xC0, 01, {0x18} },
	{0xC1, 01, {0xE4} },
	{0xC2, 01, {0x00} },
	{0xC3, 01, {0x00} },
	{0xC4, 01, {0xE4} },
	{0xC5, 01, {0x00} },
	{0xC6, 01, {0x00} },
	{0xC7, 01, {0x03} },
	{0xFF, 1, {0x23} },
	{REGFLAG_DELAY, 5, {} },
	{0xFB, 1, {0x01} },
	{0x00, 1, {0x80} },
	{0x12, 1, {0xAB} },
	{0x15, 1, {0xF5} },
	{0x16, 1, {0x0B} },
	{0x07, 1, {0x00} },
	{0x08, 1, {0x01} },

	//page selection cmd start
	{0xFF, 01, {0x20} },
	{0xFB, 01, {0x01} },
	//page selection cmd end
	//R(+) MCR cmd
	{0xB0, 16, {0x00, 0x17, 0x00, 0x26, 0x00, 0x40, 0x00, 0x56, 0x00, 0x6A, 0x00, 0x7C, 0x00, 0x8C, 0x00, 0x9B} },
	{0xB1, 16, {0x00, 0xA8, 0x00, 0xD7, 0x00, 0xFC, 0x01, 0x37, 0x01, 0x65, 0x01, 0xAC, 0x01, 0xE7, 0x01, 0xE8} },
	{0xB2, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x83, 0x02, 0xBC, 0x02, 0xE2, 0x03, 0x0D, 0x03, 0x24, 0x03, 0x36} },
	{0xB3, 12, {0x03, 0x49, 0x03, 0x5E, 0x03, 0x6F, 0x03, 0x90, 0x03, 0xA6, 0x03, 0xFF} },
	//G(+) MCR
	{0xB4, 16, {0x00, 0x20, 0x00, 0x2E, 0x00, 0x46, 0x00, 0x5B, 0x00, 0x6F, 0x00, 0x81, 0x00, 0x91, 0x00, 0xA0} },
	{0xB5, 16, {0x00, 0xAD, 0x00, 0xDB, 0x01, 0x00, 0x01, 0x3A, 0x01, 0x67, 0x01, 0xAD, 0x01, 0xE7, 0x01, 0xE9} },
	{0xB6, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x84, 0x02, 0xBD, 0x02, 0xE2, 0x03, 0x0C, 0x03, 0x24, 0x03, 0x36} },
	{0xB7, 12, {0x03, 0x49, 0x03, 0x5C, 0x03, 0x6D, 0x03, 0x8F, 0x03, 0xCF, 0x03, 0xFF} },
	//B(+) MCR
	{0xB8, 16, {0x00, 0x01, 0x00, 0x13, 0x00, 0x32, 0x00, 0x4B, 0x00, 0x61, 0x00, 0x75, 0x00, 0x86, 0x00, 0x96} },
	{0xB9, 16, {0x00, 0xA4, 0x00, 0xD4, 0x00, 0xFA, 0x01, 0x35, 0x01, 0x63, 0x01, 0xAA, 0x01, 0xE5, 0x01, 0xE6} },
	{0xBA, 16, {0x02, 0x1C, 0x02, 0x5B, 0x02, 0x83, 0x02, 0xBE, 0x02, 0xE5, 0x03, 0x1F, 0x03, 0x33, 0x03, 0x4C} },
	{0xBB, 12, {0x03, 0x71, 0x03, 0xE2, 0x03, 0xE3, 0x03, 0xE4, 0x03, 0xEE, 0x03, 0xFF} },
	//page selection cmd start
	{0xFF, 1, {0x21} },
	{0xFB, 1, {0x01} },
	//page selection cmd end
	//R(-) MCR cmd
	{0xB0, 16, {0x00, 0x17, 0x00, 0x26, 0x00, 0x40, 0x00, 0x56, 0x00, 0x6A, 0x00, 0x7C, 0x00, 0x8C, 0x00, 0x9B} },
	{0xB1, 16, {0x00, 0xA8, 0x00, 0xD7, 0x00, 0xFC, 0x01, 0x37, 0x01, 0x65, 0x01, 0xAC, 0x01, 0xE7, 0x01, 0xE8} },
	{0xB2, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x83, 0x02, 0xBC, 0x02, 0xE2, 0x03, 0x0D, 0x03, 0x24, 0x03, 0x36} },
	{0xB3, 12, {0x03, 0x49, 0x03, 0x5E, 0x03, 0x6F, 0x03, 0x90, 0x03, 0xA6, 0x03, 0xFF} },
	//G(-) MCR
	{0xB4, 16, {0x00, 0x20, 0x00, 0x2E, 0x00, 0x46, 0x00, 0x5B, 0x00, 0x6F, 0x00, 0x81, 0x00, 0x91, 0x00, 0xA0} },
	{0xB5, 16, {0x00, 0xAD, 0x00, 0xDB, 0x01, 0x00, 0x01, 0x3A, 0x01, 0x67, 0x01, 0xAD, 0x01, 0xE7, 0x01, 0xE9} },
	{0xB6, 16, {0x02, 0x1E, 0x02, 0x5C, 0x02, 0x84, 0x02, 0xBD, 0x02, 0xE2, 0x03, 0x0C, 0x03, 0x24, 0x03, 0x36} },
	{0xB7, 12, {0x03, 0x49, 0x03, 0x5C, 0x03, 0x6D, 0x03, 0x8F, 0x03, 0xCF, 0x03, 0xFF} },
	//B(-) MCR
	{0xB8, 16, {0x00, 0x01, 0x00, 0x13, 0x00, 0x32, 0x00, 0x4B, 0x00, 0x61, 0x00, 0x75, 0x00, 0x86, 0x00, 0x96} },
	{0xB9, 16, {0x00, 0xA4, 0x00, 0xD4, 0x00, 0xFA, 0x01, 0x35, 0x01, 0x63, 0x01, 0xAA, 0x01, 0xE5, 0x01, 0xE6} },
	{0xBA, 16, {0x02, 0x1C, 0x02, 0x5B, 0x02, 0x83, 0x02, 0xBE, 0x02, 0xE5, 0x03, 0x1F, 0x03, 0x33, 0x03, 0x4C} },
	{0xBB, 12, {0x03, 0x71, 0x03, 0xE2, 0x03, 0xE3, 0x03, 0xE4, 0x03, 0xEE, 0x03, 0xFF} },
	// open tp term pump
	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x9E, 1, {0x00} },
	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0x13, 1, {0x00} },
	{0x15, 1, {0xB4} },
	{0x1F, 1, {0x55} },
	{0x26, 1, {0x0F} },
	//LVD setting
	{0xFF, 1, {0X20} },
	{0xFB, 1, {0x01} },
	{0x31, 1, {0x57} },
	{0x32, 1, {0x48} },
	{0xFF, 1, {0XF0} },
	{0xFB, 1, {0x01} },
	{0xCF, 1, {0x22} },
	{0xFF, 1, {0x10} },
	{REGFLAG_DELAY, 10, {} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x24} },
	{0x68, 2, {0x03, 0x01} },//manual  dim
	{0xBA, 1, {0x02} },
	{0x29, 0, {} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 80, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level_12bit[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_enable[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2C} },
};

#endif

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x01} },
};

static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x00} },
};

static void push_table(struct LCM_setting_table *table,
				unsigned int count,
				unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
				table[i].para_list, force_update);
		}
	}
}

static void cabc_push_table(void *cmdq, struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;
	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 232;
	params->dsi.vertical_frontporch = 8;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 26; //14
	params->dsi.horizontal_backporch = 34;   //160
	params->dsi.horizontal_frontporch = 34;  //120
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 500;
#else
	params->dsi.PLL_CLOCK = 362; //300
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
	if (product_hw_id == 1)
		params->dsi.PLL_CLOCK = 359;
	else {
		params->dsi.PLL_CLOCK = 362;
		params->dsi.horizontal_backporch = 38;
		}
#endif
#ifdef CONFIG_MTK_LCM_MIPI_CLK_CONTROL
	if (rf_hw_id == 1) {
		params->dsi.horizontal_backporch = 70;   //160
		params->dsi.horizontal_frontporch = 66;
		params->dsi.PLL_CLOCK = 387;
		printk("%s 1732 rf_hw_id PLL_CLOCK %d\n", __func__, params->dsi.PLL_CLOCK);
	}
#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x53;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
	params->dsi.HS_PRPR = 0x6;
	params->dsi.HS_TRAIL = 0x7;

}


static void lcm_init_power(void)
{
	printk("%s nt36525 -> enter power on begin\n", __func__);
	lcm_vddi_setting(1);
	MDELAY(3);
	lcm_enp_setting(1);
	MDELAY(3);
	lcm_enn_setting(1);
	MDELAY(5);
	#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
	if (product_hw_id == 1)
		;
	else {
		lcm_bias_set_avdd_n_avee(60);
		lcm_bias_set_control(80);
		}
	#endif
	printk("%s nt36525 -> enter power on end\n", __func__);
}

static void lcm_suspend_power(void)
{
	pr_err("[LCM]nt36525- lcm_suspend power off begin\n");
	#ifdef CONFIG_MTK_LCM_RESET_TOGGLE
	if (phone_shutdown_state)
		lcm_reset_setting(0);
	else
		pr_err("[LCM]nt36525- reset keep high level\n");
	#else
	if (lcm_ldo_vision) {
	lcm_reset_setting(0);
	} else {
		if (phone_shutdown_state)
			lcm_reset_setting(0);
		else
			pr_err("[LCM]nt36525- reset keep high level\n");
	}
	#endif
	MDELAY(10);
	lcm_enn_setting(0);
	MDELAY(5);
	lcm_enp_setting(0);
	MDELAY(5);
	#ifdef CONFIG_MTK_LCM_RESET_TOGGLE
	pr_err("[LCM]nt36525- vddi keep high level\n");
	#else
	if (lcm_ldo_vision) {
	lcm_vddi_setting(0);
	} else {
		pr_err("[LCM]nt36525- vddi not control\n");
	}
	#endif
	panel_reset_state = 0;// clear reset state
	pr_err("[LCM]nt36525- lcm_suspend power off end\n");
}
/*
static void lcm_resume_power(void)
{

}
*/
static void lcm_init(void)
{
	unsigned int data_array[16];
	pr_err("[LCM]nt36525- init begin\n");
	lcm_reset_setting(1);
	MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(3);
	lcm_reset_setting(1);
	MDELAY(90);
	panel_reset_state = 1; // set reset state
	panel_off_state = 0;
	dsi_set_hs_test(1);
	data_array[0] = 0x00000508;
	dsi_set_cmdq(data_array, 1, 1);
	dsi_set_hs_test(0);
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
	if (product_hw_id == 1)
		push_table(init_setting,
		sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	else
		push_table(init_setting_12bit,
		sizeof(init_setting_12bit) / sizeof(struct LCM_setting_table), 1);
#else
	push_table(init_setting,
		sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
#endif
	pr_err("[LCM]nt36525- init end\n");
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
		1);
	panel_off_state = 1;
	pr_err("[LCM]nt36525- lcm_suspend display off\n");
}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_reset_for_touch(void)
{
	//lcm_reset_setting(1);
	//MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(2);
	lcm_reset_setting(1);
	MDELAY(3);
	panel_reset_state = 1;
}
static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height)
{
#if LCM_DSI_CMD_MODE
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
#endif
}

#define LCM_ID_NT36525 (0x95)

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);

	SET_RESET_PIN(1);
	MDELAY(20);

	array[0] = 0x00023700;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0];		/* we only need ID */

	pr_debug("[LCM]%s,nt36525 debug: nt36525 id = 0x%08x\n", __func__, id);

	if (id == LCM_ID_NT36525)
		return 1;
	else
		return 0;

}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x53, buffer, 1);

	if (buffer[0] != 0x24) {
		pr_debug("[LCM][LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	pr_debug("[LCM][LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	pr_debug("[LCM]ATA check size = 0x%x,0x%x,0x%x,0x%x\n",
		x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	/* read id return two byte,version and id */
	data_array[0] = 0x00043700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
	    && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
	if (product_hw_id == 1) {
		pr_err("[LCM]%s,nt35595 backlight: level = %d\n", __func__, level);
		level = 255;
		bl_level[0].para_list[0] = level;
		push_table(bl_level,
			sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	} else {
		if (level <= 30 && level > 0)
			level = 30;
		if (level != 0 && dimming_enable == 0) {
		push_table(lcm_cmd_backlight_dimming_enable,
			sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 1;
		}
		if (level == 0)
			dimming_enable = 0;
		bl_level_12bit[0].para_list[0] = (unsigned char)((level>>8)&0xFF);
		bl_level_12bit[0].para_list[1] = (unsigned char)(level&0xFFFF);
		pr_err("%s: level=%d, high_bit=0x%x, low_bit=0x%x\n", __func__, level, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
		push_table(bl_level_12bit,
			sizeof(bl_level_12bit) / sizeof(struct LCM_setting_table), 1);
	}
#else
	pr_err("[LCM]%s,nt35595 backlight: level = %d\n", __func__, level);
	level = 255;
	bl_level[0].para_list[0] = level;
	push_table(bl_level,
		sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
#endif
}

#if 0 /* defined but not used */
static void lcm_setbacklight(unsigned int level)
{
	pr_debug("[LCM]%s,nt35595 backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = level;

	push_table(bl_level,
		sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}
#endif

static void *lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
/* customization: 1. V2C config 2 values, */
/* C2V config 1 value; 2. config mode control register */
	if (mode == 0) {	/* V2C */
		lcm_switch_mode_cmd.mode = CMD_MODE;
		/* mode control addr */
		lcm_switch_mode_cmd.addr = 0xBB;
		/* enabel GRAM firstly, ensure writing one frame to GRAM */
		lcm_switch_mode_cmd.val[0] = 0x13;
		/* disable video mode secondly */
		lcm_switch_mode_cmd.val[1] = 0x10;
	} else {		/* C2V */
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		/* disable GRAM and enable video mode */
		lcm_switch_mode_cmd.val[0] = 0x03;
	}
	return (void *)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}

static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	return 0x50;
}

static void lcm_cabc_vivo_open(void *handle, unsigned char leveldimming, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(lcm_cmd_cabc_off)))
		cabc_push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(lcm_cmd_cabc_level1)))
		cabc_push_table(handle, lcm_cmd_cabc_level1, sizeof(lcm_cmd_cabc_level1) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 2) && (sizeof(lcm_cmd_cabc_level2)))
		cabc_push_table(handle, lcm_cmd_cabc_level2, sizeof(lcm_cmd_cabc_level2) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 3) && (sizeof(lcm_cmd_cabc_level3)))
		cabc_push_table(handle, lcm_cmd_cabc_level3, sizeof(lcm_cmd_cabc_level3) / sizeof(struct LCM_setting_table), 1);
      printk("vincent=----lcm_cabc_vivo_open dim:%d,level:%d\n", leveldimming, levelsetting);
}

static void lcm_cabc_vivo_close(void *handle, unsigned char level)
{
	if (sizeof(lcm_cmd_cabc_off))
		cabc_push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	 printk("vincent=----lcm_cabc_vivo_colose level:%d\n", level);
}

struct LCM_DRIVER nt36525_hdp_dsi_vdo_txd_lm3697_622_lcm_drv = {
	.name = "nt36525_hdp_dsi_vdo_txd_lm3697_622_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_init_power,
	.suspend_power = lcm_suspend_power,
	.esd_check = lcm_esd_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.switch_mode = lcm_switch_mode,
	.get_id	    = lcm_get_id,
	.lcm_cabc_open  = lcm_cabc_vivo_open,
    .lcm_cabc_close = lcm_cabc_vivo_close,
    .lcm_reset = lcm_reset_for_touch,
};
