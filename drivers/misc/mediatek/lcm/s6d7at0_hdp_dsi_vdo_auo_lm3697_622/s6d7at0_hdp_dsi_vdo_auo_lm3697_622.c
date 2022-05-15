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
extern unsigned int allGestureSwitchIntoBitmap(void);


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
#ifndef CONFIG_FPGA_EARLY_PORTING
extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;
extern unsigned int lcm_ldo_vision;
extern unsigned int lcm_id_version;
extern unsigned int rf_hw_id;
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
extern unsigned int product_hw_id;
static unsigned int dimming_enable;
extern void lcm_bias_set_avdd_n_avee(int value);
extern void lcm_bias_set_control(int level);
#endif
#endif
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

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[80];
};

static struct LCM_setting_table lcm_suspend_setting_enter_gesture[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 100, {} },
	{0x24, 0, {} },
	{REGFLAG_DELAY, 50, {} },
};

static struct LCM_setting_table lcm_suspend_setting_disable_gesture[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 100, {} },
	{0x4F, 1, {0x01} },
	{REGFLAG_DELAY, 100, {} },
};

static struct LCM_setting_table init_setting[] = {
	{REGFLAG_DELAY, 1, {} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0C} },
	{0xF2, 1, {0x84} },
	{0xB0, 1, {0x0A} },
	{0xF4, 1, {0xB7} },
	{0xC0, 15, {0x23, 0x01, 0x7B, 0x01, 0x00, 0xFF, 0x00, 0x44, 0x0F, 0xFF, 0x70, 0x40, 0x88, 0xE6, 0xA5} },
	{0xC1, 36, {0xBF, 0x7F, 0x7F, 0x25, 0x66, 0x7D, 0x7F, 0x82, 0x8A, 0x8C, 0x94, 0xA0, 0xAE, 0xBA, 0xC2, 0xD4, 0xDE, 0xF0,
		0xFA, 0xFF, 0x88, 0x96, 0xA5, 0xB4, 0xBE, 0xC8, 0xCD, 0xD0, 0xD3, 0xD7, 0xDF, 0xE6, 0xEA, 0xF0, 0xF5, 0xFF} },
	{0x73, 6, {0xB0, 0xA5, 0x00, 0x01, 0x61, 0x00} },
	{0xB3, 16, {0x01, 0xBE, 0x07, 0x20, 0x20, 0x0A, 0x08, 0x08, 0x07, 0x6F, 0x01, 0x29, 0x01, 0x29, 0x01, 0x29} },
	{0xED, 70, {0x66, 0x40, 0x00, 0x93, 0x00, 0x6A, 0x01, 0x67, 0x40, 0x00, 0x93, 0x00, 0x4A, 0x01, 0x66, 0x44,
	0x00, 0x93, 0x00, 0x2A, 0x01, 0x67, 0x44, 0x00, 0x93, 0x00, 0x0A, 0x01, 0x66, 0x48, 0x00, 0x93,
	0x00, 0xEA, 0x01, 0x78, 0x48, 0x00, 0x93, 0x00, 0xCA, 0x01, 0x67, 0x44, 0x00, 0x93, 0x00, 0xAA,
	0x01, 0x68, 0x44, 0x00, 0x93, 0x00, 0x8A, 0x01, 0x40, 0x40, 0x80, 0x08, 0x40, 0x48, 0x80, 0x08,
	0x40, 0xC0, 0x00, 0x00, 0x00, 0x00} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x53, 1, {0x24} },
	{0x55, 1, {0x01} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 110, {} },
	{0x29, 0, {} },
	{REGFLAG_DELAY, 10, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x01} },
};

static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x00} },
};
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
static struct LCM_setting_table init_setting_bit12[] = {
	{REGFLAG_DELAY, 1, {} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0C} },
	{0xF2, 1, {0x84} },
	{0xB0, 1, {0x0A} },
	{0xF4, 1, {0xB7} },
	{0xC0, 15, {0x23, 0x01, 0x7B, 0x01, 0x00, 0xFF, 0x00, 0x04, 0x0F, 0xFF, 0x70, 0x40, 0x88, 0xE6, 0xA5} },
	{0xC1, 36, {0xBF, 0x7F, 0x7F, 0x25, 0x66, 0x7D, 0x7F, 0x82, 0x8A, 0x8C, 0x94, 0xA0, 0xAE, 0xBA, 0xC2, 0xD4, 0xDE, 0xF0,
		0xFA, 0xFF, 0x88, 0x96, 0xA5, 0xB4, 0xBE, 0xC8, 0xCD, 0xD0, 0xD3, 0xD7, 0xDF, 0xE6, 0xEA, 0xF0, 0xF5, 0xFF} },
	{0x73, 6, {0xB0, 0xA5, 0x00, 0x01, 0x61, 0x00} },
	{0xB3, 16, {0x01, 0xBE, 0x07, 0x20, 0x20, 0x0A, 0x08, 0x08, 0x07, 0x6F, 0x01, 0x29, 0x01, 0x29, 0x01, 0x29} },
	{0xED, 70, {0x66, 0x40, 0x00, 0x93, 0x00, 0x6A, 0x01, 0x67, 0x40, 0x00, 0x93, 0x00, 0x4A, 0x01, 0x66, 0x44,
	0x00, 0x93, 0x00, 0x2A, 0x01, 0x67, 0x44, 0x00, 0x93, 0x00, 0x0A, 0x01, 0x66, 0x48, 0x00, 0x93,
	0x00, 0xEA, 0x01, 0x78, 0x48, 0x00, 0x93, 0x00, 0xCA, 0x01, 0x67, 0x44, 0x00, 0x93, 0x00, 0xAA,
	0x01, 0x68, 0x44, 0x00, 0x93, 0x00, 0x8A, 0x01, 0x40, 0x40, 0x80, 0x08, 0x40, 0x48, 0x80, 0x08,
	0x40, 0xC0, 0x00, 0x00, 0x00, 0x00} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x53, 1, {0x24} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 110, {} },
	{0x29, 0, {} },
	{REGFLAG_DELAY, 10, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level_12bit[] = {
	{0x51, 2, {0xFF, 0xF1} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_enable[] = {
	{0x53, 1, {0x2C} },
};
#endif

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

	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 96;
	params->dsi.vertical_frontporch = 251;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 8; //14
	params->dsi.horizontal_backporch = 8;   //160
	params->dsi.horizontal_frontporch = 24;  //120
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 500;
#else
	params->dsi.PLL_CLOCK = 360; //300
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
#ifdef CONFIG_MTK_LCM_MIPI_CLK_CONTROL
		params->dsi.horizontal_backporch = 8;
		params->dsi.horizontal_frontporch = 24;
		params->dsi.PLL_CLOCK = 359;
#endif
#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
	if (product_hw_id == 1)
		params->dsi.PLL_CLOCK = 359;
	else {
		params->dsi.PLL_CLOCK = 362;
		params->dsi.horizontal_backporch = 12;
		}
#endif
#ifdef CONFIG_MTK_LCM_MIPI_CLK_CONTROL
		if (rf_hw_id == 1) {
			params->dsi.horizontal_backporch = 40;   //160
			params->dsi.horizontal_frontporch = 50;
			params->dsi.PLL_CLOCK = 387;
		}
#endif

	printk("%s rf_hw_id PLL_CLOCK %d\n", __func__, params->dsi.PLL_CLOCK);
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
	printk("%s s6d7at0 -> enter power on begin\n", __func__);
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
		lcm_bias_set_avdd_n_avee(55);
		lcm_bias_set_control(80);
		}
#endif
	printk("%s s6d7at0 -> enter power on end\n", __func__);
}

static void lcm_suspend_power(void)
{
	pr_err("[LCM]s6d7at0- lcm_suspend power off begin\n");
	#ifdef CONFIG_MTK_LCM_RESET_TOGGLE
	pr_err("[LCM]s6d7at0- reset keep high level\n");
	#else
	if (lcm_ldo_vision) {
	lcm_reset_setting(0);
	} else {
		pr_err("[LCM]s6d7at0- reset keep high level\n");
	}
	#endif
	MDELAY(10);
	lcm_enn_setting(0);
	MDELAY(5);
	lcm_enp_setting(0);
	MDELAY(5);
	#ifdef CONFIG_MTK_LCM_RESET_TOGGLE
	pr_err("[LCM]s6d7at0- vddi keep high level\n");
	#else
	if (lcm_ldo_vision) {
	lcm_vddi_setting(0);
	} else {
		pr_err("[LCM]s6d7at0- vddi not control\n");
	}
	#endif
	panel_reset_state = 0;// clear reset state
	pr_err("[LCM]s6d7at0- lcm_suspend power off end\n");
}
/*
static void lcm_resume_power(void)
{

}
*/
static void lcm_init(void)
{
	pr_err("[LCM]s6d7at0- init begin\n");
	lcm_reset_setting(1);
	MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(3);
	lcm_reset_setting(1);
	MDELAY(10);
	panel_reset_state = 1; // set reset state
	panel_off_state = 0;
	#ifdef CONFIG_VIVO_LCM_PD1732DF_CONTROL
	if (product_hw_id == 1)
		push_table(init_setting,
		sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	else
		push_table(init_setting_bit12,
		sizeof(init_setting_bit12) / sizeof(struct LCM_setting_table), 1);
	#else
	push_table(init_setting,
		sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	#endif
	pr_err("[LCM]s6d7at0- init end\n");
}

static void lcm_suspend(void)
{
	unsigned int gs_mode = allGestureSwitchIntoBitmap();
	if (gs_mode != 0)
	push_table(lcm_suspend_setting_enter_gesture,
		sizeof(lcm_suspend_setting_enter_gesture) / sizeof(struct LCM_setting_table),
		1);
	else
		push_table(lcm_suspend_setting_disable_gesture,
		sizeof(lcm_suspend_setting_disable_gesture) / sizeof(struct LCM_setting_table),
		1);
	panel_off_state = 1;
	pr_err("[LCM]s6d7at0- lcm_suspend display off and gs mode %d\n", gs_mode);
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

#define LCM_ID_s6d7at0 (0x95)

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

	pr_debug("[LCM]%s,s6d7at0 debug: s6d7at0 id = 0x%08x\n", __func__, id);

	if (id == LCM_ID_s6d7at0)
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
		pr_err("[LCM]%s,s6d7at0 backlight: level = %d\n", __func__, level);
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
		level = level << 4;
		level = level | 0x0001;
		pr_err("[LCM]%s,s6d7at0 backlight: level<<4 = 0x%x\n", __func__, level);
		bl_level_12bit[0].para_list[0] = (unsigned char)((level>>8)&0xFF);
		bl_level_12bit[0].para_list[1] = (unsigned char)(((level&0x00FF))|0x01);
		pr_err("%s: level=%d, high_bit=0x%x, low_bit=0x%x\n", __func__, level, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
		push_table(bl_level_12bit,
			sizeof(bl_level_12bit) / sizeof(struct LCM_setting_table), 1);
	}
	#else
	pr_err("[LCM]%s,s6d7at0 backlight: level = %d\n", __func__, level);
	level = 255;
	bl_level[0].para_list[0] = level;
	push_table(bl_level,
		sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	#endif
}


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
	if (lcm_id_version == 0x78)
		return 0x20;
	else if (lcm_id_version == 0x43)
		return 0x21;
	else if (lcm_id_version == 0x47)
		return 0x22;
	else
		return 0x20;
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

struct LCM_DRIVER s6d7at0_hdp_dsi_vdo_auo_lm3697_622_lcm_drv = {
	.name = "s6d7at0_hdp_dsi_vdo_auo_lm3697_622_drv",
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
