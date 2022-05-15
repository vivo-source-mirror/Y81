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

#define I2C_I2C_LCD_BIAS_CHANNEL 6
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL	/* for I2C channel 0 */
#define I2C_ID_NAME "ktd2151_dcdc"
#define TPS_ADDR 0x3E
/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info ktd2151_board_info __initdata = {
						I2C_BOARD_INFO(I2C_ID_NAME,
								TPS_ADDR) };

static struct i2c_client *ktd2151_i2c_client;
extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;


/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int ktd2151_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int ktd2151_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct ktd2151_dev {
	struct i2c_client *client;

};

static const struct i2c_device_id ktd2151_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver ktd2151_iic_driver = {
	.id_table = ktd2151_id,
	.probe = ktd2151_probe,
	.remove = ktd2151_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ktd2151_dcdc",
		   },
};

static int ktd2151_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	pr_err("[LCM]ktd2151_iic_probe in kernel\n");
	pr_debug("[LCM]TPS: info==>name=%s addr=0x%x\n",
		client->name, client->addr);
	ktd2151_i2c_client = client;
	return 0;
}

static int ktd2151_remove(struct i2c_client *client)
{
	pr_debug("[LCM]ktd2151_remove\n");
	ktd2151_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}
/*
static int ktd2151_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = ktd2151_i2c_client;
	char write_data[2] = { 0 };

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_debug("[LCM]ktd2151 write data fail !!\n");
	return ret;
}
*/
static int __init ktd2151_iic_init(void)
{
	pr_err("[LCM]ktd2151_iic_init\n");
	i2c_register_board_info(TPS_I2C_BUSNUM, &ktd2151_board_info, 1);
	pr_debug("[LCM]ktd2151_iic_init2\n");
	i2c_add_driver(&ktd2151_iic_driver);
	pr_debug("[LCM]ktd2151_iic_init success\n");
	return 0;
}

static void __exit ktd2151_iic_exit(void)
{
	pr_debug("[LCM]ktd2151_iic_exit\n");
	i2c_del_driver(&ktd2151_iic_driver);
}


module_init(ktd2151_iic_init);
module_exit(ktd2151_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK ktd2151 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH		(720)
#define FRAME_HEIGHT		(1440)

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
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x4F, 1, {0x01} },
	{REGFLAG_DELAY, 120, {} },
};

static struct LCM_setting_table init_setting[] = {
	
	{REGFLAG_DELAY, 1, {} },
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x05, 1, {0xA9} },
	{0x07, 1, {0x6E} },
	{0x08, 1, {0xBC} },
	{0x0E, 1, {0x87} },
	{0x0F, 1, {0x55} },
	{0x69, 1, {0xA9} },
	{0x87, 1, {0x02} },
	{0x89, 1, {0xB3} },
	{0x8A, 1, {0xA8} },
	{0x8B, 1, {0xB3} },
	{0x8C, 1, {0xA8} },
	{0x94, 1, {0x40} },
	{0x95, 1, {0xE1} },
	{0x96, 1, {0x09} },
	{0xFF, 1, {0x24} },
	{0xFB, 1, {0x01} },
	{0x00, 1, {0x00} },
	{0x01, 1, {0x08} },
	{0x02, 1, {0x23} },
	{0x03, 1, {0x04} },
	{0x04, 1, {0x05} },
	{0x05, 1, {0x1F} },
	{0x06, 1, {0x1E} },
	{0x07, 1, {0x15} },
	{0x08, 1, {0x14} },
	{0x09, 1, {0x17} },
	{0x0A, 1, {0x16} },
	{0x0B, 1, {0x00} },
	{0x0C, 1, {0x00} },
	{0x16, 1, {0x00} },
	{0x17, 1, {0x08} },
	{0x18, 1, {0x23} },
	{0x19, 1, {0x04} },
	{0x1A, 1, {0x05} },
	{0x1B, 1, {0x1F} },
	{0x1C, 1, {0x1E} },
	{0x1D, 1, {0x0D} },
	{0x1E, 1, {0x0C} },
	{0x1F, 1, {0x0F} },
	{0x20, 1, {0x0E} },
	{0x21, 1, {0x00} },
	{0x22, 1, {0x00} },
	{0x37, 1, {0x44} },
	{0x2F, 1, {0x0A} },
	{0x30, 1, {0x08} },
	{0x33, 1, {0x08} },
	{0x34, 1, {0x0A} },
	{0x39, 1, {0x00} },
	{0x3A, 1, {0x60} },
	{0x3B, 1, {0x8C} },
	{0x3D, 1, {0x52} },
	{0x3F, 1, {0x0D} },
	{0x43, 1, {0x0D} },
	{0x47, 1, {0x44} },
	{0x4A, 1, {0x05} },
	{0x4B, 1, {0x70} },
	{0x4C, 1, {0x51} },
	{0x4D, 1, {0x12} },
	{0x4E, 1, {0x34} },
	{0x51, 1, {0x43} },
	{0x52, 1, {0x21} },
	{0x55, 1, {0x47} },
	{0x56, 1, {0x44} },
	{0x58, 1, {0x21} },
	{0x59, 1, {0x00} },
	{0x5A, 1, {0x0A} },
	{0x5B, 1, {0x9F} },
	{0x5C, 1, {0x8F} },
	{0x5D, 1, {0x0A} },
	{0x5E, 1, {0x04} },
	{0x60, 1, {0x80} },
	{0x61, 1, {0x68} },
	{0x64, 1, {0x10} },
	{0x68, 1, {0x12} },
	{0x69, 1, {0x34} },
	{0x6A, 1, {0x43} },
	{0x6B, 1, {0x21} },
	{0x6C, 1, {0x47} },
	{0x6D, 1, {0x44} },
	{0x6E, 1, {0x21} },
	{0x6F, 1, {0x00} },
	{0x70, 1, {0x0A} },
	{0x71, 1, {0x9F} },
	{0x72, 1, {0x8F} },
	{0x73, 1, {0x0A} },
	{0x74, 1, {0x04} },
	{0x85, 1, {0x02} },
	{0x92, 1, {0xB9} },
	{0x93, 1, {0x0A} },
	{0x94, 1, {0x0C} },
	{0x9D, 1, {0x30} },
	{0xAB, 1, {0x00} },
	{0xAC, 1, {0x00} },
	{0xAD, 1, {0x00} },
	{0xB0, 1, {0x14} },
	{0xB1, 1, {0xA5} },
	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	{0x0A, 1, {0x82} },
	{0x0B, 1, {0xBE} },
	{0x0C, 1, {0x01} },
	{0x17, 1, {0x82} },
	{0x18, 1, {0x06} },
	{0x19, 1, {0x0F} },
	{0x1E, 1, {0x00} },
	{0x1F, 1, {0x0A} },
	{0x20, 1, {0x9F} },
	{0x21, 1, {0x0A} },
	{0x22, 1, {0x9F} },
	{0x23, 1, {0x14} },
	{0x24, 1, {0xA5} },
	{0x25, 1, {0x00} },
	{0x26, 1, {0x0A} },
	{0x27, 1, {0x9F} },
	{0x28, 1, {0x0A} },
	{0x29, 1, {0x9F} },
	{0x2A, 1, {0x14} },
	{0x2B, 1, {0xA5} },
	{0x2F, 1, {0x55} },
	{0x30, 1, {0x00} },
	{0x31, 1, {0x00} },
	{0x32, 1, {0x00} },
	{0x33, 1, {0x0A} },
	{0x34, 1, {0x9F} },
	{0x35, 1, {0x0A} },
	{0x36, 1, {0x9F} },
	{0x37, 1, {0x14} },
	{0x38, 1, {0xA5} },
	{0x40, 1, {0x11} },
	{0x41, 1, {0x80} },
	{0x42, 1, {0x0A} },
	{0x43, 1, {0x8C} },
	{0x44, 1, {0x0A} },
	{0x45, 1, {0x8C} },
	{0x46, 1, {0x14} },
	{0x47, 1, {0x82} },
	{0x4A, 1, {0x00} },
	{0x4B, 1, {0x05} },
	{0x4C, 1, {0x8C} },
	{0x4F, 1, {0x0A} },
	{0x50, 1, {0x9F} },
	{0x51, 1, {0x0A} },
	{0x52, 1, {0x9F} },
	{0x53, 1, {0x14} },
	{0x54, 1, {0xA5} },
	{0x55, 1, {0x8F} },
	{0x56, 1, {0x2A} },
	{0x58, 1, {0x00} },
	{0x59, 1, {0x00} },
	{0x5A, 1, {0x22} },
	{0x5B, 1, {0xC0} },
	{0x5C, 1, {0x00} },
	{0x5D, 1, {0x05} },
	{0x5E, 1, {0xB0} },
	{0x61, 1, {0x0A} },
	{0x62, 1, {0x9F} },
	{0x63, 1, {0x0A} },
	{0x64, 1, {0x9F} },
	{0x65, 1, {0x14} },
	{0x66, 1, {0xA5} },
	{0x6B, 1, {0x44} },
	{0x6C, 1, {0x0D} },
	{0x6D, 1, {0x0D} },
	{0x6E, 1, {0x0F} },
	{0x6F, 1, {0x0F} },
	{0x78, 1, {0x00} },
	{0x79, 1, {0x60} },
	{0x7A, 1, {0x00} },
	{0x7B, 1, {0x01} },
	{0x8A, 1, {0x02} },
	{0xCA, 1, {0x1C} },
	{0xCC, 1, {0x1C} },
	{0xCD, 1, {0x00} },
	{0xCE, 1, {0x00} },
	{0xCF, 1, {0x1C} },
	{0xD0, 1, {0x00} },
	{0xD1, 1, {0x00} },
	{0xD3, 1, {0x11} },
	{0xD4, 1, {0xCC} },
	{0xD5, 1, {0x11} },
	{0xFF, 1, {0x26} },
	{0xFB, 1, {0x01} },
	{0x00, 1, {0xA1} },
	{0x04, 1, {0x4B} },
	{0x0C, 1, {0x0B} },
	{0x0D, 1, {0x01} },
	{0x0E, 1, {0x02} },
	{0x0F, 1, {0x06} },
	{0x10, 1, {0x07} },
	{0x11, 1, {0x00} },
	{0x13, 1, {0x30} },
	{0x14, 1, {0x80} },
	{0x16, 1, {0x81} },
	{0x19, 1, {0x15} },
	{0x1A, 1, {0x0D} },
	{0x1B, 1, {0x12} },
	{0x1C, 1, {0x82} },
	{0x1D, 1, {0x00} },
	{0x1E, 1, {0xB9} },
	{0x1F, 1, {0xB9} },
	{0x24, 1, {0x01} },
	{0x25, 1, {0xB9} },
	{0x2F, 1, {0x03} },
	{0x30, 1, {0x96} },
	{0x31, 1, {0x11} },
	{0x32, 1, {0x11} },
	{0x34, 1, {0x02} },
	{0x35, 1, {0xB9} },
	{0x36, 1, {0x67} },
	{0x37, 1, {0x11} },
	{0x38, 1, {0x11} },
	{0x3F, 1, {0x04} },
	{0x40, 1, {0xB9} },
	{0x41, 1, {0x00} },
	{0x42, 1, {0x00} },
	{0x49, 1, {0x00} },
	{0x58, 1, {0xE1} },
	{0x59, 1, {0xE1} },
	{0x5A, 1, {0xE1} },
	{0x5B, 1, {0xE1} },
	{0x5C, 1, {0x01} },
	{0x5D, 1, {0x02} },
	{0x5E, 1, {0x04} },
	{0x5F, 1, {0x00} },
	{0x60, 1, {0x00} },
	{0x61, 1, {0x00} },
	{0x62, 1, {0x00} },
	{0x63, 1, {0x60} },
	{0x64, 1, {0xB0} },
	{0x65, 1, {0x05} },
	{0x66, 1, {0xB0} },
	{0x67, 1, {0x0A} },
	{0x68, 1, {0x9F} },
	{0x69, 1, {0x0A} },
	{0x6A, 1, {0x9F} },
	{0x6B, 1, {0x00} },
	{0x6C, 1, {0x00} },
	{0x6D, 1, {0x00} },
	{0x70, 1, {0x14} },
	{0x71, 1, {0xA5} },
	{0x73, 1, {0x0A} },
	{0x74, 1, {0x9F} },
	{0x75, 1, {0x0A} },
	{0x76, 1, {0x9F} },
	{0x77, 1, {0x14} },
	{0x78, 1, {0xA5} },
	{0x7A, 1, {0x0A} },
	{0x7B, 1, {0x9F} },
	{0x7C, 1, {0x0A} },
	{0x7D, 1, {0x9F} },
	{0x7E, 1, {0x14} },
	{0x7F, 1, {0xA5} },
	{0x82, 1, {0x60} },
	{0x83, 1, {0xB0} },
	{0x84, 1, {0x05} },
	{0x85, 1, {0xB0} },
	{0x86, 1, {0x0A} },
	{0x87, 1, {0x9F} },
	{0x88, 1, {0x0A} },
	{0x89, 1, {0x9F} },
	{0x8A, 1, {0x14} },
	{0x8B, 1, {0xA5} },
	{0x8C, 1, {0x00} },
	{0x8D, 1, {0x00} },
	{0x8E, 1, {0x01} },
	{0x8F, 1, {0x00} },
	{0x90, 1, {0x00} },
	{0x91, 1, {0x00} },
	{0x92, 1, {0x05} },
	{0x93, 1, {0xA0} },
	{0x94, 1, {0x00} },
	{0x96, 1, {0x00} },
	{0x99, 1, {0x19} },
	{0x9A, 1, {0xA0} },
	{0x9B, 1, {0x19} },
	{0x9C, 1, {0x08} },
	{0xFF, 1, {0x23} },
	{0xFB, 1, {0x01} },
	{0x15, 1, {0x81} },
	{0x16, 1, {0x0A} },
	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0x13, 1, {0x00} },
	{0xFF, 1, {0xD0} },
	{0xFB, 1, {0x01} },
	{0x2A, 1, {0x10} },
	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	{0x9E, 1, {0x00} },
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x51, 1, {0xFF} },
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x01} },
	//{0xB0,1,{0x01}},
	{0xBA, 1, {0x02} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 0, {} },
	{REGFLAG_DELAY, 10, {} },
	//{0xFF,1,{0x25}},
	//{0xEC,1,{0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
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
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch = 10;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 2; //14
	params->dsi.horizontal_backporch = 100;   //160
	params->dsi.horizontal_frontporch = 100;  //120
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 500;
#else
	params->dsi.PLL_CLOCK = 343; //300
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x53;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;

}

#ifdef BUILD_LK
#ifndef CONFIG_FPGA_EARLY_PORTING
#define ktd2151_SLAVE_ADDR_WRITE  0x7C
static struct mt_i2c_t ktd2151_i2c;

static int ktd2151_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;

	ktd2151_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;	/* I2C2; */
	/* Since i2c will left shift 1 bit, */
	/* we need to set FAN5405 I2C address to >>1 */
	ktd2151_i2c.addr = (ktd2151_SLAVE_ADDR_WRITE >> 1);
	ktd2151_i2c.mode = ST_MODE;
	ktd2151_i2c.speed = 100;
	len = 2;

	ret_code = i2c_write(&ktd2151_i2c, write_data, len);
	/* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

	return ret_code;
}

#else

/* extern int mt8193_i2c_write(u16 addr, u32 data); */
/* extern int mt8193_i2c_read(u16 addr, u32 *data); */

/* #define ktd2151_write_byte(add, data)  mt8193_i2c_write(add, data) */
	/* #define ktd2151_read_byte(add)  mt8193_i2c_read(add) */

#endif
#endif


static void lcm_init_power(void)
{
	printk("%s nt36525 -> enter\n", __func__);
	lcm_vddi_setting(1);
	MDELAY(1);
	lcm_enp_setting(1);
	MDELAY(5);
	lcm_enn_setting(1);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(10);
	lcm_reset_setting(0);
	MDELAY(10);
	lcm_reset_setting(1);
	MDELAY(20);
	//lcm_bkg_setting(1);
	panel_reset_state = 1; // set reset state
}

static void lcm_suspend_power(void)
{
	pr_err("[LCM]nt36525- lcm_suspend\n");
	lcm_reset_setting(0);
	MDELAY(10);
	lcm_enn_setting(0);
	MDELAY(5);
	lcm_enp_setting(0);
	MDELAY(5);
	lcm_vddi_setting(0);
	panel_reset_state = 0;// clear reset state
}
/*
static void lcm_resume_power(void)
{

}
*/
static void lcm_init(void)
{

/*
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
*/
	pr_err("[LCM]nt36525- init\n");
	/*MDELAY(1);
	lcm_vddi_setting(1);
	MDELAY(1);
	lcm_enp_setting(1);
	MDELAY(5);
	lcm_enn_setting(1);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(10);
	lcm_reset_setting(0);
	MDELAY(10);
	lcm_reset_setting(1);
	MDELAY(20);
	lcm_bkg_setting(1);*/
	panel_off_state = 0;
	push_table(init_setting,
		sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
/*#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ZERO);
#endif
#endif*/
	push_table(lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
		1);
	panel_off_state = 1;
	pr_err("[LCM]nt36525- lcm_suspend\n");
	lcm_reset_setting(0);
	MDELAY(10);
	lcm_enn_setting(0);
	MDELAY(5);
	lcm_enp_setting(0);
	MDELAY(5);
	lcm_vddi_setting(0);
}

static void lcm_resume(void)
{
	lcm_init();
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

	pr_err("[LCM]%s,nt35595 backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = level;

	push_table(bl_level,
		sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
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
	return 0x10;
}


struct LCM_DRIVER nt36525_hdp_dsi_vdo_tianma_lm3697_lcm_drv = {
	.name = "nt36525_hdp_dsi_vdo_tianma_lm3697_drv",
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
};
