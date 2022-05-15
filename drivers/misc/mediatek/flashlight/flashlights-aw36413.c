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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

#define HOPE_ADD_IRQ_FLASH
#ifdef HOPE_ADD_IRQ_FLASH
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#endif

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef AW36413_DTNAME
#define AW36413_DTNAME "mediatek,flashlights_aw36413"
#endif
#ifndef AW36413_DTNAME_I2C
#define AW36413_DTNAME_I2C "mediatek,flashlights_aw36413_i2c"
#endif

#define AW36413_NAME "flashlights-aw36413"

/* define registers */
#define AW36413_REG_ENABLE (0x01)
#define AW36413_MASK_ENABLE_LED1 (0x01)
#define AW36413_MASK_ENABLE_LED2 (0x02)
#define AW36413_DISABLE (0x00)
#define AW36413_ENABLE_LED1 (0x01)
#define AW36413_ENABLE_LED1_TORCH (0x09)
#define AW36413_ENABLE_LED1_FLASH (0x0D)
#define AW36413_ENABLE_LED2 (0x02)
#define AW36413_ENABLE_LED2_TORCH (0x0A)
#define AW36413_ENABLE_LED2_FLASH (0x0E)

#define AW36413_REG_TORCH_LEVEL_LED1 (0x05)
#define AW36413_REG_FLASH_LEVEL_LED1 (0x03)
#define AW36413_REG_TORCH_LEVEL_LED2 (0x06)
#define AW36413_REG_FLASH_LEVEL_LED2 (0x04)
#define AW36413_REG_FLAGS_1 (0x0A)
#define AW36413_REG_FLAGS_2 (0x0B)
#define AW36413_DEVICE_ID (0x0C)

#define AW36413_REG_TIMING_CONF (0x08)
#define AW36413_TORCH_RAMP_TIME (0x10)
#define AW36413_FLASH_TIMEOUT   (0x0F)
#define AW36413_FLASH_TIMEOUT_90MS   (0x08)
#define AW36413TT_FLASH_TIMEOUT (0x09)

#define AW36413_REG_DEVICE_ID (0x0C)

/* define channel, level */
#define AW36413_CHANNEL_NUM 2
#define AW36413_CHANNEL_CH1 0
#define AW36413_CHANNEL_CH2 1

#define AW36413_LEVEL_NUM 26
#define AW36413_LEVEL_TORCH 7

#define AW36413_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(aw36413_mutex);
static DEFINE_MUTEX(pin_set_mutex);
static struct work_struct aw36413_work_ch1;
static struct work_struct aw36413_work_ch2;

/* define pinctrl */
#define AW36413_PINCTRL_PIN_HWEN 0
#define AW36413_PINCTRL_PINSTATE_LOW 0
#define AW36413_PINCTRL_PINSTATE_HIGH 1
#define AW36413_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define AW36413_PINCTRL_STATE_HWEN_LOW  "hwen_low"
#ifdef HOPE_ADD_IRQ_FLASH
#define AW36413_PINCTRL_STATE_FLASH_IRQ "flash_irq"
#endif

static struct pinctrl *aw36413_pinctrl;
static struct pinctrl_state *aw36413_hwen_high;
static struct pinctrl_state *aw36413_hwen_low;
#ifdef HOPE_ADD_IRQ_FLASH
static int g_flash_irq_num;
static  unsigned int g_gpio_pin;
static unsigned int g_gpio_headset_deb;
static struct pinctrl_state *aw36413_flash_irq;
static unsigned int g_accdet_eint_type = IRQ_TYPE_LEVEL_LOW;
static struct delayed_work ir_delayed_work;
static unsigned int led_count;
static unsigned int irq_enable_count;
static ktime_t StartTime;
#endif


/* define usage count */
static int use_count;
static int lock_touch;  /*hope add*/
static int lock_touch_sub; /*hope add*/

/* define i2c */
static struct i2c_client *aw36413_i2c_client;

/* platform data */
struct aw36413_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* aw36413 chip data */
struct aw36413_chip_data {
	struct i2c_client *client;
	struct aw36413_platform_data *pdata;
	struct mutex lock;
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int aw36413_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	aw36413_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw36413_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw36413_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	aw36413_hwen_high = pinctrl_lookup_state(aw36413_pinctrl, AW36413_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(aw36413_hwen_high)) {
		pr_err("Failed to init (%s)\n", AW36413_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(aw36413_hwen_high);
	}
	aw36413_hwen_low = pinctrl_lookup_state(aw36413_pinctrl, AW36413_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(aw36413_hwen_low)) {
		pr_err("Failed to init (%s)\n", AW36413_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(aw36413_hwen_low);
	}
	
	return ret;
}

static int aw36413_pinctrl_init_1(void)
{
	int ret = 0;
	
#ifdef HOPE_ADD_IRQ_FLASH
	aw36413_flash_irq = pinctrl_lookup_state(aw36413_pinctrl, AW36413_PINCTRL_STATE_FLASH_IRQ);
	if (IS_ERR(aw36413_flash_irq)) {
		pr_err("Failed to init (%s)\n", AW36413_PINCTRL_STATE_FLASH_IRQ);
		ret = PTR_ERR(aw36413_flash_irq);
	}
#endif
	return ret;
}

static int aw36413_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(aw36413_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case AW36413_PINCTRL_PIN_HWEN:
		mutex_lock(&pin_set_mutex);
		if (state == AW36413_PINCTRL_PINSTATE_LOW && !IS_ERR(aw36413_hwen_low))
			pinctrl_select_state(aw36413_pinctrl, aw36413_hwen_low);
		else if (state == AW36413_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw36413_hwen_high))
			pinctrl_select_state(aw36413_pinctrl, aw36413_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		mutex_unlock(&pin_set_mutex);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	/*pr_debug("pin(%d) state(%d)\n", pin, state);*/

	return ret;
}


/******************************************************************************
 * aw36413 operations
 *****************************************************************************/
static const int aw36413_current[AW36413_LEVEL_NUM] = {  /*current:mA */
	 33,  51,  65,  75,  93, 116, 140, 198,  246, 304,
	351, 398, 445, 503, 550, 597, 656, 703,  750, 796,
	857, 902, 961, 1008, 1067, 1100};


static const unsigned char aw36413_torch_level[AW36413_LEVEL_NUM] = {
	0x0A, 0x11, 0x15, 0x19, 0x1F, 0x27, 0x2F, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* torch duty=0x17 << current=33mA */

static const unsigned char aw36413_flash_level[AW36413_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};


static unsigned char aw36413_reg_enable;
static int aw36413_level_ch1 = -1;
static int aw36413_level_ch2 = -1;

static int aw36413_is_torch(int level)
{
	if (level >= AW36413_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw36413_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= AW36413_LEVEL_NUM)
		level = AW36413_LEVEL_NUM - 1;

	return level;
}

static int aw36413_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct aw36413_chip_data *chip = i2c_get_clientdata(client);
	aw36413_pinctrl_set(AW36413_PINCTRL_PIN_HWEN, AW36413_PINCTRL_PINSTATE_HIGH);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

/* i2c wrapper function */
static int aw36413_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char  flags_1_val, flags_2_val, timimg_conf_val;
	
	struct aw36413_chip_data *chip = i2c_get_clientdata(client);
	aw36413_pinctrl_set(AW36413_PINCTRL_PIN_HWEN, AW36413_PINCTRL_PINSTATE_HIGH);

	timimg_conf_val = aw36413_read_reg(aw36413_i2c_client, AW36413_REG_TIMING_CONF);
	flags_1_val = aw36413_read_reg(aw36413_i2c_client,AW36413_REG_FLAGS_1);
	flags_2_val = aw36413_read_reg(aw36413_i2c_client,AW36413_REG_FLAGS_2);
	/*pr_debug("flags_1_val =0x%x, flags_2_val=0x%x, timimg_conf_val = 0x%x\n", flags_1_val, flags_2_val, timimg_conf_val);*/
	if((flags_1_val & 0x7E)||(flags_2_val & 0x1E))
		pr_err("flashlight err\n");

	
	ret = aw36413_read_reg(aw36413_i2c_client,AW36413_REG_TORCH_LEVEL_LED1);
	/*pr_debug("AW36413_REG_TORCH_LEVEL_LED1 =0x%x\n",ret);*/

	ret = aw36413_read_reg(aw36413_i2c_client,AW36413_REG_FLASH_LEVEL_LED1);
	/*pr_debug("AW36413_REG_FLASH_LEVEL_LED1 =0x%x\n", ret);*/
	
	/*pr_debug("===hope aw36413_write_reg  reg =%d,val = %d\n",reg, val);*/
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}


/* flashlight enable function */
static int aw36413_enable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36413_REG_ENABLE;
	if (!aw36413_is_torch(aw36413_level_ch1)) {
		/* torch mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED1_FLASH;
	}
	val = aw36413_reg_enable;

	return aw36413_write_reg(aw36413_i2c_client, reg, val);
}

static int aw36413_enable_ch2(void)
{
#ifdef HOPE_ADD_IRQ_FLASH

	unsigned char reg, val;
	if (g_flash_irq_num > 0 && irq_enable_count == 0) {
		pr_debug("RED FLASH ON  g_flash_irq_num =%d\n", g_flash_irq_num);
		enable_irq(g_flash_irq_num);
		irq_enable_count++;
		led_count = 0;
		StartTime = ktime_get();
		reg = AW36413_REG_TIMING_CONF;
		val = AW36413_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT_90MS;   /*setting time_out = 90 ms*/
		return aw36413_write_reg(aw36413_i2c_client, reg, val);
	}else{
		pr_err("aw36413_enable_ch2 g_flash_irq_num is error g_flash_irq_num = %d, irq_enable_count = %d\n", g_flash_irq_num, irq_enable_count);
		return -1;
	}
	#else
	
	unsigned char reg, val;
	reg = AW36413_REG_ENABLE;
	if (!aw36413_is_torch(aw36413_level_ch2)) {
		/* torch mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		aw36413_reg_enable |= AW36413_ENABLE_LED2_FLASH;
	}
	val = aw36413_reg_enable;
	return aw36413_write_reg(aw36413_i2c_client, reg, val);
	
	#endif
}

static int aw36413_enable(int channel)
{
	if (channel == AW36413_CHANNEL_CH1)
		aw36413_enable_ch1();
	else if (channel == AW36413_CHANNEL_CH2)
		aw36413_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int aw36413_disable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36413_REG_ENABLE;
	if (aw36413_reg_enable & AW36413_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED1);
	} else {
		/* if LED 2 is disable, disable LED 1 and clear mode */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED1_FLASH);
	}
	val = aw36413_reg_enable;

	return aw36413_write_reg(aw36413_i2c_client, reg, val);
}

static int aw36413_disable_ch2(void)
{
	
#ifdef HOPE_ADD_IRQ_FLASH
	unsigned char reg, val;
	if (g_flash_irq_num > 0 && irq_enable_count == 1) {
		pr_debug("RED FLASH OFF  g_flash_irq_num =%d\n", g_flash_irq_num);
		disable_irq(g_flash_irq_num);
		irq_enable_count--;
		led_count = 0;
		StartTime = ktime_get();
		/* set torch current ramp time and flash timeout */
		reg = AW36413_REG_TIMING_CONF;
		val = AW36413_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT;  /*setting time_out = max default*/
		return aw36413_write_reg(aw36413_i2c_client, reg, val);
	}else{
		pr_err("aw36413_enable_ch2 g_flash_irq_num is error g_flash_irq_num = %d, irq_enable_count = %d\n", g_flash_irq_num, irq_enable_count);
		return -1;
	}
#else

	unsigned char reg, val;
	reg = AW36413_REG_ENABLE;
	if (aw36413_reg_enable & AW36413_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		aw36413_reg_enable &= (~AW36413_ENABLE_LED2_FLASH);
	}
	val = aw36413_reg_enable;
	return aw36413_write_reg(aw36413_i2c_client, reg, val);
#endif
}

static int aw36413_disable(int channel)
{
	if (channel == AW36413_CHANNEL_CH1)
		aw36413_disable_ch1();
	else if (channel == AW36413_CHANNEL_CH2)
		aw36413_disable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int aw36413_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36413_verify_level(level);

	/* set torch brightness level */
	reg = AW36413_REG_TORCH_LEVEL_LED1;
	val = aw36413_torch_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	aw36413_level_ch1 = level;

	/* set flash brightness level */
	reg = AW36413_REG_FLASH_LEVEL_LED1;
	val = aw36413_flash_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	return ret;
}

static int aw36413_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36413_verify_level(level);

	/* set torch brightness level */
	reg = AW36413_REG_TORCH_LEVEL_LED2;

	val = aw36413_torch_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, AW36413_REG_TORCH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	aw36413_level_ch2 = level;

	/* set flash brightness level */
	reg = AW36413_REG_FLASH_LEVEL_LED2;
	val = aw36413_flash_level[level];
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	return ret;
}

#ifdef HOPE_ADD_IRQ_FLASH
static int aw36413_set_red_flash_level_and_enabe_ch2(void)
{
	int ret;
	//unsigned char reg, val;
	/* set torch current ramp time and flash timeout */
	//reg = AW36413_REG_TIMING_CONF;
	//val = AW36413_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT_90MS;
	//ret = aw36413_write_reg(aw36413_i2c_client, reg, val);
	
	/* set flash brightness level */
	aw36413_reg_enable |= AW36413_ENABLE_LED2_FLASH;
	
	ret = aw36413_write_reg(aw36413_i2c_client, AW36413_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));/*write reg5 bit 7 to 0*/
	ret = aw36413_write_reg(aw36413_i2c_client, AW36413_REG_FLASH_LEVEL_LED2, 0x6E);/*6E = 1.3A*/
	ret = aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, aw36413_reg_enable);
	return ret;
}
#endif

static int aw36413_set_level(int channel, int level)
{
	int ret;
	unsigned char reg, val;
	/* set torch current ramp time and flash timeout */
	reg = AW36413_REG_TIMING_CONF;
	val = AW36413_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT;
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);
	
	if (channel == AW36413_CHANNEL_CH1)
		aw36413_set_level_ch1(level);
	else if (channel == AW36413_CHANNEL_CH2)
		aw36413_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int aw36413_init(void)
{
	int ret;
	unsigned char reg, val;

	aw36413_pinctrl_set(AW36413_PINCTRL_PIN_HWEN, AW36413_PINCTRL_PINSTATE_HIGH);
	/*msleep(20);*/
	if (lock_touch == 0 && lock_touch_sub == 0) {
	/* clear enable register */
	reg = AW36413_REG_ENABLE;
	val = AW36413_DISABLE;
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	aw36413_reg_enable = val;
	}

	/* set torch current ramp time and flash timeout */
	reg = AW36413_REG_TIMING_CONF;
	val = AW36413_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT;
	ret = aw36413_write_reg(aw36413_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int aw36413_uninit(void)
{
	if (lock_touch == 0 && lock_touch_sub == 0) {
	aw36413_disable(AW36413_CHANNEL_CH1);
	aw36413_disable(AW36413_CHANNEL_CH2);
	aw36413_pinctrl_set(AW36413_PINCTRL_PIN_HWEN, AW36413_PINCTRL_PINSTATE_LOW);
	pr_debug("aw36413_uninit AW36413_PINCTRL_PINSTATE_LOW\n");
	}
	return 0;
}

#ifdef HOPE_ADD_IRQ_FLASH
static irqreturn_t vivo_subflash_ISR(int irq, void *dev_id)
{
	pr_debug("vivo_subflash_ISR \n");
	schedule_delayed_work(&ir_delayed_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}
static void ir_delayed_func(struct work_struct *work)
{
	
	/*int ret;*/
	unsigned char reg, flags_1_val, flags_2_val;
	u64 deltaTime = ktime_us_delta(ktime_get(), StartTime);
	pr_debug("hope  %d \n", led_count);
	pr_debug("hope  deltaTime %lluus\n", ktime_us_delta(ktime_get(), StartTime));
	
	if (deltaTime > 380000 && led_count < 13) {
		
		StartTime = ktime_get();
		led_count++;
		
		reg = AW36413_REG_FLAGS_1;
		flags_1_val = aw36413_read_reg(aw36413_i2c_client,reg);
		
		reg = AW36413_REG_FLAGS_2;
		flags_2_val = aw36413_read_reg(aw36413_i2c_client,reg);

		if((flags_1_val & 0x7E)||(flags_2_val & 0x1E))
			pr_err("flashlight err\n");
		else{
			pr_debug("sub flash mode start on \n");
			aw36413_set_red_flash_level_and_enabe_ch2();
			/*aw36413_set_level(1,15);*/
			/*aw36413_enable(1);*/
			/*aw36413_set_level(0,15); use fear flash for test use*/
			/*aw36413_enable(0);*/
		}			
		
	}
}
#endif

static int set_flashlight_state(int state)
{
	aw36413_pinctrl_set(AW36413_PINCTRL_PIN_HWEN, AW36413_PINCTRL_PINSTATE_HIGH);
	pr_info("set_flashlight_state check state:%d \n", state);
	switch (state) {
	case BBK_TORCH_LOW:
		aw36413_reg_enable |= AW36413_ENABLE_LED1_TORCH;

		aw36413_write_reg(aw36413_i2c_client, AW36413_REG_TORCH_LEVEL_LED1, 0x35);/*(Brightnees code x 1.4mA)+0.997ma Torch*/
		aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, aw36413_reg_enable);
		lock_touch = 1;
		break;
	case BBK_TORCH_OFF:
		if (aw36413_reg_enable & AW36413_MASK_ENABLE_LED2) {
			/* if LED 2 is enable, disable LED 1 */
			aw36413_reg_enable &= (~AW36413_ENABLE_LED1);
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, aw36413_reg_enable);
		} else {
			/* disable LED 1 LED 2 and clear mode */
			aw36413_reg_enable &= (~AW36413_ENABLE_LED1_FLASH);
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, aw36413_reg_enable);
			aw36413_pinctrl_set(AW36413_PINCTRL_PIN_HWEN, AW36413_PINCTRL_PINSTATE_LOW);
		}
		lock_touch = 0;
		break;
	case FRONT_TORCH_ON:
		pr_debug("FRONT_TORCH_ON start\n");
		aw36413_reg_enable |= AW36413_ENABLE_LED2_TORCH;

		aw36413_write_reg(aw36413_i2c_client, AW36413_REG_TORCH_LEVEL_LED1, 0x35);/*write reg5 bit 7 to 0*/
		aw36413_write_reg(aw36413_i2c_client, AW36413_REG_TORCH_LEVEL_LED2, 0x6A);/*default 2E == 65mA  0x6A = 150mA(Brightnees code x 1.4mA)+0.997ma Torch*/
		aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, aw36413_reg_enable);
		lock_touch_sub = 1;  
		break;
	case FRONT_TORCH_OFF:
		if (aw36413_reg_enable & AW36413_MASK_ENABLE_LED1) {
			/* if LED 1 is enable, disable LED 2 */
			aw36413_reg_enable &= (~AW36413_ENABLE_LED2);
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, aw36413_reg_enable);
		} else {
			/* disable LED 1 LED 2 and clear mode */
			aw36413_reg_enable &= (~AW36413_ENABLE_LED2_FLASH);
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, aw36413_reg_enable);
			aw36413_pinctrl_set(AW36413_PINCTRL_PIN_HWEN, AW36413_PINCTRL_PINSTATE_LOW);
		}
		lock_touch_sub = 0;  
		break;
	case BBK_FLASH_AT_TEST:
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_TIMING_CONF, 0x1f);/*vivo liuguangwei change flash time out from 150ms to 400ms*/
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_FLASH_LEVEL_LED1, 0x54);
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, AW36413_ENABLE_LED1_FLASH);
		break;
	case BBK_FLASH_AT_OFF:
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_ENABLE, (~AW36413_ENABLE_LED1_FLASH));
		break;
	default:
		pr_info("set_flashlight_state No such command and arg\n");
		return -ENOTTY;
	}
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw36413_timer_ch1;
static struct hrtimer aw36413_timer_ch2;
static unsigned int aw36413_timeout_ms[AW36413_CHANNEL_NUM];

static void aw36413_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	aw36413_disable_ch1();
}

static void aw36413_work_disable_ch2(struct work_struct *data)
{
	pr_debug("lt work queue callback\n");
	aw36413_disable_ch2();
}

static enum hrtimer_restart aw36413_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw36413_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart aw36413_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&aw36413_work_ch2);
	return HRTIMER_NORESTART;
}

static int aw36413_timer_start(int channel, ktime_t ktime)
{
	if (channel == AW36413_CHANNEL_CH1)
		hrtimer_start(&aw36413_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == AW36413_CHANNEL_CH2)
		hrtimer_start(&aw36413_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int aw36413_timer_cancel(int channel)
{
	if (channel == AW36413_CHANNEL_CH1)
		hrtimer_cancel(&aw36413_timer_ch1);
	else if (channel == AW36413_CHANNEL_CH2)
		hrtimer_cancel(&aw36413_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw36413_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	int led_state;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= AW36413_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36413_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36413_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw36413_timeout_ms[channel]) {
				ktime = ktime_set(aw36413_timeout_ms[channel] / 1000,
						(aw36413_timeout_ms[channel] % 1000) * 1000000);
				aw36413_timer_start(channel, ktime);
			}
			aw36413_enable(channel);
		} else {
		if (lock_touch == 0 && lock_touch_sub == 0) {
			aw36413_disable(channel);
			aw36413_timer_cancel(channel);
			}
		}
		break;

    case FLASH_IOCTL_SET_LED_STATE:
		pr_debug("FLASH_IOCTL_SET_LED_STATE(channel %d): arg: %d\n",
				channel, (int)fl_arg->arg);
		led_state = (int)fl_arg->arg;
		set_flashlight_state(led_state);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = AW36413_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = AW36413_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = aw36413_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = aw36413_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = AW36413_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw36413_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw36413_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw36413_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&aw36413_mutex);
	if (set) {
		if (!use_count)
			ret = aw36413_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = aw36413_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&aw36413_mutex);

	return ret;
}

static ssize_t aw36413_strobe_store(struct flashlight_arg arg)
{
	int i;

	if (arg.channel == AW36413_CHANNEL_CH2 && arg.level == 27){
		if(arg.dur == 200)
			set_flashlight_state(10);
		if(arg.dur == 300)
			set_flashlight_state(11);
	}else{
		aw36413_set_driver(1);
		if(arg.channel == AW36413_CHANNEL_CH2 && arg.level == 26){
			pr_debug("====hope arg.channel = %d, arg.level = %d\n", arg.channel, arg.level);
			aw36413_level_ch2 = arg.level;
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_TORCH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/	
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_FLASH_LEVEL_LED2, 0x6E);/*0x6E = 1.3A,0x65 = 1.2A*/
			for (i = 0; i < 10; i++){
				aw36413_enable(arg.channel);
				msleep(arg.dur);
				aw36413_disable(arg.channel);
				msleep(330);
				pr_debug("====hope arg.dur = %d, disable = 330\n", arg.dur);
			}
		}else{
			aw36413_set_level(arg.channel, arg.level);
		}
#if 0
		if(arg.channel == AW36413_CHANNEL_CH1 && arg.level == 26){
			pr_debug("====hope arg.channel = %d, arg.level = %d\n", arg.channel, arg.level);
			aw36413_level_ch1 = arg.level;
			/*aw36413_write_reg(aw36413_i2c_client, AW36413_REG_TORCH_LEVEL_LED1, (0x7f & 0xbf));default value is 0xbf,bit7 should set to 0*/
			/*aw36413_write_reg(aw36413_i2c_client, AW36413_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));default value is 0xbf,bit7 should set to 0*/	
			aw36413_write_reg(aw36413_i2c_client, AW36413_REG_FLASH_LEVEL_LED1, 0x6E);
		}else{
			aw36413_set_level(arg.channel, arg.level);
		}
#endif
		aw36413_timeout_ms[arg.channel] = 0;
		aw36413_enable(arg.channel);
		msleep(arg.dur);
		aw36413_disable(arg.channel);
		aw36413_set_driver(0);
		}
	return 0;
}

static struct flashlight_operations aw36413_ops = {
	aw36413_open,
	aw36413_release,
	aw36413_ioctl,
	aw36413_strobe_store,
	aw36413_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw36413_chip_init(struct aw36413_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw36413_init();
	 */

	return 0;
}

static int aw36413_parse_dt(struct device *dev,
		struct aw36413_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, AW36413_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int aw36413_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw36413_platform_data *pdata = dev_get_platdata(&client->dev);
	struct aw36413_chip_data *chip;
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct aw36413_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = aw36413_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	aw36413_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&aw36413_work_ch1, aw36413_work_disable_ch1);
	INIT_WORK(&aw36413_work_ch2, aw36413_work_disable_ch2);

	/* init timer */
	hrtimer_init(&aw36413_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36413_timer_ch1.function = aw36413_timer_func_ch1;
	hrtimer_init(&aw36413_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36413_timer_ch2.function = aw36413_timer_func_ch2;
	aw36413_timeout_ms[AW36413_CHANNEL_CH1] = 100;
	aw36413_timeout_ms[AW36413_CHANNEL_CH2] = 100;

	/* init chip hw */
	aw36413_chip_init(chip);

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	pr_debug("aw36413 AW36413_DEVICE_ID = %d\n", aw36413_read_reg(aw36413_i2c_client, AW36413_DEVICE_ID));
	if(aw36413_read_reg(aw36413_i2c_client, AW36413_DEVICE_ID) == 0x12){
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &aw36413_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(AW36413_NAME, &aw36413_ops)) {
			err = -EFAULT;
			goto err_free;
		}
		aw36413_pinctrl_init_1();
	}
	}else{
		err = -EFAULT;
		goto err_free;
	}
	/*set_flashlight_state(11); hope for test use */
	pr_debug("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	devm_pinctrl_put(aw36413_pinctrl);
	if(aw36413_pinctrl == NULL)
		pr_err("aw36413_pinctrl == NULL");
	else
		pr_err("aw36413_pinctrl have value");
	kfree(chip);
err_out:
	return err;
}

static int aw36413_i2c_remove(struct i2c_client *client)
{
	struct aw36413_platform_data *pdata = dev_get_platdata(&client->dev);
	struct aw36413_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(AW36413_NAME);

	/* flush work queue */
	flush_work(&aw36413_work_ch1);
	flush_work(&aw36413_work_ch2);

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw36413_i2c_id[] = {
	{AW36413_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id aw36413_i2c_of_match[] = {
	{.compatible = AW36413_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver aw36413_i2c_driver = {
	.driver = {
		.name = AW36413_NAME,
#ifdef CONFIG_OF
		.of_match_table = aw36413_i2c_of_match,
#endif
	},
	.probe = aw36413_i2c_probe,
	.remove = aw36413_i2c_remove,
	.id_table = aw36413_i2c_id,
};

#ifdef CONFIG_OF
static const struct of_device_id aw36413_of_match[] = {
	{.compatible = AW36413_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw36413_of_match);
#else
static struct platform_device aw36413_platform_device[] = {
	{
		.name = AW36413_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw36413_platform_device);
#endif

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw36413_probe(struct platform_device *dev)
{
#ifdef HOPE_ADD_IRQ_FLASH
	int ret=0;
	struct device_node *node = NULL;
	u32 ints1[4] = { 0 };
#endif
	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (aw36413_pinctrl_init(dev)) {
		pr_debug("Failed to init pinctrl.\n");
		return -1;
	}
	
	if (i2c_add_driver(&aw36413_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

#ifdef HOPE_ADD_IRQ_FLASH
	pinctrl_select_state(aw36413_pinctrl, aw36413_flash_irq);
	node = of_find_matching_node(node, aw36413_of_match);
	if (node) {
		g_gpio_pin = of_get_named_gpio(node, "deb-gpios", 0);
		ret = of_property_read_u32(node, "debounce", &g_gpio_headset_deb);
		if (ret < 0) {
			pr_debug("debounce time not found\n");
			return ret;
		}
		/*gpio_set_debounce(g_gpio_pin, g_gpio_headset_deb);*/
		
		g_flash_irq_num = irq_of_parse_and_map(node, 0);
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		g_accdet_eint_type = ints1[1];
		pr_debug("[flash] gpiopin:%d debounce:%d accdet_irq:%d accdet_eint_type:%d\n",
				g_gpio_pin, g_gpio_headset_deb, g_flash_irq_num, g_accdet_eint_type);
		ret = request_irq(g_flash_irq_num, vivo_subflash_ISR, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "flash_irq", NULL);
		if (ret != 0) {
			pr_debug("[flash]EINT IRQ LINE NOT AVAILABLE\n");
			goto ir_irq_err;
		} else {
			pr_debug("[flash]accdet set EINT finished, accdet_irq=%d, headsetdebounce=%d\n",
				     g_flash_irq_num, g_gpio_headset_deb);
		}
		
		INIT_DELAYED_WORK(&ir_delayed_work, ir_delayed_func);
		
		disable_irq_nosync(g_flash_irq_num);
				
		pr_debug("hope \n");
	} else {
		pr_debug("[flash]%s can't find compatible node\n", __func__);
	}
	
	return 0;
	
ir_irq_err:
	free_irq(g_flash_irq_num, NULL);
	return -1;
#endif
	pr_debug("Probe done.\n");

	return 0;
}

static int aw36413_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&aw36413_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}


static struct platform_driver aw36413_platform_driver = {
	.probe = aw36413_probe,
	.remove = aw36413_remove,
	.driver = {
		.name = AW36413_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw36413_of_match,
#endif
	},
};

static int __init flashlight_aw36413_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw36413_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw36413_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_aw36413_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&aw36413_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_aw36413_init);
module_exit(flashlight_aw36413_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ziyu Jiang <jiangziyu@meizu.com>");
MODULE_DESCRIPTION("MTK Flashlight AW36413 Driver");

