/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "regulator.h"
#include "upmu_common.h"


static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1050,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2900,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcamd"},
	{"vcamio"},
	{"vcamaf"},
	{"vcama_sub"},
	{"vcamd_sub"},
	{"vcamio_sub"},
	{"vcama_main2"},
	{"vcamd_main2"},
	{"vcamio_main2"},
	{"vcama_sub2"},
	{"vcamd_sub2"},
	{"vcamio_sub2"}
};

static struct REGULATOR reg_instance;
static int regulator_status[REGULATOR_TYPE_MAX_NUM] = {0};
static void check_for_regulator_get(struct REGULATOR *preg, struct device *pdevice, int index);
static void check_for_regulator_put(struct REGULATOR *preg, int index);
static struct device_node *of_node_record = NULL;



static void imgsensor_oc_handler1(void)
{
	pr_debug("[regulator]%s enter vcama oc %d\n",
		__func__,
		gimgsensor.status.oc);
	gimgsensor.status.oc = 1;
}
static void imgsensor_oc_handler2(void)
{
	pr_debug("[regulator]%s enter vcamd oc %d\n",
		__func__,
		gimgsensor.status.oc);
	gimgsensor.status.oc = 1;

}
static void imgsensor_oc_handler3(void)
{
	pr_debug("[regulator]%s enter vcamio oc %d\n",
		__func__,
		gimgsensor.status.oc);
	gimgsensor.status.oc = 1;
}


enum IMGSENSOR_RETURN imgsensor_oc_interrupt(bool enable)
{
	pr_debug("[regulator] %s %d\n", __func__, enable);

	mdelay(5);

	gimgsensor.status.oc = 0;

	if (enable) {
		/* enable interrupt after power on */
		/* At least delay 3ms after power for recommendation */
		pmic_enable_interrupt(INT_VCAMA_OC, 1, "camera");
		pmic_enable_interrupt(INT_VCAMD_OC, 1, "camera");
		pmic_enable_interrupt(INT_VCAMIO_OC, 1, "camera");
	} else {
		/* Disable interrupt before power off */
		pmic_enable_interrupt(INT_VCAMA_OC, 0, "camera");
		pmic_enable_interrupt(INT_VCAMD_OC, 0, "camera");
		pmic_enable_interrupt(INT_VCAMIO_OC, 0, "camera");
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_oc_init(void)
{
	/* Register your interrupt handler of OC interrupt at first */
	pmic_register_interrupt_callback(INT_VCAMA_OC, imgsensor_oc_handler1);
	pmic_register_interrupt_callback(INT_VCAMD_OC, imgsensor_oc_handler2);
	pmic_register_interrupt_callback(INT_VCAMIO_OC, imgsensor_oc_handler3);

	gimgsensor.status.oc  = 0;
	gimgsensor.imgsensor_oc_irq_enable = imgsensor_oc_interrupt;

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_init(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	struct REGULATOR_CTRL    *pregulator_ctrl = regulator_control;
	struct device            *pdevice;
	struct device_node       *pof_node;
	int i;

	pdevice  = gimgsensor_device;
	pof_node = pdevice->of_node;
	pdevice->of_node =
		of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");
	
	of_node_record = pdevice->of_node;
	
	if (pdevice->of_node == NULL) {
		pr_err("regulator get cust camera node failed!\n");
		pdevice->of_node = pof_node;
		return IMGSENSOR_RETURN_ERROR;
	}

	for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++, pregulator_ctrl++) {
		preg->pregulator[i] =
		    regulator_get(pdevice, pregulator_ctrl->pregulator_type);

		if (preg->pregulator[i] == NULL)
			pr_err("regulator[%d]  %s fail!\n",
				i, pregulator_ctrl->pregulator_type);

		atomic_set(&preg->enable_cnt[i], 0);
		
		regulator_status[i] = 1;
	}


	pdevice->of_node = pof_node;
	imgsensor_oc_init();
	return IMGSENSOR_RETURN_SUCCESS;
}
static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int i;

	for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
		if (preg->pregulator[i] != NULL) {
			for (; atomic_read(&preg->enable_cnt[i]) > 0; ) {
				regulator_disable(preg->pregulator[i]);
				atomic_dec(&preg->enable_cnt[i]);
			}
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	enum   REGULATOR_TYPE reg_type_offset;
	atomic_t	*enable_cnt;


	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
		pin < IMGSENSOR_HW_PIN_AVDD    ||
		pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
		pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH)
		return IMGSENSOR_RETURN_ERROR;

	reg_type_offset = (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN)
		? REGULATOR_TYPE_MAIN_VCAMA
		: (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB)
		? REGULATOR_TYPE_SUB_VCAMA
		: (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2)
		? REGULATOR_TYPE_MAIN2_VCAMA
		: REGULATOR_TYPE_SUB2_VCAMA;

	
	pr_debug("regulator_dbg regulator_set sensor_idx %d, regulator %s, status %d\n", sensor_idx, regulator_control[(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD)].pregulator_type,regulator_status[(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD)]);
	check_for_regulator_get(preg, gimgsensor_device, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
	
	pregulator =
		preg->pregulator[reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	enable_cnt =
	    preg->enable_cnt + (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD);

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
			
			if ((pin == IMGSENSOR_HW_PIN_DVDD)&&(pin_state == IMGSENSOR_HW_PIN_STATE_LEVEL_1050)) {
				pin_state = IMGSENSOR_HW_PIN_STATE_LEVEL_1000;
				pmic_config_interface(0x1c94, 5 ,0xf, 0);	
			}
			
			if (regulator_set_voltage(
				pregulator,
				regulator_voltage[
				    pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				 pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				pr_err(
				    "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				    pin,
				    regulator_voltage[
				   pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(pregulator)) {
				pr_err(
				    "[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				    pin,
				    regulator_voltage[
				   pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);

				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator)) {
				/*pr_debug("[regulator]%d is enabled\n", pin);*/

				if (regulator_disable(pregulator)) {
					pr_err(
					    "[regulator]fail to regulator_disable, powertype: %d\n",
					    pin);
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			
			check_for_regulator_put(preg, (reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
			atomic_dec(enable_cnt);
		}
	} else {
		pr_err("regulator == NULL %d %d %d\n",
		    reg_type_offset,
		    pin,
		    IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static void check_for_regulator_get(struct REGULATOR *preg, struct device *pdevice, int index)
{
 struct device_node *pof_node;

	if(regulator_status[index]==0) {
		pof_node = pdevice->of_node;
		pdevice->of_node = of_node_record;
		 
		preg->pregulator[index] = regulator_get(pdevice, regulator_control[index].pregulator_type);
		pdevice->of_node = pof_node;
		regulator_status[index] = 1;
		pr_debug("regulator_dbg regulator_get %s, of_node:%p\n", regulator_control[index].pregulator_type, of_node_record);
	}
}

static void check_for_regulator_put(struct REGULATOR *preg, int index) 
{
	if(regulator_status[index]==1)
		{
			regulator_put(preg->pregulator[index]);
			regulator_status[index]=0;
			pr_debug("regulator_dbg regulator_put %s\n", regulator_control[index].pregulator_type);
		}
}




static struct IMGSENSOR_HW_DEVICE device = {
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.id        = IMGSENSOR_HW_ID_REGULATOR
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

