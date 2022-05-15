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

#include "imgsensor_sensor.h"

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/string.h>

#include "kd_camera_typedef.h"
#include "kd_camera_feature.h"


#include "imgsensor_hw.h"
#if defined(CONFIG_MTK_CAM_PD1803)
#include "linux/gpio.h"
extern int DVDD_GPIO98;
#endif 

#if defined(CONFIG_MTK_CAM_PD1732)
#include "linux/gpio.h"
extern int GPIO171;
extern int GPIO173;
extern int GPIO174;
extern int gpio171_value;
extern int gpio173_value;
extern int gpio174_value;
#endif 

char *imgsensor_sensor_idx_name[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	IMGSENSOR_SENSOR_IDX_NAME_MAIN,
	IMGSENSOR_SENSOR_IDX_NAME_SUB,
	IMGSENSOR_SENSOR_IDX_NAME_MAIN2,
};

enum IMGSENSOR_RETURN imgsensor_hw_release_all(struct IMGSENSOR_HW *phw)
{
	int i;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (phw->pdev[i]->release != NULL)
			(phw->pdev[i]->release)(phw->pdev[i]->pinstance);
	}
	return IMGSENSOR_RETURN_SUCCESS;
}
enum IMGSENSOR_RETURN imgsensor_hw_init(struct IMGSENSOR_HW *phw)
{
	struct IMGSENSOR_HW_SENSOR_POWER      *psensor_pwr;
	struct IMGSENSOR_HW_CFG               *pcust_pwr_cfg;
	struct IMGSENSOR_HW_CUSTOM_POWER_INFO *ppwr_info;
	int i, j;
	
	
	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (hw_open[i] != NULL)
			(hw_open[i])(&phw->pdev[i]);

		if (phw->pdev[i]->init != NULL)
			(phw->pdev[i]->init)(phw->pdev[i]->pinstance);
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		psensor_pwr = &phw->sensor_pwr[i];

#if defined(CONFIG_MTK_CAM_PD1732)
		//gpio_direction_input(GPIO171);
		gpio171_value = gpio_get_value(GPIO171);
		gpio173_value = gpio_get_value(GPIO173);
		gpio174_value = gpio_get_value(GPIO174);
		pr_err("gpio171_value =%d, gpio173_value =%d, gpio174_value =%d\n", gpio171_value, gpio173_value, gpio174_value);
		if (gpio173_value == 0){  /*PD1732DF_EX/PD1732DA*/
			if (gpio174_value == 1)
				pcust_pwr_cfg = imgsensor_custom_config2;
			else
				pcust_pwr_cfg = imgsensor_custom_config3;
		}
		else{/*PD1732F_EX/PD1732A*/
			if(gpio171_value == 0)
				pcust_pwr_cfg = imgsensor_custom_config1; //use gpio control avdd
			else
#endif
				pcust_pwr_cfg = imgsensor_custom_config;

#if defined(CONFIG_MTK_CAM_PD1732)
		}
#endif
		
		while (pcust_pwr_cfg->sensor_idx != i)
			pcust_pwr_cfg++;

		if (pcust_pwr_cfg->sensor_idx == IMGSENSOR_SENSOR_IDX_NONE)
			continue;

		ppwr_info = pcust_pwr_cfg->pwr_info;
		while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE) {
			for (j = 0; j < IMGSENSOR_HW_ID_MAX_NUM; j++)
				if (ppwr_info->id == phw->pdev[j]->id)
					break;

			psensor_pwr->id[ppwr_info->pin] = j;
			ppwr_info++;
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN imgsensor_hw_power_sequence(
	struct IMGSENSOR_HW             *phw,
	enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
	enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
	struct IMGSENSOR_HW_POWER_SEQ   *ppower_sequence,
	char *pcurr_idx)
{
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr =
	    &phw->sensor_pwr[sensor_idx];

	struct IMGSENSOR_HW_POWER_SEQ    *ppwr_seq = ppower_sequence;
	struct IMGSENSOR_HW_POWER_INFO   *ppwr_info;
	struct IMGSENSOR_HW_DEVICE       *pdev;
	int                               pin_cnt = 0;

	while (ppwr_seq->idx != NULL &&
		ppwr_seq < ppower_sequence + IMGSENSOR_HW_SENSOR_MAX_NUM &&
		strcmp(ppwr_seq->idx, pcurr_idx)) {
		ppwr_seq++;
	}

	if (ppwr_seq->idx == NULL)
		return IMGSENSOR_RETURN_ERROR;

	ppwr_info = ppwr_seq->pwr_info;

	while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE &&
		ppwr_info < ppwr_seq->pwr_info + IMGSENSOR_HW_POWER_INFO_MAX) {

		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON &&
		   ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
			pdev = phw->pdev[psensor_pwr->id[ppwr_info->pin]];
		/*pr_debug(
		 *  "sensor_idx = %d, pin=%d, pin_state_on=%d, hw_id =%d\n",
		 *  sensor_idx,
		 *  ppwr_info->pin,
		 *  ppwr_info->pin_state_on,
		 * psensor_pwr->id[ppwr_info->pin]);
		 */
#if defined(CONFIG_MTK_CAM_PD1803)
			/* power on gpio98 when power on DVDD */
			if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN && 
				ppwr_info->pin == IMGSENSOR_HW_PIN_DVDD)
				gpio_set_value(DVDD_GPIO98, 1);
#endif
			if (pdev->set != NULL)
				pdev->set(
				    pdev->pinstance,
				    sensor_idx,
				    ppwr_info->pin,
				    ppwr_info->pin_state_on);

			mdelay(ppwr_info->pin_on_delay);
		}

		ppwr_info++;
		pin_cnt++;
	}

	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF) {
		while (pin_cnt) {
			ppwr_info--;
			pin_cnt--;

			if (ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
#if defined(CONFIG_MTK_CAM_PD1803)				
				/* power off gpio98 when power off DVDD */
				if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN && 
						ppwr_info->pin == IMGSENSOR_HW_PIN_DVDD)
					gpio_set_value(DVDD_GPIO98, 0);
#endif
				pdev =
				    phw->pdev[psensor_pwr->id[ppwr_info->pin]];
				mdelay(ppwr_info->pin_on_delay);

				if (pdev->set != NULL)
					pdev->set(
					    pdev->pinstance,
					    sensor_idx,
					    ppwr_info->pin,
					    ppwr_info->pin_state_off);
			}
		}
	}

	/* wait for power stable */
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
		mdelay(5);
	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_power(
	struct IMGSENSOR_HW     *phw,
	struct IMGSENSOR_SENSOR *psensor,
	char *curr_sensor_name,
	enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;

#if defined(CONFIG_IMGSENSOR_MAIN)  || \
		defined(CONFIG_IMGSENSOR_SUB)   || \
		defined(CONFIG_IMGSENSOR_MAIN2) || \
		defined(CONFIG_IMGSENSOR_SUB2)
	char *pcustomize_sensor = NULL;

	switch (sensor_idx) {
	case IMGSENSOR_SENSOR_IDX_MAIN:
		pcustomize_sensor = IMGSENSOR_STRINGIZE(CONFIG_IMGSENSOR_MAIN);
		break;
	case IMGSENSOR_SENSOR_IDX_SUB:
		pcustomize_sensor = IMGSENSOR_STRINGIZE(CONFIG_IMGSENSOR_SUB);
		break;
	case IMGSENSOR_SENSOR_IDX_MAIN2:
		pcustomize_sensor = IMGSENSOR_STRINGIZE(CONFIG_IMGSENSOR_MAIN2);
		break;
	case IMGSENSOR_SENSOR_IDX_SUB2:
		pcustomize_sensor = IMGSENSOR_STRINGIZE(CONFIG_IMGSENSOR_SUB2);
		break;
	default:
		break;
	}

	if (strlen(pcustomize_sensor) > 2 &&
		!strstr(pcustomize_sensor, curr_sensor_name))
		return IMGSENSOR_RETURN_ERROR;
#endif

	pr_info(
	    "sensor_idx %d, power %d curr_sensor_name %s\n",
	    sensor_idx,
	    pwr_status,
	    curr_sensor_name);

	imgsensor_hw_power_sequence(
	    phw,
	    sensor_idx,
	    pwr_status,
	    platform_power_sequence,
	    imgsensor_sensor_idx_name[sensor_idx]);

	imgsensor_hw_power_sequence(
	    phw,
	    sensor_idx,
	    pwr_status,
	    sensor_power_sequence,
	    curr_sensor_name);

	return IMGSENSOR_RETURN_SUCCESS;
}

