/* drivers/input/touchscreen/sec_ts_fn.c
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

 
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/unaligned.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include "sec_ts.h"
#include <linux/vivo_ts_function.h>

int bbk_slsi_get_rawordiff_data(int which, int *data)
{
	struct sec_ts_data *ts = g_ts_data;
	unsigned int readbytes = 0xFF;
	int *pRead = NULL;
	int ret = 0;
	int i = 0;
	
	VTI("enter");

	VTI("ts->rx_count=%d, ts->tx_count=%d", ts->rx_count, ts->tx_count);
	/* set data length, allocation buffer memory */
	readbytes = ts->rx_count * ts->tx_count;

	pRead = data;

	switch (which) {
	case TYPE_RAW_DATA:
		ret = run_rawdata_read_all(ts);
		if (ret < 0)
			VTE("run_rawdata_read_all fail");
		break;
	case TYPE_SIGNAL_DATA:
		ret = run_delta_read_all(ts);
		if (ret < 0)
			VTE("run_signaldata_read_all fail");
		break;
	case TYPE_AMBIENT_BASELINE:
		ret = run_baseline_read_all(ts);
		if (ret < 0)
			VTE("run_ambient_data_read_all fail");
		break;
	}

	if (ret == 0) {
		for (i = 0; i < ts->rx_count*ts->tx_count; i++) {
			/*for (j = 0; j < ts->rx_count; j++)*/
				/*pRead[(i * ts->tx_count) + j] = (int)ts->pFrame[(i* ts->tx_count) +j];*/
				pRead[i] = (int)ts->pFrame[i];
			
		}
	}

	/*data = pRead;	*/
	
	/*kfree(pRead);	*/
	return ret;
}


int bbk_slsi_fw_update(const struct firmware *firmware)
{
	struct sec_ts_data *ts = g_ts_data;
	int error = 0;
	
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("[ERROR] Touch is stopped");		
		return -ERROR;
	}
	
	if (ts->client->irq)
		disable_irq(ts->client->irq);

	if (sec_ts_firmware_update(ts, firmware->data, firmware->size, 0) < 0) {
		VTI("update fw fail");
		error = -1;
	} else {
		VTI("update fw success");
		error = 0;
	}

	sec_ts_save_version_of_ic(ts);

	if (ts->client->irq)
		enable_irq(ts->client->irq);

	return error;	
}

int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len);
static int last_state = TOUCHSCREEN_NORMAL;
extern int bbk_xxsw_reset(struct sec_ts_data *ts);
int bbk_xxsw_reset(struct sec_ts_data *ts)
{
		int ret;

	VTI("sw reset");
	return 0;
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
	if (ret < 0) {
		VTE("%s: write fail, sw_reset\n", __func__);
	}

	sec_ts_delay(300);

	return ret;
}

int sec_sence_ctl(struct sec_ts_data *ts, int state)
{
	unsigned char cmd = SEC_TS_CMD_SELECT_MODE;
	unsigned char param = 0;
	int ret = 0;
	
	if (state) {
		param = SEC_TS_PARAM_SENCE_ON;
		ret = ts->sec_ts_i2c_write(ts, cmd, &param, 1);
	} else {
		param = SEC_TS_PARAM_SENCE_OFF;
		ret = ts->sec_ts_i2c_write(ts, cmd, &param, 1);
	}

	if (ret < 0) {
		VTI("sec_sence_ctl send cmd fail");
	}
	return ret;
}

extern int mdss_dsi_panel_reset_and_powerctl(int enable);


/**
 * zhj add for unbalanced irq when mode change  
 * bbk_slsi_irq_enable - Enable/Disable a irq
 * @ts: pointer to touch core data
 * enable: enable or disable irq
 * return: 0 ok, <0 failed
 */
int bbk_slsi_irq_enable(struct sec_ts_data *ts, bool enable)
{
	if (enable) {
		if (!atomic_cmpxchg(&ts->irq_enabled, 0, 1)) {
			enable_irq(ts->client->irq);
			VTI("===mode_change=== Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&ts->irq_enabled, 1, 0)) {
			disable_irq(ts->client->irq);
			VTI("+++mode_change+++ Irq disabled");
		}
	}

	return 0;
}

int bbk_slsi_mode_change(int which)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;

	switch (which) {
	case TOUCHSCREEN_NORMAL:
		VTI("change to normal mode");
		if (last_state == TOUCHSCREEN_SLEEP){
			VTI("TOUCHSCREEN_SLEEP POWER");
			ret = sec_sence_ctl(ts, 1);
			if (ret < 0)
				VTI("sec_sence_ctl fail");
		}
		sec_ts_sw_reset(ts);
		bbk_slsi_irq_enable(ts, true);

		VTI("new change to normal mode end");
	break;
	case TOUCHSCREEN_GESTURE:
		VTI("change to gesture mode");
		if (last_state == TOUCHSCREEN_NORMAL) {
			
		}
		if (last_state == TOUCHSCREEN_SLEEP) {
			ret = sec_sence_ctl(ts, 1);
			if (ret < 0) 
				VTI("sec_sence_ctl fail");
		}
		ts->lowpower_mode = SEC_TS_MODE_LSI_ALL; 
		bbk_slsi_irq_enable(ts, true);

		VTI("change to gesture mode end");		
	break;
	case TOUCHSCREEN_SLEEP:
		VTI("change to sleep mode");
		if (last_state != TOUCHSCREEN_SLEEP) {
			ret = sec_sence_ctl(ts, 0);
			if (ret < 0) 
				VTI("sec_sence_ctl fail");
		}
		if (last_state == TOUCHSCREEN_NORMAL  && allGestureSwitchIntoBitmap()==0){
			VTI("POWER OFF SLEEP begin");
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			mdss_dsi_panel_reset_and_powerctl(0);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			VTI("POWER OFF SLEEP end");
		}
		bbk_slsi_irq_enable(ts, false);

		VTI("change to sleep mode end");
	break;

	default:
	break;
	}

	last_state = which;
	return ret;
}

int bbk_slsi_get_fw_version(int which)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;

	/*read from chip*/
	ret = sec_ts_save_version_of_ic(ts);
	if (ret < 0) {
		VTE("fail to save ic version");
		return -EIO;
	}

	switch (which) {
	case FW_VERSION:
		ret = (ts->plat_data->img_version_of_ic[2]<<8)|ts->plat_data->img_version_of_ic[3];
		break;
	case CONFIG_VERSION:
		ret = (ts->plat_data->para_version_of_ic[2]<<8)|ts->plat_data->para_version_of_ic[3];
		if (ts->exception_recal > 0) {
			ret |= 0x8000;
		}
		break;
	}
	return ret;
}


int bbk_slsi_set_charger_bit(int state)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret;

	if (state) {
		ret = sec_ts_charger_config(ts, 0x01);
	} else {
		ret = sec_ts_charger_config(ts, 0);
	}

	return ret;
}


int bbk_slsi_read_charger_bit(void)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	char data = 0;

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_SET_TOUCHFUNCTION, &data, 1);
	if (ret < 0) {
		VTE("%s: failed to read charger stauts(%d)\n", __func__, ret);
		return ret;
	}	
	
	return (data & SEC_TS_BIT_SETFUNC_CHARGER);
}


int bbk_slsi_readUdd(unsigned char *udd)
{
	struct sec_ts_data *ts = g_ts_data;
	unsigned char buf[32] = {0};
	int ret = 0;
	ret = get_user_nvm_data(ts, buf, 15);
	if (ret < 0)
		VTE("%s: read user data failed\n", __func__);
	else
		memcpy(udd, buf, 15);
	
	return ret;
}
int bbk_slsi_writeUdd(unsigned char *udd)
{
	struct sec_ts_data *ts = g_ts_data;

	unsigned char buf[32] = {0};

	memcpy(buf, udd, 15);
	
	return set_user_nvm_data(ts, buf, 15);
}


int bbk_slsi_idleEnableOrDisable(int state)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = -1;

	if (state) {
		ret = sec_ts_release_tmode(ts);
		if (ret < 0)
			VTE("%s: failed to enable idle\n", __func__);
	} else {
		ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
		if (ret < 0)
			VTE("%s: failed to disable idle\n", __func__);
	}
	return ret;
}

int bbk_slsi_get_module_id(void)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;
	/*
	add get modul id
	*/	
	u8 tBuff[1];
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_MODULEID, tBuff, 1);
			
	return ret;
}


int bbk_slsi_sensor_test(char *buf, void *pdata, int tmp)
{
	struct sec_ts_data *ts = g_ts_data;	
	u8 rBuff[SEC_TS_EVENT_BUFF_SIZE];
	int len = 0;
	int i = 0;	
	char selftest_str[15][7] = {
		"MIN", "MAX", "SLPRX", "SLPTX", "OPNRX",
		"OPNTX", "SRTRG", "SRTTG", "SRTRR", "SRTTT",
		"SRTTR", "pass", "fail", "Pass", "Failed"
	};

	VTI("enter");
	
	/*
	add self test
	*/
	run_trx_short_test(ts, rBuff);
	VTI("rBuff:%d %d %d %d %d %d %d %d", rBuff[0], rBuff[1], rBuff[2], rBuff[3], rBuff[4], rBuff[5], rBuff[6], rBuff[7]);
	
	/*All test result*/
	if ((rBuff[2] == 0) && ((rBuff[3]&0x7) == 0))		
		len = snprintf(buf, sizeof(selftest_str), "%s\n", selftest_str[13]);	/*all test success*/
	else
		len = snprintf(buf, sizeof(selftest_str), "%s\n", selftest_str[14]);	/*one or all test failed*/
		
	for (i = 0; i < 8; i++) {	
		len += snprintf(&buf[len], sizeof(selftest_str), "%s test:", selftest_str[i]);	
		if ((rBuff[2] >> i) & 0x1)	
			len += snprintf(&buf[len], sizeof(selftest_str), "%s\n", selftest_str[12]);
		else
			len += snprintf(&buf[len], sizeof(selftest_str), "%s\n", selftest_str[11]);
	}

	for (i = 0; i < 3; i++) {	
		len += snprintf(&buf[len], sizeof(selftest_str), "%s test:", selftest_str[i+8]);	
		if ((rBuff[3] >> i) & 0x1)	
			len += snprintf(&buf[len], sizeof(selftest_str), "%s\n", selftest_str[12]);
		else
			len += snprintf(&buf[len], sizeof(selftest_str), "%s\n", selftest_str[11]);
	}	
	
	return len;
}

int bbk_slsi_setEdgeRestainSwitch(int on)
{
	int ret = 0;
	/*
	add edge restian command
	*/
	ret = dead_zone_enable(on);
	return ret;
}

int bbk_slsi_gesture_point_get(u16 *data)
{
	uint8_t i;
	struct sec_ts_data *ts = g_ts_data;

	VTI("enter");
	for (i = 0; i < 6; i++) {
		data[i * 2]  = ts->sec_gsTrace.u16aX[i];
		data[i * 2 + 1] = ts->sec_gsTrace.u16aY[i];
	}

	VTI("end");

	return 10;/*10means Points */
}

int bbk_slsi_get_fw_debug_info(unsigned char *buf)
{
	int ret = -1;
	int len = 0;
	unsigned char buff[4] = {0};
	struct sec_ts_data *ts = g_ts_data;

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, buff, 4);
	if (ret < 0) {
		VTI("fail to read SEC_TS_READ_TS_STATUS");
		
		return snprintf(buf, 128, "fail to read chip status\n");
	}

	len += snprintf(buf, 128, "Water Reject Mode: %d\n", (buff[2] >> 0) & 0x01);
	len += snprintf(&buf[len], 128, "Active Mode: %d\n", ((buff[3] & TOUCH_MODE_STATE_TOUCH) ? 0x01 : 0x00));
	len += snprintf(&buf[len], 128, "Charge Mode: %d\n", (buff[2] >> 1) & 0x01);
	len += snprintf(&buf[len], 128, "Palm Mode: %d\n", (buff[2] >> 2) & 0x01);
	len += snprintf(&buf[len], 128, "Gesture Mode: %d\n", ((buff[1] & TOUCH_SYSTEM_MODE_LOWPOWER) && (buff[3] & TOUCH_MODE_STATE_TOUCH) ? 0x01 : 0x00));

	return len;
}

#if 1
int bbk_slsi_erase_vivo_cal(u8 val)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;
	u8 tBuff[1] = { 0x00 };
	VTI("enter");
	tBuff[0] = val;
	ret = sec_ts_i2c_write(ts, SEC_TS_VIVO_STATUS_COMMAND, tBuff, sizeof(tBuff));// Set to Doesn't have vivo offset calibration
	if (ret < 0) {
		VTI("fail to Erase Vivo Calibration flag!");
	}

	return ret;
}

int bbk_slsi_get_vivo_calibration_status(int *cali_satus)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;
	u8 vivo_cal_status[1] = {0};
	VTI("enter");
	ret = sec_ts_i2c_read(ts, SEC_TS_VIVO_STATUS_COMMAND, vivo_cal_status, 1);

	if (ret < 0) {
		VTI("failed to read Vivo Calibration status");
		return ret;
	}

	VTI("cal status is 0x%x", vivo_cal_status[0]);
	*cali_satus = vivo_cal_status[0];
	return  ret;
}

int bbk_slsi_start_force_calibration(void)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;

	VTI("enter");
	ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SEC);
	if (ret < 0) {
		VTI("fail to write OFFSET CAL SEC!");
	} else {
		VTI("success to write OFFSET CAL SEC!");
	}

	return ret;
}
#endif
