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

static int execute_selftest(struct sec_ts_data *ts, u8* data);

int sec_clear_event_stack(struct sec_ts_data *ts)
{
	int ret = 0;
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0) {
		VTI("clear event stack failed");
	}
	return ret;
}


int sec_ts_fix_tmode(struct sec_ts_data *ts, u8 mode, u8 state)
{
	int ret;
	u8 onoff[1] = {STATE_MANAGE_OFF};
	u8 tBuff[2] = { mode, state };

	VTI("%s\n", __func__);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_STATEMANAGE_ON, onoff, 1);

	sec_ts_delay(20);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CHG_SYSMODE , tBuff, sizeof(tBuff));
	sec_ts_delay(50);

	return ret;
}

int sec_ts_release_tmode(struct sec_ts_data *ts)
{
	int ret;
	u8 onoff[1] = {STATE_MANAGE_ON};

	VTI("%s\n", __func__);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_STATEMANAGE_ON, onoff, 1);
	sec_ts_delay(20);

	return ret;
}

static int sec_ts_read_frame(struct sec_ts_data *ts, u8 type, short *min,
		short *max)
{
	unsigned int readbytes = 0xFF;
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int i = 0;
	int j = 0;
	short *temp = NULL;
	//u8 * selfdata;
	u8 tBuff[1];
	int retry = 0;
	
	VTI("%s\n", __func__);

	/* set data length, allocation buffer memory */
	readbytes = ts->rx_count * ts->tx_count * 2;

	pRead = kzalloc(readbytes, GFP_KERNEL);
	if (!pRead) {
		VTE("%s: Read frame kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	/* set OPCODE and data type */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_OPCODE_MUTUAL_DATA_TYPE, &type, 1);
	if (ret < 0) {
		VTI("Set rawdata type failed\n");
		goto ErrorExit;
	}

	sec_ts_delay(200);
	while (sec_ts_i2c_read(ts, SEC_TS_OPCODE_MUTUAL_DATA_TYPE, tBuff, 1)) {
		if (tBuff[0] == type)
			break;
		if (retry++ > SEC_TS_WAIT_RETRY_CNT) {
			VTI("%s: Time Over\n", __func__);
			goto ErrorExit;
		}
		sec_ts_delay(20);
	}

	/* read data */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_TOUCH_RAWDATA, pRead, readbytes);
	if (ret < 0) {
		VTE("%s: read rawdata failed!\n", __func__);
		goto ErrorRelease;
	}

	memset(ts->pFrame, 0x00, readbytes);

	for (i = 0; i < readbytes; i += 2)
		ts->pFrame[i / 2] = pRead[i + 1] + (pRead[i] << 8);

	*min = *max = ts->pFrame[0];

#ifdef DEBUG_MSG
	VTI("02X%02X%02X readbytes=%d\n",
			pRead[0], pRead[1], pRead[2], readbytes);
#endif
	//sec_ts_print_frame(ts, min, max);
//chenpeng
if (0) {
	temp = kzalloc(readbytes, GFP_KERNEL);
	if (!temp) {
		VTE("%s: failed to alloc mem!\n", __func__);
		goto ErrorRelease;

	}

	memcpy(temp, ts->pFrame, ts->tx_count * ts->rx_count * 2);
	memset(ts->pFrame, 0x00, ts->tx_count * ts->rx_count * 2);

	for (i = 0; i < ts->tx_count; i++) {
		for (j = 0; j < ts->rx_count; j++)
			ts->pFrame[(i * ts->rx_count) + j] = temp[(i * ts->rx_count) + j];
	}

	kfree(temp);

}

ErrorRelease:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_OPCODE_MUTUAL_DATA_TYPE, &mode, 1);
	if (ret < 0)
		VTI("Set rawdata type failed\n");

ErrorExit:
	kfree(pRead);
	//kfree(selfdata);

	return ret;
}

#define PRE_DEFINED_DATA_LENGTH		208
#if 0
static int sec_ts_read_channel(struct sec_ts_data *ts, u8 type, short *min, short *max)
{
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int ii = 0;
	int jj = 0;
	u8 w_data;

	VTI("%s\n", __func__);

	pRead = kzalloc(PRE_DEFINED_DATA_LENGTH, GFP_KERNEL);
	if (IS_ERR_OR_NULL(pRead))
		return -ENOMEM;

	/* set OPCODE and data type */
	w_data = type;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_OPCODE_SELF_DATA_TYPE, &w_data, 1);
	if (ret < 0) {
		VTI("Set rawdata type failed\n");
		goto out_read_channel;
	}

	sec_ts_delay(50);

	/* read data */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_TOUCH_SELF_RAWDATA, pRead, PRE_DEFINED_DATA_LENGTH);
	if (ret < 0) {
		VTE("%s: read rawdata failed!\n", __func__);
		goto err_read_data;
	}

	/* clear all pFrame data */
	memset(ts->pFrame, 0x00, ts->tx_count * ts->rx_count * 2);

	/* d[00] ~ d[14] : TX channel
	 * d[15] ~ d[51] : none
	 * d[52] ~ d[77] : RX channel
	 * d[78] ~ d[103] : none
	 */
	for (ii = 0; ii < PRE_DEFINED_DATA_LENGTH; ii += 2) {
		if ((ii >= (ts->tx_count * 2)) && (ii < (PRE_DEFINED_DATA_LENGTH / 2)))
			continue;
		if (ii >= ((PRE_DEFINED_DATA_LENGTH / 2) + (ts->rx_count * 2)))
			break;

		ts->pFrame[jj] = ((pRead[ii] << 8) | pRead[ii + 1]);

		if (ii == 0)
			*min = *max = ts->pFrame[jj];

		*min = min(*min, ts->pFrame[jj]);
		*max = max(*max, ts->pFrame[jj]);

		VTI("%s: [%s][%d] %d\n", __func__,
				(jj < ts->tx_count) ? "TX" : "RX", jj, ts->pFrame[jj]);
		jj++;
	}

err_read_data:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_OPCODE_MUTUAL_DATA_TYPE, &mode, 1);
	if (ret < 0)
		VTI("Set rawdata type failed\n");

out_read_channel:
	kfree(pRead);

	return ret;
}
#endif

static int sec_ts_read_raw_data(struct sec_ts_data *ts,
						struct sec_ts_test_mode *mode)
{
	int ret = 0;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: [ERROR] Touch is stopped\n",
				__func__);
		goto error_power_state;
	}

	VTI("%s: %d, %s\n",
			__func__, mode->type, mode->allnode ? "ALL" : "");


	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		VTE("%s: failed to fix tmode\n",
				__func__);
		goto error_test_fail;
	}

	sec_ts_delay(200);

	ret = sec_ts_read_frame(ts, mode->type, &mode->min, &mode->max);
	if (ret < 0) {
		VTE("%s: failed to read frame\n",
				__func__);
		goto error_test_fail;
	}

	ret = sec_ts_release_tmode(ts);
	if (ret < 0) {
		VTE("%s: failed to release tmode\n",
				__func__);
		goto error_test_fail;
	}

//	kfree(buff);

	sec_ts_locked_release_all_finger(ts);

	return ret;

error_test_fail:
error_power_state:
/*	kfree(buff);*/
/* error_alloc_mem:*/

	sec_ts_locked_release_all_finger(ts);

	return ret;
}

int get_fw_ver_ic(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	char buff[16] = { 0 };
	int ret;
	u8 fw_ver[4];

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		return -EIO;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, fw_ver, 4);
	if (ret < 0) {
		VTI("%s: firmware version read error\n ", __func__);
		snprintf(buff, sizeof(buff), "%s", "NG");
		return -EIO;
	}

	snprintf(buff, sizeof(buff), "SE%02X%02X%02X",
			ts->plat_data->panel_revision, fw_ver[2], fw_ver[3]);

	VTI("%s: %s\n", __func__, buff);

	ret = (fw_ver[3]<<8)|fw_ver[2];
	return ret;
}

int get_config_ver(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	//char buff[20] = { 0 };
	int ret;
/*
	snprintf(buff, "%s_SE_%02X%02X",
		ts->plat_data->model_name,
		ts->plat_data->para_version_of_ic[2], ts->plat_data->para_version_of_ic[3]);

	VTI("%s: %s\n", __func__, buff);
*/

	ret = (ts->plat_data->para_version_of_ic[3]<<8)|ts->plat_data->para_version_of_ic[2];
	return ret;
}

int run_reference_read_all(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SEC;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_rawdata_read_all(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_RAW_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_delta_read_all(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_SIGNAL_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_baseline_read_all(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_AMBIENT_BASELINE;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int set_user_nvm_data(struct sec_ts_data *ts, u8 *data, unsigned char length)
{
	char buff[32+3] = { 0 };
	int ret = 0;
	unsigned char len = length;

	VTI("%s\n", __func__);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: [ERROR] Touch is stopped\n",	__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP_truned off");
		return -EIO;
	}

	/* SENSE OFF -> CELAR EVENT STACK -> SAVE NV -> WRITE NV -> SENSE ON */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0) {
		VTE("%s: fail to write Sense_off\n", __func__);
		goto out_nvm;
	}

	VTI("%s: SENSE OFF\n", __func__);

	sec_ts_delay(100);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0) {
		VTE("%s: i2c write clear event failed\n", __func__);
		goto out_nvm;
	}

	VTI("%s: CLEAR EVENT STACK\n", __func__);

	sec_ts_delay(100);

	sec_ts_locked_release_all_finger(ts);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_NVM_SAVE, &len, 1);
	if (ret < 0) {
		VTE("%s nvm save command failed. ret: %d\n", __func__, ret);
		goto out_nvm;
	}

	sec_ts_delay(100);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_NVM_WRITE, data, length);
	if (ret < 0) {
		VTE("%s nvm read command failed. ret: %d\n", __func__, ret);
		goto out_nvm;
	}

out_nvm:
	sec_ts_delay(200);
	if (ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0) < 0)
		VTE("%s: fail to write Sense_on\n", __func__);
	else
		VTI("%s: SENSE ON\n", __func__);

	sec_ts_delay(300);

	return ret;

}


int get_user_nvm_data(struct sec_ts_data *ts, u8 *data, int length)
{
	int ret = 0;
	u8 len = length;

	VTI("%s: length:%d", __func__, length);

	/* SENSE OFF -> CELAR EVENT STACK -> SAVE NV -> READ NV -> SENSE ON */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0) {
		VTE("%s: fail to write Sense_off\n", __func__);
		goto out_nvm;
	}

	VTI("%s: SENSE OFF\n", __func__);

	sec_ts_delay(100);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0) {
		VTE("%s: i2c write clear event failed\n", __func__);
		goto out_nvm;
	}

	VTI("%s: CLEAR EVENT STACK\n", __func__);

	sec_ts_delay(100);

	sec_ts_locked_release_all_finger(ts);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_NVM_SAVE, &len, 1);
	if (ret < 0) {
		VTE("%s nvm save command failed. ret: %d\n", __func__, ret);
		goto out_nvm;
	}

	sec_ts_delay(100);

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_NVM_READ, data, length);
	if (ret < 0) {
		VTE("%s nvm read command failed. ret: %d\n", __func__, ret);
		goto out_nvm;
	}

out_nvm:
	if (ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0) < 0)
		VTE("%s: fail to write Sense_on\n", __func__);
	else
		VTI("%s: SENSE ON\n", __func__);

	sec_ts_delay(300);

	return ret;
}

int dead_zone_enable(u8 orient)
{
	struct sec_ts_data *ts = g_ts_data;
	/*char buff[SEC_CMD_STR_LEN] = { 0 };*/
	int ret = -1;
	char data = 0;

	if (orient < 0 || orient > 2) {
		VTE("%s NG: wrong orientation\n", __func__);
		return ret;
	}

	data = orient;

	VTI("write edge state: %d", data);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_EDGERESTAIN, &data, 1);
	if (ret < 0)
		VTE("%s failed to set deadzone, ret: %d\n", __func__, ret);

	VTI("write edge state successfully");
	return ret;
};

	
u8 *set_sft_result(u8 result,u8* pSrc, u8* pDes)
{
	char str[2][5] = { "PASS ", "FAIL " };

	strlcpy(pSrc, pDes, strlen(pDes));
	pSrc += strlen(pDes);
	if (result) {
		strlcpy(pSrc, str[0], 6);
		pSrc += strlen(str[0]);
	} else {
		strlcpy(pSrc, str[1], 6);
		pSrc += strlen(str[1]);
	}
	
	return pSrc;
}
static void sec_ts_swap(u8 *a, u8 *b)
{
	u8 temp = *a;
	*a = *b;
	*b = temp;
}

static void rearrange_sft_result(u8 *data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		sec_ts_swap(&data[i], &data[i+3]);
		sec_ts_swap(&data[i+1], &data[i+2]);
	}
}

static int print_selftest_result(struct sec_ts_data *ts)
{
	int rc;
	//u8 tpara = 0x23;
	u8 *rBuff;
	u16 *rBuff16;
	int i = 0;
	int j = 0;
	int result_size = SEC_TS_SELFTEST_REPORT_SIZE + ts->tx_count * ts->rx_count * 2;

	rBuff = kzalloc(result_size, GFP_KERNEL);
	if (!rBuff) {
		VTI("allocation failed!");
		return -ENOMEM;
	}

	rc = ts->sec_ts_i2c_read(ts, SEC_TS_READ_SELFTEST_RESULT, rBuff, result_size);
	if (rc < 0) {
		VTI("Selftest execution time out!");
		goto err_exit;
	}
	rearrange_sft_result(rBuff, result_size);

	for (i = 0; i < 80; i += 4) {
		if (i % 8 == 0)
			VTI("\n");
		if (i % 4 == 0)
			VTI("VIVO_TS sec_ts : ");

		if (i / 4 == 0)
			VTI("SIG");
		else if (i / 4 == 1)
			VTI("VER");
		else if (i / 4 == 2)
			VTI("SIZ");
		else if (i / 4 == 3)
			VTI("CRC");
		else if (i / 4 == 4)
			VTI("RES");
		else if (i / 4 == 5)
			VTI("COU");
		else if (i / 4 == 6)
			VTI("PAS");
		else if (i / 4 == 7)
			VTI("FAI");
		else if (i / 4 == 8)
			VTI("CHA");
		else if (i / 4 == 9)
			VTI("AMB");
		else if (i / 4 == 10)
			VTI("RXS");
		else if (i / 4 == 11)
			VTI("TXS");
		else if (i / 4 == 12)
			VTI("RXO");
		else if (i / 4 == 13)
			VTI("TXO");
		else if (i / 4 == 14)
			VTI("RXG");
		else if (i / 4 == 15)
			VTI("TXG");
		else if (i / 4 == 16)
			VTI("RXR");
		else if (i / 4 == 17)
			VTI("TXT");
		else if (i / 4 == 18)
			VTI("RXT");
		else if (i / 4 == 19)
			VTI("TXR");

		//this 4byte's each bit represents each channel
		VTI(" %2X, %2X, %2X, %2X  ", rBuff[i], rBuff[i + 1], rBuff[i + 2], rBuff[i + 3]);

		if (i / 4 == 4) {
			if ((rBuff[i + 3] & 0x30) != 0)// RX, RX open check.
				rc = 0;
			else
				rc = 1;
		}
	}

	rBuff16 = (u16 *)&rBuff[80];
	VTI("VIVO_TS Ambient Data\n");
	for (i = 0; i < ts->tx_count; i++) {
		VTI("TX%2x :", i);
		for (j = 0; j < ts->rx_count; j++)
			VTI("%6d ", rBuff16[i*ts->rx_count + j]);
		VTI("\n");
	}

err_exit:
	kfree(rBuff);
	return rc;
}
static int execute_selftest(struct sec_ts_data *ts, u8* rBuff)
{
	int rc;
	u8 tpara = 0x23;
	//u8 *rBuff;
	//u8 *rBuffPtr;
	//int i;
	//int result_size;
	//u8 tBuff[SEC_TS_EVENT_BUFF_SIZE];
	u8 retry = 0;

	/*char selftest_str[15][5] =
	{
		"MIN","MAX","SLPRX","SLPTX","OPNRX",
		"OPNTX","SRTRG","SRTTG","SRTRR","SRTTT",
		"SRTTR", "pass","fail","Pass","Failed"
	};*/

	/*result_size = sizeof(char)*11*10+6;
	
	rBuff = kzalloc(result_size, GFP_KERNEL);
	if (!rBuff) {
		VTE("%s: allocation failed!\n", __func__);
		return -ENOMEM;
	}*/

	VTI("Self test start!");
	rc = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SELFTEST, &tpara, 1);
	if (rc < 0) {
		VTI("Send selftest cmd failed!");
		goto err_exit;
	}
	
	sec_ts_delay(1000);	//350	

	while (sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, rBuff, SEC_TS_EVENT_BUFF_SIZE)) {
		VTI("rbuff[0] = 0x%x rbuff[1] = 0x%x", rBuff[0], rBuff[1]);
		if (rBuff[0] == TYPE_STATUS_EVENT_VENDOR_INFO) {
			if (rBuff[1] == SEC_TS_ACK_SELF_TEST_DONE) {
				rc = 1;
				break;
			}
		}

		if (retry++ > SEC_TS_WAIT_RETRY_CNT) {
			VTI("Time Over");
			rc = -1;
			print_selftest_result(ts);
			goto err_exit;
		}
		sec_ts_delay(20);
	}
	
	VTI("Self test done!");

	print_selftest_result(ts);
/*	rBuffPtr = rBuff ;	
	
	if( (tBuff[2] != 0) || (tBuff[3] != 0)){  
		strcpy(rBuffPtr,selftest_str[14]);
		rBuffPtr += strlen(selftest_str[14]);
	}
	else{
		strcpy(rBuffPtr,selftest_str[13]);
		rBuffPtr += strlen(selftest_str[13]);
	}

	for( i=0;i<8; i++){		
		rBuffPtr = set_sft_result((tBuff[2]>>i)&0x1, rBuffPtr,selftest_str[i]);
	}
	
	for( i=0;i<3; i++){
		rBuffPtr = set_sft_result((tBuff[3]>>i)&0x1, rBuffPtr,selftest_str[i+8]);
	}*/
	
/*	rc = ts->sec_ts_i2c_read(ts, SEC_TS_READ_SELFTEST_RESULT, rBuff, result_size);
	if (rc < 0) {
		VTE("%s: Selftest execution time out!\n", __func__);
		goto err_exit;
	}*/	
	return rc;
err_exit:
	//kfree(rBuff);
	return rc;
}

void run_trx_short_test(void *device_data, u8* data)
{
	struct sec_ts_data *ts = g_ts_data;
	char buff[SEC_CMD_STR_LEN] = {0};
	int rc;
	char para = TO_TOUCH_MODE;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("Touch is stopped!");
		return;
	}

	disable_irq(ts->client->irq);

	rc = execute_selftest(ts, data);
	if (rc) {

		ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		sec_ts_delay(50);
		enable_irq(ts->client->irq);
		VTI("%s", buff);
		return;
	}

	ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	enable_irq(ts->client->irq);

	snprintf(buff, sizeof(buff), "%s", "NG");
	VTI("%s", buff);
	return;

}

int sec_ts_get_gesture_point (struct sec_ts_data *ts)
{
	int ret = 0;
	int i = 0;
	u8 buff[20] = { 0 };
	u16 data[20]  = { 0 };

	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_GESTURECOORD, buff, 20);
	if (ret < 0) {
		VTE("i2c read gesture coord failed\n");
		return ret;
	}
	if (buff[0] == 0) {
		ts->read_gesture_point_num = 0;
		VTI("No gesture");
		return ret;
	}

	switch (buff[0]) {
	case GESTURE_DOUBLE_CLICK:
	case GESTURE_DOWN_SLIDE:
	case GESTURE_LEFT_SLIDE:
	case GESTURE_RIGHT_SLIDE:
	case GESTURE_M:
		return 0;
	case GESTURE_UP_SLIDE:
		ret = 2;
		break;
	case GESTURE_W:
		ret = 5;
		break;
	default:
		ret = 6;
		break;
	}
	ts->read_gesture_point_num = ret;

	VTI("gesture point num is %d", ts->read_gesture_point_num);

	for (i = 0; i < 6; i++) {
		if (i >= ret) {
		data[i * 2] = 0;
		data[i * 2 + 1] = 0;
		continue;
		}
		data[i * 2] = (buff[3 * i + 1] << 4) | (buff[3 * i + 3] >> 4);
		data[i * 2 + 1] = (buff[3 * i + 2] << 4) | (buff[3 * i + 3] & 0x0f);

		ts->sec_gsTrace.u16aX[i] = data[i * 2];
		ts->sec_gsTrace.u16aY[i] = data[i * 2 + 1];

		VTI("gesture_x: %d, gesture_y: %d", ts->sec_gsTrace.u16aX[i], ts->sec_gsTrace.u16aY[i]);
	}
	if (buff[0] == GESTURE_O)  /* orientation info for 'O' */
	data[12] = buff[19];

	return ret;
}
