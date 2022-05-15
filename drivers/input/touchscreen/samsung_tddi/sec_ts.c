/* drivers/input/touchscreen/sec_ts.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
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
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/completion.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include "sec_ts.h"

#include <linux/vivo_ts_function.h>
#include <linux/vivo_ts_ic_ctl.h>
#include <linux/vivo_ts_node.h>
#include <linux/bbk_drivers_info.h>
#include <linux/vivo_touchscreen_virtual_key.h>
#include <linux/input/vivo_key.h>
#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
static int qup_i2c_suspended;
//extern int qup_i2c_suspended;

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static struct device *sec_ts_dev;
EXPORT_SYMBOL(sec_ts_dev);

#ifdef USE_RESET_DURING_POWER_ON
static void sec_ts_reset_work(struct work_struct *work);
#endif

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev);
static void sec_ts_input_close(struct input_dev *dev);
#endif

bool tsp_init_done;
struct sec_ts_data *g_ts_data;

static int sec_ts_read_information(struct sec_ts_data *ts);

u8 lv1cmd;
u8 *read_lv1_buff;
static int lv1_readsize;
static int lv1_readremain;
static int lv1_readoffset;

static ssize_t sec_ts_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regreadsize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static inline ssize_t sec_ts_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sec_ts_enter_recovery_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regread_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_ts_gesture_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static inline ssize_t sec_ts_show_error(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(sec_ts_reg, (0644), NULL, sec_ts_reg_store);
static DEVICE_ATTR(sec_ts_regreadsize, (0644), NULL, sec_ts_regreadsize_store);
static DEVICE_ATTR(sec_ts_enter_recovery, (0644), NULL, sec_ts_enter_recovery_store);
static DEVICE_ATTR(sec_ts_regread, (0644), sec_ts_regread_show, NULL);
static DEVICE_ATTR(sec_ts_gesture_status, (0644), sec_ts_gesture_status_show, NULL);

static struct attribute *cmd_attributes[] = {
	&dev_attr_sec_ts_reg.attr,
	&dev_attr_sec_ts_regreadsize.attr,
	&dev_attr_sec_ts_enter_recovery.attr,
	&dev_attr_sec_ts_regread.attr,
	&dev_attr_sec_ts_gesture_status.attr,
	NULL,
};

static struct attribute_group cmd_attr_group = {
	.attrs = cmd_attributes,
};

static inline ssize_t sec_ts_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	VTE("sec_ts :%s read only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t sec_ts_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	VTE("sec_ts :%s write only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}


int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 buf[I2C_WRITE_BUFFER_SIZE + 1];
	int ret;
	unsigned char retry;
	struct i2c_msg msg;
	int retry_count = 0;
#ifdef USE_SEC_TS_DEBUG
	int i = 0;
#endif
	
	while(qup_i2c_suspended > 0) {
		VTI("iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) {
			VTI("after 60ms delay , fts iic is still  on suspend.");
			goto err;
		}
	}

	if (len > I2C_WRITE_BUFFER_SIZE) {
		VTI("sec_ts_i2c_write len is larger than buffer size\n");
		return -1;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;
	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));
		ts->client->adapter->timeout = HZ/10;
		if ((ret = i2c_transfer(ts->client->adapter, &msg, 1)) == 1) {
			mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
			break;
		}
		mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));

		VTI("I2C retry %d", retry + 1);
		usleep_range(1 * 1000, 1 * 1000);
	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTI("%s: I2C write over retry limit\n", __func__);
		vivoTsGetVtsData()->iicErrorNum++;
		ret = -EIO;
#ifdef USE_POR_AFTER_I2C_RETRY
		schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#endif
	}
#ifdef USE_SEC_TS_DEBUG
	VTI("%s i2c_cmd: W: %02X | ", __func__, reg);
	for (i = 0; i < len; i++) 
		VTI("%02X ", data[i]);
	VTI("\n");
#endif

	if (ret == 1)
		return 0;
err:
	return -EIO;
}

int sec_ts_i2c_read(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 buf[4];
	int ret;
	unsigned char retry;
	struct i2c_msg msg[2];
	int remain = len;
	int retry_count = 0;
#ifdef USE_SEC_TS_DEBUG
	int i = 0;
#endif

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	while(qup_i2c_suspended > 0) {
		VTI("iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) {
			VTI("after 60ms delay , fts iic is still  on suspend.");
			goto err;
		}
	}

	buf[0] = reg;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	mutex_lock(&ts->i2c_mutex);
	if (len <= ts->i2c_burstmax) {

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));
			ts->client->adapter->timeout = HZ/10;
			ret = i2c_transfer(ts->client->adapter, msg, 2);
			mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
			if (ret == 2)
				break;
			usleep_range(1 * 1000, 1 * 1000);
		}

	} else {
		/*
		 * I2C read buffer is 256 byte. do not support long buffer over than 256.
		 * So, try to seperate reading data about 256 bytes.
		 */

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));
			ts->client->adapter->timeout = HZ/10;
			ret = i2c_transfer(ts->client->adapter, msg, 1);
			mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);
		}

		do {
			if (remain > ts->i2c_burstmax)
				msg[1].len = ts->i2c_burstmax;
			else
				msg[1].len = remain;

			remain -= ts->i2c_burstmax;

			for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
				mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));
				ts->client->adapter->timeout = HZ/10;
				ret = i2c_transfer(ts->client->adapter, &msg[1], 1);
				mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
				if (ret == 1)
					break;
				usleep_range(1 * 1000, 1 * 1000);
			}

			msg[1].buf += msg[1].len;

		} while (remain > 0);

	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTI("%s: I2C read over retry limit\n", __func__);
		vivoTsGetVtsData()->iicErrorNum++;
		ret = -EIO;
#ifdef USE_POR_AFTER_I2C_RETRY
		schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#endif

	}

#ifdef USE_SEC_TS_DEBUG
		VTI("%s i2c_cmd: R: %02X | ", __func__, reg);
		for (i = 0; i < len; i++) 
			VTI("%02X ", data[i]);
		VTI("\n");
#endif


	return ret;

err:
	return -EIO;
}

static int sec_ts_i2c_write_burst(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;
	int retry;
	int retry_count = 0;
#ifdef USE_SEC_TS_DEBUG
	int i = 0;
#endif

	while(qup_i2c_suspended > 0) {
		VTI("iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) {
			VTI("after 60ms delay , fts iic is still  on suspend.");
			return -EIO;
		}
	}

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));
		if ((ret = i2c_master_send(ts->client, data, len)) == len) {
			mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
			break;
		}
		VTI("iic fail,retry %d", retry);
		mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
		usleep_range(1 * 1000, 1 * 1000);
	}

	mutex_unlock(&ts->i2c_mutex);
	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTI("%s: I2C write over retry limit\n", __func__);
		vivoTsGetVtsData()->iicErrorNum++;
		ret = -EIO;
	}

#ifdef USE_SEC_TS_DEBUG
			VTI("%s i2c_cmd: R: %02X | ", __func__, data[0]);
			for (i = 1; i < len; i++) 
				VTI("%02X ", data[i]);
			VTI("\n");
#endif


	return ret;
}

static int sec_ts_i2c_read_bulk(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;
	unsigned char retry;
	int remain = len;
	struct i2c_msg msg;
	int retry_count = 0;
#ifdef USE_SEC_TS_DEBUG
	int i = 0;
#endif
	
	while(qup_i2c_suspended > 0) {
		VTI("iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) {
			VTI("after 60ms delay , fts iic is still  on suspend.");
			return -EIO;
		}
	}

	msg.addr = ts->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	mutex_lock(&ts->i2c_mutex);

	do {
		if (remain > ts->i2c_burstmax)
			msg.len = ts->i2c_burstmax;
		else
			msg.len = remain;

		remain -= ts->i2c_burstmax;

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));
			ts->client->adapter->timeout = HZ/10;
			ret = i2c_transfer(ts->client->adapter, &msg, 1);
			mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);
		}

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTI("%s: I2C read over retry limit\n", __func__);
		vivoTsGetVtsData()->iicErrorNum++;
		ret = -EIO;

		break;
	}
		msg.buf += msg.len;

	} while (remain > 0);

	mutex_unlock(&ts->i2c_mutex);

#ifdef USE_SEC_TS_DEBUG
				VTI("%s i2c_cmd: R: | ", __func__);
				for (i = 0; i < len; i++) 
					VTI("%02X ", data[i]);
				VTI("\n");
#endif


	if (ret == 1)
		return 0;

	return -EIO;
}


void sec_ts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

int sec_ts_wait_for_ready(struct sec_ts_data *ts, unsigned int ack)
{
	int rc = -1;
	int retry = 0;
	u8 tBuff[SEC_TS_EVENT_BUFF_SIZE];

	VTI("C------------------------------0x%x 0x%x", tBuff[0], tBuff[1]);
	while (sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, tBuff, SEC_TS_EVENT_BUFF_SIZE)) {
		if (((tBuff[0] >> 2) & 0x0f) == TYPE_STATUS_EVENT_ACK || ((tBuff[0] >> 2) & 0x0f) == TYPE_STATUS_EVENT_VENDOR_INFO) {
			VTI("B------------------------------0x%x 0x%x", tBuff[0], tBuff[1]);
			if (tBuff[1] == ack) {
				rc = 0;
				break;
			} 
		} 

		if (retry++ > SEC_TS_WAIT_RETRY_CNT) {
			VTI("%s: Time Over\n", __func__);
			break;
		}
		sec_ts_delay(20);
	}

	VTI(
		"%s: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X [%d]\n",
		__func__, tBuff[0], tBuff[1], tBuff[2], tBuff[3],
		tBuff[4], tBuff[5], tBuff[6], tBuff[7], retry);

	return rc;
}

int sec_ts_read_calibration_report(struct sec_ts_data *ts)
{
	int ret;
	u8 buf[5] = { 0 };

	buf[0] = SEC_TS_READ_CALIBRATION_REPORT;

	ret = sec_ts_i2c_read(ts, buf[0], &buf[1], 4);
	if (ret < 0) {
		VTI("%s: failed to read, %d\n", __func__, ret);
		return ret;
	}

	VTI("%s: count:%d, pass count:%d, fail count:%d, status:0x%X\n",
				__func__, buf[1], buf[2], buf[3], buf[4]);

	return buf[4];
}

#define MAX_EVENT_COUNT 128
#include <uapi/linux/input.h>
static void sec_sec_ts_read_event(struct sec_ts_data *ts)
{
	int ret;
	int is_event_remain = 0;
	int t_id;
	int event_id;
	int read_event_count = 0;
	int gesture_point_x = 0, gesture_point_y = 0;
	u8 read_event_buff[SEC_TS_EVENT_BUFF_SIZE];
	struct sec_ts_event_coordinate *p_event_coord;
	struct sec_ts_coordinate *coordinate;
	int left_event = 0;
	unsigned char gesture_type = 0;
	
	/* repeat READ_ONE_EVENT until buffer is empty(No event) */
	do {

		ret = sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, read_event_buff, SEC_TS_EVENT_BUFF_SIZE);
		if (ret < 0) {
			VTE("i2c read one event failed");
			return;
		}


		if (read_event_count > MAX_EVENT_COUNT) {

			VTE("event buffer overflow");

			/* write clear event stack command when read_event_count > MAX_EVENT_COUNT */
			ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
			if (ret < 0)
				VTE("i2c write clear event failed");

			return;
		}


		VTD("event %x %x %x %x %x %x %x %x",
								read_event_buff[0], read_event_buff[1], read_event_buff[2],
								read_event_buff[3], read_event_buff[4], read_event_buff[5],
								read_event_buff[6], read_event_buff[7]);
		event_id = read_event_buff[0] & 0x03;
		VTD("event id:%d", event_id);
		
		switch (event_id) {
		case 1:/*SEC_TS_STATUS_EVENT:*/
			/* tchsta == 0 && ttype == 0 && eid == 0 : buffer empty */
			if (read_event_buff[0] != 0)
				VTD("STATUS %x %x %x %x %x %x %x %x (%d)",
						read_event_buff[0], read_event_buff[1], read_event_buff[2],
						read_event_buff[3], read_event_buff[4], read_event_buff[5],
						read_event_buff[6], read_event_buff[7], is_event_remain);
			is_event_remain = 0;

			if (read_event_buff[0] == 0x09 && read_event_buff[1] == 0x02) {
				if (read_event_buff[2] == 1) {
					/* palm detect */
					VTI("===========large press dectect=============");
					vivoTsCoverMute(0, 1);
					vivoTsGetVtsData()->largePressNum++;
				} else {
					/* palm release */
					VTI("***********large press release*************");
				}
			}			
			break;

		case 0:/*SEC_TS_COORDINATE_EVENT:*/
			if (ts->input_closed) {
				VTI("device is closed");				
				is_event_remain = 0;
				break;
			}
			p_event_coord = (struct sec_ts_event_coordinate *)read_event_buff;

			t_id = (read_event_buff[0] >> 2) & 0xf;
			
			if (t_id < MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT) {
				coordinate = &ts->coord[t_id];
				coordinate->id = t_id;
				if (coordinate->id > 0)
					coordinate->id = coordinate->id - 1;
				coordinate->action = (read_event_buff[0] >> 6);
				coordinate->x = (read_event_buff[1] << 4) | (read_event_buff[3] >> 4); 
				coordinate->y = (read_event_buff[2] << 4) | (read_event_buff[3] & 0x0f); 
				coordinate->z = read_event_buff[6] & 0x2f;
				coordinate->ttype = ((read_event_buff[6] >> 6) << 2) | (read_event_buff[7] >> 6);
				coordinate->major = read_event_buff[4];
				coordinate->minor = read_event_buff[5];
				left_event = read_event_buff[7] & 0x2f;
				VTD("protocol:id:%d action:%d x:%d y:%d z:%d touch type:%d major:%d minor:%d left event:%d", coordinate->id, coordinate->action, coordinate->x, coordinate->y, coordinate->z, 
					coordinate->ttype, coordinate->major, coordinate->minor, left_event);

				if (coordinate->ttype == 0) {	/*finger in*/
				
					if (coordinate->action == SEC_TS_COORDINATE_ACTION_RELEASE) {
						vivoTsInputReport(VTS_TOUCH_UP, coordinate->id, 0, 0, 0);
						if (ts->touch_count > 0)
							ts->touch_count--;
						if (ts->touch_count == 0) {
							vivoTsReleasePoints();
						}
					} else if (coordinate->action == SEC_TS_COORDINATE_ACTION_PRESS) {
						ts->touch_count++;
						vivoTsInputReport(VTS_TOUCH_DOWN, coordinate->id, coordinate->x, coordinate->y, coordinate->major);
						VTD("++++touch_count++++(%d)", ts->touch_count);
						ret = vivoTsCoverMute(ts->touch_count, 0);
						if (ret < 0) {
							if (-2 == ret) {
								VTI("Enter 3 finger or large press mode");
							}
							if (-3 == ret) {
								VTI("Lcd shutoff, no reporting touch");
							}
						}
					} else if (coordinate->action == SEC_TS_COORDINATE_ACTION_MOVE) {					
						vivoTsInputReport(VTS_TOUCH_DOWN, coordinate->id, coordinate->x, coordinate->y, coordinate->major);
						coordinate->lx = coordinate->x;
						coordinate->ly = coordinate->y;
					} else {
						VTI("do not support coordinate action(%d)", coordinate->action);
					}
				} else {
					VTI("do not support coordinate type(%d)", coordinate->ttype);
				}
			} else {
				VTI("tid(%d) is  out of range", t_id);
			}
			if (left_event) {
				/*	is_event_remain = 1;*/
			}
			break;

		case SEC_TS_GESTURE_EVENT:
			VTI("GESTURE detect:%x %x %x %x %x %x %x %x\n", 
						read_event_buff[0], read_event_buff[1], read_event_buff[2],
						read_event_buff[3], read_event_buff[4], read_event_buff[5],
						read_event_buff[6], read_event_buff[7]);
			gesture_type = read_event_buff[1];
			if (gesture_type == 0 || gesture_type == 2 ||
				gesture_type == 3 || gesture_type == 4) {
				if (gesture_type == 0)
					vivoTsInputReport(VTS_GESTURE_EVENT, KEY_WAKEUP, -1, -1, -1);
				if (gesture_type == 2)
					vivoTsInputReport(VTS_GESTURE_EVENT, KEY_WAKEUP_SWIPE, -1, -1, -1);
				if (gesture_type == 3)
					vivoTsInputReport(VTS_GESTURE_EVENT, KEY_LEFT, -1, -1, -1);
				if (gesture_type == 4)
					vivoTsInputReport(VTS_GESTURE_EVENT, KEY_RIGHT, -1, -1, -1);

				return;
			}

			/*clear buf*/
			memset(ts->read_gesture_coordbuf, 0,SEC_TS_GESTURE_COORD_MAX*2);
			/*read gesture point data*/
			mdelay(70);
			ret = sec_ts_i2c_read(ts, SEC_TS_CMD_GESTURECOORD, ts->read_gesture_coordbuf, 20);
			if (ret < 0) {
				VTI("i2c read gesture coord failed");
				/*return;*/
			}
			VTI("Gesture Points: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ", 
			ts->read_gesture_coordbuf[0], ts->read_gesture_coordbuf[1], ts->read_gesture_coordbuf[2], ts->read_gesture_coordbuf[3], ts->read_gesture_coordbuf[4], 
			ts->read_gesture_coordbuf[5], ts->read_gesture_coordbuf[6], ts->read_gesture_coordbuf[7], ts->read_gesture_coordbuf[8], ts->read_gesture_coordbuf[9], 
			ts->read_gesture_coordbuf[10], ts->read_gesture_coordbuf[11], ts->read_gesture_coordbuf[12], ts->read_gesture_coordbuf[13], ts->read_gesture_coordbuf[14], 
			ts->read_gesture_coordbuf[15], ts->read_gesture_coordbuf[16], ts->read_gesture_coordbuf[17], ts->read_gesture_coordbuf[18], ts->read_gesture_coordbuf[19]);

			vtsGesturePointsClean();
			if (gesture_type != 7) {
				if (gesture_type == 9) {
					/*report start point*/
					gesture_point_x = (ts->read_gesture_coordbuf[1] << 4) | (ts->read_gesture_coordbuf[3] >> 4);
					gesture_point_y = (ts->read_gesture_coordbuf[2] << 4) | (ts->read_gesture_coordbuf[3] & 0xf);
					vtsGesturePointsReport(VTS_GESTURE_POINT, 0, gesture_point_x, gesture_point_y);
					/*report top point
					gesture_point_x = (ts->read_gesture_coordbuf[7] << 4) | (ts->read_gesture_coordbuf[9] >> 4);
					gesture_point_y = (ts->read_gesture_coordbuf[8] << 4) | (ts->read_gesture_coordbuf[9] & 0xf);
					vtsGesturePointsReport(VTS_GESTURE_POINT, 2, gesture_point_x, gesture_point_y);
					*/
					/*report left point*/
					gesture_point_x = (ts->read_gesture_coordbuf[13] << 4) | (ts->read_gesture_coordbuf[15] >> 4);
					gesture_point_y = (ts->read_gesture_coordbuf[14] << 4) | (ts->read_gesture_coordbuf[15] & 0xf);	
					vtsGesturePointsReport(VTS_GESTURE_POINT, 1, gesture_point_x, gesture_point_y);
					/*report bottom point
					gesture_point_x = (ts->read_gesture_coordbuf[10] << 4) | (ts->read_gesture_coordbuf[12] >> 4);
					gesture_point_y = (ts->read_gesture_coordbuf[11] << 4) | (ts->read_gesture_coordbuf[12] & 0xf);
					vtsGesturePointsReport(VTS_GESTURE_POINT, 4, gesture_point_x, gesture_point_y);
					*/
					/*report end point*/				
					gesture_point_x = (ts->read_gesture_coordbuf[4] << 4) | (ts->read_gesture_coordbuf[6] >> 4);
					gesture_point_y = (ts->read_gesture_coordbuf[5] << 4) | (ts->read_gesture_coordbuf[6] & 0xf);
					vtsGesturePointsReport(VTS_GESTURE_POINT, 2, gesture_point_x, gesture_point_y);
				} else {
			
					/*report start point*/
					gesture_point_x = (ts->read_gesture_coordbuf[1] << 4) | (ts->read_gesture_coordbuf[3] >> 4);
					gesture_point_y = (ts->read_gesture_coordbuf[2] << 4) | (ts->read_gesture_coordbuf[3] & 0xf);
					vtsGesturePointsReport(VTS_GESTURE_POINT, 0, gesture_point_x, gesture_point_y);
					/*report end point*/				
					gesture_point_x = (ts->read_gesture_coordbuf[4] << 4) | (ts->read_gesture_coordbuf[6] >> 4);
					gesture_point_y = (ts->read_gesture_coordbuf[5] << 4) | (ts->read_gesture_coordbuf[6] & 0xf);
					vtsGesturePointsReport(VTS_GESTURE_POINT, 1, gesture_point_x, gesture_point_y);
					
					if (gesture_type != 1) {		
						/*report top point*/
						gesture_point_x = (ts->read_gesture_coordbuf[7] << 4) | (ts->read_gesture_coordbuf[9] >> 4);
						gesture_point_y = (ts->read_gesture_coordbuf[8] << 4) | (ts->read_gesture_coordbuf[9] & 0xf);
						vtsGesturePointsReport(VTS_GESTURE_POINT, 2, gesture_point_x, gesture_point_y);
						/*report left point*/
						gesture_point_x = (ts->read_gesture_coordbuf[13] << 4) | (ts->read_gesture_coordbuf[15] >> 4);
						gesture_point_y = (ts->read_gesture_coordbuf[14] << 4) | (ts->read_gesture_coordbuf[15] & 0xf);	
						vtsGesturePointsReport(VTS_GESTURE_POINT, 3, gesture_point_x, gesture_point_y);
						/*report bottom point*/
						gesture_point_x = (ts->read_gesture_coordbuf[10] << 4) | (ts->read_gesture_coordbuf[12] >> 4);
						gesture_point_y = (ts->read_gesture_coordbuf[11] << 4) | (ts->read_gesture_coordbuf[12] & 0xf);
						vtsGesturePointsReport(VTS_GESTURE_POINT, 4, gesture_point_x, gesture_point_y);
						/*report right point*/
						gesture_point_x = (ts->read_gesture_coordbuf[16] << 4) | (ts->read_gesture_coordbuf[18] >> 4);
						gesture_point_y = (ts->read_gesture_coordbuf[17] << 4) | (ts->read_gesture_coordbuf[18] & 0xf);
						vtsGesturePointsReport(VTS_GESTURE_POINT, 5, gesture_point_x, gesture_point_y);
						if (gesture_type == 8) {
							if (ts->read_gesture_coordbuf[19] == 16)
								vtsGesturePointsReport(VTS_GESTURE_O_DIR, 1, -1, -1);
							else if (ts->read_gesture_coordbuf[19] == 32)
								vtsGesturePointsReport(VTS_GESTURE_O_DIR, 0, -1, -1);
						}
					}
				}
			} else {
				/*report first point*/
				gesture_point_x = (ts->read_gesture_coordbuf[1] << 4) | (ts->read_gesture_coordbuf[3] >> 4);
				gesture_point_y = (ts->read_gesture_coordbuf[2] << 4) | (ts->read_gesture_coordbuf[3] & 0xf);
				vtsGesturePointsReport(VTS_GESTURE_POINT, 0, gesture_point_x, gesture_point_y);
				/*report second point*/
				gesture_point_x = (ts->read_gesture_coordbuf[4] << 4) | (ts->read_gesture_coordbuf[6] >> 4);
				gesture_point_y = (ts->read_gesture_coordbuf[5] << 4) | (ts->read_gesture_coordbuf[6] & 0xf);
				vtsGesturePointsReport(VTS_GESTURE_POINT, 1, gesture_point_x, gesture_point_y);
				/*report 3rd point*/
				gesture_point_x = (ts->read_gesture_coordbuf[7] << 4) | (ts->read_gesture_coordbuf[9] >> 4);
				gesture_point_y = (ts->read_gesture_coordbuf[8] << 4) | (ts->read_gesture_coordbuf[9] & 0xf);
				vtsGesturePointsReport(VTS_GESTURE_POINT, 2, gesture_point_x, gesture_point_y);
				/*report 4th point*/
				gesture_point_x = (ts->read_gesture_coordbuf[10] << 4) | (ts->read_gesture_coordbuf[12] >> 4);
				gesture_point_y = (ts->read_gesture_coordbuf[11] << 4) | (ts->read_gesture_coordbuf[12] & 0xf);
				vtsGesturePointsReport(VTS_GESTURE_POINT, 3, gesture_point_x, gesture_point_y);
				/*report 5th point*/
				gesture_point_x = (ts->read_gesture_coordbuf[13] << 4) | (ts->read_gesture_coordbuf[15] >> 4);
				gesture_point_y = (ts->read_gesture_coordbuf[14] << 4) | (ts->read_gesture_coordbuf[15] & 0xf);
				vtsGesturePointsReport(VTS_GESTURE_POINT, 4, gesture_point_x, gesture_point_y);
			}

			switch (gesture_type) {
			case 1:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_UP, -1, -1, -1);
				break;
			case 5:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_E, -1, -1, -1);
				break;
			case 6:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_F, -1, -1, -1);
				break;
			case 7:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_W, -1, -1, -1);
				break;
			case 8:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_O, -1, -1, -1);
				break;
			case 9:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_C, -1, -1, -1);
				break;
			case 10:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_A, -1, -1, -1);
				break;
			case 11:
				vivoTsInputReport(VTS_GESTURE_EVENT, KEY_M, -1, -1, -1);
				break;
			default:
				VTI("not support gesture event");
				break;
			}
		/*	ret = sec_ts_get_gesture_point(ts);
			if (ret < 0) {
				VTE("sec_ts_get_gesture_point failed");
			}*/
			/*is_event_remain = 1;*/
			break;

		default:
			VTI("unknown event  %x %x %x %x %x %x",
					read_event_buff[0], read_event_buff[1], read_event_buff[2],
					read_event_buff[3], read_event_buff[4], read_event_buff[5]);

			//is_event_remain = 1;
			break;

		}
	} while (is_event_remain);
}

static irqreturn_t sec_ts_irq_thread(int irq, void *ptr)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)ptr;

	mutex_lock(&ts->eventlock);

	sec_sec_ts_read_event(ts);

	mutex_unlock(&ts->eventlock);

	return IRQ_HANDLED;
}

int get_tsp_status(void)
{
	return 0;
}
EXPORT_SYMBOL(get_tsp_status);

void sec_ts_set_charger(bool enable)
{
	return;
/*
	int ret;
	u8 noise_mode_on[] = {0x01};
	u8 noise_mode_off[] = {0x00};

	if (enable) {
		VTI("sec_ts_set_charger : charger CONNECTED!!\n");
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_NOISE_MODE, noise_mode_on, sizeof(noise_mode_on));
		if (ret < 0)
			VTI("sec_ts_set_charger: fail to write NOISE_ON\n");
	} else {
		VTI("sec_ts_set_charger : charger DISCONNECTED!!\n");
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_NOISE_MODE, noise_mode_off, sizeof(noise_mode_off));
		if (ret < 0)
			VTI("sec_ts_set_charger: fail to write NOISE_OFF\n");
	}
 */
}
EXPORT_SYMBOL(sec_ts_set_charger);

int sec_ts_glove_mode_enables(struct sec_ts_data *ts, int mode)
{
	int ret;

	if (mode)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_GLOVE | SEC_TS_BIT_SETFUNC_MUTUAL);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_GLOVE)) | SEC_TS_BIT_SETFUNC_MUTUAL);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: pwr off, glove:%d, status:%x\n", __func__,
					mode, ts->touch_functions);
		goto glove_enable_err;
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, &(ts->touch_functions), 1);
	if (ret < 0) {
		VTI("%s: Failed to send command", __func__);
		goto glove_enable_err;
	}

	VTI("%s: glove:%d, status:%x\n", __func__,
		mode, ts->touch_functions);

	return 0;

glove_enable_err:
	return -EIO;
}
EXPORT_SYMBOL(sec_ts_glove_mode_enables);

int sec_ts_set_cover_type(struct sec_ts_data *ts, bool enable)
{
	int ret;

	VTI("%s: %d\n", __func__, ts->cover_type);


	switch (ts->cover_type) {
	case SEC_TS_VIEW_WIRELESS:
	case SEC_TS_VIEW_COVER:
	case SEC_TS_VIEW_WALLET:
	case SEC_TS_FLIP_WALLET:
	case SEC_TS_LED_COVER:
	case SEC_TS_MONTBLANC_COVER:
	case SEC_TS_CLEAR_FLIP_COVER:
	case SEC_TS_QWERTY_KEYBOARD_EUR:
	case SEC_TS_QWERTY_KEYBOARD_KOR:
		ts->cover_cmd = (u8)ts->cover_type;
		break;
	case SEC_TS_CHARGER_COVER:
	case SEC_TS_COVER_NOTHING1:
	case SEC_TS_COVER_NOTHING2:
	default:
		ts->cover_cmd = 0;
		VTI("%s: not chage touch state, %d\n",
				__func__, ts->cover_type);
		break;
	}

	if (enable)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER | SEC_TS_BIT_SETFUNC_MUTUAL);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER)) | SEC_TS_BIT_SETFUNC_MUTUAL);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: pwr off, close:%d, status:%x\n", __func__,
					enable, ts->touch_functions);
		goto cover_enable_err;
	}

	if (enable) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);
		if (ret < 0) {
			VTI("%s: Failed to send covertype command: %d", __func__, ts->cover_cmd);
			goto cover_enable_err;
		}
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, &(ts->touch_functions), 1);
	if (ret < 0) {
		VTI("%s: Failed to send command", __func__);
		goto cover_enable_err;
	}

	VTI("%s: close:%d, status:%x\n", __func__,
		enable, ts->touch_functions);

	return 0;

cover_enable_err:
	return -EIO;


}
EXPORT_SYMBOL(sec_ts_set_cover_type);

/* for debugging--------------------------------------------------------------------------------------*/
static ssize_t sec_ts_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: Power off state\n", __func__);
		return -EIO;
	}

	if (size > 0)
		sec_ts_i2c_write_burst(ts, (u8 *)buf, size);

	VTI("sec_ts_reg: 0x%x, 0x%x, size %d\n", buf[0], buf[1], (int)size);
	return size;
}

static ssize_t sec_ts_regread_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	int length;
	int remain;
	int offset;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: Power off state\n", __func__);
		return -EIO;
	}

	disable_irq(ts->client->irq);

	read_lv1_buff = kzalloc(lv1_readsize, GFP_KERNEL);
	if (!read_lv1_buff) {
		VTI("%s kzalloc failed\n", __func__);
		goto malloc_err;
	}

	mutex_lock(&ts->device_mutex);
	remain = lv1_readsize;
	offset = 0;
	do {
		if (remain >= ts->i2c_burstmax)
			length = ts->i2c_burstmax;
		else
			length = remain;

		if (offset == 0)
			ret = sec_ts_i2c_read(ts, lv1cmd, &read_lv1_buff[offset], length);
		else
			ret = sec_ts_i2c_read_bulk(ts, &read_lv1_buff[offset], length);

		if (ret < 0) {
			VTI("%s: i2c read %x command, remain =%d\n", __func__, lv1cmd, remain);
			goto i2c_err;
		}

		remain -= length;
		offset += length;
	} while (remain > 0);

	VTI("%s: lv1_readsize = %d\n", __func__, lv1_readsize);
	memcpy(buf, read_lv1_buff + lv1_readoffset, lv1_readsize);

i2c_err:
	kfree(read_lv1_buff);
malloc_err:
	mutex_unlock(&ts->device_mutex);
	lv1_readremain = 0;
	enable_irq(ts->client->irq);

	return lv1_readsize;
}

static ssize_t sec_ts_gesture_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->device_mutex);
	memcpy(buf, ts->gesture_status, sizeof(ts->gesture_status));
	VTI(
				"sec_sec_ts_gesture_status_show GESTURE STATUS %x %x %x %x %x %x\n",
				ts->gesture_status[0], ts->gesture_status[1], ts->gesture_status[2],
				ts->gesture_status[3], ts->gesture_status[4], ts->gesture_status[5]);
	mutex_unlock(&ts->device_mutex);

	return sizeof(ts->gesture_status);
}

static ssize_t sec_ts_regreadsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	lv1cmd = buf[0];
	lv1_readsize = ((unsigned int)buf[4] << 24) |
			((unsigned int)buf[3] << 16) | ((unsigned int) buf[2] << 8) | ((unsigned int)buf[1] << 0);
	lv1_readoffset = 0;
	lv1_readremain = 0;
	return size;
}

static ssize_t sec_ts_enter_recovery_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int ret;
	int on;

	sscanf(buf, "%d", &on);

	if (on == 1) {
		disable_irq(ts->client->irq);
		gpio_free(pdata->gpio);

		VTI("%s: gpio free\n", __func__);
		if (gpio_is_valid(pdata->gpio)) {
			ret = gpio_request_one(pdata->gpio, GPIOF_OUT_INIT_LOW, "sec,tsp_int");
			VTI("%s: gpio request one\n", __func__);
			if (ret < 0) {
				VTI("Unable to request tsp_int [%d]: %d\n", pdata->gpio, ret);
			}
		} else {
			VTI("Failed to get irq gpio\n");
			return -EINVAL;
		}

		pdata->power(ts->client, false);
		sec_ts_delay(100);
		pdata->power(ts->client, true);
	} else {
		gpio_free(pdata->gpio);

		if (gpio_is_valid(pdata->gpio)) {
			ret = gpio_request_one(pdata->gpio, GPIOF_DIR_IN, "sec,tsp_int");
			if (ret) {
				VTI("Unable to request tsp_int [%d]\n", pdata->gpio);
				return -EINVAL;
			}
		} else {
			VTI("Failed to get irq gpio\n");
			return -EINVAL;
		}

		pdata->power(ts->client, false);
		sec_ts_delay(500);
		pdata->power(ts->client, true);
		sec_ts_delay(500);

		/* AFE Calibration */
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CALIBRATION_AMBIENT, NULL, 0);
		if (ret < 0)
			VTI("%s: fail to write AFE_CAL\n", __func__);

		sec_ts_delay(1000);
		enable_irq(ts->client->irq);
	}

	sec_ts_read_information(ts);

	return size;
}

#ifdef SEC_TS_SUPPORT_TA_MODE
int sec_ts_charger_config(struct sec_ts_data *ts, int status)
{
	int ret;

	if (status == 0x01 || status == 0x03)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_CHARGER | SEC_TS_BIT_SETFUNC_MUTUAL);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_CHARGER)) | SEC_TS_BIT_SETFUNC_MUTUAL);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: pwr off, chg:%d, status:%x\n", __func__,
					status, ts->touch_functions);
		goto charger_enable_err;
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, &(ts->touch_functions), 1);
	if (ret < 0) {
		VTI("%s: Failed to send command", __func__);
		goto charger_enable_err;
	}

	VTI("%s: chg:%d, status:%x\n", __func__,
				status, ts->touch_functions);

	return 0;

charger_enable_err:
	return -EIO;

}

static void sec_ts_ta_cb(struct sec_ts_callbacks *cb, int status)
{
	struct sec_ts_data *ts =
		container_of(cb, struct sec_ts_data, callbacks);

	VTI("[TSP] %s: status : %x\n", __func__, status);

	ts->ta_status = status;
	/* if do not completed driver loading, ta_cb will not run. */
/*
	if (!rmi4_data->init_done.done) {
		VTE("%s: until driver loading done.\n", __func__);
		return;
	}

	if (rmi4_data->touch_stopped || rmi4_data->doing_reflash) {
		VTE("%s: device is in suspend state or reflash.\n",
				__func__);
		return;
	}
*/

	sec_ts_charger_config(ts, status);
}
#endif

static int sec_ts_raw_device_init(struct sec_ts_data *ts)
{
	int ret;

	ret = IS_ERR(sec_ts_dev);
	
	sec_class = class_create(THIS_MODULE, "sec_tsp");
	if(IS_ERR(sec_class)){
		ret = PTR_ERR(sec_class);
		VTI("class PTR_ERR %d",ret);
		return ret;
	}
	if (sec_class < 0)
	{
		VTE("%s: fail - class_create\n", __func__);	
		return ret;
	}

	sec_ts_dev = device_create(sec_class, NULL, 0, ts, "sec_ts");

	if(IS_ERR(sec_ts_dev)){
		ret = PTR_ERR(sec_ts_dev);
		VTI("PTR_ERR %d",ret);
		return ret;
	}

	ret = sysfs_create_group(&sec_ts_dev->kobj, &cmd_attr_group);
	if (ret < 0) {
		VTI("%s: fail - sysfs_create_group\n", __func__);
		goto err_sysfs;
	}

/*
	ret = sysfs_create_link(&sec_ts_dev->kobj,
			&ts->input_dev->dev.kobj, "input");
	if (ret < 0) {
		VTI("%s: fail - sysfs_create_link\n", __func__);
		goto err_sysfs;
	}
*/
	return ret;
err_sysfs:
	VTI("%s: fail\n", __func__);
	return ret;
}

/* for debugging--------------------------------------------------------------------------------------*/

static int sec_ts_pinctrl_configure(struct sec_ts_data *ts, bool enable)
{
	struct pinctrl_state *state;

	VTI("%s: %s\n", __func__, enable ? "ACTIVE" : "SUSPEND");

	if (enable) {
		state = pinctrl_lookup_state(ts->plat_data->pinctrl, "tsp_active");
		if (IS_ERR(ts->plat_data->pins_default))
			VTI("could not get active pinstate\n");
	} else {
		state = pinctrl_lookup_state(ts->plat_data->pinctrl, "tsp_suspend");
		if (IS_ERR(ts->plat_data->pins_sleep))
			VTI("could not get suspend pinstate\n");
	}

	if (!IS_ERR_OR_NULL(state))
		return pinctrl_select_state(ts->plat_data->pinctrl, state);

	return 0;

}

static int sec_ts_power_ctrl(struct i2c_client *client, bool enable)
{
	struct device *dev = &client->dev;
	int retval = 0;
	static struct regulator *avdd, *vddo;

	VTI("enter!enable = %d", enable);

	return 0;	/*tddi no power ctl*/
	
	mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));	
	if (!avdd) {
		avdd = devm_regulator_get(&client->dev, "vdd_ana");

		if (IS_ERR(avdd)) {
			VTE("could not get avdd, rc = %ld", PTR_ERR(avdd));
			avdd = NULL;
			mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
			return -ENODEV;
		}
		retval = regulator_set_voltage(avdd, 3300000, 3300000);
		if (retval)
			VTE("unable to set avdd voltage to 3.3V");

		VTI("3.3V is enabled %s", regulator_is_enabled(avdd) ? "TRUE" : "FALSE");
	}
	if (!vddo) {
		vddo = devm_regulator_get(&client->dev, "vcc_i2c");

		if (IS_ERR(vddo)) {
			VTI("could not get vddo, rc = %ld", PTR_ERR(vddo));
			vddo = NULL;
			mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
			return -ENODEV;
		}
		retval = regulator_set_voltage(vddo, 1800000, 1800000);
		if (retval)
			VTI("unable to set vddo voltage to 1.8V");
		VTI("1.8V is enabled %s", regulator_is_enabled(vddo) ? "TRUE" : "FALSE");
	}

	if (enable) {
		if (0/*regulator_is_enabled(avdd)*/) {
			dev_err(dev, "%s: avdd is already enabled\n", __func__);
		} else {
			retval = regulator_enable(avdd);
			if (retval) {
				VTE("Fail to enable regulator avdd[%d]", retval);
			}
			VTI("avdd is enabled[OK]");
		}

		if (0/*regulator_is_enabled(vddo)*/) {
			dev_err(dev, "%s: vddo is already enabled\n", __func__);
		} else {
			retval = regulator_enable(vddo);
			if (retval) {
				VTE("Fail to enable regulator vddo[%d]", retval);
			}
			VTI("vddo is enabled[OK]");
		}
	} else {
		if (regulator_is_enabled(vddo)) {
			retval = regulator_disable(vddo);
			if (retval) {
				VTE("Fail to disable regulator vddo[%d]", retval);

			}
			VTI("vddo is disabled[OK]");
		} else {
			VTI("vddo is already disabled");
		}

		if (regulator_is_enabled(avdd)) {
			retval = regulator_disable(avdd);
			if (retval) {
				VTE("Fail to disable regulator avdd[%d]\n", retval);

			}
			VTI("avdd is disabled[OK]");
		} else {
			VTI("avdd is already disabled");
		}
		/*add for driver request list 31*/
		msleep(10);
	}

	mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));

	return retval;
}

static int sec_ts_parse_dt(struct i2c_client *client)
{
   struct device *dev = &client->dev;
   struct sec_ts_plat_data *pdata = dev->platform_data;
   int ret = 0;
   struct device_node *np = dev->of_node;
 /*need to assign the tsp interrupt pin
 ret = gpio_request_one(pdata->gpio, GPIOF_DIR_IN, "sec,tsp_int");
 if (ret) {
  VTE("Unable to request tsp_int [%d]\n", pdata->gpio);
  return -EINVAL;
 }
 client->irq = gpio_to_irq(pdata->gpio);*/
	pdata->gpio = of_get_named_gpio(np, "synaptics,irq-gpio", 0);
	if (gpio_is_valid(pdata->gpio)) {
		ret = gpio_request_one(pdata->gpio, GPIOF_DIR_IN | GPIOF_INIT_HIGH, "sec,tsp_int");
		if (ret) {
			VTE("Unable to request tsp_int [%d]\n", pdata->gpio);
			return -EINVAL;
		}
	} else {
		VTE("Failed to get irq gpio\n");
		return -EINVAL;
	}

	client->irq = gpio_to_irq(pdata->gpio);
VTI("gpio : %d IRQ : %d",pdata->gpio,client->irq);

   pdata->i2c_burstmax = 1024; 
   pdata->max_x = 720 - 1;        
   pdata->max_y = 1520- 1;
   pdata->grip_area = 0;
   pdata->num_rx = 0;
   pdata->num_tx = 0;  
   pdata->tspid = 1;
   pdata->tspid2 = 0;
   pdata->firmware_name = "lsi.bin";
   pdata->project_name = "Y661";
   pdata->model_name = "Xplay6";
   pdata->panel_revision = 0x0;
   pdata->power = sec_ts_power_ctrl; 
   pdata->bringup = 0;
   return ret;
}

static int sec_ts_read_information(struct sec_ts_data *ts)
{
	unsigned char data[13] = { 0 };
	int ret;

	memset(data, 0x0, 3);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, data, 3);
	if (ret < 0) {
		VTE(
					"%s: failed to read device id(%d)\n",
					__func__, ret);
		return ret;
	}

	VTI("%s: %X, %X, %X\n",
				__func__, data[0], data[1], data[2]);
	
#if 1
	memset(data, 0x0, 9);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_SUB_ID, data, 13);
	if (ret < 0) {
		VTE(
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		return ret;
	}

	VTI("%s: AP/BL(%X), DEV1:%X, DEV2:%X, nTX:%X, nRX:%X, rY:%d, rX:%d\n",
				__func__, data[0], data[1], data[2], data[3], data[4],
				(data[5] << 8) | data[6], (data[7] << 8) | data[8]);

	/* Set X,Y Resolution from IC information. */
	/*if (((data[5] << 8) | data[6]) > 0)
		ts->plat_data->max_y = ((data[5] << 8) | data[6]) - 1;

	if (((data[7] << 8) | data[8]) > 0)
		ts->plat_data->max_x = ((data[7] << 8) | data[8]) - 1;*/

	ts->tx_count = data[3];
	ts->rx_count = data[4];
#endif

	data[0] = 0;
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, data, 1);
	if (ret < 0) {
		VTE(
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		return ret;
	}

	VTI("%s: STATUS : %X\n",
				__func__, data[0]);

	return ret;
}
#if 0
int bbk_sec_get_rawordiff_data(int which, int *data)
{
	int TP_TX = 0xff & vivoTsGetTxRxNum();
	int iRow = 0;
	int TP_RX = 0xff & (vivoTsGetTxRxNum()>>8);
	int iCol = 0;
	short *tmp_data = NULL;

	VTI("get data %d(0:rawdata 1:diffdata)", which);
	if (which == 0) {
		/*get rawdata*/
	}
	
	if (which == 1) {
		/*get rawdata*/
	}

	if (NULL == tmp_data) {
		VTI("data is NULL pointer");
		return -EINVAL;
	}
	
	for (iRow = 0; iRow < TP_TX; iRow++) {
		VTI("VIVO_TS ");
		for (iCol = 0; iCol < TP_RX; iCol++) {
			data[iRow*TP_RX+iCol] = (int)(*tmp_data);
			tmp_data++;
			VTI("%d ", data[iRow*TP_RX+iCol]);	/*data[iRow*TP_RX+iCol]);*/
		}
		VTI("\n");
	}
	
	return 0;
}
#endif

static int bbk_sec_module_id(void)
{
	/*just for display in *#225# view*/
	return VTS_MVC_SAM; /* default samsung*/
}


extern unsigned int report_lcm_id(void);
static int bbk_sec_get_lcm_id(void) {
	int lcm_id;
	lcm_id = report_lcm_id();
	if (lcm_id == 0x20 || lcm_id == 0x21 || lcm_id == 0x22)
		lcm_id = 0;
	return lcm_id;
}
static int bbk_sec_get_iic_bus_state(void) {
	return qup_i2c_suspended;
}

static int bbk_slsi_early_suspend_run(void)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	
	VTI("enter");
	ts->lowpower_mode = SEC_TS_MODE_LSI_ALL;     /*To turn on all the wakeup gesture*/
	ret = sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
	if (ret < 0)
		VTI("Failed to TO_LOWPOWER_MODE");
	VTI("end");
	return 0;
}

#include "PD1803_samsung_auo_fw.h"
#include "samsung_tm_fw.h"
extern int bbk_slsi_get_fw_debug_info(unsigned char *buf);
extern int samsung_at_sensor_test(char *buf, int at_sensor_test_cmd , void *pdata, int tmp);
static bool boot_mode_normal = true;
extern enum boot_mode_t get_boot_mode(void);

static int __sec_ts_probe(struct i2c_client *client)
{
	struct sec_ts_data *ts;
	struct sec_ts_plat_data *pdata;
	static char sec_ts_phys[64] = { 0 };
	int ret = 0;
	struct vivo_ts_struct *vts_data = NULL;

	VTI("enter");
	
	if (tsp_init_done) {
		VTI("tsp already init done");
		return -ENODEV;
	}

	vts_data = vivoTsAlloc();
	if(vts_data == NULL) {
		return -1;
	}

	vts_data->client = client;
	vts_data->tsDimensionX = 720;
	vts_data->tsDimensionY = 1520;
	vivoTsSetTxRxNum(18, 32);


	vts_data->keyType = VTS_KEY_NOKEY;
	vts_data->isIndependentKeyIc = 1;
	vts_data->hasFingerPrint = 1;
	vts_data->hasHomeKey = 0;
	vts_data->hasMenuKey = 0;
	vts_data->hasBackKey = 0;
	
	vts_data->icNum = VTS_IC_SEC_TDDI;
	vts_data->mtkOrQcom = VTS_PLATFORM_MTK_NEW;//VTS_PLATFORM_MTK_NEW;

    vts_data->updateFirmware = bbk_slsi_fw_update;	
    vts_data->icModeChange = bbk_slsi_mode_change;
	vts_data->getRawOrDiffData = bbk_slsi_get_rawordiff_data;
	vts_data->getIcFirmwareOrConfigVersion = bbk_slsi_get_fw_version;
	vts_data->getModuleId = bbk_sec_module_id;
	// vts_data->getGesturePointData = bbk_slsi_gesture_point_get;
	vts_data->setChargerFlagSwitch = bbk_slsi_set_charger_bit;
	vts_data->getTsPinValue = NULL;
	vts_data->sensorTest = NULL;
	vts_data->caliSupport = 1;
	vts_data->atSensorTest = samsung_at_sensor_test;
	vts_data->idleEnableOrDisable = bbk_slsi_idleEnableOrDisable;
	vts_data->setEdgeRestainSwitch = bbk_slsi_setEdgeRestainSwitch;
	vts_data->getLcmId = bbk_sec_get_lcm_id;
	vts_data->getI2cBusState = bbk_sec_get_iic_bus_state;
	vts_data->otherInfo = bbk_slsi_get_fw_debug_info;
	vts_data->earlySuspendRun = bbk_slsi_early_suspend_run;
	vts_data->sensorTestKey = "com.detection.ts.touch_detection:MainActivity:com.detection.ts.touch_detection:0:secTpResult";
	vts_data->lcmNoiseTestKey = "com.detection.ts.noise_detection_apk:MainActivity:null:null:null";	
	ret = vivoTsInit(client, NULL, -1);
	if(ret < 0) {
		vivoTsFree();
		return -1;
	}
	vts_data->resumeEventBlank = FB_EVENT_BLANK;
	vts_data->suspendEventBlank = FB_EVENT_BLANK;

	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1803_samsung_auo_fw, sizeof(PD1803_samsung_auo_fw), "PD1803", VTS_FW_NOPIN_VALUE, VTS_MVC_SAM, 0);
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1803_samsung_auo_fw, sizeof(PD1803_samsung_auo_fw), "PD1803F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_SAM, 0);
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1803_samsung_auo_fw, sizeof(PD1803_samsung_auo_fw), "PD1803BF_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_SAM, 0);
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1803_samsung_auo_fw, sizeof(PD1803_samsung_auo_fw), "PD1732", VTS_FW_NOPIN_VALUE, VTS_MVC_SAM, 0);
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1803_samsung_auo_fw, sizeof(PD1803_samsung_auo_fw), "PD1732F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_SAM, 0);
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1803_samsung_auo_fw, sizeof(PD1803_samsung_auo_fw), "PD1732BF_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_SAM, 0);
	vivoTsFwAdd(VTS_FW_TYPE_FW, samsung_tm_fw, sizeof(samsung_tm_fw), "PD1732", VTS_FW_NOPIN_VALUE, VTS_MVC_SAM, 0x14);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		VTE("EIO err!");
		vivoTsDeInit();
		vivoTsFree();
		return -EIO;
	}

	/* power enable */
	sec_ts_power_ctrl(client, true);

	/* parse dt */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct sec_ts_plat_data), GFP_KERNEL);

		if (!pdata) {
			VTE("Failed to allocate platform data\n");
			goto error_allocate_mem;
		}

		client->dev.platform_data = pdata;

		ret = sec_ts_parse_dt(client);
		if (ret) {
			VTE("Failed to parse dt\n");
			goto error_allocate_mem;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		VTE("No platform data found\n");
		goto error_allocate_mem;
	}
	if (!pdata->power) {
		VTE("No power contorl found\n");
		goto error_allocate_mem;
	}

	pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pdata->pinctrl))
		VTE("could not get pinctrl\n");

	ts = kzalloc(sizeof(struct sec_ts_data), GFP_KERNEL);
	if (!ts) {
		VTE("%s: Failed to alloc mem for info\n", __func__);
		goto error_allocate_mem;
	}

	ts->client = client;
	VTI("%s: addr=0x%x\n", __func__, client->addr);
	client->addr = 0x48;
	VTI("%s: addr=0x%x\n", __func__, client->addr);
	ts->plat_data = pdata;
	ts->crc_addr = 0x0001FE00;
	ts->fw_addr = 0x00002000;
	ts->para_addr = 0x0001B000;
	ts->sec_ts_i2c_read = sec_ts_i2c_read;
	ts->sec_ts_i2c_write = sec_ts_i2c_write;
	ts->sec_ts_i2c_write_burst = sec_ts_i2c_write_burst;
	ts->sec_ts_i2c_read_bulk = sec_ts_i2c_read_bulk;
	ts->i2c_burstmax = pdata->i2c_burstmax;
#ifdef USE_RESET_DURING_POWER_ON
	INIT_DELAYED_WORK(&ts->reset_work, sec_ts_reset_work);
#endif

	g_ts_data = ts;

	i2c_set_clientdata(client, ts);

	if (gpio_is_valid(ts->plat_data->tspid) && gpio_is_valid(ts->plat_data->tspid2)) {
		ts->tspid_val = gpio_get_value(ts->plat_data->tspid);
		ts->tspid2_val = gpio_get_value(ts->plat_data->tspid2);
	} else {
		ts->tspid_val = 0;
		ts->tspid2_val = 0;
	}

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		VTI("%s: allocate device err!\n", __func__);
		ret = -ENOMEM;
		goto err_allocate_device;
	}

	ts->input_dev->name = "sec_touchscreen";
	snprintf(sec_ts_phys, sizeof(sec_ts_phys), "%s/input1",
			ts->input_dev->name);
	ts->input_dev->phys = sec_ts_phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->dev.parent = &client->dev;
	ts->touch_count = 0;
	ts->sec_ts_i2c_write = sec_ts_i2c_write;
	ts->sec_ts_i2c_read = sec_ts_i2c_read;

	sec_ts_raw_device_init(ts);
	/*sec_ts_fn_init(ts);*/

	atomic_set(&ts->irq_enabled, 1);/* zhj add for unbalanced irq when mode change */

	mutex_init(&ts->lock);
	mutex_init(&ts->device_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->eventlock);

	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "tsp_wakelock");
	init_completion(&ts->resume_done);

#ifdef USE_OPEN_CLOSE
	ts->input_dev->open = sec_ts_input_open;
	ts->input_dev->close = sec_ts_input_close;
#endif

	VTE("%s init resource\n", __func__);

	sec_ts_pinctrl_configure(ts, true);

	sec_ts_delay(70);
	ts->power_status = SEC_TS_STATE_POWER_ON;

	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	
	VTE("%s power enable\n", __func__);

#define SEC_TS_FW_UPDATE_ON_PROBE
#ifdef SEC_TS_FW_UPDATE_ON_PROBE
if (1) {
	ret = sec_ts_firmware_auto_update_on_probe(ts);
	if (ret < 0)
		goto err_init;
}
#else
	VTI("%s: fw update on probe disabled!\n", __func__);
#endif

	ret = sec_ts_read_information(ts);
	if (ret < 0)
		goto err_init;

	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0) {
		VTI("sec_ts_probe: fail to write Sense_on\n");
		goto err_init;
	}

	ts->pFrame = kzalloc(ts->tx_count * ts->rx_count * 2, GFP_KERNEL);
	if (!ts->pFrame) {
		VTI("%s: allocate pFrame err!\n", __func__);
		ret = -ENOMEM;
		goto err_allocate_frame;
	}

#ifdef CONFIG_TOUCHSCREN_SEC_TS_GLOVEMODE
	/*input_set_capability(ts->input_dev, EV_SW, SW_GLOVE);*/
#endif
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, ts->input_dev->keybit);
	
	/*et_bit(KEY_BLACK_UI_GESTURE, ts->input_dev->keybit);*/
#ifdef SEC_TS_SUPPORT_TOUCH_KEY
	if (ts->plat_data->support_mskey) {
		int i;

		for (i = 0 ; i < ts->plat_data->num_touchkey ; i++)
			set_bit(ts->plat_data->touchkey[i].keycode, ts->input_dev->keybit);

		set_bit(EV_LED, ts->input_dev->evbit);
		set_bit(LED_MISC, ts->input_dev->ledbit);
	}
#endif
	/*set_bit(KEY_SIDE_GESTURE, ts->input_dev->keybit);*/
	/*set_bit(KEY_SIDE_GESTURE_RIGHT, ts->input_dev->keybit);*/
	/*set_bit(KEY_SIDE_GESTURE_LEFT, ts->input_dev->keybit);*/
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#endif

	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_mt_init_slots(ts->input_dev, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_DIRECT);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->plat_data->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->plat_data->max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	/*input_set_abs_params(ts->input_dev, ABS_MT_PALM, 0, 1, 0, 0);*/
#ifdef CONFIG_SEC_FACTORY
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif

	input_set_drvdata(ts->input_dev, ts);
	i2c_set_clientdata(client, ts);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		VTI("%s: Unable to register %s input device\n", __func__, ts->input_dev->name);
		goto err_input_register_device;
	}

	VTI("sec_ts_probe request_irq = %d\n" , client->irq);

if(0) {
	ret = request_threaded_irq(client->irq, NULL, sec_ts_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, SEC_TS_I2C_NAME, ts);
	if (ret < 0) {
		VTI("sec_ts_probe: Unable to request threaded irq\n");
		goto err_irq;
	}
}
	ret = vivoTsInterruptRegister(client->irq, NULL, sec_ts_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts);
	if (ret < 0) {
		VTE("Failed to create irq thread");
		goto err_irq;
	}


#ifdef SEC_TS_SUPPORT_TA_MODE
	ts->callbacks.inform_charger = sec_ts_ta_cb;
	if (ts->plat_data->register_cb)
		ts->plat_data->register_cb(&ts->callbacks);
#endif


	device_init_wakeup(&client->dev, true);

	/*schedule_delayed_work(&ts->work_read_nv, msecs_to_jiffies(100));*/

	tsp_init_done = true;

	vivoTsAfterProbeCompleteCall(client, NULL, -1);

	return 0;

err_irq:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device:
	if (ts->input_dev)
		input_free_device(ts->input_dev);
	kfree(ts->pFrame);
err_allocate_frame:
err_init:
	wake_lock_destroy(&ts->wakelock);
err_allocate_device:
	kfree(ts);

error_allocate_mem:
	if (gpio_is_valid(pdata->gpio))
		gpio_free(pdata->gpio);
	if (gpio_is_valid(pdata->tspid))
		gpio_free(pdata->tspid);
	if (gpio_is_valid(pdata->tspid2))
		gpio_free(pdata->tspid2);

	sec_ts_power_ctrl(client, false);
	if (ret == -ECONNREFUSED)
		sec_ts_delay(100);
	ret = -ENODEV;

/*error_vivo_ts:*/
	vivoTsDeInit();
	vivoTsFree();

	return ret;
}

void sec_ts_unlocked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
 			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

 			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			VTI("%s: [RA] tID:%d mc: %d tc:%d, v:%02X%02X, cal:%X(%X|%X), id(%d,%d), p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->nv, ts->cal_count, ts->tspid_val,
					ts->tspid2_val, ts->coord[i].palm_count);
 		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	input_mt_slot(ts->input_dev, 0);

	input_report_key(ts->input_dev, BTN_TOUCH, false);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, false);
#ifdef CONFIG_TOUCHSCREN_SEC_TS_GLOVEMODE
	/*input_report_switch(ts->input_dev, SW_GLOVE, false);*/
#endif
	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;

	/*input_report_key(ts->input_dev, KEY_SIDE_GESTURE_LEFT, 0);*/
	/*input_report_key(ts->input_dev, KEY_SIDE_GESTURE_RIGHT, 0);*/

	input_sync(ts->input_dev);

}

void sec_ts_locked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	mutex_lock(&ts->eventlock);

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
 			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

 			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			VTI("%s: [RA] tID:%d mc: %d tc:%d, v:%02X%02X, cal:%X(%X|%X), id(%d,%d), p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->nv, ts->cal_count, ts->tspid_val,
					ts->tspid2_val, ts->coord[i].palm_count);
 		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	input_mt_slot(ts->input_dev, 0);

	input_report_key(ts->input_dev, BTN_TOUCH, false);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, false);
#ifdef CONFIG_TOUCHSCREN_SEC_TS_GLOVEMODE
	/*input_report_switch(ts->input_dev, SW_GLOVE, false);*/
#endif
	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;

	/*input_report_key(ts->input_dev, KEY_SIDE_GESTURE_LEFT, 0);*/
	/*input_report_key(ts->input_dev, KEY_SIDE_GESTURE_RIGHT, 0);*/

	input_sync(ts->input_dev);

	mutex_unlock(&ts->eventlock);

}

#ifdef USE_RESET_DURING_POWER_ON
static void sec_ts_reset_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
							reset_work.work);

	u8 temp_lpm = 0;
	u8 temp_status = 0;

	mutex_lock(&(vivoTsGetVtsData()->i2cResetMutex));

	if (!tsp_init_done) {
		VTI("is not done, return");
		mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
		return;
	}

	VTI("%s\n", __func__);

	temp_lpm = ts->lowpower_mode;
	temp_status = ts->power_status;

	ts->lowpower_mode = 0;
	
	sec_ts_stop_device(ts);

	sec_ts_delay(30);

	sec_ts_start_device(ts);

	ts->lowpower_mode = temp_lpm;
	if ((ts->lowpower_mode) && (temp_status < SEC_TS_STATE_POWER_ON))
		sec_ts_input_close(ts->input_dev);
	
	mutex_unlock(&(vivoTsGetVtsData()->i2cResetMutex));
}
#endif

#if 0
static void sec_ts_read_nv_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
							work_read_nv.work);

	ts->nv = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);
	ts->cal_count = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_CAL_COUNT);

	VTI("%s: fac_nv:%02X, cal_nv:%02X\n", __func__, ts->nv, ts->cal_count);
}
#endif

int sec_ts_set_lowpowermode(struct sec_ts_data *ts, u8 mode)
{
	int ret;
	u8 data;

	VTI("enter");
	VTI("%s: %s(%X)\n", __func__,
			mode == TO_LOWPOWER_MODE ? "ENTER" : "EXIT", ts->lowpower_mode);

	if (mode) {
		/*data = ((ts->lowpower_mode >> 4) & 0xF);*/
		data = 0x3;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_GESTURE_MODE, &data, 1);
		if (ret < 0)
			VTE("Failed to set");
	}
	sec_ts_delay(300);
	VTI("delay");
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &mode, 1);
	VTI("SEC_TS_CMD_SET_POWER_MODE iic ret%d",ret);
	if (ret < 0)
		VTE("failed");

	sec_ts_delay(50);
	
	sec_ts_locked_release_all_finger(ts);

	if (device_may_wakeup(&ts->client->dev)) {
		if (mode)
			enable_irq_wake(ts->client->irq);
		else
			disable_irq_wake(ts->client->irq);
	}

	ts->lowpower_status = mode;

	VTI("end");
	return ret;
}

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);
	int ret;

	ts->input_closed = false;

	VTI("%s\n", __func__);

	if (ts->lowpower_status) {
#ifdef USE_RESET_EXIT_LPM
		schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#else
		sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);

#endif
	} else {
		ret = sec_ts_start_device(ts);
		if (ret < 0)
			VTI("%s: Failed to start device\n", __func__);
	}

	return 0;
}

static void sec_ts_input_close(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);

	ts->input_closed = true;

	VTI("%s\n", __func__);

#ifdef USE_RESET_DURING_POWER_ON
	cancel_delayed_work(&ts->reset_work);
#endif

	if (ts->lowpower_mode) {
		/*sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);*/
		sec_ts_sw_reset(ts);

		ts->power_status = SEC_TS_STATE_LPM_RESUME;
	} else {
		sec_ts_stop_device(ts);
	}

}
#endif

static int sec_ts_remove(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	VTI("%s\n", __func__);

#ifdef USE_RESET_DURING_POWER_ON
	cancel_delayed_work(&ts->reset_work);
#endif
	/*sec_ts_fn_remove(ts);*/

	free_irq(client->irq, ts);

#ifdef CONFIG_TOUCHSCREEN_DUMP_MODE
	p_ghost_check = NULL;
#endif
	device_init_wakeup(&client->dev, false);
	wake_lock_destroy(&ts->wakelock);

	input_mt_destroy_slots(ts->input_dev);
	input_unregister_device(ts->input_dev);

	ts->input_dev = NULL;
	ts->plat_data->power(ts->client, false);

	kfree(ts);
	return 0;
}

static void sec_ts_shutdown(struct i2c_client *client)
{
	VTI("%s\n", __func__);

	sec_ts_remove(client);
}

int sec_ts_stop_device(struct sec_ts_data *ts)
{
	VTI("%s\n", __func__);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: already power off\n", __func__);
		goto out;
	}

	ts->power_status = SEC_TS_STATE_POWER_OFF;

	disable_irq(ts->client->irq);
	sec_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts->client, false);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(false);

	sec_ts_pinctrl_configure(ts, false);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

int sec_ts_start_device(struct sec_ts_data *ts)
{
	int ret;

	VTI("%s\n", __func__);

	sec_ts_pinctrl_configure(ts, true);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_ON) {
		VTI("%s: already power on\n", __func__);
		goto out;
	}

	sec_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts->client, true);
	sec_ts_delay(70);
	ts->power_status = SEC_TS_STATE_POWER_ON;
	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(true);

#ifdef SEC_TS_SUPPORT_TA_MODE
	if (ts->ta_status)
		sec_ts_charger_config(ts, ts->ta_status);
#endif

	if (ts->flip_enable) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);

		ts->touch_functions = ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER | SEC_TS_BIT_SETFUNC_MUTUAL;
		VTI("%s: cover cmd write type:%d, mode:%x, ret:%d", __func__, ts->touch_functions, ts->cover_cmd, ret);
	} else {
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER)) | SEC_TS_BIT_SETFUNC_MUTUAL);
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, &ts->touch_functions, 1);
	if (ret < 0)
		VTE(
				"%s: Failed to send glove_mode command", __func__);

	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		VTI("sec_ts_probe: fail to write Sense_on\n");

	enable_irq(ts->client->irq);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

#ifdef CONFIG_PM
static int sec_ts_pm_suspend(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
/*
	mutex_lock(&ts->input_dev->mutex);
	if (ts->input_dev->users)
		sec_ts_stop_device(ts);
	mutex_unlock(&ts->input_dev->mutex);
*/

	if (ts->lowpower_mode) {
		ts->power_status = SEC_TS_STATE_LPM_SUSPEND;
		/*reinit_completion(&ts->resume_done);*/
	}

	return 0;
}

static int sec_ts_pm_resume(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
/*
	mutex_lock(&ts->input_dev->mutex);
	if (ts->input_dev->users)
		sec_ts_start_device(ts);
	mutex_unlock(&ts->input_dev->mutex);
*/

	if (ts->lowpower_mode) {
		ts->power_status = SEC_TS_STATE_LPM_RESUME;
		complete_all(&ts->resume_done);
	}

	return 0;
}
#endif

static int sec_ts_init_thread(void *dev)
{
	return __sec_ts_probe((struct i2c_client *)dev);
}

static int sec_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	kthread_run(sec_ts_init_thread, client, dev_name(&client->dev));
	return 0;
}


static const struct i2c_device_id sec_ts_id[] = {
	{ SEC_TS_I2C_NAME, 0 },
	{ },
};

#ifdef CONFIG_PM
static const struct dev_pm_ops sec_ts_dev_pm_ops = {
	.suspend = sec_ts_pm_suspend,
	.resume = sec_ts_pm_resume,
};
#endif

#ifdef CONFIG_OF
static struct of_device_id sec_ts_match_table[] = {
	{ .compatible = "samsung,tddi",},
	{ },
};
#else
#define sec_ts_match_table NULL
#endif

static struct i2c_driver sec_ts_driver = {
	.probe		= sec_ts_probe,
	.remove		= sec_ts_remove,
	.shutdown	= sec_ts_shutdown,
	.id_table	= sec_ts_id,
	.driver = {
		.owner    = THIS_MODULE,
		.name	= SEC_TS_I2C_NAME,
#ifdef CONFIG_OF
		.of_match_table = sec_ts_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &sec_ts_dev_pm_ops,
#endif
	},
};

extern unsigned int is_atboot;
//extern unsigned int power_off_charging_mode;
static int __init sec_ts_init(void)
{
	int lcm_id = 0;
	enum boot_mode_t boot_mode;
	VTI("enter");
	
	
/*
	if(is_atboot==1 || power_off_charging_mode==1){
		VTI("TS is in at mood or power off charging mode!");
		return 0;		 
	}
*/
	lcm_id = report_lcm_id();
	VTI("lcm id is 0x%x", lcm_id);
	/*for MTK 6762  0x20,0x21,0x22 is samsung+AUO; 0x14 is samsung+TM*/
	if (lcm_id != 0x20 && lcm_id != 0x21 && lcm_id != 0x22 && lcm_id != 0x14) {
		VTI("not samsung tddi chip");
		return 0;
	}
	
	boot_mode = get_boot_mode();
	VTI("mode=%d", boot_mode);
	if( boot_mode==KERNEL_POWER_OFF_CHARGING_BOOT  ||
		boot_mode==LOW_POWER_OFF_CHARGING_BOOT ||
		boot_mode==META_BOOT ||
		boot_mode==FACTORY_BOOT ||
		boot_mode==ADVMETA_BOOT) {
		VTI("in (%d) mode,we do not load driver", boot_mode);
		boot_mode_normal = false;
	} else {
		boot_mode_normal = true;
	}


	if(is_atboot==1 || !boot_mode_normal){
		VTI("TS is in at mood of power off charging mode.");
		return 0;
	}
	return i2c_add_driver(&sec_ts_driver);
}

static void __exit sec_ts_exit(void)
{
	i2c_del_driver(&sec_ts_driver);
}

MODULE_AUTHOR("Yougnhee, Won<younghee46.won@samsung.com>");
MODULE_DESCRIPTION("Samsung Electronics TouchScreen driver");
MODULE_LICENSE("GPL");

module_init(sec_ts_init);
module_exit(sec_ts_exit);
