/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 13132 $
 * $Date: 2017 -06 -07 18:25:27 + 0800 (Wed, 07 Jun 2017) $
 *
 * This program is free software; you can redistribute it and / or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */


#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>

#include "nt36xxx.h"
#include <linux/vivo_ts_function.h>

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"

#define I2C_TANSFER_LENGTH  64

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static int32_t xdata_i[2048] = {0};
static int32_t xdata_q[2048] = {0};

#if (_CustomerFunction_)
	int32_t xdata_custom_1732[2048] = {0};
#endif
static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
#if (_CustomerFunction_)
	#define NVTcustom_FlashID "nvt_custom_flashid"
	static struct proc_dir_entry *NVTcustomer_proc_FlashID;

#endif
/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
 *******************************************************/
void nvt_change_mode_1732(uint8_t mode)
{
	uint8_t buf[8] = {0};

	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (ts_1732->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts_1732->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);

	/*-- - set mode-- - */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0-- - pipe 0. 1-- - pipe 1.
 *******************************************************/
uint8_t nvt_get_fw_pipe_1732(void)
{
	uint8_t buf[8] = {0};

	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (ts_1732->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts_1732->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);

	/*-- - read fw status-- - */
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_I2C_READ_1732(ts_1732->client, I2C_FW_Address, buf, 2);

	/*NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]); */

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
 *******************************************************/
void nvt_read_mdata_1732(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[I2C_TANSFER_LENGTH + 1] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	/*-- - set xdata sector address & length-- - */
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts_1732->x_num * ts_1732->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	/*printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len); */

	/*read xdata : step 1 */
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		/*-- - change xdata index-- - */
		buf[0] = 0xFF;
		buf[1] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 16) & 0xFF;
		buf[2] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 8) & 0xFF;
		CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);

		/*-- - read xdata by I2C_TANSFER_LENGTH */
		for (j = 0; j < (XDATA_SECTOR_SIZE / I2C_TANSFER_LENGTH); j++) {
			/*-- - read data-- - */
			buf[0] = I2C_TANSFER_LENGTH * j;
			CTP_I2C_READ_1732(ts_1732->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

			/*-- - copy buf to xdata_tmp-- - */
			for (k = 0; k < I2C_TANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
				/*printk("0x%02X, 0x%04X\n", buf[k + 1], (XDATA_SECTOR_SIZE *i + I2C_TANSFER_LENGTH *j + k)); */
			}
		}
		/*printk("addr=0x%05X\n", (head_addr + XDATA_SECTOR_SIZE *i)); */
	}

	/*read xdata : step2 */
	if (residual_len != 0) {
		/*-- - change xdata index-- - */
		buf[0] = 0xFF;
		buf[1] = ((xdata_addr + data_len - residual_len) >> 16) & 0xFF;
		buf[2] = ((xdata_addr + data_len - residual_len) >> 8) & 0xFF;
		CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);

		/*-- - read xdata by I2C_TANSFER_LENGTH */
		for (j = 0; j < (residual_len / I2C_TANSFER_LENGTH + 1); j++) {
			/*-- - read data-- - */
			buf[0] = I2C_TANSFER_LENGTH * j;
			CTP_I2C_READ_1732(ts_1732->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

			/*-- - copy buf to xdata_tmp-- - */
			for (k = 0; k < I2C_TANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
				/*printk("0x%02X, 0x%04x\n", buf[k + 1], ((dummy_len + data_len - residual_len) + I2C_TANSFER_LENGTH *j + k)); */
			}
		}
		/*printk("addr=0x%05X\n", (xdata_addr + data_len - residual_len)); */
	}

	/*-- - remove dummy data and 2bytes - to -1data-- - */
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	/*read button xdata : step3 */
	/*-- - change xdata index-- - */
	buf[0] = 0xFF;
	buf[1] = (xdata_btn_addr >> 16) & 0xFF;
	buf[2] = ((xdata_btn_addr >> 8) & 0xFF);
	CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);

	/*-- - read data-- - */
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_I2C_READ_1732(ts_1732->client, I2C_FW_Address, buf, (TOUCH_KEY_NUM * 2 + 1));

	/*-- -2bytes - to -1data-- - */
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts_1732->x_num * ts_1732->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (ts_1732->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts_1732->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data from IQ to rss function.

return:
	n.a.
 *******************************************************/
void nvt_read_mdata_1732_rss_1732(uint32_t xdata_i_addr, uint32_t xdata_q_addr, uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr)
{
	int i = 0;

	nvt_read_mdata_1732(xdata_i_addr, xdata_btn_i_addr);
	memcpy(xdata_i, xdata, ((ts_1732->x_num * ts_1732->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	nvt_read_mdata_1732(xdata_q_addr, xdata_btn_q_addr);
	memcpy(xdata_q, xdata, ((ts_1732->x_num * ts_1732->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	for (i = 0; i < (ts_1732->x_num * ts_1732->y_num + TOUCH_KEY_NUM); i++) {
		xdata[i] = (int32_t)int_sqrt((unsigned long)(xdata_i[i] * xdata_i[i]) + (unsigned long)(xdata_q[i] * xdata_q[i]));
	}
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
 *******************************************************/
void nvt_get_mdata_1732(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts_1732->x_num;
    *m_y_num = ts_1732->y_num;
    memcpy(buf, xdata, ((ts_1732->x_num * ts_1732->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts_1732->fw_ver, ts_1732->x_num, ts_1732->y_num, ts_1732->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts_1732->y_num; i++) {
		for (j = 0; j < ts_1732->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts_1732->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts_1732->x_num * ts_1732->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1-- - call next function.
	NULL-- - not call next function and sequence loop
	stop.
 *******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL-- - no next and call sequence
	stop function.
 *******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
 *******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops_1732 = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops_1732 = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen / proc / nvt_fw_version open
	function.

return:
	n.a.
 *******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif

	if (nvt_get_fw_info_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts_1732->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops_1732);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen / proc / nvt_baseline open function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif

	if (nvt_clear_fw_status_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	nvt_change_mode_1732(TEST_MODE_2);

	if (nvt_check_fw_status_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	if (ts_1732->carrier_system) {
		nvt_read_mdata_1732_rss_1732(ts_1732->mmap->BASELINE_ADDR, ts_1732->mmap->BASELINE_Q_ADDR,
				ts_1732->mmap->BASELINE_BTN_ADDR, ts_1732->mmap->BASELINE_BTN_Q_ADDR);
	} else {
		nvt_read_mdata_1732(ts_1732->mmap->BASELINE_ADDR, ts_1732->mmap->BASELINE_BTN_ADDR);
	}

	nvt_change_mode_1732(NORMAL_MODE);

	mutex_unlock(&ts_1732->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops_1732);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen / proc / nvt_raw open function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif

	if (nvt_clear_fw_status_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	nvt_change_mode_1732(TEST_MODE_2);

	if (nvt_check_fw_status_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	if (ts_1732->carrier_system) {
		if (nvt_get_fw_pipe_1732() == 0)
			nvt_read_mdata_1732_rss_1732(ts_1732->mmap->RAW_PIPE0_ADDR, ts_1732->mmap->RAW_PIPE0_Q_ADDR,
				ts_1732->mmap->RAW_BTN_PIPE0_ADDR, ts_1732->mmap->RAW_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_1732_rss_1732(ts_1732->mmap->RAW_PIPE1_ADDR, ts_1732->mmap->RAW_PIPE1_Q_ADDR,
				ts_1732->mmap->RAW_BTN_PIPE1_ADDR, ts_1732->mmap->RAW_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe_1732() == 0)
			nvt_read_mdata_1732(ts_1732->mmap->RAW_PIPE0_ADDR, ts_1732->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata_1732(ts_1732->mmap->RAW_PIPE1_ADDR, ts_1732->mmap->RAW_BTN_PIPE1_ADDR);
	}

	nvt_change_mode_1732(NORMAL_MODE);

	NVT_LOG("--\n");

	mutex_unlock(&ts_1732->lock);

	return seq_open(file, &nvt_seq_ops_1732);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen / proc / nvt_diff open function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif

	if (nvt_clear_fw_status_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	nvt_change_mode_1732(TEST_MODE_2);

	if (nvt_check_fw_status_1732()) {
		mutex_unlock(&ts_1732->lock);
		return -EAGAIN;
	}

	nvt_get_fw_info_1732();

	if (ts_1732->carrier_system) {
		if (nvt_get_fw_pipe_1732() == 0)
			nvt_read_mdata_1732_rss_1732(ts_1732->mmap->DIFF_PIPE0_ADDR, ts_1732->mmap->DIFF_PIPE0_Q_ADDR,
				ts_1732->mmap->DIFF_BTN_PIPE0_ADDR, ts_1732->mmap->DIFF_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_1732_rss_1732(ts_1732->mmap->DIFF_PIPE1_ADDR, ts_1732->mmap->DIFF_PIPE1_Q_ADDR,
				ts_1732->mmap->DIFF_BTN_PIPE1_ADDR, ts_1732->mmap->DIFF_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe_1732() == 0)
			nvt_read_mdata_1732(ts_1732->mmap->DIFF_PIPE0_ADDR, ts_1732->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata_1732(ts_1732->mmap->DIFF_PIPE1_ADDR, ts_1732->mmap->DIFF_BTN_PIPE1_ADDR);
	}

	nvt_change_mode_1732(NORMAL_MODE);

	mutex_unlock(&ts_1732->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops_1732);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#if (_CustomerFunction_)
extern int bbk_nt_sensor_test_1732(char *buf, void *pdata, int tmp);
static int32_t nvtc_FlashID_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}
	NVT_LOG("++\n");
#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif
	mutex_unlock(&ts_1732->lock);
	{
		int32_t i32ret = 0;
		uint8_t testBuf[128] = {0};
		i32ret = bbk_nt_sensor_test_1732(testBuf, NULL, -1);/*<-Watch!Contain Mutex Lock/unLock*/
		NVT_LOG("FinalResult[%s]\n", testBuf);
		NVT_LOG("Detail[%s]\n", &testBuf[7]);
	}
	NVT_LOG("--\n");
	return seq_open(file, &nvt_seq_ops_1732);
}
static const struct file_operations nvtcustom_FlashID = {
	.owner = THIS_MODULE,
	.open = nvtc_FlashID_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif /*_CustomerFunction_*/
/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0-- - succeed. -12-- - failed.
 *******************************************************/
int32_t nvt_extra_proc_init_1732(void)
{
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL, &nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/nvt_fw_version Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_fw_version Succeeded!\n");
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL, &nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/nvt_baseline Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_baseline Succeeded!\n");
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL, &nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/nvt_raw Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_raw Succeeded!\n");
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL, &nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/nvt_diff Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_diff Succeeded!\n");
	}

#if (_CustomerFunction_)
	NVTcustomer_proc_FlashID = proc_create(NVTcustom_FlashID, 0444, NULL, &nvtcustom_FlashID);
#endif
	return 0;
}



extern int nvt_sw_reset_idle_1732(void);
extern int32_t Update_Firmware_1732(void);
extern int32_t Resume_PD_1732(void);
extern void nvt_bootloader_reset_1732(void);
extern int32_t Init_BootLoader_1732(void);


#define SectorSpecify (30)/*30 < Specify 0x1E000~0x1EFFF >, 31 < Specify 0x1F000~0x1FFF > */
#define FLASH_ID_READ_LEN 32
#define FLASH_ID_WRITE_LEN 32
uint8_t u8A_IDread_1732[FLASH_ID_READ_LEN + 1] = {"999999999"};
uint8_t u8A_IDwrite_1732[64]   = {"1234567890*ABCDEFGHJ123456789012"};/*len is 32 */

int32_t nvtc_Erase_Flash_1732_Sector(void)
{
	uint8_t buf[64] = {0};
	int32_t ret = 0;
	/*int32_t count = 0; */
	int32_t i = SectorSpecify;
	int32_t Flash_Address = 0;
	int32_t retry = 0;

	/* Write Enable */
	buf[0] = 0x00;
	buf[1] = 0x06;
	ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("Write Enable (for Write Status Register) error!!(%d)\n", ret);
		return ret;
	}
	/* Check 0xAA (Write Enable) */
	retry = 0;
	while (1) {
		mdelay(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Write Enable for Write Status Register) error!!(%d)\n", ret);
			return ret;
		}
		if (buf[1] == 0xAA) {
			break;
		}
		retry++;
		if (unlikely(retry > 20)) {
			NVT_ERR("Check 0xAA (Write Enable for Write Status Register) error!! status=0x%02X\n", buf[1]);
			ret = -1;
			return ret;
		}
	}

	/* Write Status Register */
	buf[0] = 0x00;
	buf[1] = 0x01;
	buf[2] = 0x00;
	ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("Write Status Register error!!(%d)\n", ret);
		return ret;
	}
	/* Check 0xAA (Write Status Register) */
	retry = 0;
	while (1) {
		mdelay(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Write Status Register) error!!(%d)\n", ret);
			return ret;
		}
		if (buf[1] == 0xAA) {
			break;
		}
		retry++;
		if (unlikely(retry > 20)) {
			NVT_ERR("Check 0xAA (Write Status Register) error!! status=0x%02X\n", buf[1]);
			ret = -1;
			return ret;
		}
	}

	/* Read Status */
	retry = 0;
	while (1) {
		mdelay(5);
		buf[0] = 0x00;
		buf[1] = 0x05;
		ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Read Status (for Write Status Register) error!!(%d)\n", ret);
			return ret;
		}

		/* Check 0xAA (Read Status) */
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[2] = 0x00;
		ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 3);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Read Status for Write Status Register) error!!(%d)\n", ret);
			return ret;
		}
		if ((buf[1] == 0xAA) && (buf[2] == 0x00)) {
			break;
		}
		retry++;
		if (unlikely(retry > 100)) {
			NVT_ERR("Check 0xAA (Read Status for Write Status Register) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
			ret = -1;
			return ret;
		}
	}

	/*for (i = 0; i < count; i++) { */
	i = SectorSpecify;/*31[20170301, jx] < Specify 0x1F000~0x1FFF > */
	{
		/* Write Enable */
		buf[0] = 0x00;
		buf[1] = 0x06;
		ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write Enable error!!(%d,%d)\n", ret, i);
			return ret;
		}
		/* Check 0xAA (Write Enable) */
		retry = 0;
		while (1) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Write Enable) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if (buf[1] == 0xAA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Write Enable) error!! status=0x%02X\n", buf[1]);
				ret = -1;
				return ret;
			}
		}

		/*Flash_Address = i * FLASH_SECTOR_SIZE; */
		Flash_Address = i << 12;
			NVT_LOG("Sector Erase !!(0x%x,%d)\n", Flash_Address, i);


		/* Sector Erase */
		buf[0] = 0x00;
		buf[1] = 0x20;    /* Command : Sector Erase */
		buf[2] = ((Flash_Address >> 16) & 0xFF);
		buf[3] = ((Flash_Address >> 8) & 0xFF);
		buf[4] = (Flash_Address & 0xFF);
		ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 5);
		if (ret < 0) {
			NVT_ERR("Sector Erase error!!(%d,%d)\n", ret, i);
			return ret;
		}
		/* Check 0xAA (Sector Erase) */
		retry = 0;
		while (1) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Sector Erase) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if (buf[1] == 0xAA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Sector Erase) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
				ret = -1;
				return ret;
			}
		}

		/* Read Status */
		retry = 0;
		while (1) {
			mdelay(5);
			buf[0] = 0x00;
			buf[1] = 0x05;
			ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Read Status error!!(%d,%d)\n", ret, i);
				return ret;
			}

			/* Check 0xAA (Read Status) */
			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Read Status) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if ((buf[1] == 0xAA) && (buf[2] == 0x00)) {
				break;
			}
			retry++;
			if (unlikely(retry > 100)) {
				NVT_ERR("Check 0xAA (Read Status) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
				ret = -1;
				return ret;
			}
		}
	}

	NVT_LOG("Erase OK \n");
	return 0;
}

int32_t nvtc_Write_Flash_ID_1732(uint8_t *u8Am_IDdata)
{

/*	uint8_t u8StrBuff[32 + 1]; */

	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = ts_1732->mmap->RW_FLASH_DATA_ADDR;
	uint32_t Flash_Address = 0;
	int32_t i = 0, j = 0, k = 0;
	uint8_t tmpvalue = 0;
/*	int32_t count = 0; */
	int32_t ret = 0;
	int32_t retry = 0;

	int32_t sizeMax = FLASH_ID_WRITE_LEN;

	int32_t u32Len = strlen(u8Am_IDdata);

	NVT_LOG("++\n");
	NVT_LOG("(u32Len,u8Am_IDdata) =[%d,%s]\n", u32Len, u8Am_IDdata);
	/* change I2C buffer index */
	buf[0] = 0xFF;
	buf[1] = XDATA_Addr >> 16;
	buf[2] = (XDATA_Addr >> 8) & 0xFF;
	ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("change I2C buffer index error!!(%d)\n", ret);
		return ret;
	}

	/*for (i = 0; i < count; i++) { */
	i = SectorSpecify;
	{
		/*Flash_Address = i * 256; */
		Flash_Address = i << 12; /* << 12 is 4096 */

		/* Write Enable */
		buf[0] = 0x00;
		buf[1] = 0x06;
		ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write Enable error!!(%d)\n", ret);
			return ret;
		}
		/* Check 0xAA (Write Enable) */
		retry = 0;
		while (1) {
			udelay(100);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Write Enable) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if (buf[1] == 0xAA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Write Enable) error!! status=0x%02X\n", buf[1]);
				ret = -1;
				return ret;
			}
		}

		/* Write Page : 256 bytes */
		j = 0;
		{
			buf[0] = (XDATA_Addr + j) & 0xFF;
			for (k = 0; k < 32; k++) {
				/*buf[1 + k] = fw_entry_1732->data[Flash_Address + j + k]; */
				buf[1 + k] = u8Am_IDdata[k];
			}
			ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_BLDR_Address, buf, 33);
			if (ret < 0) {
				NVT_ERR("Write Page error!!(%d), j=%d\n", ret, j);
				return ret;
			}
		}
		tmpvalue = (Flash_Address >> 16) + ((Flash_Address >> 8) & 0xFF) + (Flash_Address & 0xFF) + 0x00 + (sizeMax - 1);


		for (k = 0; k < sizeMax; k++)
			tmpvalue += u8Am_IDdata[k];

		tmpvalue = 255 - tmpvalue + 1;

		/* Page Program */
		buf[0] = 0x00;
		buf[1] = 0x02;
		buf[2] = ((Flash_Address >> 16) & 0xFF);
		buf[3] = ((Flash_Address >> 8) & 0xFF);
		buf[4] = (Flash_Address & 0xFF);
		buf[5] = 0x00;
	  /*buf[6] = min(fw_entry_1732->size - Flash_Address, (size_t)256) - 1; */
	    buf[6] = sizeMax - 1;
		buf[7] = tmpvalue;
		ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 8);
		if (ret < 0) {
			NVT_ERR("Page Program error!!(%d), i=%d\n", ret, i);
			return ret;
		}
		/* Check 0xAA (Page Program) */
		retry = 0;
		while (1) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Page Program error!!(%d)\n", ret);
				return ret;
			}
			if (buf[1] == 0xAA || buf[1] == 0xEA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Page Program) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
				ret = -1;
				return ret;
			}
		}
		if (buf[1] == 0xEA) {
			NVT_ERR("Page Program error!! i=%d\n", i);
			ret = -3;
			return ret;
		}

		/* Read Status */
		retry = 0;
		while (1) {
			mdelay(5);
			buf[0] = 0x00;
			buf[1] = 0x05;
			ret = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Read Status error!!(%d)\n", ret);
				return ret;
			}

			/* Check 0xAA (Read Status) */
			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Read Status) error!!(%d)\n", ret);
				return ret;
			}
			if (((buf[1] == 0xAA) && (buf[2] == 0x00)) || (buf[1] == 0xEA)) {
				break;
			}
			retry++;
			if (unlikely(retry > 100)) {
				NVT_ERR("Check 0xAA (Read Status) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
				ret = -1;
				return ret;
			}
		}
		if (buf[1] == 0xEA) {
			NVT_ERR("Page Program error!! i=%d\n", i);
			ret = -4;
			return ret;
		}

		/*NVT_LOG("Programming...%2d%%\r", ((i * 100) / count)); */
	}
	NVT_LOG("--\n");
	return 0;
}

int32_t nvtc_write_projectid_1732(uint8_t *u8Am_IDdata)
{
	uint8_t buf[64] = {0};
	int retval = 0;/*NO_ERR; */

	/*-- - stop fw-- - */
	nvt_sw_reset_idle_1732();
	/*-------------------------- - */

	/* Step 1 : initial bootloader */
	retval = Init_BootLoader_1732();
	if (retval) {
		return retval;
	}

	/* Step 2 : Resume PD */
	retval = Resume_PD_1732();
	if (retval) {
		return retval;
	}

	/* Step 3 -1 : unlock */
	buf[0] = 0x00;
	buf[1] = 0x35;
	retval = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
	if (retval < 0) {
		NVT_ERR("%s: write unlock error!!(%d)\n", __func__, retval);
		return retval;
	}
	msleep(10);

	/* Step 3 -2 : Erase */
	retval = nvtc_Erase_Flash_1732_Sector();
	if (retval) {
		NVT_ERR("%s: erase error!!(%d)\n", __func__, retval);
		return retval;
	}


	/* Step 4 : Program */
	nvtc_Write_Flash_ID_1732(u8Am_IDdata);
	/*-- - reset ic to run fw-- - */
	nvt_bootloader_reset_1732();
	/*nvt_check_fw_reset_state_1732(RESET_STATE_INIT); */
	/*---------------------------------- - */
	return retval;
}

int32_t nvtc_read_projectid_1732(uint8_t *u8Am_IDread)
{
	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = ts_1732->mmap->RW_FLASH_DATA_ADDR;
	int retval = 0;//NO_ERR;
	VTI("++\n");

	/*-- - stop fw-- - */
	nvt_sw_reset_idle_1732();
	/*-------------------------- - */

	/* Step 1 : initial bootloader */
	retval = Init_BootLoader_1732();
	if (retval) {
		return retval;
	}

	/* Step 2 : Resume PD */
	retval = Resume_PD_1732();
	if (retval) {
		return retval;
	}

	/* Step 3 : unlock */
	buf[0] = 0x00;
	buf[1] = 0x35;
	retval = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 2);
	if (retval < 0) {
		NVT_ERR("%s: write unlock error!!(%d)\n", __func__, retval);
		return retval;
	}
	msleep(10);

	/*Step 4 : Flash Read Command */
	buf[0] = 0x00;
	buf[1] = 0x03;
	buf[2] = 0x01;
/*	buf[3] = 0xF0; */
	buf[3] = 0xE0;/* / jxTest */
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x20;
	retval = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 7);
	if (retval < 0) {
		NVT_ERR("%s: write Read Command error!!(%d)\n", __func__, retval);
		return retval;
	}
	msleep(10);

	/* Check 0xAA (Read Command) */
	buf[0] = 0x00;
	buf[1] = 0x00;
	retval = CTP_I2C_READ_1732(ts_1732->client, I2C_HW_Address, buf, 2);
	if (retval < 0) {
		NVT_ERR("%s: Check 0xAA (Read Command) error!!(%d)\n", __func__, retval);
		return retval;
	}
	if (buf[1] != 0xAA) {
		NVT_ERR("%s: Check 0xAA (Read Command) error!! status=0x%02X\n", __func__, buf[1]);
		retval = -1;
		return retval;
	}
	msleep(10);

	/*Step 5 : Read Flash Data */
	buf[0] = 0xFF;
	buf[1] = XDATA_Addr >> 16;
	buf[2] = (XDATA_Addr >> 8) & 0xFF;
	retval = CTP_I2C_WRITE_1732(ts_1732->client, I2C_HW_Address, buf, 3);
	if (retval < 0) {
		NVT_ERR("%s: change index error!!(%d)\n", __func__, retval);
		return retval;
	}
	msleep(10);

	/* Read Back */
	buf[0] = XDATA_Addr & 0xFF;
	retval = CTP_I2C_READ_1732(ts_1732->client, I2C_BLDR_Address, buf, 33);
	if (retval < 0) {
		NVT_ERR("%s: Check 0xAA (Read Command) error!!(%d)\n", __func__, retval);
		return retval;
	}
	msleep(10);

	/*buf[3~3 + 32]		 = > u8A_IDread_1732 */
	strlcpy(u8Am_IDread, &buf[1], 16);/*FLASH_ID_READ_LEN); */
	NVT_LOG("u8Am_IDread=%s\n", u8Am_IDread);

	/*-- - reset ic to run fw-- - */
	nvt_bootloader_reset_1732();
	/*---------------------------------- - */

	/*[20170811, jx] */
	if (retval == 2)
		retval = 0;/*NO_ERR */

	VTI("--\n");
	return retval;
}

#define	INT_Enable						0x77 
#define	INT_Disable 					0x78 
#define	IDLE_Enable 					0x79 
#define	IDLE_Disable 					0x7A 
#define	CMD_EDGE_REJECT_VERTICAL	0x7C
#define	CMD_EDGE_REJECT_LEFT_Up  	0x7D
#define	CMD_EDGE_REJECT_RIGHT_Up 	0x7E  

#define	IDLE_SET_Disable 	0
#define	IDLE_SET_Enable 	1

#define	EDGE_REJECT_VERTICAL 		1 
#define	EDGE_REJECT_LEFT_Up 		2 
#define	EDGE_REJECT_RIGHT_Up		0 
static int8_t nvt_customizeCmd(uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	//NVT_LOG("++\n");
	mutex_lock(&ts_1732->lock);
	NVT_LOG("++ Cmd=0x%02X\n",u8Cmd);//[20171101]Modi
	for (retry = 0; retry < 20; retry++) {
		/*-- - set xdata index to EVENT BUF ADDR-- - */
		buf[0] = 0xFF;
		buf[1] = (ts_1732->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts_1732->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);

		/*-- - switch HOST_CMD-- - */
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_I2C_READ_1732(ts_1732->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry == 20)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}
	mutex_unlock(&ts_1732->lock);
	NVT_LOG("--\n");
	return ret;
}
extern uint8_t bTouchIsAwake_1732;
int8_t nvt_customizeCmd_WaitSet_1732(uint8_t u8WaitAddr, uint8_t u8WaitStatus, uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 100;/*50; */

	NVT_LOG("++parameter(0x%02X,0x%02X,0x%02X)\n", u8WaitAddr, u8WaitStatus, u8Cmd);
	mutex_lock(&ts_1732->lock);
	VTI("start");
	for (i = 0; i < retry; i++) {
		/*-- - set xdata index to EVENT BUF ADDR-- - */
		buf[0] = 0xFF;
		buf[1] = (ts_1732->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts_1732->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);

		/*-- - read fw status-- - */
		buf[0] = u8WaitAddr;
		buf[1] = 0x00;
		CTP_I2C_READ_1732(ts_1732->client, I2C_FW_Address, buf, (2 + 1));

		VTD("read %x %x ,retyr = %d", u8WaitAddr, buf[1], i);
		/*if (buf[1] == u8WaitStatus || buf[1] == 0xa3)
	  //if ((buf[1] & 0xF0) == u8WaitStatus)
			break;*/

		if (buf[1] >= u8WaitStatus) {   //[20170815,jx]set      >= u8WaitStatus , is for A->B->C mode
			break;
		}
		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("HANDSHAKING failed, i=%d, buf[1]=0x%02X,0x%02X\n", i, buf[1],buf[2]);
		i = -1;
		mutex_unlock(&ts_1732->lock);
		return i;
	} else {
		/*return 0; */
		NVT_LOG("HANDSHAKING OK    , i=%d, buf[1]=0x%02X,0x%02X\n", i, buf[1], buf[2]);
	}

	/*for (i = 0; i < retry; i++) {*/
		//---set xdata index to EVENT BUF ADDR---
		//buf[0] = 0xFF;
		//buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		//buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		//CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		/*-- - switch HOST_CMD-- - */
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 2);
		bTouchIsAwake_1732 = 0;
		msleep(35);

		/*buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		VTD("read %x ,retyr = %d", buf[1], i);
		if (buf[1] == 0x00) {
			bTouchIsAwake_1732 = 0;
			break;
		}
	}*/

	if (unlikely(i >= retry)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		i = -1;
		mutex_unlock(&ts_1732->lock);
		return i;
	}
	VTI("end");
	mutex_unlock(&ts_1732->lock);
	return 0;
}


#define	EVENTBUFFER_STATUS_OFF 	0x51 
#define	EVENTBUFFER_STATUS_DC	0x52 
#define	EVENTBUFFER_STATUS_AC	0x53 

#define	USB_PLUGOUT	(0)
#define	USB_PLUGIN	(1)

int tpd_usb_plugin_1732(int plugin)
{
	int ret = -1;

	switch (plugin) {
	case USB_PLUGOUT:
		NVT_LOG("usb plug [out] .\n");
		ret = nvt_customizeCmd(EVENTBUFFER_STATUS_OFF);
		if (ret < 0) {
			NVT_LOG("tpd_usb_plugin_1732 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_OFF);
		}
		break;

	case USB_PLUGIN:
	default:			/*default is AC		 */
		NVT_LOG("usb plug [in ] .\n");
		ret = nvt_customizeCmd(EVENTBUFFER_STATUS_AC);
		if (ret < 0) {
			NVT_LOG("tpd_usb_plugin_1732 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_AC);
		}
		break;
    }
	return ret;
}

int vivo_nt_readUdd_1732(unsigned char *udd)
{
	int32_t i32ret = 0;

	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}
#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif
	i32ret = nvtc_read_projectid_1732(udd);

	mutex_unlock(&ts_1732->lock);
	NVT_LOG("i32ret=[%d]\n", i32ret);/* / jx */
	return	i32ret;
}
int vivo_nt_writeUdd_1732(unsigned char *udd)
{
	int32_t i32ret = 0;

	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}
#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif

	nvtc_write_projectid_1732(udd);

	mutex_unlock(&ts_1732->lock);

	NVT_LOG("i32ret=[%d]\n", i32ret);/* / jx */
	return	i32ret;
}


int32_t i32chargerFlag_1732 = 0;

int vivo_nt_set_charger_bit_1732(int state)
{
	int ret = 0;

/*	if (vivoTsGetVtsData()->hasLcdShutoff) {
		VTI("lcd is shutoff, not to write charger");
		return 0;
	}*/

	VTI("write charger bit:%d", state);
	ret = tpd_usb_plugin_1732(state);
	if (ret < 0) {
		VTE("write charge bit fail");
	}
	return 0;
}

#if(1)
int vivo_nt_read_charger_bit_1732(void)
{	
	uint8_t buf[8] = {0};
	buf[0] = 0xFF;
	buf[1] = (ts_1732->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts_1732->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE_1732(ts_1732->client, I2C_FW_Address, buf, 3);
	buf[0] = 0x62;//Specify 11A62
	buf[1] = 0xFF;
	CTP_I2C_READ_1732(ts_1732->client, I2C_FW_Address, buf, 2);
	NVT_LOG("charger_bit=[%d]\n",buf[1]);///jx
	if ((buf[1] == 0x00)||(buf[1] == 0x01))
		return ((int)buf[1]);	
	else
		return (-1);		
}
#else
int vivo_nt_read_charger_bit_1732(void)
{
	int32_t i32ret = 0;
	if ((i32chargerFlag_1732 == USB_PLUGOUT) || (i32chargerFlag_1732 == USB_PLUGIN)) {
		i32ret = i32chargerFlag_1732;
	} else {
		i32ret = (-1);
	}
		NVT_LOG("i32ret=[%d]\n", i32ret);/* / jx */
	return	i32ret;
}

#endif
int vivo_nt_idle_ctl_1732(int state)
{
	int32_t i32ret = 0;
	if (state == IDLE_SET_Disable)
		i32ret = nvt_customizeCmd(IDLE_Disable);
	else if (state == IDLE_SET_Enable)
		i32ret = nvt_customizeCmd(IDLE_Enable);
	else
		i32ret = (-1);

		NVT_LOG("para=[%d],i32ret=[%d]\n", state, i32ret);/* / jx */
	return	i32ret;
}

int vivo_nt_set_edge_restain_switch_1732(int on)
{
	int32_t ret = 0;

/*	if (vivoTsGetVtsData()->hasLcdShutoff) {
		VTI("lcd is shutoff, not to set edge restain");
		return 0;
	}*/

//	if(on == EDGE_REJECT_OFF)
//		i32ret = nvt_customizeCmd(EDGE_REJECT_Disable);
	if (on == EDGE_REJECT_VERTICAL)		/*shuping */
		ret = nvt_customizeCmd(CMD_EDGE_REJECT_VERTICAL);
	else if (on == EDGE_REJECT_LEFT_Up)		/*hengping */
		ret = nvt_customizeCmd(CMD_EDGE_REJECT_LEFT_Up);
	else if (on == EDGE_REJECT_RIGHT_Up)
		ret = nvt_customizeCmd(CMD_EDGE_REJECT_RIGHT_Up);

	if (ret < 0) {
		VTI("write edge state fail");
	}
	return 0;
}

int vivo_nt_get_rawordiff_data_1732(int which, int *data)
{
	int32_t i32ret = 0;
	int x = 0, y = 0;
	VTI("enter");

	if (mutex_lock_interruptible(&ts_1732->lock)) {
		return -ERESTARTSYS;
	}
#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_1732(false);
#endif

	if (nvt_clear_fw_status_1732()) {
		NVT_ERR("EINVAL!\n");
		mutex_unlock(&ts_1732->lock);
		return -EINVAL;
	}

	nvt_change_mode_1732(TEST_MODE_2);

	if (nvt_check_fw_status_1732()) {

		NVT_ERR("EINVAL!\n");
		mutex_unlock(&ts_1732->lock);
		return -EINVAL;
	}


	if (nvt_get_fw_info_1732()) {
		NVT_ERR("EINVAL!\n");
		mutex_unlock(&ts_1732->lock);
		return -EINVAL;
	}

	if (which == 0) {/*0, raw */
		if (nvt_get_fw_pipe_1732() == 0)
			nvt_read_mdata_1732(ts_1732->mmap->RAW_PIPE0_ADDR, ts_1732->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata_1732(ts_1732->mmap->RAW_PIPE1_ADDR, ts_1732->mmap->RAW_BTN_PIPE1_ADDR);
	} else if (which == 1) {/*1 */
		if (nvt_get_fw_pipe_1732() == 0)
			nvt_read_mdata_1732(ts_1732->mmap->DIFF_PIPE0_ADDR, ts_1732->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata_1732(ts_1732->mmap->DIFF_PIPE1_ADDR, ts_1732->mmap->DIFF_BTN_PIPE1_ADDR);
	} else {
				NVT_ERR("EINVAL!\n");
		i32ret = (-EINVAL);
	}
		VTI("(x,y)=(%d,%d)", ts_1732->x_num, ts_1732->y_num);	/* / x = 18 y = 30 */

if (i32ret != (-EINVAL)) {
	/*ts_1732->y_num = ts_1732->y_num -2; */

	for (y = 0; y < ts_1732->y_num; y++) {
		printk("VIVO_TS");
		for (x = 0; x < ts_1732->x_num; x++) {
			 *(data + x * ts_1732->y_num + y) = *(xdata + y * ts_1732->x_num + x);
			printk("%6d ", *(xdata + y * ts_1732->x_num + x));
		}
		printk("\n");
	}
}
/*	if (i32ret != (-EINVAL)) { */
	/*	memcpy(data, xdata, ((ts_1732->x_num * ts_1732->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));	 */
/*	} */


	nvt_change_mode_1732(NORMAL_MODE);

	mutex_unlock(&ts_1732->lock);
	NVT_LOG("--\n");

	return i32ret;
}

#endif
