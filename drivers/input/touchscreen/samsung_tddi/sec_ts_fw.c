/* drivers/input/touchscreen/sec_ts_fw.c
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
#include <linux/vmalloc.h>

#include <linux/uaccess.h>
/*#include <asm/gpio.h>*/

#include "sec_ts.h"
#include <linux/vivo_ts_function.h>

#define SEC_TS_FW_BLK_SIZE		256

enum {
	BUILT_IN = 0,
	UMS,
	BL,
	FFU,
};

typedef struct {
	u32 signature;			/* signature */
	u32 version;			/* App img version */
	u32 totalsize;			/* total size */
	u32 param_area;			/* parameter area */
	u32 flag;			/* mode select/bootloader mode */
	u32 setting;			/* HWB settings */
	u32 checksum;			/* checksum */
	u32 boot_addr;
	u32 fw_ver;
	u32 boot_dddr2;
	u32 flash_addr[3];
	u32 chunk_num[3];
} fw_header;

typedef struct {
	u32 signature;
	u32 addr;
	u32 size;
	u32 reserved;
} fw_chunk;

#if 0
static int sec_ts_enter_fw_mode(struct sec_ts_data *ts)
{
	int ret;
	u8 fw_update_mode_passwd[] = {0x55, 0xAC};
	u8 fw_status;
	u8 id[3];

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_ENTER_FW_MODE, fw_update_mode_passwd, sizeof(fw_update_mode_passwd));
	sec_ts_delay(20);
	if (ret < 0) {
		VTI("%s: write fail, enter_fw_mode\n", __func__);
		return 0;
	}

	VTI("%s: write ok, enter_fw_mode - 0x%x 0x%x 0x%x\n", __func__, SEC_TS_CMD_ENTER_FW_MODE, fw_update_mode_passwd[0], fw_update_mode_passwd[1]);

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &fw_status, 1);
	if (ret < 0) {
		VTI("%s: read fail, read_boot_status\n", __func__);
		return 0;
	}
	if (fw_status != SEC_TS_STATUS_BOOT_MODE) {
		VTI("%s: enter fail! read_boot_status = 0x%x\n", __func__, fw_status);
		return 0;
	}

	VTI("%s: Success! read_boot_status = 0x%x\n", __func__, fw_status);

	sec_ts_delay(10);

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, id, 3);
	if (ret < 0) {
		VTI("%s: read id fail\n", __func__);
		return 0;
	}

	ts->boot_ver[0] = id[0];
	ts->boot_ver[1] = id[1];
	ts->boot_ver[2] = id[2];

	VTI("%s: read_boot_id = %02X%02X%02X\n", __func__, id[0], id[1], id[2]);

	return 1;
}
#endif

static int sec_ts_vrom_reset(struct sec_ts_data *ts)
{
	int ret;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_VROM_RESET, NULL, 0);
	if (ret < 0) {
		VTI("%s: write fail, sw_reset\n", __func__);
		return 0;
	}

	sec_ts_delay(100);

	VTI("%s: sw_reset\n", __func__);
	
	return ret;
}

int sec_ts_sw_reset(struct sec_ts_data *ts)
{
	int ret;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
	if (ret < 0) {
		VTI("%s: write fail, sw_reset\n", __func__);
		return 0;
	}

	sec_ts_delay(300);
	VTI("%s: sw_reset\n", __func__);

	return ret;
}

static void sec_ts_save_version_of_bin(struct sec_ts_data *ts, const fw_header *fw_hd)
{
	u32 *para_ver = (u32 *)fw_hd + 0x701A;

	ts->plat_data->img_version_of_bin[3] = ((fw_hd->version >> 24) & 0xff);
	ts->plat_data->img_version_of_bin[2] = ((fw_hd->version >> 16) & 0xff);
	ts->plat_data->img_version_of_bin[1] = ((fw_hd->version >> 8) & 0xff);
	ts->plat_data->img_version_of_bin[0] = ((fw_hd->version >> 0) & 0xff);

	ts->plat_data->para_version_of_bin[3] = (((*para_ver) >> 24) & 0xff);
	ts->plat_data->para_version_of_bin[2] = (((*para_ver) >> 16) & 0xff);
	ts->plat_data->para_version_of_bin[1] = (((*para_ver) >> 8) & 0xff);
	ts->plat_data->para_version_of_bin[0] = (((*para_ver) >> 0) & 0xff);

	VTI("img_ver of bin = %x.%x.%x.%x\n",
			ts->plat_data->img_version_of_bin[0],
			ts->plat_data->img_version_of_bin[1],
			ts->plat_data->img_version_of_bin[2],
			ts->plat_data->img_version_of_bin[3]);
	VTI("para_ver of bin = %x.%x.%x.%x\n",
			ts->plat_data->para_version_of_bin[0],
			ts->plat_data->para_version_of_bin[1],
			ts->plat_data->para_version_of_bin[2],
			ts->plat_data->para_version_of_bin[3]);
}

int sec_ts_save_version_of_ic(struct sec_ts_data *ts)
{
	u8 img_ver[4];
	u8 para_ver[4];
	int ret;

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, img_ver, 4);
	sec_ts_delay(5);
	if (ret < 0) {
		VTI("%s: Image version read error\n ", __func__);
		return -EIO;
	}
	VTI("IC Image version info : %x.%x.%x.%x",
					img_ver[0], img_ver[1], img_ver[2], img_ver[3]);

	ts->plat_data->img_version_of_ic[0] = img_ver[0];
	ts->plat_data->img_version_of_ic[1] = img_ver[1];
	ts->plat_data->img_version_of_ic[2] = img_ver[2];
	ts->plat_data->img_version_of_ic[3] = img_ver[3];

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_PARA_VERSION, para_ver, 4);
	if (ret < 0) {
		VTI("%s: parameter version read error\n ", __func__);
		return -EIO;
	}
	VTI("IC parameter version info : %x.%x.%x.%x ",
					 para_ver[0], para_ver[1], para_ver[2], para_ver[3]);

	ts->plat_data->para_version_of_ic[0] = para_ver[0];
	ts->plat_data->para_version_of_ic[1] = para_ver[1];
	ts->plat_data->para_version_of_ic[2] = para_ver[2];
	ts->plat_data->para_version_of_ic[3] = para_ver[3];

	return 1;
}

static int sec_ts_check_firmware_version(struct sec_ts_data *ts, const u8 *fw_info)
{
	fw_header *fw_hd;
	u8 buff[1];
	int i;
	int ret;
	/*
	 * sec_ts_check_firmware_version
	 * return value = 1 : firmware download needed,
	 * return value = 0 : skip firmware download
	 */

	fw_hd = (fw_header *)fw_info;

	sec_ts_save_version_of_bin(ts, fw_hd);

	/* irmware download if READ_BOOT_STATUS = 0x10 */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, buff, 1);
	if (ret < 0) {
		VTE("fail to read BootStatus");
		return -EIO;
	}

	if (buff[0] == SEC_TS_STATUS_BOOT_MODE) {
		VTI("CRC check err,need update firmware!ReadBootStatus = 0x%x, Firmware download Start!", buff[0]);
		return 1;
	}
	VTI("CRC check OK!");

	ret = sec_ts_save_version_of_ic(ts);
	if (ret < 0) {
		VTE("fail to read ic version");
		return -EIO;
	}

	/* ver[0] : IC version
	 * ver[1] : Project version
	 */
	if ((ts->plat_data->img_version_of_ic[0] != ts->plat_data->img_version_of_bin[0]) ||
		((ts->plat_data->img_version_of_ic[1] != ts->plat_data->img_version_of_bin[1]))) {

		VTI("do not matched version info");
		return 0;
	}

	for (i = 3; i >= 2; i--) {
		if (ts->plat_data->img_version_of_ic[i] != ts->plat_data->img_version_of_bin[i]
			|| ts->plat_data->para_version_of_ic[i] != ts->plat_data->para_version_of_bin[i]) {

			VTI("firmware base version or config version not same,need update firmware!");
			return 1;
		} else {
			continue;
		}
	}

	VTI("firmware base version and config version are same,no need update firmware!");
	return 0;
}

static u8 sec_ts_checksum(u8 *data, int offset, int size)
{
	int i;
	u8 checksum = 0;

	for (i = 0; i < size; i++)
		checksum += data[i + offset];

	return checksum;
}

static int sec_ts_flash_set_datanum(struct sec_ts_data *ts, u16 num)
{
	u8 tData[2];
	int ret;

	tData[0] = (num >> 8) & 0xFF;
	tData[1] = num & 0xFF;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_DATA_NUM, tData, 2);
	if (ret < 0)
		VTI("%s: Set datanum Fail %d\n",
				__func__, num);

	return ret;
}

static int sec_ts_flash_cs_control(struct sec_ts_data *ts, bool cs_level)
{
	u8 tData;
	int ret;

	tData = cs_level ? 1 : 0;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CS_CONTROL, &tData, 1);
	if (ret < 0)
		VTI("%s: %s control Fail!\n",
				__func__, cs_level ? "CS High" : "CS Low");
	return ret;
}

static int  sec_ts_wren(struct sec_ts_data *ts)
{
	u8 tData[2];
	int ret;

	sec_ts_flash_cs_control(ts, CS_LOW);

	sec_ts_flash_set_datanum(ts, 6);

	tData[0] = FLASH_CMD_WREN;
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_FLASH_SEND_DATA, &tData[0], 1);
	if (ret < 0)
		VTI("%s: Send WREN fail!\n", __func__);
	sec_ts_delay(10);
	sec_ts_flash_cs_control(ts, CS_HIGH);

	return ret;
}

static u8 sec_ts_rdsr(struct sec_ts_data *ts)
{
	u8 tData[2];

	sec_ts_flash_cs_control(ts, CS_LOW);

	sec_ts_flash_set_datanum(ts, 2);

	tData[0] = FLASH_CMD_RDSR;
	ts->sec_ts_i2c_write(ts, SEC_TS_CMD_FLASH_SEND_DATA, tData, 1);
	sec_ts_delay(10);
	sec_ts_flash_set_datanum(ts, 1);

	ts->sec_ts_i2c_read(ts, SEC_TS_CMD_FLASH_READ_DATA, tData, 1);

	sec_ts_flash_cs_control(ts, CS_HIGH);

	return tData[0];
}

static bool IsFlashBusy(struct sec_ts_data *ts)
{
	u8 tBuf;
	sec_ts_wren(ts);
	tBuf = sec_ts_rdsr(ts);
	if ((tBuf & SEC_TS_FLASH_WIP_MASK) == SEC_TS_FLASH_WIP_MASK)
		return true;
	else
		return false;
}

static int sec_ts_wait_for_flash_busy(struct sec_ts_data *ts)
{
	int retry_cnt = 0;
	int ret = 0;

	while (IsFlashBusy(ts)) {
		sec_ts_delay(10);

		if (retry_cnt++ > SEC_TS_WAIT_RETRY_CNT) {
			VTI("%s: Retry Cnt over!\n", __func__);
			ret = -1;
		}
	}

	return ret;
}

static int sec_ts_cmd_flash_se(struct sec_ts_data *ts, u32 flash_addr)
{
	int ret;
	u8 tBuf[5];

	if (IsFlashBusy(ts)) {
		VTI("%s: flash busy, flash_addr = %X\n", __func__, flash_addr);
		return false;
	}

	VTI("xxxxxxx1");
	sec_ts_wren(ts);
	VTI("xxxxxxx2");

	sec_ts_flash_cs_control(ts, CS_LOW);
	VTI("xxxxxxx3");

	sec_ts_flash_set_datanum(ts, 5);
	VTI("xxxxxxx4");

	tBuf[0] = SEC_TS_CMD_FLASH_SEND_DATA;
	tBuf[1] = FLASH_CMD_SE;
	tBuf[2] = (flash_addr >> 16) & 0xFF;
	tBuf[3] = (flash_addr >>  8) & 0xFF;
	tBuf[4] = (flash_addr >>  0) & 0xFF;
	ret = ts->sec_ts_i2c_write_burst(ts, tBuf, 5);
		VTI("xxxxxxx5");
	sec_ts_flash_cs_control(ts, CS_HIGH);
		VTI("xxxxxxx6");
	if (ret < 0) {
		VTI("%s: Send sector erase cmd fail!\n", __func__);
		return ret;
	}

	ret = sec_ts_wait_for_flash_busy(ts);
	if (ret < 0)
		VTI("%s: Time out! - flash busy wait\n", __func__);
	VTI("xxxxxxx7");

	return ret;
}

static int sec_ts_FlashSectorErase(struct sec_ts_data *ts, u32 sector_idx)
{
	u32 addr;
	int ret;

	//addr = sector_idx * BYTE_PER_SECTOR;
	addr = sector_idx * BYTE_PER_PAGE;

	ret = sec_ts_cmd_flash_se(ts, addr);
	if (ret < 0)
		VTI("%s : ret = %d, sector_idx = %d\n", __func__, ret, sector_idx);
	sec_ts_delay(10);

	return ret;
}

static bool sec_ts_flashpagewrite(struct sec_ts_data *ts, u32 page_idx, u8 *page_data)
{
	int ret;
	int i, j;
	u8 *tCmd;
	u8 copy_data[3 + 256];
	int copy_left = 256 + 3;
	int copy_size = 0;
	//int copy_max = ts->i2c_burstmax - 1;
	int copy_max = 200;

	copy_data[0] = (u8)((page_idx >> 8) & 0xFF);
	copy_data[1] = (u8)((page_idx >> 0) & 0xFF);
	for (i = 0; i < 256; i++)
		copy_data[2 + i] = page_data[i];
	copy_data[2 + 256] = sec_ts_checksum(copy_data, 0, 2 + 256);

	sec_ts_flash_cs_control(ts, CS_LOW);
	while (copy_left > 0) {
		int copy_cur = (copy_left > copy_max) ? copy_max : copy_left;
		tCmd = (u8 *)kzalloc(copy_cur + 1, GFP_KERNEL);
		if (copy_size == 0)
			tCmd[0] = 0xD9;
		else
			tCmd[0] = 0xDA;

		for (j = 0; j < copy_cur; j++)
			tCmd[j+1] = copy_data[copy_size + j];
		ret = ts->sec_ts_i2c_write_burst(ts, tCmd, 1+copy_cur);
		if (ret < 0) {
			VTI("flash page write,iic write burst fail");
			break;
		}
		copy_size += copy_cur;
		copy_left -= copy_cur;
		kfree(tCmd);
	}

	sec_ts_delay(10);
	VTD("write5");
	sec_ts_flash_cs_control(ts, CS_HIGH);
	VTD("write6");
	
	return ret;
}

static int sec_ts_flashwrite(struct sec_ts_data *ts, u32 mem_addr, u8 *mem_data, u32 mem_size)
{
	int ret;
	int page_idx;
	int size_left;
	int size_copy;
	u32 flash_page_size;
	u32 page_idx_start;
	u32 page_idx_end;
	u32 page_num;
	u8 page_buf[SEC_TS_FLASH_SIZE_256];

	if (0 == mem_size)
		return 0;

	flash_page_size = SEC_TS_FLASH_SIZE_256;
	page_idx_start = mem_addr / flash_page_size;
	page_idx_end = (mem_addr + mem_size - 1) / flash_page_size;
	page_num = page_idx_end - page_idx_start + 1;

	VTI("%s: page_idx_start=%X, page_idx_end=%X\n",	__func__, page_idx_start, page_idx_end);

	VTI("erase flash");
	for (page_idx = (int)((page_num - 1)/16); page_idx >= 0; page_idx--) {
		VTI("erase flash page:%d", page_idx);
		ret = sec_ts_FlashSectorErase(ts, (u32)(page_idx_start + page_idx * 16));
		if (ret < 0) {
			VTI("%s: Sector erase fail! sector_idx = %08X\n",
					__func__, page_idx_start + page_idx * 16);
			return -EIO;
		}
	}

	VTI("%s flash sector erase done\n", __func__);

	sec_ts_delay(page_num + 10);

	size_left = (int)mem_size;
	size_copy = (int)(mem_size % flash_page_size);
	if (size_copy == 0)
		size_copy = (int)flash_page_size;

	memset(page_buf, 0, SEC_TS_FLASH_SIZE_256);

	VTI("write flash");
	for (page_idx = (int)page_num - 1; page_idx >= 0; page_idx--) {
		VTI("write flash page:%d", page_idx);
		memcpy(page_buf, mem_data + (page_idx * flash_page_size), size_copy);
		ret = sec_ts_flashpagewrite(ts, (u32)(page_idx + page_idx_start), page_buf);
		if (ret < 0) {
			VTI("%s fw write failed, page_idx = %d\n", __func__, page_idx);
			goto err;
		}

		size_copy = (int)flash_page_size;
		sec_ts_delay(5);
	}

	return mem_size;
err:
	return -EIO;
}

static int sec_ts_memoryblockread(struct sec_ts_data *ts, u32 mem_addr, int mem_size, u8 *buf)
{
	int ret;
	u8 cmd[5];
	u8 *data;

	if (mem_size >= 64 * 1024) {
		VTI("%s mem size over 64K\n", __func__);
		return -EIO;
	}
	sec_ts_flash_cs_control(ts, CS_LOW);

	sec_ts_flash_set_datanum(ts, 5);


	cmd[0] = (u8)SEC_TS_CMD_FLASH_SEND_DATA;
	cmd[1] = (u8)0x03;
	cmd[2] = (u8)((mem_addr >> 16) & 0xff);
	cmd[3] = (u8)((mem_addr >> 8) & 0xff);
	cmd[4] = (u8)((mem_addr >> 0) & 0xff);

	ret = ts->sec_ts_i2c_write_burst(ts, cmd, 5);
	if (ret < 0) {
		VTI("%s send command failed, %02X\n", __func__, cmd[0]);
		return -EIO;
	}

	udelay(10);
	cmd[0] = (u8)SEC_TS_CMD_SET_DATA_NUM;
	cmd[1] = (u8)((mem_size >> 8) & 0xff);
	cmd[2] = (u8)((mem_size >> 0) & 0xff);

	ret = ts->sec_ts_i2c_write_burst(ts, cmd, 3);
	if (ret < 0) {
		VTI("%s send command failed, %02X\n", __func__, cmd[0]);
		return -EIO;
	}

	udelay(10);
	cmd[0] = (u8)SEC_TS_CMD_FLASH_READ_DATA;

	data = buf;


	ret = ts->sec_ts_i2c_read(ts, cmd[0], data, mem_size);
	if (ret < 0) {
		VTI("%s memory read failed\n", __func__);
		return -EIO;
	}

	sec_ts_flash_cs_control(ts, CS_HIGH);
/*
	ret = ts->sec_ts_i2c_write(ts, cmd[0], NULL, 0);
	ret = ts->sec_ts_i2c_read_bulk(ts, data, mem_size);
*/
	return 0;
}

static int sec_ts_memoryread(struct sec_ts_data *ts, u32 mem_addr, u8 *mem_data, u32 mem_size)
{
	int ret;
	int retry = 3;
	int read_size = 0;
	int unit_size;
	int max_size = 1024;
	int read_left = (int)mem_size;

	while (read_left > 0) {
		unit_size = (read_left > max_size) ? max_size : read_left;
		retry = 3;
		do {
			ret = sec_ts_memoryblockread(ts, mem_addr, unit_size, mem_data + read_size);
			if (read_size == 0 && mem_data[0] == 0)
				ret = -1;

			if (retry-- == 0) {
				VTI("%s fw read fail mem_addr=%08X,unit_size=%d\n",
							__func__, mem_addr, unit_size);
				return -EIO;
			}

		} while (ret < 0);

		mem_addr += unit_size;
		read_size += unit_size;
		read_left -= unit_size;
	}

	return read_size;

}

static int sec_ts_chunk_update(struct sec_ts_data *ts, u32 addr, u32 size, u8 *data)
{
	u32 fw_size;
	u32 write_size;
	u8 *mem_rb;
	int ret = 0;

	fw_size = size;

	VTI("flash data");
	write_size = sec_ts_flashwrite(ts, addr, data, fw_size);
	if (write_size != fw_size) {
		VTI("%s fw write failed\n", __func__);
		ret = -1;
		goto err_write_fail;
	}

	mem_rb = (u8 *)vzalloc(fw_size);
	if (!mem_rb) {
		VTI("%s kzalloc failed\n", __func__);
		ret = -1;
		goto err_write_fail;
	}

	
	VTI("compare flash");
if (0) {
	if (sec_ts_memoryread(ts, addr, mem_rb, fw_size) >= 0) {
		u32 ii;

		for (ii = 0; ii < fw_size; ii++) {
			//VTI("compare flash data:%d", ii);
			if (data[ii] != mem_rb[ii]) {
				VTI("fw compare fail,data[%d]:%d != mem_rb[%d]:%d", ii, data[ii], ii, mem_rb[ii]);
				//break;
			}
		}

		if (fw_size != ii) {
			VTI("fw data compar  fail");
			ret = -1;
			goto out;
		}
	} else {
		ret = -1;
		goto out;
	}
}
	VTI("%s verify done(%d)\n", __func__, ret);

out:
	vfree(mem_rb);
err_write_fail:
	sec_ts_delay(10);

	return ret;
}

int sec_ts_firmware_update(struct sec_ts_data *ts, const u8 *data, size_t size, int bl_update)
{
	int i;
	int ret;
	fw_header *fw_hd;
	//fw_chunk *fw_ch;
	u8 fw_status = 0;
	u8 *fd = (u8 *)data;
	u8 tBuff[3];
	unsigned char num_chunk = 0;
	/*int cali_info = 0;*/

	VTI("enter,fw size:%d", (int)size);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0) {
		VTI("%s: fail to write Sense_off\n", __func__);
		return ret;
	}

	fw_hd = (fw_header *)fd;
	//fd += sizeof(fw_header);
	num_chunk = fw_hd->chunk_num[0] & 0xff;
	
	VTI("num_chunk : %d", num_chunk);

	for (i = 0; i < num_chunk; i++) {
		ret = sec_ts_chunk_update(ts, 0x00, size, fd);
		if (ret < 0) {
			//printk(KERN_ERR "%s: firmware chunk write failed, addr=%08X, size = %d\n", __func__, fw_ch->addr, fw_ch->size);
			return -EIO;
		}
	}

	if (ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &fw_status, 1) < 0) {
		VTI("%s: read fail, read_boot_status = 0x%x\n", __func__, fw_status);
		return -EIO;
	}

	if (fw_status != SEC_TS_STATUS_APP_MODE) {
			VTI("%s: fw update sequence done, BUT read_boot_status = 0x%x\n", __func__, fw_status);
		if (sec_ts_vrom_reset(ts) < 0) {
			VTI("%s: vrom reset fail\n", __func__);
			return -EIO;
		}
	} else {
		sec_ts_sw_reset(ts);
	}

	if (!bl_update) {
if (0) {	
#if defined(CALIBRATION_BY_FACTORY)
		if ((ts->cal_count == 0) || (ts->cal_count == 0xFF)) {
			VTI("%s: RUN OFFSET CALIBRATION(%d)\n", __func__, ts->cal_count);

			ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SEC);
			if (ret < 0)
				VTI("%s:  fail to write OFFSET CAL SEC!\n", __func__);
		} else {
			VTI("%s: DO NOT CALIBRATION(%d)\n", __func__, ts->cal_count);
		}

		/* always calibration after fw update */
		VTI("%s: RUN OFFSET CALIBRATION\n", __func__);
		cali_info = sec_ts_read_calibration_report(ts);
		if (cali_info == 0x00 || cali_info == 0xFF) {
			VTI("The chip has not been calibration before this boot!!!!!!");
			ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SEC);
			if (ret < 0)
				VTI("%s: fail to write OFFSET CAL SEC!\n", __func__);
		} else {
			VTI("chip has been calibration");
		}
#endif
}

		/* Sense_on */
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
		if (ret < 0) {
			VTI("%s: write fail, Sense_on\n", __func__);
			return 0;
		}

		if (ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &fw_status, 1) < 0) {
			VTI("%s: read fail, read_boot_status = 0x%x\n", __func__, fw_status);
			return -EIO;
		}

		if (fw_status != SEC_TS_STATUS_APP_MODE) {
			VTI("%s: fw update sequence done, BUT read_boot_status = 0x%x\n", __func__, fw_status);
			return -EIO;
		}

		VTI("%s: fw update Success! read_boot_status = 0x%x\n", __func__, fw_status);

		return 1;
	} else {

		if (ts->sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, tBuff, 3) < 0) {
			VTI("%s: read device id fail after bl fw download\n", __func__);
			return -EIO;
		}

		if (tBuff[0] == 0xA0) {
			VTI("%s: bl fw download success - device id = %02X\n", __func__, tBuff[0]);
			return 1;
		} else {
			VTI("%s: bl fw id does not match - device id = %02X\n", __func__, tBuff[0]);
			return -EINVAL;
		}
	}


	return 0;
}

int sec_ts_firmware_update_bl(struct sec_ts_data *ts)
{
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int result = -1;

	disable_irq(ts->client->irq);

	snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", SEC_TS_DEFAULT_BL_NAME);

	VTI("%s: initial bl update %s\n", __func__, fw_path);

	/* Loading Firmware------------------------------------------ */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		VTI("%s: bt is not available\n", __func__);
		goto err_request_fw;
	}
	VTI("%s: request bt done! size = %d\n", __func__, (int)fw_entry->size);

	result = sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 1);

err_request_fw:
	release_firmware(fw_entry);
	enable_irq(ts->client->irq);

	return result;
}

int sec_ts_bl_update(struct sec_ts_data *ts)
{
	int ret;
	u8 tCmd[5] = { 0xDE, 0xAD, 0xBE, 0xEF };
	u8 tBuff[3];

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_READ_BL_UPDATE_STATUS, tCmd, 4);
	if (ret < 0) {
		VTI("%s: bl update command send fail!\n", __func__);
		goto err;
	}
	sec_ts_delay(10);

	do {
		ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BL_UPDATE_STATUS, tBuff, 1);
		if (ret < 0) {
			VTI("%s: read bl update status fail!\n", __func__);
			goto err;
		}
		sec_ts_delay(2);

	} while (tBuff[0] == 0x1);

	tCmd[0] = 0x55;
	tCmd[1] = 0xAC;
	ret = ts->sec_ts_i2c_write(ts, 0x57, tCmd, 2);
	if (ret < 0) {
		VTI("%s: write passwd fail!\n", __func__);
		goto err;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, tBuff, 3);

	if (tBuff[0]  == 0xB4) {
		VTI("%s: bl update completed!\n", __func__);
		ret = 1;
	} else {
		VTI("%s: bl updated but bl version not matching, ver=%02X\n", __func__, tBuff[0]);
		goto err;
	}

	return ret;
err:
	return -EIO;
}

int sec_ts_firmware_update_on_probe(struct sec_ts_data *ts)
{
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int result = -1;

	if (ts->plat_data->bringup) {
		VTI("%s: bringup. do not update\n", __func__);
		return 0;
	}

	disable_irq(ts->client->irq);

	if (!ts->plat_data->firmware_name)
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", SEC_TS_DEFAULT_FW_NAME);
	else
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", ts->plat_data->firmware_name);

	/* read cal status */
	ts->cal_status = sec_ts_read_calibration_report(ts);

	VTI("%s: initial firmware update %s, cal:%X\n",
					__func__, fw_path, ts->cal_status);

	/* Loading Firmware */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		VTI("%s: firmware is not available\n", __func__);
		goto err_request_fw;
	}
	VTI("%s: request firmware done! size = %d\n", __func__, (int)fw_entry->size);

	result = sec_ts_check_firmware_version(ts, fw_entry->data);

	if (result <= 0)
		goto err_request_fw;

	//ts->cal_count = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_CAL_COUNT);
	if (sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 0) < 0)
		result = -1;
	else
		result = 0;

	sec_ts_save_version_of_ic(ts);

err_request_fw:
	release_firmware(fw_entry);
	enable_irq(ts->client->irq);
	return result;
}
extern int bbk_slsi_erase_vivo_cal(u8 val);
extern int bbk_slsi_get_vivo_calibration_status(int *);
static int sec_ts_force_calibration_on_probe(struct sec_ts_data *ts)
{
	int cali_info = 0;
	int ret = 0;
	ret = bbk_slsi_get_vivo_calibration_status(&cali_info);
	if (ret < 0) {
		VTI("get calibration status fail");
		goto END;
	}
	/*0xA5 is for compatible the old software version*/
	if (cali_info != 0xA5 && cali_info != 0xB1 && cali_info != 0xA1) {
		VTI("The chip has not been force calibration or probe calibration before this boot!!!!!!");
		ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SEC);
		if (ret < 0) {
			VTI("fail to write OFFSET CAL SEC!");
			msleep(200);
			bbk_slsi_erase_vivo_cal(0x00);
			msleep(200);
			goto END;
		}else{
			msleep(200);
			bbk_slsi_erase_vivo_cal(0xB1);
			msleep(200);
			ret = bbk_slsi_get_vivo_calibration_status(&cali_info);
			if ((ret < 0) || (cali_info != 0xB1)) {
				ret = -1;
				VTI("bbk_slsi_erase_vivo_cal 0xB1 fail");
				goto END;
			}
			VTI("force calibration on probe successful!!!");
		}
	} else {
		VTI("chip has been calibration before!");
	}
END:
	return ret;

}

#define EXCEPTION_THRESHOLD	50
static int sec_ts_rawdata_exception_calibration(struct sec_ts_data *ts)
{
	int i;
	int ret = 0;
	unsigned char buf[6];
	unsigned short count_less_than_800 = 0;
	unsigned short count_larger_than_1500 = 0;
	unsigned short count_ignore = 0;
	for (i = 0; i < 5; i++) {
		ret = sec_ts_sw_reset(ts);
		msleep(300);
		buf[0] = 0xdd;
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_READ_FW_RAWDATA_EXCEPTION, buf, 1);
		if (ret < 0) {
			break;
		}
		msleep(200);
		ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_FW_RAWDATA_EXCEPTION, buf, sizeof(buf));
		if (ret < 0) {
			break;
		}
		count_less_than_800 = ((buf[0] & 0xff) << 8) | (buf[1] & 0xff);
		count_larger_than_1500 = ((buf[2] & 0xff) << 8) | (buf[3] & 0xff);
		count_ignore = ((buf[4] & 0xff) << 8) | (buf[5] & 0xff);
		VTI("<800[%d] >1500[%d] ignore[%d]", count_less_than_800, count_larger_than_1500, count_ignore);
		if (count_less_than_800 < count_ignore) {
			ret = -1;
			break;
		}
		if ((count_less_than_800 - count_ignore) < EXCEPTION_THRESHOLD) {
			ret = 1;
			break;
		}
		ret = 0;
	}
	return ret;
}

int sec_ts_firmware_auto_update_on_probe(struct sec_ts_data *ts)
{
	unsigned char *fw_data = NULL;
	int fw_size = 0;
	int result = -1;
	int retry = 0;
	bool isFwUpdated = true;
	int ret = 0;
	int cali_info = 0;
	int lcmId = 0;

	if (ts->plat_data->bringup) {
		VTI("%s: bringup. do not update\n", __func__);
		return 0;
	}

	/* Loading Firmware */
	fw_data = vivoTsGetFw(VTS_FW_TYPE_FW, &fw_size);
	if (fw_data == NULL || fw_size == 0) {
		VTI("get fw fail in probe");
		return -EIO;
	}
	VTI("fw_size is 0x%x", fw_size);

	/* read cal status */
	ts->cal_status = sec_ts_read_calibration_report(ts);

	result = sec_ts_check_firmware_version(ts, fw_data);
	if (result <= 0) {
		isFwUpdated = false;
		goto err_request_fw;
	}
/*
	result = bbk_slsi_get_vivo_calibration_status(&cali_info);
	if(result < 0) {
		VTI("get calibration status fail");
		goto err_request_fw;
	}
	if (cali_info != 0xA5 && cali_info != 0xB1 && cali_info != 0xA1) {
		VTI("The first reboot erase calibration data");
		sec_ts_FlashSectorErase(ts, 480);
  		sec_ts_FlashSectorErase(ts, 481);
	}
*/	
	//ts->cal_count = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_CAL_COUNT);
	if (sec_ts_firmware_update(ts, fw_data, fw_size, 0) < 0)
		result = -1;
	else
		result = 0;

	sec_ts_save_version_of_ic(ts);

err_request_fw:
	while(sec_ts_force_calibration_on_probe(ts) < 0) {
		retry++;
		if (retry >= 3) {
			VTI("calibration still fail after 3 times");
			return result;
		}
	}
	lcmId = vivoTsGetLcmId();
	if (lcmId == 0) {
		if (isFwUpdated == false) {
			ret = sec_ts_rawdata_exception_calibration(ts);
			if (ret == 0) {
				VTI("We need to re cal because of the bad rawdata!");
				ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SEC);
				if (ret < 0) {
					VTI("fail to write OFFSET CAL SEC!");
					msleep(200);
					bbk_slsi_erase_vivo_cal(0x00);
					msleep(200);
					ts->exception_recal = -1;
				} else {
					msleep(200);
					bbk_slsi_erase_vivo_cal(0xB1);
					msleep(200);
					ret = bbk_slsi_get_vivo_calibration_status(&cali_info);
					if ((ret < 0) || (cali_info != 0xB1)) {
						ret = -1;
						VTI("bbk_slsi_erase_vivo_cal 0xB1 fail");
						ts->exception_recal = -1;
					} else {
						VTI("force calibration on probe successful!!!");
						ts->exception_recal = 1;
					}
				}
			}
		}
	}
	return result;
}


static int sec_ts_load_fw_from_bin(struct sec_ts_data *ts)
{
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int error = 0;

	if (ts->client->irq)
		disable_irq(ts->client->irq);

	if (!ts->plat_data->firmware_name)
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", SEC_TS_DEFAULT_FW_NAME);
	else
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", ts->plat_data->firmware_name);

	VTI("%s: initial firmware update  %s\n", __func__, fw_path);

	/* Loading Firmware */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		VTI("%s: firmware is not available\n", __func__);
		error = -1;
		goto err_request_fw;
	}
	VTI("%s: request firmware done! size = %d\n", __func__, (int)fw_entry->size);

	if (sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 0) < 0)
		error = -1;
	else
		error = 0;

	sec_ts_save_version_of_ic(ts);

err_request_fw:
	release_firmware(fw_entry);
	if (ts->client->irq)
		enable_irq(ts->client->irq);

	return error;
}

static int sec_ts_load_fw_from_ums(struct sec_ts_data *ts)
{
	fw_header *fw_hd;
	struct file *fp;
	mm_segment_t old_fs;
	long fw_size, nread;
	int error = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(SEC_TS_DEFAULT_UMS_FW, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		VTI("%s: failed to open %s.\n", __func__,
				SEC_TS_DEFAULT_UMS_FW);
		error = -ENOENT;
		goto open_err;
	}

	fw_size = fp->f_path.dentry->d_inode->i_size;

	if (0 < fw_size) {
		unsigned char *fw_data;

		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data,
				fw_size, &fp->f_pos);

		VTI("%s: start, file path %s, size %ld Bytes\n",
				__func__, SEC_TS_DEFAULT_UMS_FW, fw_size);

		if (nread != fw_size) {
			VTI("%s: failed to read firmware file, nread %ld Bytes\n",
					__func__, nread);
			error = -EIO;
		} else {
			fw_hd = (fw_header *)fw_data;
/*
			sec_ts_check_firmware_version(ts, fw_data);
*/
			VTI("%s: firmware version %08X\n ", __func__, fw_hd->fw_ver);

			if (ts->client->irq)
				disable_irq(ts->client->irq);
			if (sec_ts_firmware_update(ts, fw_data, fw_size, 0) < 0)
				goto done;

			sec_ts_save_version_of_ic(ts);
		}

		if (error < 0)
			VTI("%s: failed update firmware\n",
					__func__);

done:
		if (ts->client->irq)
			enable_irq(ts->client->irq);
		kfree(fw_data);
	}

	filp_close(fp, NULL);

open_err:
	set_fs(old_fs);
	return error;
}

static int sec_ts_load_fw_from_ffu(struct sec_ts_data *ts)
{
	const struct firmware *fw_entry;
	const char *fw_path = SEC_TS_DEFAULT_FFU_FW;
	int result = -1;

	if (!fw_path) {
		VTI("%s: Firmware name is not defined\n",
			__func__);
		return -EINVAL;
	}

	disable_irq(ts->client->irq);

	VTI("%s: Load firmware : %s\n", __func__, fw_path);

	/* Loading Firmware */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		VTI("%s: firmware is not available\n", __func__);
		goto err_request_fw;
	}
	VTI("%s: request firmware done! size = %d\n", __func__, (int)fw_entry->size);

	sec_ts_check_firmware_version(ts, fw_entry->data);

	if (sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 0) < 0)
		result = -1;
	else
		result = 0;

	sec_ts_save_version_of_ic(ts);

err_request_fw:
	release_firmware(fw_entry);
	enable_irq(ts->client->irq);
	return result;
}

int sec_ts_firmware_update_on_hidden_menu(struct sec_ts_data *ts, int update_type)
{
	int ret = 0;

	/* Factory cmd for firmware update
	 * argument represent what is source of firmware like below.
	 *
	 * 0 : [BUILT_IN] Getting firmware which is for user.
	 * 1 : [UMS] Getting firmware from sd card.
	 * 2 : none
	 * 3 : [FFU] Getting firmware from air.
	 */

	switch (update_type) {
	case BUILT_IN:
		ret = sec_ts_load_fw_from_bin(ts);
		break;
	case UMS:
		ret = sec_ts_load_fw_from_ums(ts);
		break;
	case FFU:
		ret = sec_ts_load_fw_from_ffu(ts);
		break;
	case BL:
		ret = sec_ts_firmware_update_bl(ts);
		if (ret < 0) {
			break;
		} else if (!ret) {
			ret = sec_ts_firmware_update_on_probe(ts);
			break;
		} else {
			ret = sec_ts_bl_update(ts);
			if (ret < 0)
				break;
			ret = sec_ts_firmware_update_on_probe(ts);
			if (ret < 0)
				break;
		}
		break;
	default:
		VTI("%s: Not support command[%d]\n",
				__func__, update_type);
		break;
	}

	//sec_ts_check_custom_library(ts);

	return ret;
}

extern struct sec_ts_data *g_ts_data;
extern int bbk_slsi_start_force_calibration(void);
extern int bbk_slsi_irq_enable(struct sec_ts_data *ts, bool enable);

int samsung_at_sensor_test(char *buf, int at_sensor_test_cmd , void *pdata, int tmp)
{

	int count = 0, ret = 0;
	int cali_satus = 0;

	vivoTsReleasePointsAndKeys();
	
	if (VTS_SENSOR_TEST_CALIBRATION == at_sensor_test_cmd) {/*calibrate*/
		/*show tag*/
		ret = bbk_slsi_get_vivo_calibration_status(&cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return 0;
		}

		/*erase tag*/
		VTI("erase cali tag");
		ret = bbk_slsi_erase_vivo_cal(0);
		msleep(200);
		if (ret < 0) {
			VTI("erase tag fail");
			return 0;
		}

		/*show tag*/
		ret = bbk_slsi_get_vivo_calibration_status(&cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return 0;
		}

		msleep(100);
		/*cali*/
		ret = bbk_slsi_start_force_calibration();
		if (ret < 0) {
			VTI("cali fail");
			msleep(200);
			bbk_slsi_erase_vivo_cal(0x00);
			msleep(200);
			return 0;
		} else {
			msleep(200);
			bbk_slsi_erase_vivo_cal(0xA1);
		}

		msleep(200);
		/*show tag*/
		ret = bbk_slsi_get_vivo_calibration_status(&cali_satus);
		if (ret >= 0 && (cali_satus == 0xa1)) {
			count = snprintf(buf, 1024, "Pass\nCalibration pass.\n");
			VTI("Calibration pass.");
		} else {
			count = snprintf(buf, 1024, "Failed\nCalibration fail.\n");
			VTI("Calibration fail.");
		}
	} else if (VTS_SENSOR_TEST_CHECK == at_sensor_test_cmd) {
			VTI("MP Flag not set!");
	} else {
		VTI("no cmd ");
	}

	sec_ts_sw_reset(g_ts_data);

	VTI("at_sensor_test result:%s\n", buf);
	at_sensor_test_cmd = 0;

	vivoTsReleasePointsAndKeys();
	
	return count;
}

int sec_ts_execute_force_calibration(struct sec_ts_data *ts, int cal_mode)
{
	int rc = -1;

	VTI("%s\n", __func__);

	disable_irq(ts->client->irq);
	rc = sec_clear_event_stack(ts);
	if (rc < 0) {
		VTI("sec_clear_event_stack fail");
	}
//	rc = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
//	if (rc < 0) {
//		VTI("sec_ts_fix_tmode fail");
//	}

	if (ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CALIBRATION_AMBIENT, NULL, 0) < 0) {
		VTI("%s: Write Cal commend failed!\n", __func__);
		enable_irq(ts->client->irq);
		return rc;
	}

	sec_ts_delay(1000);

	rc = sec_ts_wait_for_ready(ts, SEC_TS_ACK_OFFSET_CAL_DONE);
	
	enable_irq(ts->client->irq);
	return rc;
}
EXPORT_SYMBOL(sec_ts_firmware_update_on_hidden_menu);
