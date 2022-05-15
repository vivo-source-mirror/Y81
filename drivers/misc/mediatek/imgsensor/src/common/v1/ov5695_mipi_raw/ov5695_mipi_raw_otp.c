#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>


#include "../imgsensor_i2c.h"
#include "ov5695mipiraw_Sensor.h"

#define PFX "OV5695_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_SUB_OTP_DATA_SIZE 0xF /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_SUB_EEPROM_WRITE_ID 0x20
#define VIVO_SUB_I2C_SPEED 400
/*#define VIVO_SUB_MAX_OFFSET 0x1bff*/
/*#define VIVO_SUB_VENDOR_SUNNY 0x01*/
/*#define VIVO_SUB_VENDOR_TRULY 0x02*/
#define VIVO_SUB_VENDOR_QTECH 0x05
/*#define VIVO_SUB_VENDOR_OFILM 0x09*/
/*#define VIVO_SUB_VENDOR_LENS_ID 0x07*/
/*#define VIVO_SUB_VENDOR_VCM_ID 0x00*/
/*#define VIVO_SUB_VENDOR_DRIVERIC_ID 0x00*/
#define VIVO_SUB_VENDOR_PLATFORM_ID 0x01
#define VIVO_SUB_VENDOR_PLATFORM_ID_1 0x11
#define VIVO_SUB_VENDOR_PLATFORM_ID_2 0x21
#define VIVO_SUB_VENDOR_PLATFORM_ID_3 0x31

static unsigned char vivo_sub_otp_data[VIVO_SUB_OTP_DATA_SIZE];
unsigned char vivo_sub_otp_data_ov5695[7];
static unsigned int group_valid_addr = 0x700C - 0x700C;
/*static unsigned const int ModuleInfo_sub_flag = 0x150c;   0x190d - 0x0401*/
static unsigned int ModuleInfo_sub_addr;
/*static unsigned int ModuleInfo_sub_checksum_addr;*/
/*static unsigned const int Awb_sub_flag = 0;*/
static unsigned int Awb_sub_addr;
static unsigned int Awb_sub_checksum_addr;

/*static unsigned const int Sn_sub_flag = 0x1528;  0x1929 - 0x0401*/
/*static unsigned int Sn_sub_addr;*/
/*static unsigned int Sn_sub_checksum_addr;*/

#if 0
unsigned const int Lsc_sub_flag = 0x04b2 - 0x0401;
unsigned int Lsc_sub_addr;
unsigned int Lsc_sub_checksum_addr;
#endif

static int checksum;
otp_error_code_t SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_sub_ov5695[13];

static void write_cmos_sensor_16_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	kdSetI2CSpeed(VIVO_SUB_I2C_SPEED); 
	iWriteRegI2C(pu_send_cmd, 3, VIVO_SUB_EEPROM_WRITE_ID);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	
	kdSetI2CSpeed(400); 

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, VIVO_SUB_EEPROM_WRITE_ID);

	return get_byte;
}

static int ov5695_otp_read_setting_init_continuous_read(kal_uint16 addr)
{
	int i;
	write_cmos_sensor_16_8(0x0100, 0x01); /*stream on*/
	write_cmos_sensor_16_8(0x5001, 0xC2); 
	write_cmos_sensor_16_8(0x3d84, 0xC0); /*read OTP into buffer*/
	write_cmos_sensor_16_8(0x3d88, 0x70); /*OTP start address*/
	write_cmos_sensor_16_8(0x3d89, 0x0C); /*OTP start address*/
	write_cmos_sensor_16_8(0x3d8A, 0x70); /*OTP end address*/
	write_cmos_sensor_16_8(0x3d8B, 0x1A); /*OTP end address*/
	write_cmos_sensor_16_8(0x3d81, 0x01);
	mdelay(10);
	for (i = addr; i <= 0x701A; i++) {
		vivo_sub_otp_data[i-addr] = read_cmos_sensor_16_8(i);
		/*LOG_INF("vivo_sub_otp_data[0x%x] = 0x%x\n", i, vivo_sub_otp_data[i-addr]);*/
	}
	
	for (i = 0x700C; i <= 0x701A; i++) {
		write_cmos_sensor_16_8(i, 0x00); /*clear OTP buffer, recommended use continuous write to accelarate*/
	}
	
	write_cmos_sensor_16_8(0x5001, 0xCA);
	write_cmos_sensor_16_8(0x0100, 0x00); /*stream off*/
	return 0;
}

/* extern unsigned int is_atboot;guojunzheng add*/
int vivo_sub_otp_read_ov5695(void)
{
	int i = 0;
	int offset = 0x700C;
	int check_if_ModuleInfo_group_valid = 1;
	int check_if_AWB_group_valid = 1;
	/*int check_if_SN_group_valid = 1;*/
	int check_if_group_valid = 0;

	/* This operation takes a long time, we need to skip it. guojunzheng add begin
	if (is_atboot == 1) {
		LOG_INF("AT mode skip vivo_sub_otp_read\n");
		return 1;
	}
	 guojunzheng add end */

	ov5695_otp_read_setting_init_continuous_read(offset);
	LOG_INF("read_ov5695_data,group_valid = 0x%x\n", vivo_sub_otp_data[group_valid_addr]);
	LOG_INF("read_ov5695_data\n");
#if 0	
	for (i = 0; i < VIVO_SUB_OTP_DATA_SIZE; i++) {
			LOG_INF("read_ov5695_data[0x%x][%d] = 0x%x\n", (0x700C+i),i, vivo_sub_otp_data[i]);
	}
#endif
	if (0x01 == ((vivo_sub_otp_data[group_valid_addr]&0xC0)>>6)) { /*Group 1*/
		ModuleInfo_sub_addr = 0x700D - 0x700C;
		/*ModuleInfo_sub_checksum_addr = ModuleInfo_sub_addr + 8;*/
		LOG_INF("ModuleInfo Group 1 !!!\n");
	} else if (0x01 == ((vivo_sub_otp_data[group_valid_addr]&0x30)>>4)) { /*Group 2*/
		ModuleInfo_sub_addr = 0x7014 - 0x700C;
		/*ModuleInfo_sub_checksum_addr = ModuleInfo_sub_addr + 8;*/
		LOG_INF("ModuleInfo Group 2 !!!\n");
	} else {
		check_if_ModuleInfo_group_valid = 0;
		LOG_INF("ModuleInfo flag error!!!    ModuleInfo flag :%d\n", vivo_sub_otp_data[group_valid_addr]);
	}

	if (0x01 == ((vivo_sub_otp_data[group_valid_addr]&0xC0)>>6)) { /*Group 1*/
		Awb_sub_addr = 0x700D - 0x700C;
		Awb_sub_checksum_addr = Awb_sub_addr + 6;
		LOG_INF("AWB Group 1 !!!\n");
	} else if (0x01 == ((vivo_sub_otp_data[group_valid_addr]&0x30)>>4)) { /*Group 2*/
		Awb_sub_addr = 0x7014 - 0x700C;
		Awb_sub_checksum_addr = Awb_sub_addr + 6;
		LOG_INF("AWB Group 2 !!!\n");
	} else {
		check_if_AWB_group_valid = 0;
		LOG_INF("AWB flag error!!!    AWB flag :%d\n", vivo_sub_otp_data[group_valid_addr]);
	}

	#if 0
	if (0x01 == vivo_sub_otp_data[Sn_sub_flag]) { /*Group 1*/
		Sn_sub_addr = 0x192A - 0x0401;
		Sn_sub_checksum_addr = Sn_sub_addr + 25;
		LOG_INF("Sn Group 1 !!!\n");
	} else if (0x13 == vivo_sub_otp_data[Sn_sub_flag]) { /*Group 2*/
		Sn_sub_addr = 0x1944 - 0x0401;
		Sn_sub_checksum_addr = Sn_sub_addr + 25;
		LOG_INF("Sn Group 2 !!!\n");
	} else if (0x37 == vivo_sub_otp_data[Sn_sub_flag]) { /*Group 3*/
		Sn_sub_addr = 0x195E - 0x0401;
		Sn_sub_checksum_addr = Sn_sub_addr + 25;
		LOG_INF("Sn Group 3 !!!\n");
	} else {
		check_if_SN_group_valid = 0;
		LOG_INF("Sn flag error!!!    Sn  flag :%d\n", vivo_sub_otp_data[Sn_sub_flag]);
	}
	#endif
	
	LOG_INF("read_ov5695_data,moduleinfo_addr=0x%x,awb_addr= 0x%x\n", ModuleInfo_sub_addr, Awb_sub_addr);

	check_if_group_valid = check_if_AWB_group_valid & check_if_ModuleInfo_group_valid /*& check_if_SN_group_valid*/;
	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if ((VIVO_SUB_VENDOR_QTECH != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x00])) {
			SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Module ID error!!!    otp_error_code:%d\n", SUB_OTP_ERROR_CODE_OV5695);
			return 0;
		} else if ((VIVO_SUB_VENDOR_PLATFORM_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x01]) && (VIVO_SUB_VENDOR_PLATFORM_ID_1 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x01]) &&(VIVO_SUB_VENDOR_PLATFORM_ID_2 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x01]) && (VIVO_SUB_VENDOR_PLATFORM_ID_3 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x01])/*(VIVO_SUB_VENDOR_LENS_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x04]) || (VIVO_SUB_VENDOR_VCM_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x06]) || (VIVO_SUB_VENDOR_DRIVERIC_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x07])*/) {
			SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code =%d\n", SUB_OTP_ERROR_CODE_OV5695);
			return 0;
		} /*else if ((0xff != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x08]) || (0x00 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x09]) || (0x0b != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0a]) || (0x01 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0b])) {
			SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", vivo_sub_otp_data[ModuleInfo_sub_addr + 0x08], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x09], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0a], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0b]);
			return 0;
		}*/
		#if 0
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_sub_addr; i < ModuleInfo_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[ModuleInfo_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("ModuleInfo_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_OV5695);
			return 0;
			}
		#endif
		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_sub_addr; i < Awb_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Awb_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_OV5695);
			return 0;
		}
		
		#if 0
		/****SN_checksum****/
		checksum = 0;
		for (i = Sn_sub_addr; i < Sn_sub_checksum_addr; i++) {
			/*LOG_INF("vivo_sub_otp_data[0x%x] = 0x%x\n", i, vivo_sub_otp_data[i]);*/
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Sn_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_OV5695);
			return 0;
		}
		#endif
		
		sn_inf_sub_ov5695[0] = 0x00;
		
		#if 0
		for (i = 0; i < 12; i++) {
			sn_inf_sub_ov5695[i +1] = (MUINT32)vivo_sub_otp_data[i + Sn_sub_addr];
			LOG_INF("sn_inf_sub_ov5695[%d] = 0x%x, vivo_sub_otp_data[0x%x] = 0x%x\n", i+1  , sn_inf_sub_ov5695[i+1],  i +Sn_sub_addr, vivo_sub_otp_data[i + Sn_sub_addr]);
		}
		#endif
		#if 0
		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_sub_addr; i < Lsc_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Lsc_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_OV5695);
			return 0;
		}
		#endif
		/*memcpy(vivo_sub_otp_data_ov5695, &vivo_sub_otp_data[ModuleInfo_sub_addr], 8);*/
		/*memcpy(&vivo_sub_otp_data_ov5695[8], &vivo_sub_otp_data[0], 1);*/
		/*memcpy(&vivo_sub_otp_data_ov5695[9], &vivo_sub_otp_data[Awb_sub_addr], 12);*/
		
		memcpy(&vivo_sub_otp_data_ov5695, &vivo_sub_otp_data[group_valid_addr], 1);
		memcpy(&vivo_sub_otp_data_ov5695[1], &vivo_sub_otp_data[Awb_sub_addr], 6);
		/* add for print ov5695 data*/
/*		
		for (i = 0; i < 7; i++) {
			LOG_INF("vivo_sub_otp_data_ov5695[%d] = 0x%x\n", i, vivo_sub_otp_data_ov5695[i]);
		}
*/		
		return 1;
	} else {
		SUB_OTP_ERROR_CODE_OV5695 = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE_OV5695);
		return 0;
	}
}
/*vivo cfx add end*/
