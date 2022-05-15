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
#include "hi556mipiraw_Sensor.h"

#define PFX "hi556_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_SUB_OTP_DATA_SIZE 0x1600 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_SUB_EEPROM_WRITE_ID 0x40
#define VIVO_SUB_I2C_SPEED 400
#define VIVO_SUB_MAX_OFFSET 0x1bff
/*#define VIVO_SUB_VENDOR_SUNNY 0x01*/
/*#define VIVO_SUB_VENDOR_TRULY 0x02*/
#define VIVO_SUB_VENDOR_QTECH 0x05
/*#define VIVO_SUB_VENDOR_OFILM 0x09*/
#define VIVO_SUB_VENDOR_LENS_ID 0x07
#define VIVO_SUB_VENDOR_PLATFROM_ID 0x01
#define VIVO_SUB_VENDOR_VCM_ID 0x00
#define VIVO_SUB_VENDOR_DRIVERIC_ID 0x00
/*#define VIVO_SUB_VENDOR_PLATFORM_ID 0x00*/

static unsigned char vivo_sub_otp_data[VIVO_SUB_OTP_DATA_SIZE];
unsigned char vivo_sub_otp_data_hi556[1880];
unsigned const int ModuleInfo_sub_flag = 0x150c;   /*0x190d - 0x0401*/
unsigned int ModuleInfo_sub_addr;
unsigned int ModuleInfo_sub_checksum_addr;
unsigned const int Awb_sub_flag = 0;
unsigned int Awb_sub_addr;
unsigned int Awb_sub_checksum_addr;

unsigned const int Sn_sub_flag = 0x1528;  /*0x1929 - 0x0401*/
unsigned int Sn_sub_addr;
unsigned int Sn_sub_checksum_addr;

#if 0
unsigned const int Lsc_sub_flag = 0x04b2 - 0x0401;
unsigned int Lsc_sub_addr;
unsigned int Lsc_sub_checksum_addr;
#endif

static int checksum;
otp_error_code_t SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	kdSetI2CSpeed(VIVO_SUB_I2C_SPEED); 
	iWriteRegI2C(pu_send_cmd, 3, VIVO_SUB_EEPROM_WRITE_ID);
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	
	kdSetI2CSpeed(400); 

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, VIVO_SUB_EEPROM_WRITE_ID);

	return get_byte;
}

static int HI556_otp_read_setting_init_continuous_read(kal_uint16 addr)
{
	int i;
	write_cmos_sensor_8(0x0a02, 0x01); /*Fast sleep on*/
	write_cmos_sensor_8(0x0a00, 0x00); /*stand by on*/
	mdelay(10);
	write_cmos_sensor_8(0x0f02, 0x00); /*pll disable*/
	write_cmos_sensor_8(0x011a, 0x01); /*CP TRIM_H*/
	write_cmos_sensor_8(0x011b, 0x09); /*IPGM TRIM_H*/
	write_cmos_sensor_8(0x0d04, 0x01); /*Fsync(otp busy) Output Enable*/
	write_cmos_sensor_8(0x0d00, 0x07); /*Fsync(otp busy) Output Drivability*/
	write_cmos_sensor_8(0x003e, 0x10); /*OTP R/W mode*/
	write_cmos_sensor_8(0x0a00, 0x01); /*stand by off*/

	write_cmos_sensor_8(0x010a, ((addr)>>8)&0xff);/*start address H*/
	write_cmos_sensor_8(0x010b, (addr)&0xff);/*start address L*/
	write_cmos_sensor_8(0x0102, 0x01);/*single read*/
	for (i = 0; i < VIVO_SUB_OTP_DATA_SIZE; i++) {
		vivo_sub_otp_data[i] = read_cmos_sensor(0x108);
	}
	write_cmos_sensor_8(0x0a00, 0x00);
	mdelay(10);
	write_cmos_sensor_8(0x003e, 0x00);
	write_cmos_sensor_8(0x0a00, 0x01);
	return 0;
}

/* extern unsigned int is_atboot;guojunzheng add*/
int vivo_sub_otp_read(void)
{
	int i = 0;
	int offset = 0x0401;
	int check_if_ModuleInfo_group_valid = 1;
	int check_if_AWB_group_valid = 1;
	int check_if_SN_group_valid = 1;
	int check_if_group_valid = 0;
    int R_unit = 0, B_unit = 0, R_golden = 0, B_golden = 0, G_unit = 0, G_golden = 0;
	long long t1, t2, t3, t4, t5, t6, t, temp;
	/* This operation takes a long time, we need to skip it. guojunzheng add begin
	if (is_atboot == 1) {
		LOG_INF("[cfx++]AT mode skip vivo_sub_otp_read\n");
		return 1;
	}
	 guojunzheng add end */

	HI556_otp_read_setting_init_continuous_read(offset);
	LOG_INF("[cfx++]read_hi556_data,moduleinfo_flag = 0x%x,awb_flag = 0x%x , sn_flag = 0x%x\n", vivo_sub_otp_data[ModuleInfo_sub_flag], vivo_sub_otp_data[Awb_sub_flag], vivo_sub_otp_data[Sn_sub_flag]);
	LOG_INF("[cfx++]read_hi556_data\n");
	
	/*for (i = 0; i < VIVO_SUB_OTP_DATA_SIZE; i++) {
			LOG_INF("read_hi556_data[0x%x][%d] = 0x%x\n", (0x0401+i),i, vivo_sub_otp_data[i]);
	}*/
	
	if (0x01 == vivo_sub_otp_data[ModuleInfo_sub_flag]) { /*Group 1*/
		ModuleInfo_sub_addr = 0x190E - 0x0401;
		ModuleInfo_sub_checksum_addr = ModuleInfo_sub_addr + 8;
		LOG_INF("[cfx++]ModuleInfo Group 1 !!!\n");
		
	} else if (0x13 == vivo_sub_otp_data[ModuleInfo_sub_flag]) { /*Group 2*/
		ModuleInfo_sub_addr = 0x1917 - 0x0401;
		ModuleInfo_sub_checksum_addr = ModuleInfo_sub_addr + 8;
		LOG_INF("[cfx++]ModuleInfo Group 2 !!!\n");
	} else if (0x37 == vivo_sub_otp_data[ModuleInfo_sub_flag]) { /*Group 3*/
		ModuleInfo_sub_addr = 0x1920 - 0x0401;
		ModuleInfo_sub_checksum_addr = ModuleInfo_sub_addr + 8;
		LOG_INF("[cfx++]ModuleInfo Group 3 !!!\n");
	} else {
		check_if_ModuleInfo_group_valid = 0;
		LOG_INF("[cfx++]ModuleInfo flag error!!!    ModuleInfo flag :%d\n", vivo_sub_otp_data[ModuleInfo_sub_flag]);
	}

	if (0x01 == vivo_sub_otp_data[Awb_sub_flag]) { /*Group 1*/
		Awb_sub_addr = 0x0402 - 0x0401;
		Awb_sub_checksum_addr = Awb_sub_addr + 12;
		LOG_INF("[cfx++]AWB Group 1 !!!\n");
		LOG_INF("lilin Awb_sub_addr:%d\n", Awb_sub_addr);
	} else if (0x13 == vivo_sub_otp_data[Awb_sub_flag]) { /*Group 2*/
		Awb_sub_addr = 0x040F - 0x0401;
		Awb_sub_checksum_addr = Awb_sub_addr + 12;
		LOG_INF("[cfx++]AWB Group 2 !!!\n");
		LOG_INF("lilin Awb_sub_addr:%d\n", Awb_sub_addr);
	} else if (0x37 == vivo_sub_otp_data[Awb_sub_flag]) { /*Group 3*/
		Awb_sub_addr = 0x41C - 0x0401;
		Awb_sub_checksum_addr = Awb_sub_addr + 12;
		LOG_INF("[cfx++]AWB Group 3 !!!\n");
		LOG_INF("lilin Awb_sub_addr:%d\n", Awb_sub_addr);
	} else {
		check_if_AWB_group_valid = 0;
		LOG_INF("[cfx++]AWB flag error!!!    AWB flag :%d\n", vivo_sub_otp_data[Awb_sub_flag]);
	}
      
	
	if (0x01 == vivo_sub_otp_data[Sn_sub_flag]) { /*Group 1*/
		Sn_sub_addr = 0x192A - 0x0401;
		Sn_sub_checksum_addr = Sn_sub_addr + 25;
		LOG_INF("[cfx++]Sn Group 1 !!!\n");
	} else if (0x13 == vivo_sub_otp_data[Sn_sub_flag]) { /*Group 2*/
		Sn_sub_addr = 0x1944 - 0x0401;
		Sn_sub_checksum_addr = Sn_sub_addr + 25;
		LOG_INF("[cfx++]Sn Group 2 !!!\n");
	} else if (0x37 == vivo_sub_otp_data[Sn_sub_flag]) { /*Group 3*/
		Sn_sub_addr = 0x195E - 0x0401;
		Sn_sub_checksum_addr = Sn_sub_addr + 25;
		LOG_INF("[cfx++]Sn Group 3 !!!\n");
	} else {
		check_if_SN_group_valid = 0;
		LOG_INF("[cfx++]Sn flag error!!!    Sn  flag :%d\n", vivo_sub_otp_data[Sn_sub_flag]);
	}
	
	LOG_INF("[cfx++]read_hi556_data,moduleinfo_addr=0x%x,awb_addr= 0x%x, sn = %x\n", ModuleInfo_sub_addr, Awb_sub_addr,Sn_sub_addr);

	check_if_group_valid = check_if_AWB_group_valid & check_if_ModuleInfo_group_valid & check_if_SN_group_valid;
	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if ((VIVO_SUB_VENDOR_QTECH != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x00])) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]Module ID error!!!    otp_error_code:%d\n", SUB_OTP_ERROR_CODE);
			return 0;
		} else if ((VIVO_SUB_VENDOR_LENS_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x04])/* || (VIVO_SUB_VENDOR_VCM_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x06]) || (VIVO_SUB_VENDOR_DRIVERIC_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x07])*/) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code =%d\n", SUB_OTP_ERROR_CODE);
			return 0;
		} else if ((VIVO_SUB_VENDOR_PLATFROM_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x07])/* || (VIVO_SUB_VENDOR_VCM_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x06]) || (VIVO_SUB_VENDOR_DRIVERIC_ID != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x07])*/) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]Platform ID  Error!!!    otp_error_code =%d\n", SUB_OTP_ERROR_CODE);
			return 0;
		} 
		/*else if ((0xff != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x08]) || (0x00 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x09]) || (0x0b != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0a]) || (0x01 != vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0b])) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", vivo_sub_otp_data[ModuleInfo_sub_addr + 0x08], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x09], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0a], vivo_sub_otp_data[ModuleInfo_sub_addr + 0x0b]);
			return 0;
		}*/
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_sub_addr; i < ModuleInfo_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
			LOG_INF("checksum value:%d, OTP checksum value:%d\n", checksum, vivo_sub_otp_data[ModuleInfo_sub_checksum_addr]);
		if (vivo_sub_otp_data[ModuleInfo_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE);
			return 0;
			}
		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_sub_addr; i < Awb_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
			LOG_INF("checksum value:%d, OTP checksum value:%d\n", checksum, vivo_sub_otp_data[ModuleInfo_sub_checksum_addr]);
		if (vivo_sub_otp_data[Awb_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE);
			return 0;
		}
	   R_unit = vivo_sub_otp_data[Awb_sub_addr];
	   R_unit = (R_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr+1]);
	   B_unit = vivo_sub_otp_data[Awb_sub_addr+2];
	   B_unit = (B_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr+3]);
	   G_unit = vivo_sub_otp_data[Awb_sub_addr+4];
	   G_unit = (G_unit << 8) | (vivo_sub_otp_data[Awb_sub_addr+5]);
       
	   R_golden = vivo_sub_otp_data[Awb_sub_addr+6];
	   R_golden = (R_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr+7]);
	   B_golden = vivo_sub_otp_data[Awb_sub_addr+8];
	   B_golden = (B_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr+9]);
	   G_golden = vivo_sub_otp_data[Awb_sub_addr+10];
	   G_golden = (G_golden << 8) | (vivo_sub_otp_data[Awb_sub_addr+11]);
       
	   /****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
	   /****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
	   LOG_INF("lilin_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
	   t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
	   t2 = R_golden*R_golden;
	   t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
	   t4 = G_golden*G_golden;
	   t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
	   t6 = B_golden*B_golden;
	   temp = t1/t2 + t3/t4 + t5/t6 ;
	   t = temp - 10485;
	   LOG_INF("lilin_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
	   if (t > 0) {
	SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
	LOG_INF("[cfx++]lilin AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%	otp_error_code:%d\n",	temp,	SUB_OTP_ERROR_CODE);
	return 0;
	   }
		
		
		/****SN_checksum****/
		checksum = 0;
		for (i = Sn_sub_addr; i < Sn_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Sn_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE);
			return 0;
		}
		#if 0
		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_sub_addr; i < Lsc_sub_checksum_addr; i++) {
			checksum += vivo_sub_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_sub_otp_data[Lsc_sub_checksum_addr] != checksum) {
			SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE);
			return 0;
		}
		#endif
	    /****check if awb out of range[high cct]****/
		memcpy(vivo_sub_otp_data_hi556, &vivo_sub_otp_data[ModuleInfo_sub_addr], 8);
		memcpy(&vivo_sub_otp_data_hi556[8], &vivo_sub_otp_data[0], 1);
		memcpy(&vivo_sub_otp_data_hi556[9], &vivo_sub_otp_data[Awb_sub_addr], 12);
		/* add for print hi556 data*/
		for (i = 0; i < 21; i++) {
			LOG_INF("read_hi556_data[%d] = 0x%x\n", i, vivo_sub_otp_data_hi556[i]);
		}
		return 1;
	} else {
		SUB_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", SUB_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
