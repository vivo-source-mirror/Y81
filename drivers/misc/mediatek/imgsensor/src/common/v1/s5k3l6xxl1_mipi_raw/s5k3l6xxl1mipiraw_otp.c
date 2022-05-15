#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "s5k3l6xxl1mipiraw_Sensor.h"

#define PFX "S5K3L6XXL1_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1404];*/
#define VIVO_OTP_DATA_SIZE 0x0D88 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)+sizeof(pdaf)*/
//#define AF_RANGE_GOT  //if got the range of AF_inf and AF_mac , open this define ,the default AF range is 10%!!!!
#define VIVO_EEPROM_WRITE_ID 0xA0
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x0D88
/*#define VIVO_VENDOR_SUNNY 0x01*/
/*#define VIVO_VENDOR_TRULY 0x02*/
#define VIVO_VENDOR_QTECH 0x05
/*#define VIVO_VENDOR_OFILM 0x09*/
#define VIVO_VENDOR_LENS_ID 0x10
#define VIVO_VENDOR_VCM_ID 0x09
#define VIVO_VENDOR_DRIVERIC_ID 0x05
#define VIVO_VENDOR_PLATFORM_ID 0x01

static unsigned char vivo_otp_data[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001f;
static unsigned const int Af_addr = 0x07f7;
static unsigned const int Af_checksum_addr = 0x0807;
static unsigned const int Awb_addr = 0x0070;
static unsigned const int Awb_checksum_addr = 0x007d;
static unsigned const int Lsc_addr = 0x0080;
static unsigned const int Lsc_checksum_addr = 0x07cd;
static unsigned const int PD_Proc1_addr = 0x0808;
static unsigned const int PD_Proc1_checksum_addr = 0x09f9;
static unsigned const int PD_Proc2_addr = 0x9fa;
static unsigned const int PD_Proc2_checksum_addr = 0x0d87;
#ifdef AF_RANGE_GOT
static unsigned const int AF_inf_golden = 314;
static unsigned const int AF_mac_golden = 706;
#endif
static int checksum = 0;
otp_error_code_t S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_main_s5k3l6xxl1[13];
unsigned  int s5k3l6xxl1_flag;

static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
    if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)data,  1,  VIVO_EEPROM_WRITE_ID) < 0) {
		return false;
	}
    return true;
}
int s5k3l6xxl1_vivo_otp_read(void)
{
	int i = 0;
	int offset = ModuleInfo_addr;
	int check_if_group_valid = 0;
	int R_unit = 0,B_unit = 0,R_golden = 0,B_golden = 0,G_unit = 0,G_golden = 0;
		
	#ifdef AF_RANGE_GOT
	int diff_inf = 0, diff_mac = 0, diff_inf_macro = 0;
	int AF_inf = 0, AF_mac = 0;
	#endif
	
	long long t1,t2,t3,t4,t5,t6,t,temp;
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if(!vivo_read_eeprom(offset, &vivo_otp_data[i])){
			LOG_INF("[cfx++]read_vivo_eeprom 0x%0x %d fail \n", offset, vivo_otp_data[i]);
			return 0;
		}
		/*LOG_INF("[cfx++1]read_vivo_eeprom vivo_otp_data[0x%0x] =0x%x\n",offset, vivo_otp_data[i]);*/
		offset++;
	}
	//check_if_group_valid = vivo_otp_data[ModuleInfo_addr] | vivo_otp_data[Awb_addr] | vivo_otp_data[Af_addr] | vivo_otp_data[Lsc_addr] | vivo_otp_data[PD_Proc1_addr] | vivo_otp_data[PD_Proc2_addr];
	if((0x01 == vivo_otp_data[ModuleInfo_addr]) && (0x01 == vivo_otp_data[Awb_addr]) && (0x01 == vivo_otp_data[Af_addr]) && (0x01 == vivo_otp_data[Lsc_addr]) && (0x01 == vivo_otp_data[PD_Proc1_addr]) && (0x01 == vivo_otp_data[PD_Proc2_addr]))
	{
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid =%d\n", check_if_group_valid);
	}

	
	if ((vivo_otp_data[0x08] != VIVO_VENDOR_VCM_ID) ||(vivo_otp_data[0x09] != VIVO_VENDOR_DRIVERIC_ID) ){
		S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
		s5k3l6xxl1_flag = 0;
		LOG_INF("VCM ID or Driver_IC ID  Error!!!    s5k3l6xx_flag =%d\n", s5k3l6xxl1_flag);
	}else{
		s5k3l6xxl1_flag = 1;
		LOG_INF("s5k3l6xx_flag = %d\n", s5k3l6xxl1_flag);
	}
	
	if (check_if_group_valid == 0x01) //all the data is valid
	{
		/////ModuleInfo 
		if (VIVO_VENDOR_QTECH != vivo_otp_data[0x01]) ///xuyongfu need check 2nd vendor
		{
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]ModuleInfo_info error!!!    otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
		} else if ((vivo_otp_data[0x02] != VIVO_VENDOR_PLATFORM_ID) || (vivo_otp_data[0x06] != 0x01) || (vivo_otp_data[0x07] != VIVO_VENDOR_LENS_ID) || (vivo_otp_data[0x09] != VIVO_VENDOR_DRIVERIC_ID))
		{
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]: Platform ID or Sensor or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
		} else if (vivo_otp_data[0x08] != VIVO_VENDOR_VCM_ID) {
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]:  VCM ID Error!!!    otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;		
		}
		else if((0xff != vivo_otp_data[0x0a]) || (0x00 != vivo_otp_data[0x0b]) || (0x0b != vivo_otp_data[0x0c]) || (0x01 != vivo_otp_data[0x0d]))
		{
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[cfx++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", vivo_otp_data[0x0a], vivo_otp_data[0x0b], vivo_otp_data[0x0c], vivo_otp_data[0x0d]);
			return 0;
		}
		//////ModuleInfo_checksum
		checksum = 0;
		for (i = ModuleInfo_addr + 1; i < ModuleInfo_checksum_addr; i++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[ModuleInfo_checksum_addr] != checksum )
        {
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
        }
	
	sn_inf_main_s5k3l6xxl1[0] = 0x00;
	#if 0
	for (i = 0; i < 12; i++) {
		sn_inf_main_s5k3l6xxl1[i +1] = (MUINT32)vivo_sub_otp_data[i + Sn_sub_addr];
		LOG_INF("sn_inf_main_s5k3l6xxl1[%d] = 0x%x, vivo_sub_otp_data[0x%x] = 0x%x\n", i+1, sn_inf_main_s5k3l6xxl1[i+1],	i +Sn_sub_addr, vivo_sub_otp_data[i + Sn_sub_addr]);
	}
	#endif
		
		//////AWB_checksum
		checksum = 0;
        for(i = Awb_addr+1; i < Awb_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Awb_checksum_addr] != checksum )
        {
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]AWB_checksum error!!!   otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
        }
		
		//////AF_checksum
		checksum = 0;
        for(i = Af_addr+1; i < Af_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Af_checksum_addr] != checksum )
        {
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]AF_checksum error!!!   otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
        }
		
		//////LSC_checksum
		checksum = 0;
        for(i = Lsc_addr+1; i < Lsc_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[Lsc_checksum_addr] != checksum )
        {
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]LSC_checksum error!!!   otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
		}

		/****PDAF_checksum****/
		checksum = 0;
        for(i = PD_Proc1_addr+1; i < PD_Proc1_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[PD_Proc1_checksum_addr] != checksum )
        {
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]PD_Proc1_data_checksum error!!!   otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
        }
		checksum = 0;//PD_Proc2_data
		for(i = PD_Proc2_addr+1; i < PD_Proc2_checksum_addr; i ++)
        {              
			checksum += vivo_otp_data[i];						
        }  
			checksum = checksum % 0xff + 1;
		if( vivo_otp_data[PD_Proc2_checksum_addr] != checksum )
        {
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[cfx++]PD_Proc2_data_checksum error!!!   otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
        }
		
		/****check if awb out of range[high cct]****/
		R_unit = vivo_otp_data[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data[Awb_addr+2]);
		B_unit = vivo_otp_data[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data[Awb_addr+4]);
		G_unit = vivo_otp_data[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data[Awb_addr+6]);

		R_golden = vivo_otp_data[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data[Awb_addr+8]);
		B_golden = vivo_otp_data[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data[Awb_addr+10]);
		G_golden = vivo_otp_data[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld ,t2 = %lld ,t3 = %lld ,t4 = %lld ,t5 = %lld ,t6 = %lld ,temp = %lld ,t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if(t > 0) 
		{
			S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[cfx++]AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5K3L6XXL1_OTP_ERROR_CODE);
			return 0;
		}

		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data[Af_addr+6];
		AF_inf = (AF_inf << 8) | (vivo_otp_data[Af_addr+7]);
				
		AF_mac = vivo_otp_data[Af_addr+8];
		AF_mac = (AF_mac << 8) | (vivo_otp_data[Af_addr+9]);
        diff_inf_macro = AF_mac - AF_inf;
        
		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
   
        /* tyj add for distinguishing VCM */
		if (vivo_otp_data[0x08] == 0x07) {    /* 0x07 or 0x02 Shicoh, 0x01 TDK */ 
            if (diff_inf > 100 || diff_mac > 120 || diff_inf_macro > 500 || diff_inf_macro < 280){  /*AF code out of range*/
			    S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			    LOG_INF("[cfx++]AF out of range error!!!   otp_error_code:%d   VCM ID = 0x%x AF_inf = %d AF_mac = %d\n", S5K3L6XXL1_OTP_ERROR_CODE, vivo_otp_data[0x07], AF_inf, AF_mac);
			    return 0;
		    }
        }
        /* tyj add for distinguishing end */
        else{
			LOG_INF("[tyj++]distinguish vcm VCM ID = 0x%x inf = %d mac = %d\n", vivo_otp_data[0x08], AF_inf, AF_mac);
		    if (diff_inf > 100 || diff_mac > 110 || diff_inf_macro > 500 || diff_inf_macro < 280) {  /*AF code out of range*/
			    S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			    LOG_INF("[cfx++]AF out of range error!!!   otp_error_code:%d   VCM ID = 0x%x AF_inf = %d AF_mac = %d\n", S5K3L6XXL1_OTP_ERROR_CODE, vivo_otp_data[0x07], AF_inf, AF_mac);
			    return 0;
		    }
        }
		#endif

		/*cfx add for pdaf data start 20161223*/
		/*for(i = 0;i < 1404;i++)
		{
			if(i == 0)
				LOG_INF("[cfx++]read_S5K3L6XXL1_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*cfx add for pdaf data end*/

		return 1;
	} else {
		S5K3L6XXL1_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("[cfx++]invalid otp data. error!!!   otp_error_code:%d\n", S5K3L6XXL1_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
