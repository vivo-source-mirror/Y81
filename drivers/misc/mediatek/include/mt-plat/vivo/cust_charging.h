#ifndef __CUST_CHARGING_H
#define __CUST_CHARGING_H

#include <mt-plat/mtk_auxadc_intf.h>

/************************************************************
 *
 *   [adc cust define]

 #ifdef CONFIG_MTK_PMIC_CHIP_MT6356
	//mt6356
	AUXADC_LIST_BATADC,
	AUXADC_LIST_MT6356_START = AUXADC_LIST_BATADC,
	AUXADC_LIST_VCDT,
	AUXADC_LIST_BATTEMP,
	AUXADC_LIST_BATID,
	AUXADC_LIST_VBIF,
	AUXADC_LIST_MT6356_CHIP_TEMP,
	AUXADC_LIST_DCXO,
	AUXADC_LIST_ACCDET,
	AUXADC_LIST_TSX,
	AUXADC_LIST_HPOFS_CAL,
	AUXADC_LIST_ISENSE,
	AUXADC_LIST_MT6356_BUCK1_TEMP,
	AUXADC_LIST_MT6356_BUCK2_TEMP,
	AUXADC_LIST_MT6356_END = AUXADC_LIST_MT6356_BUCK2_TEMP,
#endif
 *
 ***********************************************************/
#define BAT_ADC_IN_CHANNEL              3//13//1  need check on mt6763
#define NTC_CHANNEL                     AUXADC_LIST_BATTEMP//12//2
#define AUX_ISENSE                      AUXADC_LIST_ISENSE//PMIC_AUX_ISENSE_AP//MT6350_AUX_ISENSE
#define AUX_BATSNS                      AUXADC_LIST_BATADC//PMIC_AUX_BATSNS_AP//MT6350_AUX_BATSNS
#define AUX_VCDT                        AUXADC_LIST_VCDT//PMIC_AUX_VCDT_AP//MT6350_AUX_VCDT


/************************************************************
 *
 *   [charger voltage divider resistance]
 *
 ***********************************************************/
#define R_CHARGER_1                     330
#define R_CHARGER_2                     39
#define R_BAT_SENSE                     4
#define R_I_SENSE                       4


/************************************************************
 *
 *   [battery_id cust define]
 *
 ***********************************************************/
#define NEW_GIOP_OPERATE
#define DECIDE_BATTERY_ID_BY_C
//#define BUILT_IN_BATTERY_RISE


/************************************************************
 *
 *   [battery cust define]
 *
 ***********************************************************/
//#define CUST_CAR_BY_IAVG

#define SHAKE_PROOF                     6

#define BATTERY_POWER_OFF_VOLTAGE       3400

#define BATTERY_THIN_ENDURANCE_SPEED    6
#define BATTERY_NORMAL_ENDURANCE_SPEED  15
#define BATTERY_PLUMP_ENDURANCE_SPEED   18

#define SMOOTH_RISE_PERCENT_THR         93
#define SMOOTH_RISE_CUMULATIVE          10
#define SMOOTH_RISE_KEEP_CUMULATIVE     20

#define PERCNET_SYNC_CUMULATIVE         9
#define BATTERY_FULL_SYNC_CUMULATIVE    3

#define SOFT_CHARGING_TERM_PERCENT      99
#define SOFT_CHARGING_TERM_CUMULATIVE   60

#define PERCNET_TRACK_INTERVAL_3        3
#define PERCNET_TRACK_INTERVAL_4        4
#define PERCNET_TRACK_INTERVAL_5        5

#ifdef CUST_CAR_BY_IAVG
#define SYNC_BY_C_V_DELTA_THRESHOLD     6
#else
#define SYNC_BY_C_V_DELTA_THRESHOLD     4
#endif
#define SYNC_BY_V_C_DELTA_THRESHOLD     8
#define SYNC_BY_V_C_DELTA_FOR_LOW_TEMP  SYNC_BY_V_C_DELTA_THRESHOLD

#define C_V_DELTA_SYNC_HIGH             80
#define C_V_DELTA_SYNC_LOW              20

#define THIN_LOW_PERCENT_STAY           26
#define NORMAL_LOW_PERCENT_STAY         30

#define PERCENT_2_ENDURANCE             12
#define PERCENT_1_ENDURANCE             20
#define PERCENT_1_EXTRA                 3330

#define SYSTEM_NORMAL_CURRENT           3000
#define SYSTEM_LOADING_CURRENT          7200


/************************************************************
 *
 *   [battery ntc temperature]
 *
 ***********************************************************/
#define BAT_NTC_100

#if defined(BAT_NTC_10)
#define NTC_PULL_UP_RES                 16900	
#define NTC_OVER_CRITICAL_LOW           27000	

#elif defined(BAT_NTC_47)

#define NTC_PULL_UP_RES                 61900	
#define NTC_OVER_CRITICAL_LOW           100000	

#elif defined(BAT_NTC_100)

#define NTC_PULL_UP_RES                 180000
#define NTC_OVER_CRITICAL_LOW           865161
#endif

#define NTC_PULL_UP_VOLT                1800

#define TEMPERATURE_MAX                 60
#define TEMPERATURE_MIN                 -20
#define NTC_TABLE_SIZE                  17

struct ntc_temperature {
	int temperature;
	int resistance;
};


/************************************************************
 *
 *   [meter curve profile structure]
 *
 ***********************************************************/
#define CURVE_STANDARD_SIZE             101
struct curve_profile {
        int dod;        /* depth of discharge */
	int voltage;    /* open circuit voltage */
        int resistance; /* resistance of cell */
};


/************************************************************
 *
 *   [gauge master]
 *
 ***********************************************************/
#define GAUGE_VOLTAGE_BASE              0
#define GAUGE_COULOMB_COUNTER           1
#define GAUGE_MASTER_V3                 2

#define GAUGE_MASTER                    GAUGE_COULOMB_COUNTER


/************************************************************
 *
 *   [fg define]
 *
 ***********************************************************/
#define METER_RESISTANCE                0

#define LOW_TEMP_DISCHG_CAR_TUNE        9
#define HIGH_TEMP_DISCHG_CAR_TUNE       3

#define CAR_TUNE_VALUE                  100
#define R_FG_VALUE                      10
#define R_FG_BOARD_BASE                 1000
#define R_FG_BOARD_SLOPE                1000


#define AGING_TUNING_VALUE              103
#define CURRENT_DETECT_R_FG             10

#define POWERON_DELTA_CAPACITY_THR      20
#define POWERON_LOW_CAPACITY_THR        3
#define HIGH_BAT_CAPACITY_THR           95

#define VBAT_LOW_POWER_WAKEUP		    3500
#define VBAT_NORMAL_WAKEUP              3600
#define NORMAL_WAKEUP_PERIOD            (35*60)/*1200*/
#define LOW_POWER_WAKEUP_PERIOD         300
#define POWEROFF_WAKEUP_PERIOD          30
#define VBAT_MINERR_OFFSET              1000


/************************************************************
 *
 *   [oam define]
 *
 ***********************************************************/
#define SOC_BY_SW_FG
#define OAM_RUNNING_INTERVAL            60
#define CUST_R_SENSE                    68//56
#define I_SENSE_OFFSET                  0
#define OCV_BOARD_COMPESATE             0
#define OAM_D5                          1
#define FG_METER_RESISTANCE             0
#define CUST_HW_CC                      0
#define CUST_R_FG_OFFSET                0
#define DIFF_PRESS_COUNT                10


/************************************************************
 *
 *   [switch charging]
 *
 ***********************************************************/
#if 1/*defined(CONFIG_VIVO_BQ25890_FAST_CHARGING) || defined(CONFIG_VIVO_BQ24296_NORMAL_CHARGING)*/
#define SWCHR_POWER_PATH
#else
#define EXTERNAL_SWCHR_SUPPORT
#endif

#if defined(SWCHR_POWER_PATH)
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
#define PMIC_HW_OCV_ADC_RG              PMIC_AUXADC_ADC_OUT_WAKEUP_SWCHR
#else
#define PMIC_HW_OCV_ADC_RG              PMIC_RG_ADC_OUT_WAKEUP_SWCHR
#endif
#else
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
#define PMIC_HW_OCV_ADC_RG              PMIC_AUXADC_ADC_OUT_WAKEUP_PCHR
#else
#define PMIC_HW_OCV_ADC_RG              PMIC_RG_ADC_OUT_WAKEUP_PCHR
#endif
#endif

#define I2C_CHANNEL                     I2C1
#define TASK_PERIOD                     10
#define CHARGING_I2C_MASTER_CLOCK       380

#define ADJUST_VINDPM_THR               4200
#define ADJUST_VINDPM_THR_REC           4050

#define CHARGING_OVER_TIME_THR          (10*60*60)
#define CHARGING_FULL_CHECK_TIMES       6
#define HIGH_BATTERY_VOLTAGE_SUPPORT

#define BATTERY_AVERAGE_SIZE            6//10
#define MAX_POWER_DET_TIME              60

typedef enum {
	DEV_UNKNOWN = 0,
	DEV_BQ2589X = 1,
	DEV_BQ25890H = 3,
} CHARGER_IC_DEV_REV;

#endif/* #ifndef __CUST_CHARGING_H */
