#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/wait.h>		/* For wait queue*/
#include <linux/sched.h>	/* For wait queue*/
#include <linux/kthread.h>	/* For Kthread_run */
#include <linux/platform_device.h>	/* platform device */
#include <linux/time.h>

#include <linux/netlink.h>	/* netlink */
#include <linux/kernel.h>
#include <linux/socket.h>	/* netlink */
#include <linux/skbuff.h>	/* netlink */
#include <net/sock.h>		/* netlink */
#include <linux/cdev.h>		/* cdev */

#include <linux/err.h>	/* IS_ERR, PTR_ERR */
#include <linux/reboot.h>	/*kernel_power_off*/
#include <linux/proc_fs.h>
#include <linux/of_fdt.h>	/*of_dt API*/
#include <linux/vmalloc.h>

//#include <linux/wakelock.h>


#define GETBIT(x,n)                     (x & (0x1<<n))
#define SETBIT(x,n)                     (x = (x | (0x1<<n)))
#define CLRBIT(x,n)                     (x = (x & ~(0x1<<n)))

#define BATTERY_NAME                    "battery"
#define METER_NAME                      "meter"
#define CHARGE_NAME                     "charge"
#define SLAVE_CHARGE_NAME               "slave_charge"

#define GETARRAYNUM(array)      (sizeof(array)/sizeof(array[0]))


/************************************************************
 *
 *   [macro define]
 *
 ***********************************************************/
#define CHARGING_I2C_MASTER_CLOCK       380

//#define ADJUST_VINDPM_THR               4150
//#define ADJUST_VINDPM_THR_REC           4050

#define BTC_SIZE                        8
#define TC_SMART_FBON_SIZE              6
#define TC_SMART_FBOFF_SIZE             5


/************************************************************
 *
 *   [devices tree data]
 *
 ***********************************************************/
typedef enum {
        ROW = 0,
        COL = 1,
} DIMENSION_ENUM;

#define SIZE_OF_BTC                     6
struct btc {
        int tmin;
        int tmax;
        int cinput;
        int vbat_thr;
        int scale;
        int scale_step;
};
#define INIT_BTC(min, max, input, thr, s, sstep) \
{ \
        .tmin = min, \
        .tmax = max, \
        .cinput = input, \
        .vbat_thr = thr, \
        .scale = s, \
        .scale_step = sstep, \
}

#define SIZE_OF_TC                      4
struct tc {
        int tmin;
        int tmax;
        int cinput;
        int cinbat;
};
#define INIT_TC(min, max, input, inbat) \
{ \
        .tmin = min, \
        .tmax = max, \
        .cinput = input, \
        .cinbat = inbat, \
}


/************************************************************
 *
 *   [debug]
 *
 ***********************************************************/
#define LOG_CRTI                        1
#define LOG_FULL                        2
extern int enable_log;

//pr_err(fmt, ##args);
#define pc_print(num, fmt, args...) \
        do { \
                if (enable_log >= (int)num) { \
                        printk(KERN_ERR " [%s] "pr_fmt(fmt), __func__, ##args); \
                } \
        } while(0)


/************************************************************
 *
 *   [reg attr structure]
 *
 ***********************************************************/
struct reg_attr {
        uint8_t reg_enum;
        uint8_t reg_mask;
        uint8_t reg_shift;
        struct device_attribute attribute;
};

#define INIT_REG_ATTR(REG_ENUM) \
{ \
        .reg_enum = REG_ENUM, \
        .reg_mask = REG_ENUM##_MASK, \
        .reg_shift = REG_ENUM##_SHIFT, \
        .attribute = __ATTR(REG_ENUM, 0664, show_reg, store_reg), \
}


/************************************************************
 *
 *   [control structure]
 *
 ***********************************************************/
struct ctrl {
        int ctrl_enum;
        int (*func)(void *data);
};

#define INIT_CTRL(num, method) \
{ \
        .ctrl_enum = num, \
        .func = method, \
}


/************************************************************
 *
 *   [battery supplier]
 *
 ***********************************************************/
typedef enum {
        BATTERY_ID_NOT_LOADED = -1,
        BATTERY_UNKNOWN_SUPPLIER = 0,
        BATTERY_FIRST_SUPPLIER,
        BATTERY_SECOND_SUPPLIER,
        BATTERY_THIRD_SUPPLIER,
        BATTERY_FOURTH_SUPPLIER,
        BATTERY_FIFTH_SUPPLIER,
        BATTERY_DEFAULT_SUPPLIER = 100,
} BATTERY_VENDOR;

struct battery_id {
        int id;
        const char *name;
        bool (*check_vendor)(struct battery_id *bid);
        int (*get_vendor)(struct battery_id *bid);
};


/************************************************************
 *
 *   [battery]
 *
 ***********************************************************/
typedef enum {
        BATTERY_CMD_GET_UI_SOC,
        BATTERY_CMD_GET_UI_SOC_READY,
        BATTERY_CMD_GET_VENDOR,
        BATTERY_CMD_GET_STATUS,
        BATTERY_CMD_GET_VOLTAGE,
        BATTERY_CMD_GET_VOLTAGE_AVG,
        BATTERY_CMD_GET_TEMPERATURE,
        BATTERY_CMD_GET_TEMPERATURE_AVG,
        BATTERY_CMD_SYNC_BATTERY_FULL,
		BATTERY_CMD_GET_CURRENT,
        BATTERY_CMD_NUMBER,
} BATTERY_CTRL_CMD;
typedef int (*BATTERY_CONTROL)(BATTERY_CTRL_CMD cmd, void *data);

struct battery {
        const char *name;
        struct power_control *pwr;
        int (*initial)(struct power_control *p);
        BATTERY_CONTROL battery_ctrl;
        void *battery_private;
        bool suspend;
        bool battery_full;
};


/************************************************************
 *
 *   [meter]
 *
 ***********************************************************/
typedef enum {
        METER_CMD_INIT,
        METER_CMD_RESET,
        METER_CMD_GET_CAPACITY,
        METER_CMD_GET_COULOMB,
        METER_CMD_GET_COULOMB_DELTA,
        METER_CMD_GET_CURRENT,
        METER_CMD_GET_IM_CURRENT,
        METER_CMD_GET_CURRENT_SIGN,
        METER_CMD_GET_OCV,
        METER_CMD_GET_ZCV,
        METER_CMD_GET_NPERCENT_ZCV,
        METER_CMD_GET_NPERCENT_POINT,
        METER_CMD_GET_QMAX,
        METER_CMD_GET_QMAX_LOADED,
        METER_CMD_GET_V_C_DELTA,
        METER_CMD_GET_DELTA_IS_ERR,
        METER_CMD_NUMBER,
} METER_CTRL_CMD;
typedef int (*METER_CONTROL)(METER_CTRL_CMD cmd, void *data);

struct meter {
        const char *name;
        struct power_control *pwr;
        METER_CONTROL meter_ctrl;
        int (*initial)(struct power_control *p);
        void *meter_private;
};


/************************************************************
 *
 *   [charge]
 *
 ***********************************************************/
typedef enum {
        CHARGE_CMD_INIT,
        CHARGE_CMD_DUMP,
        CHARGE_CMD_HIZ,
        CHARGE_CMD_GET_HIZ,
        CHARGE_CMD_ENABLE,
        CHARGE_CMD_SET_CV_VOLTAGE,
        CHARGE_CMD_GET_CURRENT,
        CHARGE_CMD_SET_CURRENT,
        CHARGE_CMD_SET_INPUT_CURRENT,
        CHARGE_CMD_GET_CHARGING_STATUS,
        CHARGE_CMD_GET_CHARGER_VOLTAGE,
        CHARGE_CMD_SET_WATCH_DOG,
        CHARGE_CMD_RESET_WATCH_DOG,
        CHARGE_CMD_GET_CHARGER_DET_STATUS,
        CHARGE_CMD_GET_CHARGER_TYPE,
        CHARGE_CMD_CHARGER_TYPE_RETRY,
        CHARGE_CMD_SET_CHARGING_STEAL,
        CHARGE_CMD_VPH_POWER_SWITCH,
        CHARGE_CMD_GET_VINDPM,
        CHARGE_CMD_SET_ABS_VINDPM,
        CHARGE_CMD_DYNAMIC_ADJUST_VINDPM,
        CHARGE_CMD_GET_CONFIG_OTG_BIT,
        CHARGE_CMD_CONFIG_OTG_MODE,
        CHARGE_CMD_CHECK_OTG_MODE,
		CHARGE_CMD_CHECK_SAFE_REG,

        CHARGE_CMD_HVDCP_ENABLE,
        CHARGE_CMD_FORCE_DPDM,
        CHARGE_CMD_SET_FORCE_VINDPM,
        CHARGE_CMD_FULL_HV_SWITCH,
        CHARGE_CMD_RESET_TO_DEFAULT,
        CHARGE_CMD_SET_ICO_MA,
        CHARGE_CMD_GET_CHARGER_REG_TYPE,
        CHARGE_CMD_CHARGER_HV_PREVENT_STATIC,

        CHARGE_CMD_GET_IS_PCM_TIMER_TRIGGER,
        CHARGE_CMD_SET_PLATFORM_RESET,
        CHARGE_CMD_GET_PLATFORM_BOOT_MODE,
        CHARGE_CMD_GET_PLATFORM_BOOT_REASON,
        CHARGE_CMD_SET_POWER_OFF,
#ifdef CONFIG_VIVO_DUAL_CHARGING
		CHARGE_CMD_GET_SLAVE_CHG_EXIST,
		CHARGE_CMD_GET_IS_DPM_TRIGGERED,
#endif
        CHARGE_CMD_NUMBER,
} CHARGE_CTRL_CMD;
typedef int (*CHARGE_CONTROL)(CHARGE_CTRL_CMD cmd, void *data);

struct charge {
        const char *name;
        struct power_control *pwr;
        struct delayed_work power_detect;
        struct delayed_work charger_type_detect;
        CHARGE_CONTROL charge_ctrl;
        int (*initial)(struct power_control *p);
        bool charger_type_lag;
		bool is_exist;
		bool is_powerpath_supported;
        void *charge_private;
};
#define BQ25890H_PN			0x3
#define BQ24157_VC			0x2
#define FAN54005_VC			0x4
typedef enum {
	BQ_UNKNOW = 0,
	BQ_25890H = 1,
	BQ_25601D = 2,
	BQ_24157 = 3,
} BQ_ENUM;

/************************************************************
 *
 *   [power_control]
 *
 ***********************************************************/
struct power_routine_data {
        int boot_mode;
        int boot_reason;
        int spm_period;
        bool warning_test;
        uint32_t warning;
#define BATTERY_CHARGING_ERROR          0x0
#define BATTERY_CC_CHARGING             0x1
#define BATTERY_CHARGING_FULL           0x2
#define BATTERY_RE_CHARGING             0x3
        uint8_t battery_charging_state;
        int battery_voltage;
        int i_current;
        int battery_temperature;
#define UNKNOWN_SOC                     -1
        int soc;
        int zcv;

        bool full_meter_reset;

        bool charger_exist;
        bool charger_in_int;
        bool charge_enable;
        bool charge_hiz;
        bool charge_protection;
        bool charger_type_done;
		bool bat_is_charging;
        bool fast_charge;
		int otg_retry_count;
#define OTG_RETRY_COUNTS		6

#define CHARGER_OUT                     0
#define CHARGER_IN                      1
        int charger_state;
        int charger_type;
		bool dcp_type_detected;
        int charger_voltage;
        int inlimt_current;
        int charging_current;
        int charger_ico;
        int charging_time;
        int sleep_duration;

        bool call_state;
        bool cooperation_contract;
        bool large_charging;
        bool charging_steal;
        bool charging_vph_switch;
        bool spm_timeout;
        struct timespec charging_happen_time;
        struct timespec charging_start_time;
};

struct power_control {
        const char *name;
        int usboe_gpio;
        struct device_node *node;
        struct battery *bat;
        struct meter *mtr;
        struct charge *chg;
        struct power_routine_data *data;
        struct mutex thd_mutex;
        struct wakeup_source suspend_lock;
        struct wakeup_source fg_lock;
        wait_queue_head_t thd_wq;
        struct hrtimer thd_hrtimer;
        struct timespec thd_sleep_time;
        struct list_head *charger_interrupt;
        struct list_head *charger_int_in;
        struct list_head *charger_int_out;
        struct list_head *charger_foolproof;
        struct list_head *running;
        struct list_head *collect_info;
        struct list_head *warining;
        struct list_head *switch_charging;
		struct list_head *switch_charging_protection;
        struct list_head *update;
        struct list_head *full_reserve;

#define BATTERY_READY_BIT               0
#define METER_READY_BIT                 1
#define CHARGE_READY_BIT                2
#define PCS_READY_BIT                   3
#define INIT_READY                      0xF
#define INIT_COMPLETE                   0xFF
        uint8_t init_state;

#define BATTERY_TIMEOUT_BIT             0
#define METER_TIMEOUT_BIT               1
#define FG_WAKEUP_BIT                   2
#define CHARGER_WAKEUP_BIT              3
        uint8_t thd_event;

        bool thd_sleep;
        bool initialized;

        int qmax_mah;
        int large_current_timeout;
        int normal_current;
        int protection_current;
        bool smart_charging;
        struct btc btc_data[BTC_SIZE];
        struct tc smart_fbon_tc_data[TC_SMART_FBON_SIZE];
        struct tc smart_fboff_tc_data[TC_SMART_FBOFF_SIZE];
        struct power_supply *fs_psy;
        struct fuelsummary_ex *pc_fex;
        struct pinctrl *pin_ctrl;
        struct pinctrl_state *usboe_default;
        struct pinctrl_state *usboe_enable;
        struct pinctrl_state *usboe_disable;
};


/************************************************************
 *
 *   [warning]
 *
 ***********************************************************/
typedef enum {
        NORMAL_NO_WARNING = 0,
        CHARGER_VOLTAGE_HIGH,
        BATTERY_TEMPERATURE_HIGH,
        CURRENT_OVER_PROTECTION,
        BATTERY_VOLTAGE_HIGH,
        CHARGING_OVER_TIME,
        BATTERY_TEMPERATURE_LOW,
        BATTERY_ID_ERROR,
        BATTERY_TEMPERATURE_OFFSET,
        BATTERY_NOT_ORIGINAL,
        CHARGING_EXCEPTION,
        BATTERY_VOLTAGE_TEMPERATURE_HIGH,
        TEST_WARNING,
        WARNING_NUMBER,
} WARNING_ENUM;
/*
struct warning_s {
        int mark;
        struct list_head list;
        int (*check)(struct power_control *p);
};

#define BATTERY_WARNING(num, check) \
{ \
        .ctrl_enum = num, \
        .func = method, \
}
*/


/************************************************************
 *
 *   [compare define]
 *
 ***********************************************************/
#define BETWEEN(val, min, max) ((val) >= (min) && (val) <= (max))


/************************************************************
 *
 *   [command]
 *
 ***********************************************************/
#define CMD_NAME_SIZE                   64
struct command {
        char name[CMD_NAME_SIZE];
        struct list_head list;
        int (*execute)(struct power_control *p);
};

#define INIT_COMMAND(func) \
{ \
        .name = #func, \
        .execute = func, \
}

typedef int (*COMMAND)(struct power_control *p);


/************************************************************
 *
 *   [export function]
 *
 ***********************************************************/
typedef enum {
        BATTERY_VOLTAGE = 0,
        BATTERY_TEMPERATURE,
        BATTERY_CURRENT,
        CHARGER_VOLTAGE,
        AVG_NUMBER,
} AVG_ENUM;

//#define pmic_init_wake_lock(lock, name)	wakeup_source_init(lock, name)
#define vivo_wake_lock(lock)	__pm_stay_awake(lock)
#define vivo_wake_unlock(lock)	__pm_relax(lock)

extern struct command *init_command(COMMAND func, const char *name);

extern int get_average_value(AVG_ENUM type, int *buf, int size, int element);
extern int value_to_parameter(const int *parameter, const int array_size, const int val);
extern int parameter_to_index(const int *parameter, const int array_size, const int val);
extern int find_closest_level(const int *list, int number, int level);
extern bool get_usboe_status(void);
extern void config_usboe(bool enable);
extern bool get_battery_is_charging(void);

extern void set_vendor_name(const char *vendor);
extern void wake_up_bat(void);
extern void fg_wake_up(void);
extern void do_chrdet_int_task(void);
extern void do_chrdet_int_task_chr_ic(void);

extern int pc_register_battery(struct battery *b);
extern int pc_register_meter(struct meter *m);
extern int pc_register_charge(struct charge *c);
extern void pc_charger_type_ready(struct power_control *p);


#endif/* #ifndef __POWER_CONTROL_H */
