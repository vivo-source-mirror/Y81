#ifndef __CHARGING_PLATFORM_INTERACT_H
#define __CHARGING_PLATFORM_INTERACT_H

#include <linux/types.h>


#ifdef battery_log
#undef battery_log
#endif

#define BAT_LOG_CRTI                    1
#define BAT_LOG_FULL                    2
#define BAT_LOG_DEBG                    3

#ifndef battery_log
#define battery_log(num, fmt, args...) \
do { \
        if ((int)num <= 1) { \
                pr_err(fmt, ##args); \
        } \
} while(0)
#endif


/************************************************************
 *
 *   [pcs supply export function]
 *
 ***********************************************************/
extern unsigned long BAT_Get_Battery_Voltage(int polling_mode);
extern int get_bat_charging_current_level(void);
extern int charge_get_vchg(void);
extern int charge_get_type(void);
extern bool pmic_chrdet_status(void);
extern bool upmu_is_chr_det(void);
extern void do_chrdet_int_task(void);

extern void wake_up_bat(void);
extern int meter_get_i_sign(void);
extern int meter_get_im_i(void);
extern int battery_get_vbat(void);
extern int vivo_battery_get_soc(void);
extern int vivo_battery_get_uisoc(void);
extern void config_otg_mode(bool enable);

extern int car_tune_value;


#endif/* #ifndef __CHARGING_PLATFORM_INTERACT_H */
