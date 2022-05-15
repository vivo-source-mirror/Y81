#ifndef __PLATFORM_INTERACT_H
#define __PLATFORM_INTERACT_H


#include <linux/slab.h>
#include <mach/upmu_hw.h>
#include <mach/upmu_sw.h>
#include <mach/mtk_pmic.h>
#include <mtk_sleep.h>
#include <mtk_boot.h>
#include <mtk_boot_reason.h>
#include <upmu_common.h>
#include <vivo/cust_charging.h>

#include <mt-plat/mtk_auxadc_intf.h>


/************************************************************
 *
 *   [platform supply function]
 *
 ***********************************************************/
extern void arch_reset(char mode, const char *cmd);

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);

#if (GAUGE_VOLTAGE_BASE == GAUGE_MASTER)
extern int get_battery_plug_out_status(void);
extern uint32_t PMIC_IMM_GetOneChannelValue(mt6350_adc_ch_list_enum dwChannel, int deCount, int trimd);
#else
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
extern uint32_t PMIC_IMM_GetOneChannelValue(pmic_adc_ch_list_enum dwChannel, int deCount, int trimd);
extern uint32_t mt6353_upmu_get_rgs_vcdt_hv_det(void);
extern uint32_t mt6353_upmu_get_rgs_chrdet(void);
extern void mt6353_upmu_set_rg_vcdt_hv_vth(uint32_t val);

extern void mt6353_upmu_set_baton_tdet_en(uint32_t val);
extern uint32_t mt6353_upmu_get_baton_tdet_en(void);
extern void mt6353_upmu_set_rg_baton_en(uint32_t val);
extern uint32_t mt6353_upmu_get_rgs_baton_undet(void);
extern void upmu_interrupt_chrdet_int_en(uint32_t val);
#endif
#endif


extern int mtkts_bts_get_hw_temp(void);
extern bool get_fb_suspend_state(void);
extern bool usb_is_configured(void);
extern void clear_usb_is_configured(void);
extern bool mtk_is_usb_id_pin_short_gnd(void);
extern int hw_charging_get_charger_type(void);
extern uint32_t mt_get_bl_brightness(void);


#if defined(CONFIG_USB_MTK_HDRC) || defined(CONFIG_USB_MU3D_DRV)
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
#else
#define mt_usb_connect()                do { } while (0)
#define mt_usb_disconnect()             do { } while (0)
#endif


/************************************************************
 *
 *   [local function]
 *
 ***********************************************************/
extern int get_bts_temp(void);
extern bool get_screen_off_state(void);
extern void platform_usb_connect(void);
extern void platform_usb_disconnect(void);
extern int pmic_get_channel_value(u8 list);//extern int pmic_get_channel_value(int channel);
extern int ap_get_channel_value(int channel);
extern bool is_power_path_supported(void);

#endif/* #ifndef __PLATFORM_INTERACT_H */
