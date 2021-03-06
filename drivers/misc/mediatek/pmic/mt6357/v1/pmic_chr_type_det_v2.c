/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>

#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include "include/pmic.h"

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
#include <mt-plat/mtk_usb2jtag.h>
#endif

/* #define __FORCE_USB_TYPE__ */
#define __SW_CHRDET_IN_PROBE_PHASE__

static CHARGER_TYPE g_chr_type;
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static struct work_struct chr_work;
#endif
static DEFINE_MUTEX(chrdet_lock);
static struct power_supply *chrdet_psy;

#if !defined(CONFIG_FPGA_EARLY_PORTING)
static bool first_connect = true;
#endif

static int chrdet_inform_psy_changed(CHARGER_TYPE chg_type,
				bool chg_online)
{
	int ret = 0;
	union power_supply_propval propval;

	pr_info("charger type: %s: online = %d, type = %d\n", __func__,
		chg_online, chg_type);

	/* Inform chg det power supply */
	if (chg_online) {
		propval.intval = chg_online;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0)
			pr_notice("%s: psy online failed, ret = %d\n",
				__func__, ret);

		propval.intval = chg_type;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret < 0)
			pr_notice("%s: psy type failed, ret = %d\n",
				__func__, ret);

		return ret;
	}

	propval.intval = chg_type;
	ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		pr_notice("%s: psy type failed, ret(%d)\n", __func__, ret);

	propval.intval = chg_online;
	ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_ONLINE,
				&propval);
	if (ret < 0)
		pr_notice("%s: psy online failed, ret(%d)\n", __func__, ret);
	return ret;
}

#if defined(CONFIG_FPGA_EARLY_PORTING)
/* FPGA */
int hw_charging_get_charger_type(void)
{
	/* Force Standard USB Host */
	g_chr_type = STANDARD_HOST;
	chrdet_inform_psy_changed(g_chr_type, 1);
	return g_chr_type;
}

#else
/* EVB / Phone */
static void hw_bc11_init(void)
{
	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0);
	/* bc11_RST=1 */
	bc11_set_register_value(PMIC_RG_BC11_RST, 1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(PMIC_RG_BC11_BB_CTRL, 1);
	/* add pull down to prevent PMIC leakage */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	msleep(50);

	Charger_Detect_Init();
}

static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPU_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA1(void)
{
	unsigned int wChargerAvail = 0;

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);//vref=1.2v
#else
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
#endif
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*enable the voltage source to DM*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	/*reset to default value*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	if (wChargerAvail == 1) {
		bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
		pr_info("charger type: DCP, keep DM voltage source in stepB2\n");
	}
	return wChargerAvail;

}

static void hw_bc11_done(void)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 0x0);
	Charger_Detect_Release();
}
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
static void dump_charger_name(CHARGER_TYPE type)
{
	switch (type) {
	case CHARGER_UNKNOWN:
		pr_info("charger type: %d, CHARGER_UNKNOWN\n", type);
		break;
	case STANDARD_HOST:
		pr_info("charger type: %d, Standard USB Host\n", type);
		break;
	case CHARGING_HOST:
		pr_info("charger type: %d, Charging USB Host\n", type);
		break;
	case NONSTANDARD_CHARGER:
		pr_info("charger type: %d, Non-standard Charger\n", type);
		break;
	case STANDARD_CHARGER:
		pr_info("charger type: %d, Standard Charger\n", type);
		break;
	case APPLE_2_1A_CHARGER:
		pr_info("charger type: %d, APPLE_2_1A_CHARGER\n", type);
		break;
	case APPLE_1_0A_CHARGER:
		pr_info("charger type: %d, APPLE_1_0A_CHARGER\n", type);
		break;
	case APPLE_0_5A_CHARGER:
		pr_info("charger type: %d, APPLE_0_5A_CHARGER\n", type);
		break;
	default:
		pr_info("charger type: %d, Not Defined!!!\n", type);
		break;
	}
}
#endif
int hw_charging_get_charger_type(void)
{
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
			CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;
			unsigned long long t1, t2;
			int retry = 0;
			int timeout = 20;//200

			msleep(200);
			mutex_lock(&chrdet_lock);
			if (first_connect == true) {
				/* add make sure USB Ready */
				if (is_usb_rdy() == false) {
					pr_info("CDP, block\n");
					while (is_usb_rdy() == false && timeout > 0) {
						msleep(100);
						timeout--;
					}
					if (timeout == 0) {
						pr_info("CDP, timeout\n");
						mutex_unlock(&chrdet_lock);
						return NONSTANDARD_CHARGER;
					}
					else
						pr_info("CDP, free\n");
				} else
					pr_info("CDP, PASS\n");
				//first_connect = false;
			}
			mutex_unlock(&chrdet_lock);
	
			do {
					mutex_lock(&chrdet_lock);
					t1 = sched_clock();
					hw_bc11_init();//Step initial
	
					if (1 != upmu_get_rgs_chrdet()) {
							pr_info("usb is not present, break\n");
							hw_bc11_done();
							mutex_unlock(&chrdet_lock);
							break;
					}
	
					if (hw_bc11_DCD()) {//Step DCD
							if (hw_bc11_stepA1()) {//Step A1
									CHR_Type_num = APPLE_2_1A_CHARGER;
									pr_info("step A1 : Apple 2.1A CHARGER!\n");
							} else {
									CHR_Type_num = NONSTANDARD_CHARGER;
									pr_info("step A1 : Non STANDARD CHARGER!\n");
							}
					} else {
							if (hw_bc11_stepA2()) {//Step A2
									if (hw_bc11_stepB2()) {//Step B2
											CHR_Type_num = STANDARD_CHARGER;
											pr_info("step B2 : STANDARD CHARGER!\n");
											mutex_unlock(&chrdet_lock);
											break;
									} else {
											CHR_Type_num = CHARGING_HOST;
											pr_info("step B2 : Charging Host!\n");
									}
							} else {
									CHR_Type_num = STANDARD_HOST;
									pr_info("step A2 : Standard USB Host!\n");
							}
					}
	
			/* pull up DP current source. Skip hw_bc11_done(). */
			if (CHR_Type_num != STANDARD_CHARGER)
				hw_bc11_done();//Finally setting
			else
				pr_info("charger type: skip bc11 release for BC12 DCP SPEC\n");
	
					t2 = sched_clock();
					pr_info("chr_type_num=%d, retry=%d, pdetect_time=%lld\n",
									CHR_Type_num, retry, (t2-t1));
	
					mutex_unlock(&chrdet_lock);
			} while((NONSTANDARD_CHARGER == CHR_Type_num) && (retry++ < 1));
	
			return CHR_Type_num;
#else

	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
	if (usb2jtag_mode()) {
		pr_info("[USB2JTAG] in usb2jtag mode, skip charger detection\n");
		return STANDARD_HOST;
	}
#endif

	hw_bc11_init();

	if (hw_bc11_DCD()) {
		if (hw_bc11_stepA1())
			CHR_Type_num = APPLE_2_1A_CHARGER;
		else
			CHR_Type_num = NONSTANDARD_CHARGER;
	} else {
		if (hw_bc11_stepA2()) {
			if (hw_bc11_stepB2())
				CHR_Type_num = STANDARD_CHARGER;
			else
				CHR_Type_num = CHARGING_HOST;
		} else
			CHR_Type_num = STANDARD_HOST;
	}

	if (CHR_Type_num != STANDARD_CHARGER)
		hw_bc11_done();
	else
		pr_info("charger type: skip bc11 release for BC12 DCP SPEC\n");

	dump_charger_name(CHR_Type_num);


#ifdef __FORCE_USB_TYPE__
	CHR_Type_num = STANDARD_HOST;
	pr_info("charger type: Froce to STANDARD_HOST\n");
#endif

	return CHR_Type_num;
#endif

}

/* Charger Detection */
void do_charger_detect(void)
{
	if (!mt_usb_is_device()) {
		g_chr_type = CHARGER_UNKNOWN;
		pr_info("charger type: UNKNOWN, Now is usb host mode. Skip detection\n");
		return;
	}

	if (is_meta_mode()) {
		/* Skip charger type detection to speed up meta boot */
		pr_notice("charger type: force Standard USB Host in meta\n");
		if (pmic_get_register_value(PMIC_RGS_CHRDET)) {
			g_chr_type = STANDARD_HOST;
			chrdet_inform_psy_changed(g_chr_type, 1);
		} else {
			g_chr_type = CHARGER_UNKNOWN;
			chrdet_inform_psy_changed(g_chr_type, 0);
		}
		return;
	}

	mutex_lock(&chrdet_lock);

	if (pmic_get_register_value(PMIC_RGS_CHRDET)) {
		pr_info("charger type: charger IN\n");
		g_chr_type = hw_charging_get_charger_type();
		chrdet_inform_psy_changed(g_chr_type, 1);
	} else {
		pr_info("charger type: charger OUT\n");
		g_chr_type = CHARGER_UNKNOWN;
		chrdet_inform_psy_changed(g_chr_type, 0);
	}

	mutex_unlock(&chrdet_lock);
}



/* PMIC Int Handler */
void chrdet_int_handler(void)
{
	/*
	 * pr_notice("[chrdet_int_handler]CHRDET status = %d....\n",
	 *	pmic_get_register_value(PMIC_RGS_CHRDET));
	 */
	if (!pmic_get_register_value(PMIC_RGS_CHRDET)) {
		int boot_mode = 0;

		hw_bc11_done();
		boot_mode = get_boot_mode();

		if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
		    || boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			pr_info("[chrdet_int_handler] Unplug Charger/USB\n");
#ifndef CONFIG_TCPC_CLASS
			orderly_poweroff(true);
#else
			return;
#endif
		}
	}
	do_charger_detect();
}


/* Charger Probe Related */
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static void do_charger_detection_work(struct work_struct *data)
{
	if (pmic_get_register_value(PMIC_RGS_CHRDET))
		do_charger_detect();
}
#endif
#if defined(CONFIG_VIVO_CHARGING_NEW_ARCH)
extern void do_chrdet_int_task_pmic(void);
#endif
static int __init pmic_chrdet_init(void)
{
	mutex_init(&chrdet_lock);
	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy) {
		pr_notice("%s: get power supply failed\n", __func__);
#if defined(CONFIG_VIVO_CHARGING_NEW_ARCH)
		pmic_register_interrupt_callback(INT_CHRDET_EDGE, do_chrdet_int_task_pmic);
		pmic_enable_interrupt(INT_CHRDET_EDGE, 1, "PMIC");
#endif
		return -EINVAL;
	}

#ifdef __SW_CHRDET_IN_PROBE_PHASE__
	/* do charger detect here to prevent HW miss interrupt*/
	INIT_WORK(&chr_work, do_charger_detection_work);
	schedule_work(&chr_work);
#endif

	pmic_register_interrupt_callback(INT_CHRDET_EDGE, chrdet_int_handler);
	pmic_enable_interrupt(INT_CHRDET_EDGE, 1, "PMIC");

	return 0;
}

late_initcall(pmic_chrdet_init);

#endif /* CONFIG_FPGA_EARLY_PORTING */
