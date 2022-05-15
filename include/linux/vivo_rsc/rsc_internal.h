/*
 * kernel/rsc/rsc_internal.h
 *
 * VIVO Resource Control.
 *
 */

#ifndef __RSC_INTERNAL_H__
#define __RSC_INTERNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/list.h>
#include <linux/uidgid.h>
#include <linux/fs.h>
#include <linux/vivo_rsc_ioctl.h>

/* operation */
#define MAX(a, b)		((a) >= (b) ? (a) : (b))
#define MIN(a, b)		((a) >= (b) ? (b) : (a))

/* LOCK */
#define rsc_lock(lock)		mutex_lock(lock)
#define rsc_unlock(lock)	mutex_unlock(lock)


struct rsc_global_attr {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *a, struct attribute *b,
					const char *c, size_t count);
};

#ifndef define_rsc_one_global_ro
#define define_rsc_one_global_ro(_name)		\
static struct rsc_global_attr _name =		\
__ATTR(_name, 0440, show_##_name, NULL)
#endif

#ifndef define_rsc_one_global_rw
#define define_rsc_one_global_rw(_name)		\
static struct rsc_global_attr _name =		\
__ATTR(_name, 0640, show_##_name, store_##_name)
#endif


/* LOG */
#undef TAG
#define TAG     "^RSC "

#define rsc_err(fmt, args...)		\
	pr_err(TAG"[ERROR]"fmt, ##args)
#define rsc_warn(fmt, args...)		\
	pr_warn(TAG"[WARNING]"fmt, ##args)
#define rsc_info(fmt, args...)		\
	pr_warn(TAG""fmt, ##args)
#define rsc_dbg(type, fmt, args...)				\
	do {											\
		if (rsc_debug & ALL || rsc_debug & type)	\
			rsc_info(fmt, ##args);					\
	} while (0)
#define rsc_ver(fmt, args...)			\
	do {					\
		if (rsc_debug == ALL)		\
			rsc_info(fmt, ##args);	\
	} while (0)
#define rsc_cont(fmt, args...)		\
	pr_cont(fmt, ##args)

#define rsc_main(fmt, args...)			\
	do {					\
		if (rsc_debug & MAIN)		\
			rsc_info(fmt, ##args);	\
	} while (0)

/* default value is print*/
#define rsc_cpucap(fmt, args...)			\
			do {					\
				if (!(rsc_debug & CPU_CAPINF_OFF))		\
					rsc_info(fmt, ##args);	\
			} while (0)

#define FUNC_LV_MODULE		BIT(0)	/* module, platform driver interface */
#define FUNC_LV_API			BIT(1)	/* mt_rsc driver global function */
#define FUNC_LV_MAIN		BIT(2)	/* mt_rsc driver main function */
#define FUNC_LV_HICA		BIT(3)	/* mt_rsc driver HICA related function */
#define FUNC_LV_POLICY		BIT(4)	/* mt_rsc driver other policy function */

#define FUNC_ENTER(lv)	\
	do { if ((lv) & rsc_func_lv_mask) rsc_info(">> %s()\n", __func__); } while (0)
#define FUNC_EXIT(lv)	\
	do { if ((lv) & rsc_func_lv_mask) rsc_info("<< %s():%d\n", __func__, __LINE__); } while (0)

/*==============================================================*/
/* Enum								*/
/*==============================================================*/
enum {
	NO_LOG	= 0,
	ALL		= 1 << 0,/*1*/
	MAIN	= 1 << 1,/*2*/
	PROFILE = 1 << 2,/*4*/
	CPU_CAP = 1 << 3,/*8*/
	CPU_CAPINF_OFF = 1 << 4,/* 16 , 1 means off*/
	CPU_TASK_USAGE = 1 << 5,/* 32*/
	UID_IO = 1 << 6,/* 64*/
	UID_LMK = 1 << 7,/* 128*/
	CPU_TOP = 1 << 8,/* 256*/
	IOLIMIT = 1 << 9,/* 512*/
};

enum rsc_type {
	RSC_TYPE_CPU_LEVEL = 0,
	RSC_TYPE_CPU_INTERNAL,
	RSC_TYPE_GPU_LEVEL,
	RSC_TYPE_MEM_LEVEL,
	RSC_TYPE_BUS_LEVEL,
	RSC_TYPE_IO_LEVEL,
	RSC_TYPE_SCHED_LEVEL,
	RSC_TYPE_IOSCHED_LEVEL,
	RSC_TYPE_VM_OPTS_LEVEL,
	RSC_TYPE_LOW_MEM_KILLER_LEVEL,
	RSC_TYPE_MAX,
};

enum rsc_mode {
	RSC_MODE_LOW_POWER = 0,
	RSC_MODE_NORMAL,
	RSC_MODE_PERFORMANCE,
};

enum rsc_plat_mode {
	RSC_PLAT_MTK = 0,
	RSC_PLAT_SAMSUNG,
	RSC_PLAT_QUALCOMM,
};

enum power_state_search_policy {
	PERFORMANCE = 0,
	LOW_POWER,
};

#define CPU_PERD_FREQ_ID 		89889966557766
#define CPU_POWER_FREQ_ID 		89888568958888

#define CPU_INTERNAL_FREQ_ID 	89155075917309

#define CPU_LEVEL_FREQ_ID 		89168689977309

#define RSC_END_PID 0xffffffff

#define RSC_UID_IO_EXIT 	(1 << 0)
#define RSC_CPU_TOP_EXIT 	(1 << 1)

/*==============================================================*/
/* Data Structures*/
/*==============================================================*/

#define MAX_VAL_NUM 4

struct rsc_qos_udate {
	int val;
	int timeout;/*ms*/
	u64 settime;
};

struct rsc_qos_val {
	struct rsc_qos_udate data;
	bool is_new_add;
};

struct rsc_upper_req {
	struct list_head node;

	enum rsc_type types;

	/* hash of package name */
	u64 id;

	/* hash of package name */
	char *name;

	/* virtual level */
	int minlevel;
	int maxlevel;

	/* -1 means no time limit */
	long timeout;

	int v_num;
	int req_num;
	unsigned int req_bitmap;
	struct rsc_qos_udate *pval;

};

struct rsc_pm_qos_req {
	/* hash of package name */
	u64 id;

	/* hash of package name */
	char *name;
	/* virtual level */
	int minlevel;
	int maxlevel;
	/* -1 means no time limit */
	long timeout;
	/* request to pm qos module */
	struct pm_qos_request rsc_level_min;
	struct pm_qos_request rsc_level_max;
	int v_num;
	int req_num;
	unsigned int req_bitmap;
	struct rsc_qos_val *pval;
	struct pm_qos_request *rsc_qos_req;
	bool is_new_add;
	/* lock */
	struct mutex lock;
	/* for profile */
	unsigned long last_atime;
	unsigned long counts;

	/* link to rsc_policy_data.pm_qos_req_head */
	struct list_head link;
};

struct rsc_policy_data {
	/* settings */
	const char *name;
	enum rsc_type types;

	/* status */
	bool is_enabled;
	/* lock */
	struct mutex lock;
	/* link to rsc_data.sub_module_list*/
	struct list_head link;
	/* pm qos list header */
	struct list_head pm_qos_req_head;

	/* callbacks */
	void (*update_cb)(struct rsc_pm_qos_req *req);
	void (*mode_change_cb)(enum rsc_mode mode);
};

struct rsc_data {
	/* status */
	enum rsc_mode cur_mode;
	enum rsc_plat_mode cur_plat;	/* mtk/samsung/qualcomm */

	bool is_enabled;
	bool is_in_suspend;

	/* platform dev/driver */
	const struct dev_pm_ops rsc_pm_ops;
	struct platform_device rsc_pdev;
	struct platform_driver rsc_pdrv;

	/* rsc core data */
	struct mutex lock;
	struct list_head sub_module_list;
};

#define RSC_MAX_ID_SIZE	16
#define RSC_ID_PREFIX "RSC_ID_"
struct rc_id_map_t {
	u32 id;
	const char *name;
};

static const struct rc_id_map_t const rsc_ids[RSC_ID_END - RSC_ID_START] = {
	{RSC_ID_PERFD, RSC_ID(RSC_ID_PERFD)},
	{RSC_ID_BBKLOG, RSC_ID(RSC_ID_BBKLOG)},
	{RSC_ID_POWER, RSC_ID(RSC_ID_POWER)},
	{RSC_ID_VIVOD, RSC_ID(RSC_ID_VIVOD)},
	{RSC_ID_BIGDATA, RSC_ID(RSC_ID_BIGDATA)},
	{RSC_ID_SHELL, RSC_ID(RSC_ID_SHELL)},
	{RSC_ID_OTHER, RSC_ID(RSC_ID_OTHER)}
};

/*==============================================================*/
/* Global variables						*/
/*==============================================================*/
extern struct rsc_data rsc_main_info;
extern struct kobject *rsc_root_dir;
extern struct proc_dir_entry *vivo_rsc;

extern unsigned int rsc_func_lv_mask;
extern unsigned int rsc_debug;

/*==============================================================*/
/* APIs								*/
/*==============================================================*/
/* procfs */
extern int rsc_sys_api_init(void);

/* main */
extern int rsc_main_register_sub_module(struct rsc_policy_data *policy);
extern void rsc_main_unregister_sub_module(struct rsc_policy_data *policy);
extern int rsc_main_update(struct rsc_upper_req *up, struct rsc_policy_data *policy);

/* profiling */
extern int rsc_profile_init(void);
extern void rsc_profile_exit(void);
extern int rsc_cpu_initialized;
extern int update_rsc_cpu_freq(u64 id, struct rsc_qos_udate *cluster_freq, int input_num, unsigned int req_bitmap);
extern int rsc_chown_to_system(struct kobject *kobj, const struct attribute *attr);
#if defined(CONFIG_RSC_UID_IO) && defined(CONFIG_RSC_CPU_TOP)
int rsc_top_process_notifier(struct notifier_block *self,
			unsigned long cmd, void *v);
#endif

#define SYSTEM_ID KUIDT_INIT(1000)
#define SYSTEM_GROUP_ID KGIDT_INIT(1000)

#ifdef __cplusplus
}
#endif

#endif /*__RSC_INTERNAL_H__*/
