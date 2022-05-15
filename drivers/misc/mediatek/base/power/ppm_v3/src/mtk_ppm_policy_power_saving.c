/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

#include "mtk_ppm_internal.h"


static void ppm_power_saving_update_limit_cb(void);
static void ppm_power_saving_status_change_cb(bool enable);

/* other members will init by ppm_main */
static struct ppm_policy_data power_saving_policy = {
	.name			= __stringify(PPM_POLICY_POWER_SAVING),
	.lock			= __MUTEX_INITIALIZER(power_saving_policy.lock),
	.policy			= PPM_POLICY_POWER_SAVING,
	.priority		= PPM_POLICY_PRIO_USER_SPECIFY_BASE,
	.update_limit_cb	= ppm_power_saving_update_limit_cb,
	.status_change_cb	= ppm_power_saving_status_change_cb,
};

struct ppm_userlimit_data power_saving_data = {
	.is_freq_limited_by_user = false,
	.is_core_limited_by_user = false,
};


/* MUST in lock */
static bool ppm_power_saving_is_policy_active(void)
{
	if (!power_saving_data.is_core_limited_by_user &&
		!power_saving_data.is_freq_limited_by_user)
		return false;
	else
		return true;
}

static void ppm_power_saving_update_limit_cb(void)
{
	unsigned int i;
	struct ppm_policy_req *req = &power_saving_policy.req;

	FUNC_ENTER(FUNC_LV_POLICY);

	if (power_saving_data.is_core_limited_by_user || power_saving_data.is_freq_limited_by_user) {
		ppm_clear_policy_limit(&power_saving_policy);

		for (i = 0; i < req->cluster_num; i++) {
			req->limit[i].min_cpu_core = (power_saving_data.limit[i].min_core_num == -1)
				? req->limit[i].min_cpu_core
				: power_saving_data.limit[i].min_core_num;
			req->limit[i].max_cpu_core = (power_saving_data.limit[i].max_core_num == -1)
				? req->limit[i].max_cpu_core
				: power_saving_data.limit[i].max_core_num;

			req->limit[i].min_cpufreq_idx = (power_saving_data.limit[i].min_freq_idx == -1)
				? req->limit[i].min_cpufreq_idx
				: power_saving_data.limit[i].min_freq_idx;
			req->limit[i].max_cpufreq_idx = (power_saving_data.limit[i].max_freq_idx == -1)
				? req->limit[i].max_cpufreq_idx
				: power_saving_data.limit[i].max_freq_idx;
		}
	}

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_power_saving_status_change_cb(bool enable)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: power_saving policy status changed to %d\n", __func__, enable);

	FUNC_EXIT(FUNC_LV_POLICY);
}

unsigned int mt_ppm_power_saving_cpu_core(unsigned int cluster_num, struct ppm_limit_data *data)
{
	int i = 0;
	int min_core, max_core;
	bool is_limit = false;

	/* Error check */
	if (cluster_num > NR_PPM_CLUSTERS) {
		ppm_err("@%s: Invalid cluster num = %d\n", __func__, cluster_num);
		return -1;
	}

	if (!data) {
		ppm_err("@%s: limit data is NULL!\n", __func__);
		return -1;
	}

	for (i = 0; i < cluster_num; i++) {
		min_core = data[i].min;
		max_core = data[i].max;

		/* invalid input check */
		if (min_core != -1 && min_core < (int)get_cluster_min_cpu_core(i)) {
			ppm_err("@%s: Invalid input! min_core for cluster %d = %d\n", __func__, i, min_core);
			return -1;
		}
		if (max_core != -1 && max_core > (int)get_cluster_max_cpu_core(i)) {
			ppm_err("@%s: Invalid input! max_core for cluster %d = %d\n", __func__, i, max_core);
			return -1;
		}

		/* check is all limit clear or not */
		if (min_core != -1 || max_core != -1)
			is_limit = true;

		/* sync to max_core if min > max */
		if (min_core != -1 && max_core != -1 && min_core > max_core)
			data[i].min = data[i].max;
	}

	ppm_lock(&power_saving_policy.lock);
	if (!power_saving_policy.is_enabled) {
		ppm_warn("@%s: power_saving policy is not enabled!\n", __func__);
		ppm_unlock(&power_saving_policy.lock);
		return -1;
	}

	/* update policy data */
	for (i = 0; i < cluster_num; i++) {
		min_core = data[i].min;
		max_core = data[i].max;

		if (min_core != power_saving_data.limit[i].min_core_num ||
			max_core != power_saving_data.limit[i].max_core_num) {
			power_saving_data.limit[i].min_core_num = min_core;
			power_saving_data.limit[i].max_core_num = max_core;
			ppm_info("update power_saving min/max core for cluster %d = %d/%d\n",
				i, min_core, max_core);
		}
	}

	power_saving_data.is_core_limited_by_user = is_limit;
	power_saving_policy.is_activated = ppm_power_saving_is_policy_active();

	ppm_unlock(&power_saving_policy.lock);
	mt_ppm_main();

	return 0;
}

unsigned int mt_ppm_power_saving_cpu_freq(unsigned int cluster_num, struct ppm_limit_data *data)
{
	int i = 0;
	int min_freq, max_freq, min_freq_idx, max_freq_idx;
	bool is_limit = false;
	
	/* Error check */	
	if (cluster_num > NR_PPM_CLUSTERS) {		
		ppm_err("@%s: Invalid cluster num = %d\n", __func__, cluster_num);		
		return -1;	
	}	

	if (!data) {		
		ppm_err("@%s: limit data is NULL!\n", __func__);		
		return -1;	
	}	ppm_lock(&power_saving_policy.lock);	

	if (!power_saving_policy.is_enabled) {		
		ppm_warn("@%s: power_saving policy is not enabled!\n", __func__);		
		ppm_unlock(&power_saving_policy.lock);		
		return -1;	
	}	

	/* update policy data */	
	for (i = 0; i < cluster_num; i++) {		
		min_freq = data[i].min;		
		max_freq = data[i].max;		
		/* check is all limit clear or not */		
		if (min_freq != -1 || max_freq != -1)			
			is_limit = true;		

		/* freq to idx */		
		min_freq_idx = (min_freq == -1) ? -1				
			: ppm_main_freq_to_idx(i, min_freq, CPUFREQ_RELATION_L);		
		max_freq_idx = (max_freq == -1) ? -1				
			: ppm_main_freq_to_idx(i, max_freq, CPUFREQ_RELATION_H);		

		/* sync to max_freq if min < max */		
		if (min_freq_idx != -1 && max_freq_idx != -1 && min_freq_idx < max_freq_idx)			
			min_freq_idx = max_freq_idx;		

		/* write to policy data */		
		if (min_freq_idx != power_saving_data.limit[i].min_freq_idx ||
			max_freq_idx != power_saving_data.limit[i].max_freq_idx) {			
			power_saving_data.limit[i].min_freq_idx = min_freq_idx;			
			power_saving_data.limit[i].max_freq_idx = max_freq_idx;			
			ppm_info("update power_saving min/max freq for cluster %d = %d(%d)/%d(%d)\n",
				i, min_freq, min_freq_idx, max_freq, max_freq_idx);		
		}	
	}	

	power_saving_data.is_freq_limited_by_user = is_limit;	
	power_saving_policy.is_activated = ppm_power_saving_is_policy_active();	

	ppm_unlock(&power_saving_policy.lock);	
	mt_ppm_main();	

	
	return 0;
}


static int ppm_power_saving_cpu_core_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < power_saving_policy.req.cluster_num; i++) {
		seq_printf(m, "cluster %d: min_core_num = %d, max_core_num = %d\n",
			i, power_saving_data.limit[i].min_core_num, power_saving_data.limit[i].max_core_num);
	}

	return 0;
}

static ssize_t ppm_power_saving_cpu_core_proc_write(struct file *file, const char __user *buffer,
					size_t count,	loff_t *pos)
{
	int i = 0, data;
	struct ppm_limit_data core_limit[NR_PPM_CLUSTERS];
	unsigned int arg_num = NR_PPM_CLUSTERS * 2; /* for min and max */
	char *tok, *tmp;
	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	tmp = buf;
	while ((tok = strsep(&tmp, " ")) != NULL) {
		if (i == arg_num) {
			ppm_err("@%s: number of arguments > %d!\n", __func__, arg_num);
			goto out;
		}

		if (kstrtoint(tok, 10, &data)) {
			ppm_err("@%s: Invalid input: %s\n", __func__, tok);
			goto out;
		} else {
			if (i % 2) /* max */
				core_limit[i/2].max = data;
			else /* min */
				core_limit[i/2].min = data;

			i++;
		}
	}

	if (i < arg_num)
		ppm_err("@%s: number of arguments < %d!\n", __func__, arg_num);
	else
		mt_ppm_power_saving_cpu_core(NR_PPM_CLUSTERS, core_limit);

out:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_power_saving_cpu_freq_proc_show(struct seq_file *m, void *v)
{	
	int i;	
	for (i = 0; i < power_saving_policy.req.cluster_num; i++) {		
		seq_printf(m, "cluster %d: min_freq_idx = %d, max_freq_idx = %d\n",			
			i, power_saving_data.limit[i].min_freq_idx, power_saving_data.limit[i].max_freq_idx);	
	}	

	return 0;
}

static ssize_t ppm_power_saving_cpu_freq_proc_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *pos)
{	
	int i = 0, data;	
	struct ppm_limit_data freq_limit[NR_PPM_CLUSTERS];
	unsigned int arg_num = NR_PPM_CLUSTERS * 2;/* for min and max */
	char *tok, *tmp;	
	char *buf = ppm_copy_from_user_for_proc(buffer, count);	

	if (!buf) {	
		return -EINVAL;
	}
	tmp = buf;	

	while ((tok = strsep(&tmp, " ")) != NULL) {		
		if (i == arg_num) {			
			ppm_err("@%s: number of arguments > %d!\n", __func__, arg_num);			
			goto out;		
		}		

		if (kstrtoint(tok, 10, &data)) {			
			ppm_err("@%s: Invalid input: %s\n", __func__, tok);			
			goto out;		
		} else {			
			if (i % 2) /* max */
				freq_limit[i/2].max = data;
			else /* min */
				freq_limit[i/2].min = data;
			
			i++;		
		}	
	}
	
	if (i < arg_num)
		ppm_err("@%s: number of arguments < %d!\n", __func__, arg_num);
	else		
		mt_ppm_power_saving_cpu_freq(NR_PPM_CLUSTERS, freq_limit);

out:	
	free_page((unsigned long)buf);	
	return count;
}


PROC_FOPS_RW(power_saving_cpu_core);
PROC_FOPS_RW(power_saving_cpu_freq);


static int __init ppm_power_saving_policy_init(void)
{
	int i, ret = 0;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(power_saving_cpu_core),
		PROC_ENTRY(power_saving_cpu_freq),
	};

	FUNC_ENTER(FUNC_LV_POLICY);

	/* create procfs */
	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, policy_dir, entries[i].fops)) {
			ppm_err("%s(), create /proc/ppm/policy/%s failed\n", __func__, entries[i].name);
			ret = -EINVAL;
			goto out;
		}
	}

	power_saving_data.limit = kcalloc(ppm_main_info.cluster_num, sizeof(*power_saving_data.limit), GFP_KERNEL);
	if (!power_saving_data.limit) {
		ret = -ENOMEM;
		goto out;
	}

	/* init power_saving_data */
	for_each_ppm_clusters(i) {
		power_saving_data.limit[i].min_freq_idx = -1;
		power_saving_data.limit[i].max_freq_idx = -1;
		power_saving_data.limit[i].min_core_num = -1;
		power_saving_data.limit[i].max_core_num = -1;
	}

	if (ppm_main_register_policy(&power_saving_policy)) {
		ppm_err("@%s: power_saving policy register failed\n", __func__);
		kfree(power_saving_data.limit);
		ret = -EINVAL;
		goto out;
	}

	ppm_info("@%s: register %s done!\n", __func__, power_saving_policy.name);

out:
	FUNC_EXIT(FUNC_LV_POLICY);

	return ret;
}

static void __exit ppm_power_saving_policy_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	kfree(power_saving_data.limit);

	ppm_main_unregister_policy(&power_saving_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}

module_init(ppm_power_saving_policy_init);
module_exit(ppm_power_saving_policy_exit);

