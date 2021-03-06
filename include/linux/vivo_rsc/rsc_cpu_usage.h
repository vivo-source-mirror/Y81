/*
 * include/linux/vivo_rsc/rsc_cpu_usage.h
 *
 * VIVO Resource Control.
 *
 * stat task cpu usage.
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */

#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/kernel_stat.h>

/*debug switch*/
#if 0
#define SHOW_TASK_INFO_IN_PRINTK
#endif

#if 0
#define RECORD_GET_LOCK_FAIL
#endif

#if 0
#define RECORD_STAT_EXPTIME
#endif

#if 0
#define RECORD_CPUBIT
#endif

#if 0
#define RECORD_DEBUG_LOG
#endif

#if 0
#define  RECORD_IDLE_TICK_CNT
#endif

#if 0
#define  RSC_USE_TASKLOCK_LIST
#endif

/*it can save many cpu usage*/
#define SORT_IN_USER_PROCESS

/*notify by loading, if not defined, notify by percpu loading*/
#define RSC_NOTIFY_BY_LOADING

/*use system cpustat[CPUTIME_IRQ] and cpustat[CPUTIME_SOFTIRQ]*/
#define RSC_USE_SYS_IRQ_TIME

/*
RSC_STAT_MAX_TASK is not allow set to 256, must be 8,16,32,64,128,
see task_cpu_usage_map_cnt
*/
#define RSC_STAT_MAX_TASK 16
#define RSC_STAT_MAX_TASK_MASK	(RSC_STAT_MAX_TASK - 1)

#define RSC_STAT_IVALID_IDX 0xff

#define RSC_GET_LOCK_FAIL_MAX_TASK	8

#define RSC_MAX_HASH_TASK	256
#define RSC_MAX_HASH_TASK_MASK	(RSC_MAX_HASH_TASK - 1)

#define CPU_USAGE_NAME ktop
#define CPU_USAGE_SET ktop_set
#define CPU_USAGE_PERCENT ktop_percent
#define CPU_USAGE_FPID ktop_fpid

#define RSC_SHOW_MAX_TASK 32

struct rsc_task_cpu_usage_t {
	struct task_struct *task;
	/*struct thread_info *tinf;*/
	u32 utime;
	u32 stime;
	u32 time;
	pid_t pid;
	pid_t tgid;
	/*parent pid*/
	pid_t ppid;
	/*pid_t ptgid;*/
#ifdef RECORD_STAT_EXPTIME
	u32 exptime;
	u32 minexptime;
#endif
	u32 seq;
#ifdef RECORD_CPUBIT
	u16	cpubit;
#endif
	u8 is_exit;
	char comm[TASK_COMM_LEN];
	char tgcomm[TASK_COMM_LEN];
};

DECLARE_PER_CPU(struct rsc_task_cpu_usage_t,
	task_cpu_usage[RSC_STAT_MAX_TASK]);
DECLARE_PER_CPU(u8, task_cpu_usage_tbl[RSC_STAT_MAX_TASK]);
DECLARE_PER_CPU(u32, task_cpu_usage_cnt);
DECLARE_PER_CPU(u32, task_cpu_usage_idx);
#ifdef RECORD_DEBUG_LOG
DECLARE_PER_CPU(u32, task_call_cnt);
#endif
#ifdef RECORD_IDLE_TICK_CNT
DECLARE_PER_CPU(u32, task_cpu_usage_idle);
#endif
DECLARE_PER_CPU(u32, task_cpu_usage_notidle);
DECLARE_PER_CPU(u32, task_cpu_usage_total);
DECLARE_PER_CPU(u32, task_cpu_usage_user);
/*hardware irq*/
DECLARE_PER_CPU(u32, task_cpu_usage_hirq);
/*soft irq*/
DECLARE_PER_CPU(u32, task_cpu_usage_sirq);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) || defined(RSC_USE_SYS_IRQ_TIME)
/*hardware irq*/
DECLARE_PER_CPU(u64, task_cpu_usage_hirq_time);
/*soft irq*/
DECLARE_PER_CPU(u64, task_cpu_usage_sirq_time);
#endif
DECLARE_PER_CPU(u8,
	task_cpu_usage_map_cnt[RSC_MAX_HASH_TASK]);

DECLARE_PER_CPU(u8,
	task_cpu_usage_map_idx[RSC_MAX_HASH_TASK]);

#ifdef RECORD_GET_LOCK_FAIL
extern atomic_t task_get_lock_fail_cnt;
extern struct rsc_task_cpu_usage_t task_cpu_usage_fail[RSC_GET_LOCK_FAIL_MAX_TASK];
#endif

extern atomic_t __read_mostly task_cpu_usage_enable ;
extern u32 __read_mostly task_cpu_usage_seq;
extern const u32 max_stat_task;

#ifdef RECORD_GET_LOCK_FAIL
static inline void update_tasklock_fail(struct task_struct *p,
	int cpu, int tasklock, const char *str)
{
	unsigned int cnt;
	struct rsc_task_cpu_usage_t *fitm;
	int maxrec;

	/*make sure group_leader is NULL can record*/
	if (tasklock)
		maxrec = RSC_GET_LOCK_FAIL_MAX_TASK/2;
	else
		maxrec = RSC_GET_LOCK_FAIL_MAX_TASK;

	if (atomic_read(&task_get_lock_fail_cnt) < maxrec) {
		cnt = atomic_inc_return(&task_get_lock_fail_cnt);
		if (cnt < RSC_GET_LOCK_FAIL_MAX_TASK) {
			fitm = &task_cpu_usage_fail[cnt];
			fitm->time = p->rsc_utime[cpu] + p->rsc_stime[cpu];
			fitm->utime = p->rsc_utime[cpu];
			fitm->stime = p->rsc_stime[cpu];
			fitm->pid = p->pid;
			fitm->tgid = p->tgid;
			fitm->task = p;
			/*fitm->tinf = current_thread_info();*/
			fitm->seq = p->rsc_seq;
#ifdef RECORD_CPUBIT
			fitm->cpubit = 1 << cpu;
#endif
			fitm->is_exit = 0;
			strlcpy(fitm->comm, p->comm, TASK_COMM_LEN);
			strlcpy(fitm->tgcomm, str, TASK_COMM_LEN);
			fitm->ppid = RSC_END_PID;
		}
	}
}
#endif

static inline void update_tgcomm(struct rsc_task_cpu_usage_t *itm,
	struct task_struct *p, int cpu)
{
	struct task_struct *father;
#ifdef RSC_USE_TASKLOCK_LIST
	if (read_trylock(&tasklist_lock)) {
	struct task_struct *pt;
#else
	struct task_struct *pt;

	rcu_read_lock();
#endif
		if (thread_group_leader(p)) {
		/*if (current->group_leader == p) {*/
			pt = rcu_dereference(p->real_parent);
			father = pt;
		} else {
			pt = rcu_dereference(p->group_leader);
			father = rcu_dereference(p->real_parent);
		}
		if (pt) {
			strlcpy(itm->tgcomm, pt->comm, TASK_COMM_LEN);
		#if 1
			if (father)
				itm->ppid = father->pid;
			else
				itm->ppid = RSC_END_PID;
		#else
			itm->ppid = pt->pid;
		#endif
			/*itm->ptgid = pt->tgid;*/
		} else {
			strlcpy(itm->tgcomm, "group_leader", TASK_COMM_LEN);
			itm->ppid = RSC_END_PID;
			/*itm->ptgid = RSC_END_PID;*/
#ifdef RECORD_GET_LOCK_FAIL
			update_tasklock_fail(p, cpu, 0, "group_leader");
#endif
		}
#ifdef RSC_USE_TASKLOCK_LIST
		read_unlock(&tasklist_lock);
	} else {
		strlcpy(itm->tgcomm, "tasklist_lock", TASK_COMM_LEN);
		itm->ppid = RSC_END_PID;
	#ifdef RECORD_GET_LOCK_FAIL
		update_tasklock_fail(p, cpu, 1, "tasklist_lock");
	#endif
	}
#else
	rcu_read_unlock();
#endif
}

/*Note: call in timer tick, no need to protect the global var
	call backtrace:
	dump_backtrace+0x0/0x1f0
	show_stack+0x20/0x28
	dump_stack+0xb0/0xec
	rsc_task_usage_insert+0x10c/0x990
	account_process_tick+0x4c/0xd4
	update_process_times+0x30/0x68
	tick_sched_handle.isra.11+0x58/0x6c
	tick_sched_timer+0x84/0x1a0
	__hrtimer_run_queues+0x1b0/0x334
	hrtimer_interrupt+0xa8/0x1c8
	arch_timer_handler_virt+0x3c/0x48
	handle_percpu_devid_irq+0xe4/0x1d8
	generic_handle_irq+0x30/0x44
	__handle_domain_irq+0x90/0xbc
	gic_handle_irq+0xe0/0x1a4
*/
static __always_inline void rsc_task_usage_insert(int cpu,
		u32 user_tick, u32 idle, u32 irqt, u32 sirqt,
		u32 ksoftirqtick, u32 ticks)
{
	int i, j;
	int size, idx, pos, preidx, tmp, l;
	int len, add;
	struct rsc_task_cpu_usage_t *itm;
	struct rsc_task_cpu_usage_t *data;
	u8 *tbl;
	struct task_struct *p;
	u32 rsc_time;
	u8 *map_cnt;
	u8 *map_idx;
	int map_pid;
#ifdef RECORD_STAT_EXPTIME
	int tidx = 0;
	u64 start, end;
#endif
	p = current;

	if (!atomic_read(&task_cpu_usage_enable))
		return;

	if (unlikely(irqt) || unlikely(sirqt)) {
		per_cpu(task_cpu_usage_hirq, cpu) += irqt;
		per_cpu(task_cpu_usage_notidle, cpu) += irqt + sirqt;
		per_cpu(task_cpu_usage_total, cpu) += irqt + sirqt;
		per_cpu(task_cpu_usage_sirq, cpu) += sirqt;
		return;
	}

	if (unlikely(idle)) {
#ifdef RECORD_IDLE_TICK_CNT
		per_cpu(task_cpu_usage_idle, cpu) += ticks;
#endif
		per_cpu(task_cpu_usage_total, cpu) += ticks;
		return;
	}

	per_cpu(task_cpu_usage_sirq, cpu) += ksoftirqtick;
	per_cpu(task_cpu_usage_notidle, cpu) += ticks;
	per_cpu(task_cpu_usage_total, cpu) += ticks;
#ifdef RECORD_STAT_EXPTIME
	start = local_clock();
#endif
	map_pid = p->pid & RSC_MAX_HASH_TASK_MASK;

	if (unlikely(task_cpu_usage_seq != p->rsc_seq)) {
		/*
		   also need to clear in task fork(clear fucntion: dup_task_struct)
		*/
		p->rsc_seq = task_cpu_usage_seq;
#ifdef RECORD_CPUBIT
		p->rsc_cpubit = 0;
#endif
		memset(p->rsc_utime, 0, sizeof(p->rsc_utime));
		memset(p->rsc_stime, 0, sizeof(p->rsc_stime));
		/*rsc_info("%s: rsc_seq: %d\n", __func__, task_cpu_usage_seq);*/
		/*dump_stack();*/
	}

	rsc_time = p->rsc_utime[cpu] + p->rsc_stime[cpu];
#ifdef RECORD_CPUBIT
	if (!(p->rsc_cpubit & (1 << cpu)))
		p->rsc_cpubit |= (1 << cpu);
#endif
	if (user_tick) {
		per_cpu(task_cpu_usage_user, cpu) += ticks;
		p->rsc_utime[cpu] += ticks;
	} else
		p->rsc_stime[cpu] += ticks;

	data = per_cpu(task_cpu_usage, cpu);
	tbl = per_cpu(task_cpu_usage_tbl, cpu);
	map_cnt = per_cpu(task_cpu_usage_map_cnt, cpu);
	map_idx = per_cpu(task_cpu_usage_map_idx, cpu);

	if (unlikely(!map_cnt[map_pid])) {
		/*rsc_info("hash find comm: %16s p->pid: %4d is none\n",
				p->comm, p->pid);*/
		len = per_cpu(task_cpu_usage_cnt, cpu);
		goto newtask;
	} else {
		pos = map_idx[map_pid];
		/*if (1 == map_cnt[map_pid]) {*/
			j = tbl[pos];
			/*(RSC_STAT_IVALID_IDX != pos)*/
			if ((p == data[j].task) && (p->pid == data[j].pid) &&
				/*(current_thread_info() == data[j].tinf) && */!data[j].is_exit) {
				/*rsc_info("hash fasthit cpu: %d comm: %16s p->pid: %4d\n",
					cpu, p->comm, p->pid);*/
				goto fasthit;
			/*}*/
		}
	}

	len = per_cpu(task_cpu_usage_cnt, cpu);
	idx = per_cpu(task_cpu_usage_idx, cpu);
	for (i = 0; i < len; i++) {
		pos = (idx + i) % len;
		j = tbl[pos];
		/*
		could not use task name comm to compare,
		because the task name can be changed,
		such the name of pid 1 process changes from swapper/0 to init
		*/
		/*if ((p == data[j].task) && (p->pid == data[j].pid) &&
			!strncmp(p->comm, data[j].comm, TASK_COMM_LEN))*/
		if ((p == data[j].task) && (p->pid == data[j].pid) &&
			/*(current_thread_info() == data[j].tinf) && */!data[j].is_exit) {
			if (!rsc_time) {
				data[j].is_exit = 1;
				rsc_err("%s:  rsc_time: %d should not equal 0, data[j].time: %4d"
					" comm: %16s pid: %5d task: %p threadinfo: %p\n",
					__func__, rsc_time, data[j].time, data[j].comm,
					data[j].pid, p, current_thread_info());
				continue;
			}
			break;
		}
	}

	/*not new task*/
	if (likely(i < len)) {
fasthit:
#ifdef RECORD_DEBUG_LOG
		if (data[j].time != rsc_time)
			rsc_err("%s: time not equal! data: %4u cur task: %4u comm: %16s pid: %5d\n",
				__func__, data[j].time, rsc_time, data[j].comm, data[j].pid);
#endif
		rsc_time += ticks;
		data[j].time = rsc_time;

		if (user_tick)
			data[j].utime = p->rsc_utime[cpu];
		else
			data[j].stime = p->rsc_stime[cpu];

		l = pos;
		while (l > 0) {
			preidx = tbl[l-1];
			if (rsc_time > data[preidx].time)
				l--;
			else
				break;
		}
		if (pos > l) {
			/*swap two pos*/
			tmp = tbl[l];
			tbl[l] = tbl[pos];
			tbl[pos] = tmp;

			/*update the fasthit idx*/
			map_idx[map_pid] = l;
			preidx = data[tmp].pid & RSC_MAX_HASH_TASK_MASK;
			map_idx[preidx] = pos;
		}
#ifdef RSC_USE_TASKLOCK_LIST
		if (data[j].ppid == RSC_END_PID)
			update_tgcomm(&data[j], p, cpu);
#endif
#ifdef RECORD_STAT_EXPTIME
		tidx = j;
#endif

/*debug*/
#ifdef RECORD_DEBUG_LOG
		{
			static int printonce;
			idx = 0;

			len = per_cpu(task_cpu_usage_cnt, cpu);
			for (i = 0; i < len; i++) {
				j = tbl[i];
				if (i < (len - 1)) {
					tmp = tbl[i+1];
					if (data[j].time < data[tmp].time) {
						idx = 1;
						rsc_err("i: %2d cpu: %d j: %2d len: %2d user_tick: %d comm: %16s "
						"time: %5u utime: %5u stime: %5u pid: %5d tgid: %5d "
	#ifdef RECORD_CPUBIT
						"cpubit: 0x%x "
	#endif
						"seq: %4u swap "
						"data[j].time: %4u < data[tmp].time: %4u idx: %d\n",
						i, cpu, j, len, user_tick, data[j].comm, data[j].time,
						 data[j].utime, data[j].stime, data[j].pid,
						 data[j].tgid,
	#ifdef RECORD_CPUBIT
						 data[j].cpubit,
	#endif
						 data[j].seq, data[j].time, data[tmp].time, l);
					}
				}
			}

			if (idx && (printonce < 2)) {
				if (printonce)
					printonce++;
				for (i = 0; i < len; i++) {
					j = tbl[i];
					rsc_info("i: %2d cpu: %d j: %2d user_tick: %d comm: %16s "
						"time: %5u utime: %5u stime: %5u pid: %5d tgid: %5d "
	#ifdef RECORD_CPUBIT
						"cpubit: 0x%x "
	#endif
						"seq: %4u swap\n",
						i, cpu, j, user_tick, data[j].comm, data[j].time,
						 data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
	#ifdef RECORD_CPUBIT
						 data[j].cpubit,
	#endif
						 data[j].seq);
				}
				rsc_err("user_tick: comm: pid: %5d %16s\n", p->pid, p->comm);
				rsc_info("user_tick\n");
			}
		}
#endif
		per_cpu(task_cpu_usage_idx, cpu) = pos;
	} else {
newtask:
		/* len begin from 0*/
		/*size = min(len - 1, max_stat_task - 1);*/
		size = len - 1;
		add = 0;
		rsc_time++;
		if (likely(size == max_stat_task - 1)) {
			l = size;
			 do {
				preidx = tbl[l];
				if (rsc_time > data[preidx].time)
					l--;
				else
					break;
			} while (l >= 0);

			if (size > l) {
				preidx = tbl[l+1];
				itm = &data[preidx];
				add = 1;
				per_cpu(task_cpu_usage_idx, cpu) = l + 1;
				map_cnt[map_pid]++;
				map_idx[map_pid] = l + 1;
				map_cnt[itm->pid & RSC_MAX_HASH_TASK_MASK]--;
				/*map_idx[itm->pid & RSC_MAX_HASH_TASK_MASK] = RSC_STAT_IVALID_IDX;*/
			}
#ifdef RECORD_STAT_EXPTIME
			tidx = preidx;
#endif
		} else {
			pos = size + 1;
			tbl[pos] = pos;
			itm = &data[pos];
			add = 1;
#ifdef RECORD_DEBUG_LOG
			if (1 != rsc_time) {
				rsc_err("%s: add new rsc_time: %5u "
					" pos: %2d size: %2d p:%p comm: %s\n",
					 __func__, rsc_time,  pos, size, p, p->comm);

				for (i = 0; i <= pos; i++)
					rsc_err("%s: start i: %2d cpu: %d smallsize pos: %2d  tbl[i]: %2d comm: %16s pid: %5d "
						"rsc_time: %5u data[i].time: %u task:%p\n",
						__func__, i, cpu, pos, tbl[i], (i == pos)?p->comm:data[i].comm,
						(i == pos)?p->pid:data[i].pid, rsc_time, (i == pos)?rsc_time:data[i].time, (i == pos)?p:data[i].task);

			}
#endif

#ifdef RECORD_STAT_EXPTIME
			tidx = pos;
#endif
			per_cpu(task_cpu_usage_idx, cpu) = pos;
			if (len < max_stat_task)
				per_cpu(task_cpu_usage_cnt, cpu)++;
			map_cnt[map_pid]++;
			map_idx[map_pid] = pos;
		}

		/*add new one or replay the old one*/
		if (add) {
			itm->time = rsc_time;
			itm->utime = p->rsc_utime[cpu];
			itm->stime = p->rsc_stime[cpu];
			itm->pid = p->pid;
			itm->tgid = p->tgid;
			itm->task = p;
			/*itm->tinf = current_thread_info();*/
			itm->seq = p->rsc_seq;
#ifdef RECORD_CPUBIT
			itm->cpubit = p->rsc_cpubit;
#endif
			itm->is_exit = 0;
#ifdef RECORD_STAT_EXPTIME
			itm->exptime = 0;
			itm->minexptime = 0;
#endif
			strlcpy(itm->comm, p->comm, TASK_COMM_LEN);
			update_tgcomm(itm, p, cpu);
		}
#ifdef RECORD_DEBUG_LOG
		/*debug*/
{
		static int printonce;

		l = 0;
		for (i = 0; i < len; i++) {
			j = tbl[i];
			if (i < (len - 1)) {
				tmp = tbl[i+1];
				if (data[j].time < data[tmp].time) {
					l = 1;
					rsc_err("i: %2d cpu: %d j: %2d len: %2d user_tick: %d comm: %16s "
					"time: %5u utime: %5u stime: %5u pid: %5d tgid: %5d "
	#ifdef RECORD_CPUBIT
					"cpubit: 0x%x "
	#endif
					"seq: %4u "
						"repaly data[j].time: %4u < data[tmp].time: %4u replayidx: %d\n",
					i, cpu, j, len, user_tick, data[j].comm, data[j].time,
					 data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
	#ifdef RECORD_CPUBIT
					 data[j].cpubit,
	#endif
					 data[j].seq, data[j].time, data[tmp].time, (int)(itm - data));
				}
			}
		}

		if (l && (printonce < 2)) {
			if (printonce)
				printonce++;
			for (i = 0; i < len; i++) {
				j = tbl[i];
				rsc_err("i: %2d cpu: %d j: %2d user_tick: %d comm: %16s "
					"time: %5u utime: %5u stime: %5u pid: %5d tgid: %5d "
	#ifdef RECORD_CPUBIT
					"cpubit: 0x%x "
	#endif
					"seq: %4u replay\n",
					i, cpu, j, user_tick, data[j].comm, data[j].time,
					 data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
	#ifdef RECORD_CPUBIT
					 data[j].cpubit,
	#endif
					 data[j].seq);
			}
			rsc_err("user_tick: comm: pid: %5d %16s\n", p->pid, p->comm);
			rsc_err("user_tick\n");
		}
}
#endif
	}
#ifdef RECORD_DEBUG_LOG
	per_cpu(task_call_cnt, cpu)++;
	if (0 == (per_cpu(task_call_cnt, cpu) % 2000)) {
		for (i = 0; i < len; i++) {
			j = tbl[i];
			rsc_info("i: %2d cpu: %d j: %2d user_tick: %d comm: %16s "
				"time: %5u utime: %5u stime: %5u pid: %5d tgid: %5d  "
				"tgcomm: %16s ppid: %5d "
	#ifdef RECORD_CPUBIT
				"cpubit: 0x%x "
	#endif
				"seq: %4u\n",
				i, cpu, j, user_tick, data[j].comm, data[j].time,
				 data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
				 data[j].tgcomm, data[j].ppid,
	#ifdef RECORD_CPUBIT
				 data[j].cpubit,
	#endif
				 data[j].seq);
		}
		rsc_info("user_tick\n");
		rsc_info("user_tick\n");
	}
#endif

#ifdef RECORD_STAT_EXPTIME
	end = local_clock();
	data[tidx].exptime = max((u32)(end - start), data[tidx].exptime);
	data[tidx].minexptime = min((u32)(end - start), data[tidx].exptime);
#endif
}

