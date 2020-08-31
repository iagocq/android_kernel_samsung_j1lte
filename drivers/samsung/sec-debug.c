/*
 * Copyright (C) 2014 Samsung Electronics Co, Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kallsyms.h>
#include <linux/kernel_stat.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#endif
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/ptrace.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/sec-debug.h>
#include <linux/slab.h>
#include <linux/smp.h>

#ifdef CONFIG_USER_RESET_DEBUG
#include <linux/seq_file.h>
enum sec_debug_reset_reason_t {
	RR_S = 1,
	RR_W,
	RR_D,
	RR_K,
	RR_M,
	RR_P,
	RR_R,
	RR_B,
	RR_N,
};

static int reset_reason = RR_N;
module_param_named(reset_reason, reset_reason, uint, 0644);
#endif

extern void __iomem *regs_addr_get_va(unsigned int id);
/* For reset with command */
#define MPMU_RSR_PJ_OFFSET	(0x1028)
#define MPMU_ARSR_SWR_SHIFT	(8)
#define RESET_PANIC		(1 << MPMU_ARSR_SWR_SHIFT)
#define RESET_FORCE_UPLOAD	(2 << MPMU_ARSR_SWR_SHIFT)
#define RESET_CPASSERT		(4 << MPMU_ARSR_SWR_SHIFT)
#define MPMU_ARSR_SWR_MASK	(0xfffff0ff)

static int schedlogging;
int sec_crash_key_panic;
int sec_debug_panic = 0;

/* enable/disable sec_debug feature
 * level = 0 when enable = 0 && enable_user = 0
 * level = 1 when enable = 1 && enable_user = 0
 * level = 0x10001 when enable = 1 && enable_user = 1
 * The other cases are not considered
 */
union sec_debug_level_t sec_debug_level = {.en.kernel_fault = 1, };

module_param_named(enable, sec_debug_level.en.kernel_fault, ushort, 0644);
module_param_named(enable_user, sec_debug_level.en.user_fault, ushort, 0644);
module_param_named(level, sec_debug_level.uint_val, uint, 0644);
module_param_named(enable_schedlog, schedlogging, int, 0664);

static int __init early_debug_level(char *str)
{
	int level;

	if (get_option(&str, &level)) {
		sec_debug_level.uint_val = level;
		return 0;
	}

	return -EINVAL;
}
early_param("sec_debug_level", early_debug_level);

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
#define SCHED_LOG_MAX 2048

/*  size of 'struct task_log' :
 *  {8(time) + 16(comm) + 8(pid)} * 4(NR_CPUS) * 2048(FTRACE_LOG_MAX) = 256KB = 0x40000Bytes
 */
struct sched_log {
#ifdef CONFIG_SEC_DEBUG_TASK_LOG
	struct task_log {
		unsigned long long time;
		char comm[TASK_COMM_LEN];
		pid_t pid;
	} task[NR_CPUS][SCHED_LOG_MAX];
#endif
#ifdef CONFIG_SEC_DEBUG_IRQ_LOG
	struct irq_log {
		unsigned long long time;
		int irq;
		void *fn;
		int en;
	} irq[NR_CPUS][SCHED_LOG_MAX];
#endif
#ifdef CONFIG_SEC_DEBUG_WORK_LOG
	struct work_log {
		unsigned long long time;
		struct worker *worker;
		struct work_struct *work;
		work_func_t f;
		int en;
	} work[NR_CPUS][SCHED_LOG_MAX];
#endif
#ifdef CONFIG_SEC_DEBUG_TIMER_LOG
	struct timer_log {
		unsigned long long time;
		unsigned int type;
		void *fn;
	} timer[NR_CPUS][SCHED_LOG_MAX];
#endif
};

static struct sched_log *psec_debug_log;


#ifdef CONFIG_SEC_DEBUG_TASK_LOG
#if NR_CPUS == 2
static atomic_t task_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1) };
#elif NR_CPUS == 4
static atomic_t task_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1),
	ATOMIC_INIT(-1), ATOMIC_INIT(-1)
};
#endif
void __sec_debug_task_log(int cpu, struct task_struct *task)
{
	if (schedlogging == 0)
		return;

	unsigned i;

	i = atomic_inc_return(&task_log_idx[cpu]) & (SCHED_LOG_MAX - 1);
	psec_debug_log->task[cpu][i].time = cpu_clock(cpu);
	strcpy(psec_debug_log->task[cpu][i].comm, task->comm);
	psec_debug_log->task[cpu][i].pid = task->pid;
}
#endif
#ifdef CONFIG_SEC_DEBUG_IRQ_LOG
#if NR_CPUS == 2
static atomic_t irq_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1) };
#elif NR_CPUS == 4
static atomic_t irq_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1),
	ATOMIC_INIT(-1), ATOMIC_INIT(-1)
};
#endif
void __sec_debug_irq_log(unsigned int irq, void *fn, int en)
{
	if (schedlogging == 0)
		return;

	int cpu = raw_smp_processor_id();
	unsigned i;

	i = atomic_inc_return(&irq_log_idx[cpu]) & (SCHED_LOG_MAX - 1);
	psec_debug_log->irq[cpu][i].time = cpu_clock(cpu);
	psec_debug_log->irq[cpu][i].irq = irq;
	psec_debug_log->irq[cpu][i].fn = (void *)fn;
	psec_debug_log->irq[cpu][i].en = en;
}
#endif
#ifdef CONFIG_SEC_DEBUG_WORK_LOG
#if NR_CPUS == 2
static atomic_t work_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1) };
#elif NR_CPUS == 4
static atomic_t work_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1),
	ATOMIC_INIT(-1), ATOMIC_INIT(-1)
};
#endif
void __sec_debug_work_log(struct worker *worker,
			  struct work_struct *work, work_func_t f, int en)
{
	if (schedlogging == 0)
		return;

	int cpu = raw_smp_processor_id();
	unsigned i;

	i = atomic_inc_return(&work_log_idx[cpu]) & (SCHED_LOG_MAX - 1);
	psec_debug_log->work[cpu][i].time = cpu_clock(cpu);
	psec_debug_log->work[cpu][i].worker = worker;
	psec_debug_log->work[cpu][i].work = work;
	psec_debug_log->work[cpu][i].f = f;
	psec_debug_log->work[cpu][i].en = en;
}
#endif
#ifdef CONFIG_SEC_DEBUG_TIMER_LOG
#if NR_CPUS == 2
static atomic_t timer_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1) };
#elif NR_CPUS == 4
static atomic_t timer_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1),
	ATOMIC_INIT(-1), ATOMIC_INIT(-1)
};
#endif
void __sec_debug_timer_log(unsigned int type, void *fn)
{
	if (schedlogging == 0)
		return;

	int cpu = raw_smp_processor_id();
	unsigned i;

	i = atomic_inc_return(&timer_log_idx[cpu]) & (SCHED_LOG_MAX - 1);
	psec_debug_log->timer[cpu][i].time = cpu_clock(cpu);
	psec_debug_log->timer[cpu][i].type = type;
	psec_debug_log->timer[cpu][i].fn = fn;
}
#endif

static int checksum_sched_log(void)
{
	int sum = 0, i;
	for (i = 0; i < sizeof(*psec_debug_log); i++)
		sum += *((char *)psec_debug_log + i);

	return sum;
}

#else /* !CONFIG_SEC_DEBUG_SCHED_LOG */
static int checksum_sched_log(void)
{
	return 0;
}
#endif /* End of CONFIG_SEC_DEBUG_SCHED_LOG */

int get_sec_debug_level(void)
{
	return sec_debug_level.uint_val;
}

void sec_debug_check_crash_key(unsigned int code, int value)
{
	static bool volup_p;
	static bool voldown_p;
	static int loopcount;

	static const unsigned int VOLUME_UP = KEY_VOLUMEUP;
	static const unsigned int VOLUME_DOWN = KEY_VOLUMEDOWN;

	if (!sec_debug_level.en.kernel_fault)
		return;

	pr_debug("%s: code is %d, value is %d\n", __func__, code, value);

	/* Enter Force Upload
	 *  Hold volume down key first
	 *  and then press power key twice
	 *  and volume up key should not be pressed
	 */
	if (value) {
		if (code == VOLUME_UP)
			volup_p = true;
		if (code == VOLUME_DOWN)
			voldown_p = true;
		if (!volup_p && voldown_p) {
			if (code == KEY_POWER) {
				pr_info
				    ("%s: count for enter forced upload : %d\n",
				     __func__, ++loopcount);
				if (loopcount == 2) {
					schedlogging = 0;
					sec_crash_key_panic = 1;
					panic("Crash Key");
				}
			}
		}
	} else {
		if (code == VOLUME_UP)
			volup_p = false;
		if (code == VOLUME_DOWN) {
			loopcount = 0;
			voldown_p = false;
		}
	}
}

DEFINE_PER_CPU(struct sec_debug_core_t, sec_debug_aarch64_core_reg);
DEFINE_PER_CPU(struct sec_debug_mmu_reg_t, sec_debug_aarch64_mmu_reg);

inline void sec_debug_save_context(void)
{
	unsigned long flags;
	local_irq_save(flags);
	sec_debug_save_mmu_reg(&per_cpu(sec_debug_aarch64_mmu_reg,
					smp_processor_id()));
	sec_debug_save_core_reg(&per_cpu(sec_debug_aarch64_core_reg,
					 smp_processor_id()));

	pr_emerg("(%s) context saved(CPU:%d)\n", __func__, smp_processor_id());
	local_irq_restore(flags);
}

static inline struct task_struct *get_next_thread(struct task_struct *tsk)
{
	return container_of(tsk->thread_group.next,
			    struct task_struct, thread_group);
}

static void dump_one_task_info(struct task_struct *tsk, bool is_main)
{
	char state_array[] =
	    { 'R', 'S', 'D', 'T', 't', 'Z', 'X', 'x', 'K', 'W' };
	unsigned char idx = 0;
	unsigned int state = (tsk->state & TASK_REPORT) | tsk->exit_state;
	unsigned long wchan;
	unsigned long pc = 0;
	char symname[KSYM_NAME_LEN];
	int permitted;
	struct mm_struct *mm;

	permitted = ptrace_may_access(tsk, PTRACE_MODE_READ);
	mm = get_task_mm(tsk);
	if (mm) {
		if (permitted)
			pc = KSTK_EIP(tsk);
	}

	wchan = get_wchan(tsk);
	if (lookup_symbol_name(wchan, symname) < 0) {
		if (!ptrace_may_access(tsk, PTRACE_MODE_READ))
			sprintf(symname, "_____");
		else
			sprintf(symname, "%lu", wchan);
	}

	while (state) {
		idx++;
		state >>= 1;
	}

	pr_info
	    ("%8d %8d %8d %16lld %c(%d) %3d  %08x %08x  %16lx %c %16s [%s]\n",
	     tsk->pid, (int)(tsk->utime), (int)(tsk->stime), tsk->se.exec_start,
	     state_array[idx], (int)(tsk->state), task_cpu(tsk), (int)wchan,
	     (int)pc, (unsigned long)tsk, is_main ? '*' : ' ', tsk->comm,
	     symname);

	if (tsk->state == TASK_RUNNING
	    || tsk->state == TASK_UNINTERRUPTIBLE || tsk->mm == NULL) {
		show_stack(tsk, NULL);
		pr_info("\n");
	}
}

static void dump_all_task_info(void)
{
	struct task_struct *frst_tsk;
	struct task_struct *curr_tsk;
	struct task_struct *frst_thr;
	struct task_struct *curr_thr;

	pr_info("\n");
	pr_info(" current proc : %d %s\n", current->pid, current->comm);
	pr_info
	    (" -------------------------------------------------------------------------------------------------------------\n");
	pr_info
	    ("     pid      uTime    sTime      exec(ns)  stat  cpu   wchan   user_pc  task_struct          comm   sym_wchan\n");
	pr_info
	    (" -------------------------------------------------------------------------------------------------------------\n");

	/* processes */
	frst_tsk = &init_task;
	curr_tsk = frst_tsk;
	while (curr_tsk != NULL) {
		dump_one_task_info(curr_tsk, true);
		/* threads */
		if (curr_tsk->thread_group.next != NULL) {
			frst_thr = get_next_thread(curr_tsk);
			curr_thr = frst_thr;
			if (frst_thr != curr_tsk) {
				while (curr_thr != NULL) {
					dump_one_task_info(curr_thr, false);
					curr_thr = get_next_thread(curr_thr);
					if (curr_thr == curr_tsk)
						break;
				}
			}
		}
		curr_tsk = container_of(curr_tsk->tasks.next,
					struct task_struct, tasks);
		if (curr_tsk == frst_tsk)
			break;
	}
	pr_info
	    (" -----------------------------------------------------------------------------------\n");
}

#ifndef arch_irq_stat_cpu
#define arch_irq_stat_cpu(cpu) 0
#endif
#ifndef arch_irq_stat
#define arch_irq_stat() 0
#endif
#ifndef arch_idle_time
#define arch_idle_time(cpu) 0
#endif

static void dump_cpu_stat(void)
{
	int cpu_i, irq_j;
	unsigned long jif;
	cputime64_t user, nice, system, idle, iowait, irq, softirq, steal;
	cputime64_t guest, guest_nice;
	u64 sum = 0;
	u64 sum_softirq = 0;
	unsigned int per_softirq_sums[NR_SOFTIRQS] = { 0 };
	struct timespec boottime;
	unsigned int per_irq_sum;

	char *softirq_to_name[NR_SOFTIRQS] = {
		"HI", "TIMER", "NET_TX", "NET_RX", "BLOCK", "BLOCK_IOPOLL",
		"TASKLET", "SCHED", "HRTIMER", "RCU"
	};

	user = nice = system = idle = iowait = 0UL;
	irq = softirq = steal = 0UL;
	guest = guest_nice = 0UL;

	getboottime(&boottime);
	jif = boottime.tv_sec;
	for_each_possible_cpu(cpu_i) {
		user = user + kcpustat_cpu(cpu_i).cpustat[CPUTIME_USER];
		nice = nice + kcpustat_cpu(cpu_i).cpustat[CPUTIME_NICE];
		system = system + kcpustat_cpu(cpu_i).cpustat[CPUTIME_SYSTEM];
		idle = idle + kcpustat_cpu(cpu_i).cpustat[CPUTIME_IDLE];
		idle = idle + arch_idle_time(cpu_i);
		iowait = iowait + kcpustat_cpu(cpu_i).cpustat[CPUTIME_IOWAIT];
		irq = irq + kcpustat_cpu(cpu_i).cpustat[CPUTIME_IRQ];
		softirq =
		    softirq + kcpustat_cpu(cpu_i).cpustat[CPUTIME_SOFTIRQ];

		for_each_irq_nr(irq_j) {
			sum += kstat_irqs_cpu(irq_j, cpu_i);
		}
		sum += arch_irq_stat_cpu(cpu_i);
		for (irq_j = 0; irq_j < NR_SOFTIRQS; irq_j++) {
			unsigned int softirq_stat =
			    kstat_softirqs_cpu(irq_j, cpu_i);
			per_softirq_sums[irq_j] += softirq_stat;
			sum_softirq += softirq_stat;
		}
	}
	sum += arch_irq_stat();
	pr_info("\n");
	pr_info(" cpu     user:%llu  nice:%llu  system:%llu  idle:%llu  " "iowait:%llu  irq:%llu  softirq:%llu %llu %llu " "%llu\n", (unsigned long long)cputime64_to_clock_t(user), (unsigned long long)cputime64_to_clock_t(nice), (unsigned long long)cputime64_to_clock_t(system), (unsigned long long)cputime64_to_clock_t(idle), (unsigned long long)cputime64_to_clock_t(iowait), (unsigned long long)cputime64_to_clock_t(irq), (unsigned long long)cputime64_to_clock_t(softirq), (unsigned long long)0,	/* steal */
		(unsigned long long)0,	/* guest */
		(unsigned long long)0);	/* guest_nice */
	pr_info
	    (" -----------------------------------------------------------------------------------\n");
	for_each_online_cpu(cpu_i) {
		/* Copy values here to work around gcc-2.95.3, gcc-2.96 */
		user = kcpustat_cpu(cpu_i).cpustat[CPUTIME_USER];
		nice = kcpustat_cpu(cpu_i).cpustat[CPUTIME_NICE];
		system = kcpustat_cpu(cpu_i).cpustat[CPUTIME_SYSTEM];
		idle = kcpustat_cpu(cpu_i).cpustat[CPUTIME_IDLE];
		idle = idle + arch_idle_time(cpu_i);
		iowait = kcpustat_cpu(cpu_i).cpustat[CPUTIME_IOWAIT];
		irq = kcpustat_cpu(cpu_i).cpustat[CPUTIME_IRQ];
		softirq = kcpustat_cpu(cpu_i).cpustat[CPUTIME_SOFTIRQ];
		/* steal = kstat_cpu(cpu_i).cpustat.steal; */
		/* guest = kstat_cpu(cpu_i).cpustat.guest; */
		/* guest_nice = kstat_cpu(cpu_i).cpustat.guest_nice; */
		pr_info(" cpu %d   user:%llu  nice:%llu  system:%llu  " "idle:%llu  iowait:%llu  irq:%llu  softirq:%llu " "%llu %llu " "%llu\n", cpu_i, (unsigned long long)cputime64_to_clock_t(user), (unsigned long long)cputime64_to_clock_t(nice), (unsigned long long)cputime64_to_clock_t(system), (unsigned long long)cputime64_to_clock_t(idle), (unsigned long long)cputime64_to_clock_t(iowait), (unsigned long long)cputime64_to_clock_t(irq), (unsigned long long)cputime64_to_clock_t(softirq), (unsigned long long)0,	/* steal */
			(unsigned long long)0,	/* guest */
			(unsigned long long)0);	/* guest_nice */
	}
	pr_info
	    (" -----------------------------------------------------------------------------------\n");
	pr_info("\n");
	pr_info(" irq : %llu", (unsigned long long)sum);
	pr_info
	    (" -----------------------------------------------------------------------------------\n");
	/* sum again ? it could be updated? */
	for_each_irq_nr(irq_j) {
		per_irq_sum = 0;
		for_each_possible_cpu(cpu_i)
		    per_irq_sum += kstat_irqs_cpu(irq_j, cpu_i);
		if (per_irq_sum) {
			pr_info(" irq-%4d : %8u %s\n",
				irq_j, per_irq_sum, irq_to_desc(irq_j)->action ?
				irq_to_desc(irq_j)->action->
				name ? : "???" : "???");
		}
	}
	pr_info
	    (" -----------------------------------------------------------------------------------\n");
	pr_info("\n");
	pr_info(" softirq : %llu", (unsigned long long)sum_softirq);
	pr_info
	    (" -----------------------------------------------------------------------------------\n");
	for (cpu_i = 0; cpu_i < NR_SOFTIRQS; cpu_i++)
		if (per_softirq_sums[cpu_i])
			pr_info(" softirq-%d : %8u %s\n",
				cpu_i, per_softirq_sums[cpu_i],
				softirq_to_name[cpu_i]);
	pr_info
	    (" -----------------------------------------------------------------------------------\n");
}

/*
 * Called from dump_stack()
 * This function call does not necessarily mean that a fatal error
 * had occurred. It may be just a warning.
 */
static inline int sec_debug_dump_stack(void)
{

	sec_debug_save_context();

	/* flush L1 from each core.
	   L2 will be flushed later before reset. */
	flush_cache_all();

	return 0;
}

static inline void sec_debug_hw_reset(void *buf)
{
	pr_emerg("(%s) %s\n", __func__, linux_banner);
	pr_emerg("(%s) rebooting...\n", __func__);

	flush_cache_all();
	drain_mc_buffer();

	machine_restart(buf);

	while (1)
		pr_err("should not be happend\n");
}

static int sec_debug_panic_handler(struct notifier_block *nb,
				   unsigned long l, void *buf)
{
	unsigned int reg_val;
	void *va_pmum_rsr_pj;
	struct timespec ts;
	struct rtc_time tm;
	extern unsigned int system_rev;

	/* PXA1926 CPU Reset Status Register */
	va_pmum_rsr_pj = regs_addr_get_va(0) + MPMU_RSR_PJ_OFFSET;
	reg_val = readl(va_pmum_rsr_pj);
	reg_val &= MPMU_ARSR_SWR_MASK;

	if (!strcmp(buf, "Crash Key"))
		/* Intended panic for force upload. */
		writel(reg_val | RESET_FORCE_UPLOAD, va_pmum_rsr_pj);
	else if (!strcmp(buf, "CP Crash"))
		writel(reg_val | RESET_CPASSERT, va_pmum_rsr_pj);
	else
		/* Real Panic */
		writel(reg_val | RESET_PANIC, va_pmum_rsr_pj);

//	pr_err("(%s) checksum_sched_log: %x\n", __func__, checksum_sched_log());
	dump_all_task_info();
	dump_cpu_stat();
	/* No backtrace */
	show_state_filter(TASK_STATE_MAX);

	sec_debug_dump_stack();

	pr_emerg("Board_id %d\n", system_rev);
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("UTC :	%d-%02d-%02d %02d:%02d:%02d.%09lu \n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

	sec_debug_panic = 1;
	/* Device will be reset */
	sec_debug_hw_reset(buf);

	return NOTIFY_DONE;
}

int sec_debug_panic_dump(char *buf)
{
	return sec_debug_panic_handler(NULL, 0, buf);
}

static struct notifier_block nb_panic_block = {
	.notifier_call = sec_debug_panic_handler,
};

static u32 __initdata secdebug_base = 0x08300000;
static u32 __initdata secdebug_size = 0x00100000;
static u32 __initdata secdebug_reserved;
#ifdef CONFIG_OF
static int __init mmp_sec_debug_fdt_find_info(unsigned long node, const char *uname,
			int depth, void *data)
{
	__be32 *prop;
	unsigned long len;

	if (!of_flat_dt_is_compatible(node, "marvell,secdebug-heap"))
		return 0;

	prop = of_get_flat_dt_prop(node, "secdebug-base", &len);
	if (!prop || (len != sizeof(unsigned int))) {
		pr_err("%s: Can't find secdebug-base property\n", __func__);
		return 0;
	}
	secdebug_base = be32_to_cpu(prop[0]);

	secdebug_size = sizeof(struct sched_log);

	return 1;
}

void __init pxa1908_reserve_sec_debug_mem(void)
{
	if (sec_debug_level.uint_val == 1)
		if (of_scan_flat_dt(mmp_sec_debug_fdt_find_info, NULL)) {
			BUG_ON(memblock_reserve(secdebug_base, secdebug_size) != 0);
			pr_info("Reserved sec debug memory : %dMB at %#.8x\n",
				(unsigned)secdebug_size/0x100000,
				(unsigned)secdebug_base);
			secdebug_reserved = 1;
		}
}
#endif

static int sec_debug_probe(struct platform_device *pdev)
{
	unsigned int i, base, size, page_count;
	phys_addr_t page_start;
	pgprot_t prot;
	void *vaddr;
	struct page **pages;

	base = secdebug_base;
	size = secdebug_size;

	if (!base) {
		dev_err(&pdev->dev, "base missing\n");
		return -EINVAL;
	}

	if (!size) {
		dev_err(&pdev->dev, "size missing\n");
		return -EINVAL;
	}

	if (sec_debug_level.uint_val == 1 && secdebug_reserved) {
		page_start = base - offset_in_page(base);
		page_count = DIV_ROUND_UP(size + offset_in_page(base), PAGE_SIZE);

#ifdef CONFIG_ARM64
		prot = pgprot_writecombine(PAGE_KERNEL);
#else
		prot = pgprot_noncached(PAGE_KERNEL);
#endif
		pages = kmalloc(sizeof(struct page *) * page_count, GFP_KERNEL);
		if (!pages) {
			pr_err("%s: Failed to allocate array for %u pages\n", __func__,
			       page_count);
			return 0;
		}

		for (i = 0; i < page_count; i++) {
			phys_addr_t addr = page_start + i * PAGE_SIZE;
			pages[i] = pfn_to_page(addr >> PAGE_SHIFT);

			dma_sync_single_for_device(NULL, addr, size, DMA_TO_DEVICE);
		}
		vaddr = vmap(pages, page_count, VM_MAP, prot);
		kfree(pages);

		psec_debug_log = (struct sched_log *) vaddr;
		schedlogging = 1;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &nb_panic_block);

	return 0;
}

static struct of_device_id sec_debug_dt_ids[] = {
	{.compatible = "sec,sec_debug",},
	{}
};

MODULE_DEVICE_TABLE(of, sec_log_dt_ids);

static struct platform_driver sec_debug_driver = {
	.probe = sec_debug_probe,
	.driver = {
		   .name = "sec_debug",
		   .owner = THIS_MODULE,
		   .of_match_table = sec_debug_dt_ids,
		   },
};

module_platform_driver(sec_debug_driver);

#ifdef CONFIG_USER_RESET_DEBUG
static int set_reset_reason_proc_show(struct seq_file *m, void *v)
{
	if (reset_reason == RR_S)
		seq_printf(m, "SPON\n");
	else if (reset_reason == RR_W)
		seq_printf(m, "WPON\n");
	else if (reset_reason == RR_D)
		seq_printf(m, "DPON\n");
	else if (reset_reason == RR_K)
		seq_printf(m, "KPON\n");
	else if (reset_reason == RR_M)
		seq_printf(m, "MPON\n");
	else if (reset_reason == RR_P)
		seq_printf(m, "PPON\n");
	else if (reset_reason == RR_R)
		seq_printf(m, "RPON\n");
	else if (reset_reason == RR_B)
		seq_printf(m, "BPON\n");
	else
		seq_printf(m, "NPON\n");

	return 0;
}

static int sec_reset_reason_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, set_reset_reason_proc_show, NULL);
}

static const struct file_operations sec_reset_reason_proc_fops = {
	.open = sec_reset_reason_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init sec_debug_reset_reason_init(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create("reset_reason", S_IWUGO, NULL,
			    &sec_reset_reason_proc_fops);

	if (!entry)
		return -ENOMEM;

	return 0;
}

device_initcall(sec_debug_reset_reason_init);
#endif
