/*
 * Copyright (c) 2014 Samsung Electronics Co, Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#define DEBUG
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sec-debug.h>
#include <linux/slab.h>
#include <linux/smp.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#endif

#define LOG_LINE_MAX	256
#define FTRACE_LOG_MAX	8192
#define MAX_TEXT_LEN	135

static int ftracelogging;

static u32 __initdata secftrace_base = 0x08100000;
static u32 __initdata secftrace_size = 0x00040000;
static u32 __initdata secftrace_reserved;

/*  size of 'struct event_log' :
 *  {8(time) + 1(cpu) + 135(text)} * 4(NR_CPUS) * 4096(FTRACE_LOG_MAX) = 2304KB = 0x240000 Bytes
 */
struct ftrace_log {
	struct event_log {
		unsigned long long time;
		char cpu;
		char text[MAX_TEXT_LEN];
	} event[NR_CPUS][FTRACE_LOG_MAX];
};

static struct ftrace_log *psec_ftrace_log;

#if NR_CPUS == 2
static atomic_t event_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1) };
#elif NR_CPUS == 4
static atomic_t event_log_idx[NR_CPUS] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1),
	ATOMIC_INIT(-1), ATOMIC_INIT(-1)
};
#endif

void print_ftrace(const char *fmt, ...)
{
	if (ftracelogging == 0)
		return;

	int cpu = raw_smp_processor_id();
	unsigned int i = atomic_inc_return(&event_log_idx[cpu]) & (FTRACE_LOG_MAX - 1);
	int r;
	va_list args;
	static char textbuf[NR_CPUS][LOG_LINE_MAX];

	psec_ftrace_log->event[cpu][i].cpu = cpu;
	psec_ftrace_log->event[cpu][i].time = cpu_clock(cpu);

	va_start(args, fmt);
	r = vscnprintf(textbuf[cpu], sizeof(textbuf[cpu]), fmt, args);

	if (r < MAX_TEXT_LEN)
		strcpy(psec_ftrace_log->event[cpu][i].text, textbuf[cpu]);
	else
		strncpy(psec_ftrace_log->event[cpu][i].text, textbuf[cpu], MAX_TEXT_LEN - 1);

	va_end(args);

	return;
}

#ifdef CONFIG_OF
static int __init mmp_sec_ftrace_fdt_find_info(unsigned long node, const char *uname,
			int depth, void *data)
{
	__be32 *prop;
	unsigned long len;

	if (!of_flat_dt_is_compatible(node, "marvell,secftrace-heap"))
		return 0;

	prop = of_get_flat_dt_prop(node, "secftrace-base", &len);
	if (!prop || (len != sizeof(unsigned int))) {
		pr_err("%s: Can't find secftrace-base property\n", __func__);
		return 0;
	}
	secftrace_base = be32_to_cpu(prop[0]);

	secftrace_size = sizeof(struct ftrace_log);

	return 1;
}

void __init pxa1908_reserve_sec_ftrace_mem(void)
{
	if (sec_debug_level.uint_val == 1)
		if (of_scan_flat_dt(mmp_sec_ftrace_fdt_find_info, NULL)) {
			BUG_ON(memblock_reserve(secftrace_base, secftrace_size) != 0);
			pr_info("Reserved sec ftrace memory : %dMB at %#.8x\n",
				(unsigned)secftrace_size/0x100000,
				(unsigned)secftrace_base);
			secftrace_reserved = 1;
		}
}
#endif

static int sec_ftrace_probe(struct platform_device *pdev)
{
	unsigned int i, base, size, page_count;
	phys_addr_t page_start;
	pgprot_t prot;
	void *vaddr;
	struct page **pages;

	base = secftrace_base;
	size = secftrace_size;

	if (!base) {
		dev_err(&pdev->dev, "base missing\n");
		return -EINVAL;
	}

	if (!size) {
		dev_err(&pdev->dev, "size missing\n");
		return -EINVAL;
	}

	if (sec_debug_level.uint_val == 1 && secftrace_reserved) {
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

		psec_ftrace_log = (struct ftrace_log *) vaddr;
		ftracelogging = 1;
	}

	return 0;
}

static struct of_device_id sec_ftrace_dt_ids[] = {
	{.compatible = "sec,sec_ftrace",},
	{}
};

MODULE_DEVICE_TABLE(of, sec_ftrace_dt_ids);

static struct platform_driver sec_ftrace_driver = {
	.probe = sec_ftrace_probe,
	.driver = {
		   .name = "sec_ftrace",
		   .owner = THIS_MODULE,
		   .of_match_table = sec_ftrace_dt_ids,
		   },
};

module_platform_driver(sec_ftrace_driver);
