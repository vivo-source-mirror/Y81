#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/atomic.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif
#include <asm/page.h>
#include <asm/pgtable.h>
#include "internal.h"

#ifdef CONFIG_VIVO_MP_KEEP_INFO_SECRET
#include "linux/vivo_mp_keep_info_secret.h"
#endif

void __attribute__((weak)) arch_report_meminfo(struct seq_file *m)
{
}

#define RSC_ZONE_INFO
#ifdef RSC_ZONE_INFO
struct rsc_zone_info_t {
	const char *name;
	int nr;
};

#define RSC_ZONE_INFO_D(rname)					\
/*	(											\
		do {	(*/ 									\
			[rname] = {							\
				.name = __stringify(rname),		\
				.nr = rname,					\
			}									\
/*		) } while (0)								\
	) */

static struct rsc_zone_info_t rsc_zone_info[NR_VM_ZONE_STAT_ITEMS] = {
	RSC_ZONE_INFO_D(NR_FREE_PAGES),
	RSC_ZONE_INFO_D(NR_INACTIVE_ANON),
	RSC_ZONE_INFO_D(NR_ACTIVE_ANON),
	RSC_ZONE_INFO_D(NR_INACTIVE_FILE),
	RSC_ZONE_INFO_D(NR_ACTIVE_FILE),
	RSC_ZONE_INFO_D(NR_UNEVICTABLE),
	RSC_ZONE_INFO_D(NR_ZONE_WRITE_PENDING),
	RSC_ZONE_INFO_D(NR_MLOCK),
	RSC_ZONE_INFO_D(NR_SLAB_RECLAIMABLE),
	RSC_ZONE_INFO_D(NR_SLAB_UNRECLAIMABLE),
	RSC_ZONE_INFO_D(NR_PAGETABLE),
	RSC_ZONE_INFO_D(NR_KERNEL_STACK_KB),
	RSC_ZONE_INFO_D(NR_KAISERTABLE),
	RSC_ZONE_INFO_D(NR_BOUNCE),
#if IS_ENABLED(CONFIG_ZSMALLOC)
	RSC_ZONE_INFO_D(NR_ZSPAGES),
#endif
#ifdef CONFIG_NUMA
	RSC_ZONE_INFO_D(NUMA_HIT),
	RSC_ZONE_INFO_D(NUMA_MISS),
	RSC_ZONE_INFO_D(NUMA_FOREIGN),
	RSC_ZONE_INFO_D(NUMA_INTERLEAVE_HIT),
	RSC_ZONE_INFO_D(NUMA_LOCAL),
	RSC_ZONE_INFO_D(NUMA_OTHER),
#endif
	RSC_ZONE_INFO_D(NR_FREE_CMA_PAGES)
};

static int zoneinfo_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	long cached;
	long available;
	unsigned long pagecache;
	unsigned long wmark_low = 0;
	unsigned long pages[NR_LRU_LISTS];
	struct zone *zone;
	int lru;
	int j, k;

/*
 * display in kilobytes.
 */
#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);

	cached = global_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_page_state(NR_LRU_BASE + lru);

	for_each_zone(zone)
		wmark_low += zone->watermark[WMARK_LOW];

	/*
	 * Estimate the amount of memory available for userspace allocations,
	 * without causing swapping.
	 *
	 * Free memory cannot be taken below the low watermark, before the
	 * system starts swapping.
	 */
	available = i.freeram - wmark_low;

	/*
	 * Not all the page cache can be freed, otherwise the system will
	 * start swapping. Assume at least half of the page cache, or the
	 * low watermark worth of cache, needs to stay.
	 */
	pagecache = pages[LRU_ACTIVE_FILE] + pages[LRU_INACTIVE_FILE];
	pagecache -= min(pagecache / 2, wmark_low);
	available += pagecache;

	/*
	 * Part of the reclaimable slab consists of items that are in use,
	 * and cannot be freed. Cap this estimate at the low watermark.
	 */
	available += global_page_state(NR_SLAB_RECLAIMABLE) -
		     min(global_page_state(NR_SLAB_RECLAIMABLE) / 2, wmark_low);

	if (available < 0)
		available = 0;

	seq_printf(m,
		"MemTotal\t%8lu\t"
		"MemFree\t%8lu\t"
		"Avail\t%8lu\t"
		"Buffers\t%8lu\t"
		"Cached\t%8lu\t"
		"SwapCached\t%8lu\t"
		"SwapTotal\t%8lu\t"
		"SwapFree\t%8lu\t"
		"Shmem\t%8lu\t",
		K(i.totalram),
		K(i.freeram),
		K(available),
		K(i.bufferram),
		K(cached),
		K(total_swapcache_pages()),
		K(i.totalswap),
		K(i.freeswap),
		K(i.sharedram)
	);

	j = 0;
	for_each_zone(zone) {

		seq_printf(m,
			"zone[%d]\t"
			"total\t%8lu\t"
			"waterlow\t%8lu\t",
			j,
			K(zone->managed_pages),
			zone->watermark[WMARK_LOW]
		);

		for (k = 0; k < NR_VM_ZONE_STAT_ITEMS; k++)
			if (k == NR_KERNEL_STACK_KB)
				seq_printf(m, "%s\t%8lu\t",
					rsc_zone_info[NR_KERNEL_STACK_KB].name,
					zone_page_state(zone,
					rsc_zone_info[NR_KERNEL_STACK_KB].nr));
			else
				seq_printf(m, "%s\t%8lu\t",
					rsc_zone_info[k].name,
					K(zone_page_state(zone,
					rsc_zone_info[k].nr)));
		j++;
		if (j >= 2)
			break;
	}
	seq_printf(m, "\n");

	return 0;
#undef K
}

static int zoneinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, zoneinfo_proc_show, NULL);
}

static const struct file_operations zoneinfo_proc_fops = {
	.open		= zoneinfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static void show_val_kb(struct seq_file *m, const char *s, unsigned long num)
{
	char v[32];
	static const char blanks[7] = {' ', ' ', ' ', ' ',' ', ' ', ' '};
	int len;

	len = num_to_str(v, sizeof(v), num << (PAGE_SHIFT - 10));

	seq_write(m, s, 16);

	if (len > 0) {
		if (len < 8)
			seq_write(m, blanks, 8 - len);

		seq_write(m, v, len);
	}
	seq_write(m, " kB\n", 4);
}

static int meminfo_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	unsigned long committed;
	long cached;
	long available;
	unsigned long pages[NR_LRU_LISTS];
	int lru;

#ifdef CONFIG_VIVO_MP_KEEP_INFO_SECRET
    unsigned long mem_total = 1048576;
    unsigned long mem_free = 262144;
    unsigned long mem_available = 524288;
    unsigned long mem_buffers = 131072;
    unsigned long mem_cached = 131072;
#endif

	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);

	cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_node_page_state(NR_LRU_BASE + lru);

	available = si_mem_available();

#ifdef CONFIG_VIVO_MP_KEEP_INFO_SECRET
    if (should_keep_confidentiality() == 0) {
        mem_total = i.totalram;
        mem_free = i.freeram;
        mem_available = available;
        mem_buffers = i.bufferram;
        mem_cached = cached;
    }
#endif

#ifdef CONFIG_VIVO_MP_KEEP_INFO_SECRET
    show_val_kb(m, "MemTotal:       ", mem_total);
    show_val_kb(m, "MemFree:        ", mem_free);
    show_val_kb(m, "MemAvailable:   ", mem_available);
    show_val_kb(m, "Buffers:        ", mem_buffers);
    show_val_kb(m, "Cached:         ", mem_cached);
#else
	show_val_kb(m, "MemTotal:       ", i.totalram);
	show_val_kb(m, "MemFree:        ", i.freeram);
	show_val_kb(m, "MemAvailable:   ", available);
	show_val_kb(m, "Buffers:        ", i.bufferram);
	show_val_kb(m, "Cached:         ", cached);
#endif
	show_val_kb(m, "SwapCached:     ", total_swapcache_pages());
	show_val_kb(m, "Active:         ", pages[LRU_ACTIVE_ANON] +
					   pages[LRU_ACTIVE_FILE]);
	show_val_kb(m, "Inactive:       ", pages[LRU_INACTIVE_ANON] +
					   pages[LRU_INACTIVE_FILE]);
	show_val_kb(m, "Active(anon):   ", pages[LRU_ACTIVE_ANON]);
	show_val_kb(m, "Inactive(anon): ", pages[LRU_INACTIVE_ANON]);
	show_val_kb(m, "Active(file):   ", pages[LRU_ACTIVE_FILE]);
	show_val_kb(m, "Inactive(file): ", pages[LRU_INACTIVE_FILE]);
	show_val_kb(m, "Unevictable:    ", pages[LRU_UNEVICTABLE]);
	show_val_kb(m, "Mlocked:        ", global_page_state(NR_MLOCK));

#ifdef CONFIG_HIGHMEM
	show_val_kb(m, "HighTotal:      ", i.totalhigh);
	show_val_kb(m, "HighFree:       ", i.freehigh);
	show_val_kb(m, "LowTotal:       ", i.totalram - i.totalhigh);
	show_val_kb(m, "LowFree:        ", i.freeram - i.freehigh);
#endif

#ifndef CONFIG_MMU
	show_val_kb(m, "MmapCopy:       ",
		    (unsigned long)atomic_long_read(&mmap_pages_allocated));
#endif

	show_val_kb(m, "SwapTotal:      ", i.totalswap);
	show_val_kb(m, "SwapFree:       ", i.freeswap);
	show_val_kb(m, "Dirty:          ",
		    global_node_page_state(NR_FILE_DIRTY));
	show_val_kb(m, "Writeback:      ",
		    global_node_page_state(NR_WRITEBACK));
	show_val_kb(m, "AnonPages:      ",
		    global_node_page_state(NR_ANON_MAPPED));
	show_val_kb(m, "Mapped:         ",
		    global_node_page_state(NR_FILE_MAPPED));
	show_val_kb(m, "Shmem:          ", i.sharedram);
	show_val_kb(m, "Slab:           ",
		    global_page_state(NR_SLAB_RECLAIMABLE) +
		    global_page_state(NR_SLAB_UNRECLAIMABLE));

	show_val_kb(m, "SReclaimable:   ",
		    global_page_state(NR_SLAB_RECLAIMABLE));
	show_val_kb(m, "SUnreclaim:     ",
		    global_page_state(NR_SLAB_UNRECLAIMABLE));
	seq_printf(m, "KernelStack:    %8lu kB\n",
		   global_page_state(NR_KERNEL_STACK_KB));
	show_val_kb(m, "PageTables:     ",
		    global_page_state(NR_PAGETABLE));
#ifdef CONFIG_QUICKLIST
	show_val_kb(m, "Quicklists:     ", quicklist_total_size());
#endif

	show_val_kb(m, "NFS_Unstable:   ",
		    global_node_page_state(NR_UNSTABLE_NFS));
	show_val_kb(m, "Bounce:         ",
		    global_page_state(NR_BOUNCE));
	show_val_kb(m, "WritebackTmp:   ",
		    global_node_page_state(NR_WRITEBACK_TEMP));
	show_val_kb(m, "CommitLimit:    ", vm_commit_limit());
	show_val_kb(m, "Committed_AS:   ", committed);
	seq_printf(m, "VmallocTotal:   %8lu kB\n",
		   (unsigned long)VMALLOC_TOTAL >> 10);
	show_val_kb(m, "VmallocUsed:    ", 0ul);
	show_val_kb(m, "VmallocChunk:   ", 0ul);

#ifdef CONFIG_MEMORY_FAILURE
	seq_printf(m, "HardwareCorrupted: %5lu kB\n",
		   atomic_long_read(&num_poisoned_pages) << (PAGE_SHIFT - 10));
#endif

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	show_val_kb(m, "AnonHugePages:  ",
		    global_node_page_state(NR_ANON_THPS) * HPAGE_PMD_NR);
	show_val_kb(m, "ShmemHugePages: ",
		    global_node_page_state(NR_SHMEM_THPS) * HPAGE_PMD_NR);
	show_val_kb(m, "ShmemPmdMapped: ",
		    global_node_page_state(NR_SHMEM_PMDMAPPED) * HPAGE_PMD_NR);
#endif

#ifdef CONFIG_CMA
	show_val_kb(m, "CmaTotal:       ", totalcma_pages);
	show_val_kb(m, "CmaFree:        ",
		    global_page_state(NR_FREE_CMA_PAGES));
#endif

	hugetlb_report_meminfo(m);

	arch_report_meminfo(m);

	return 0;
}

static int meminfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_proc_show, NULL);
}

static const struct file_operations meminfo_proc_fops = {
	.open		= meminfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int meminfo_quick_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	long cached;
#define K(x) ((x) << (PAGE_SHIFT - 10))

	si_meminfo(&i);
	si_swapinfo(&i);
	cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;
	seq_printf(m,
		"%lu %lu %lu %lu %lu\n",
		K(i.totalram),
		K(i.freeram),
		K(i.bufferram),
		K(cached),
		K(i.sharedram)
		);
	return 0;

#undef K
}

static int meminfo_quick_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_quick_proc_show, NULL);
}

static const struct file_operations meminfo_quick_proc_fops = {
	.open		= meminfo_quick_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_meminfo_init(void)
{
	proc_create("meminfo", 0, NULL, &meminfo_proc_fops);
	proc_create("meminfo_quick", S_IRUGO, NULL, &meminfo_quick_proc_fops);
#ifdef RSC_ZONE_INFO
	proc_create("rsc_zoneinfo", S_IRUSR|S_IRGRP,
		NULL, &zoneinfo_proc_fops);
#endif

	return 0;
}
fs_initcall(proc_meminfo_init);
