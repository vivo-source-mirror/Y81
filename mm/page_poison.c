#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/page_ext.h>
#include <linux/poison.h>
#include <linux/ratelimit.h>

static bool __page_poisoning_enabled __read_mostly;
static bool want_page_poisoning __read_mostly;

static int early_page_poison_param(char *buf)
{
	if (!buf)
		return -EINVAL;
	return strtobool(buf, &want_page_poisoning);
}
early_param("page_poison", early_page_poison_param);

bool page_poisoning_enabled(void)
{
	return __page_poisoning_enabled;
}

static bool need_page_poisoning(void)
{
	return want_page_poisoning;
}

static void init_page_poisoning(void)
{
	/*
	 * page poisoning is debug page alloc for some arches. If either
	 * of those options are enabled, enable poisoning
	 */
	if (!IS_ENABLED(CONFIG_ARCH_SUPPORTS_DEBUG_PAGEALLOC)) {
		if (!want_page_poisoning && !debug_pagealloc_enabled())
			return;
	} else {
		if (!want_page_poisoning)
			return;
	}

	__page_poisoning_enabled = true;
}

struct page_ext_operations page_poisoning_ops = {
	.need = need_page_poisoning,
	.init = init_page_poisoning,
};

bool page_is_poisoned(struct page *page)
{
	return false;
}

static void poison_page(struct page *page)
{
	void *addr = kmap_atomic(page);

	memset(addr, PAGE_POISON, PAGE_SIZE);
	kunmap_atomic(addr);
}

static void poison_pages(struct page *page, int n)
{
	int i;

	for (i = 0; i < n; i++)
		poison_page(page + i);
}

static bool single_bit_flip(unsigned char a, unsigned char b)
{
	unsigned char error = a ^ b;

	return error && !(error & (error - 1));
}

static void check_poison_mem(struct page *page,
			     unsigned char *mem, size_t bytes)
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 5 * HZ, 10);
	unsigned char *start;
	unsigned char *end;

	start = memchr_inv(mem, PAGE_POISON, bytes);
	if (!start)
		return;

	for (end = mem + bytes - 1; end > start; end--) {
		if (*end != PAGE_POISON)
			break;
	}

	if (!__ratelimit(&ratelimit))
		return;
	else if (start == end && single_bit_flip(*start, PAGE_POISON))
		pr_err("pagealloc: single bit error on page with phys start 0x%lx\n",
			(unsigned long)page_to_phys(page));
	else
		pr_err("pagealloc: memory corruption on page with phys start 0x%lx\n",
			(unsigned long)page_to_phys(page));

	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 1, start,
			end - start + 1, 1);
	BUG_ON(1);
	dump_stack();
}

static void unpoison_page(struct page *page)
{
	void *addr;

	addr = kmap_atomic(page);
	/*
	 * Page poisoning when enabled poisons each and every page
	 * that is freed to buddy. Thus no extra check is done to
	 * see if a page was posioned.
	 */
	check_poison_mem(page, addr, PAGE_SIZE);
	kunmap_atomic(addr);
}

static void unpoison_pages(struct page *page, int n)
{
	int i;

	for (i = 0; i < n; i++)
		unpoison_page(page + i);
}

void __kernel_poison_pages(struct page *page, int numpages, int enable)
{
	if (enable)
		unpoison_pages(page, numpages);
	else
		poison_pages(page, numpages);
}

#ifndef CONFIG_ARCH_SUPPORTS_DEBUG_PAGEALLOC
void __kernel_map_pages(struct page *page, int numpages, int enable)
{
	/* This function does nothing, all work is done via poison pages */
}
#endif
