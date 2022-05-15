#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/kernel.h>

extern u32 get_devinfo_with_index(u32 index);

static int sec_proc_show(struct seq_file *m, void *v)
{
	u32 sec_en = (get_devinfo_with_index(27) & 0x00000002) ? 1 : 0;
	seq_printf(m, "%d\n", sec_en);
	return 0;
}

static int sec_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_proc_show, NULL);
}

static const struct file_operations sec_proc_fops = {
	.open		= sec_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_sec_init(void)
{
	proc_create("sec_en", 0x0, NULL, &sec_proc_fops);
	return 0;
}

static void __exit proc_sec_exit(void)
{
	remove_proc_entry("sec_en", NULL);
}

module_init(proc_sec_init);
module_exit(proc_sec_exit);

