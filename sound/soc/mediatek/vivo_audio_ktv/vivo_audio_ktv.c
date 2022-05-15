#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <asm/unistd.h>
#include <asm/delay.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/time.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include "vivo_audio_ktv.h"

#include "../common_int/mtk-soc-afe-control.h"

#define VIVO_AUDIO_KTV_DEVICE_NAME "vivo_audio_ktv_dev"

static int vivo_audio_ktv_dev_Open;
struct vivo_audio_ktv_prv g_vivo_audio_ktv;

#define VIVO_AUDIO_KTV_PERIOD_FRAME 240

#define VIVO_AUDIO_KTV_PERIOD_SIZE  1920 /*  5 *  48 * 2 *  4 */
										 /* 5ms 48K 2ch 32bits */
#define VIVO_AUDIO_KTV_BUFFER_NUM  8
										 /* 4 buffer 20ms */

uint8_t save_processed_buf[VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM];

uint8_t *vivo_audio_ktv_mmap_buf;


DECLARE_WAIT_QUEUE_HEAD(vivo_audio_ktv_wait);

static uint32_t vivo_audio_ktv_data_is_full;

static int vivo_audio_ktv_dev_open(struct inode *inode, struct file *file);
static int vivo_audio_ktv_dev_release(struct inode *inode, struct file *file);

static ssize_t vivo_audio_ktv_dev_read(
		struct file *file,
		char *buffer,
		size_t length,
		loff_t *offset);
static ssize_t vivo_audio_ktv_dev_write(
		struct file *file,
		const char __user *buffer,
		size_t length,
		loff_t *offset);

static long vivo_audio_ktv_dev_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param);

static long vivo_audio_ktv_dev_compat_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param);

static unsigned vivo_audio_ktv_dev_poll(struct file *file, poll_table *wait);

static int vivo_audio_ktv_dev_mmap(struct file *file, struct vm_area_struct *vma);

struct file_operations vivo_audio_ktv_dev_fops = {
	.owner 			= THIS_MODULE,
	.open 			= vivo_audio_ktv_dev_open,
	.release 		= vivo_audio_ktv_dev_release,
	.read 			= vivo_audio_ktv_dev_read,
	.write 			= vivo_audio_ktv_dev_write,
	.unlocked_ioctl = vivo_audio_ktv_dev_ioctl,
	.compat_ioctl   = vivo_audio_ktv_dev_compat_ioctl,
	.poll 			= vivo_audio_ktv_dev_poll,
	.mmap			= vivo_audio_ktv_dev_mmap,
};


struct miscdevice vivo_audio_ktv_dev_device = {
	.minor = VIVO_AUDIO_KTV_DEV_MINOR,
	.name  = VIVO_AUDIO_KTV_DEVICE_NAME,
	.fops = &vivo_audio_ktv_dev_fops,
	.mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH,
};



static void vivo_audio_ktv_prv_init(void)
{
	g_vivo_audio_ktv.vivo_audio_ktv_flag = 0;
	g_vivo_audio_ktv.mixer_flag = 0;
	g_vivo_audio_ktv.ears_back = 0;
	g_vivo_audio_ktv.tx_read_total = 0;
	g_vivo_audio_ktv.rx_write_total = 0;
	g_vivo_audio_ktv.buffer_size = VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM;
	g_vivo_audio_ktv.buffer_frame_size = (g_vivo_audio_ktv.buffer_size >> 2);
	*g_vivo_audio_ktv.share_buf_idx_addr = 0;


	vivo_audio_ktv_tx_init();
	vivo_audio_ktv_rx_init();

    /* set IRQ to 5ms */
	vivo_audio_ktv_set_irq_cnt();

}

void vivo_audio_ktv_set_irq_mcu_counter(void)
{
	if (g_vivo_audio_ktv.vivo_audio_ktv_flag)	{
		/* set IRQ to 5ms */
		vivo_audio_ktv_set_irq_cnt();
	}
}


static void vivo_audio_ktv_open(void)
{
	printk(KERN_ALERT "%s\n", __func__);
	g_vivo_audio_ktv.vivo_audio_ktv_flag = 1;

}

static void vivo_audio_ktv_close(void)
{
	printk(KERN_ALERT "%s\n", __func__);
	g_vivo_audio_ktv.vivo_audio_ktv_flag = 0;
	vivo_audio_ktv_data_is_full = 1;
	wake_up(&vivo_audio_ktv_wait);

}

static void vivo_audio_ktv_set_mixer_flag(unsigned long ioctl_param)
{
	printk(KERN_ALERT "%s: flag = %ld\n", __func__, ioctl_param);
	g_vivo_audio_ktv.mixer_flag = (int32_t)ioctl_param;

}

static void vivo_audio_ktv_set_ears_back(unsigned long ioctl_param)
{
	printk(KERN_ALERT "%s: flag = %ld\n", __func__, ioctl_param);
	g_vivo_audio_ktv.ears_back = !!(int32_t)ioctl_param;

}

static int vivo_audio_ktv_dev_open(struct inode *inode, struct file *file)
{

	printk(KERN_ALERT "%s\n", __func__);

	if (vivo_audio_ktv_dev_Open)
		return -EBUSY;

	vivo_audio_ktv_dev_Open++;
	try_module_get(THIS_MODULE);

	vivo_audio_ktv_data_is_full = 0;

	vivo_audio_ktv_mmap_buf = (uint8_t *)kmalloc(8192*2, GFP_KERNEL);
	if (vivo_audio_ktv_mmap_buf == NULL) {
		printk(KERN_ERR "%s malloc buf faild\n", __func__);
		return -ENOMEM;
	}
	SetPageReserved(virt_to_page(vivo_audio_ktv_mmap_buf));

	g_vivo_audio_ktv.tx_buffer_addr = vivo_audio_ktv_mmap_buf;
	g_vivo_audio_ktv.rx_buffer_addr = vivo_audio_ktv_mmap_buf;
	g_vivo_audio_ktv.share_buf_idx_addr = (uint64_t *)(vivo_audio_ktv_mmap_buf
		+ VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM);
	vivo_audio_ktv_prv_init();

	memset(save_processed_buf, 0, sizeof(uint8_t) * VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM);

	return 0;
}

static int vivo_audio_ktv_dev_release(struct inode *inode, struct file *file)
{
	printk(KERN_ALERT "%s (%p,%p)\n", __func__, inode, file);
	vivo_audio_ktv_dev_Open--;
	vivo_audio_ktv_data_is_full = 0;

	ClearPageReserved(virt_to_page(vivo_audio_ktv_mmap_buf));
	kfree(vivo_audio_ktv_mmap_buf);
	vivo_audio_ktv_mmap_buf = NULL;
	module_put(THIS_MODULE);
	return 0;

}

static ssize_t vivo_audio_ktv_dev_read(
		struct file *file,
		char *buffer,
		size_t length,
		loff_t *offset)
{

	return 0;
}

static ssize_t vivo_audio_ktv_dev_write(
		struct file *file,
		const char __user *buffer,
		size_t length,
		loff_t *offset)
{

	return 0;

}

static long vivo_audio_ktv_dev_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param)
{
	printk(KERN_ALERT "%s ioctl = %u\n", __func__, cmd);

	switch (cmd)	{
	case IOCTL_VIVO_AUDIO_KTV_OPEN:
		vivo_audio_ktv_open();
		break;
	case IOCTL_VIVO_AUDIO_KTV_CLOSE:
		vivo_audio_ktv_close();
		break;
	case IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG:
		vivo_audio_ktv_set_mixer_flag(ioctl_param);
		break;
	case IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK:
		vivo_audio_ktv_set_ears_back(ioctl_param);
		break;
	default:
		printk(KERN_ERR "%s wrong ioctrl 0x%x\n", __func__, cmd);
		return -ENOTTY;

	}
	return 0;
}

static long vivo_audio_ktv_dev_compat_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param)
{
	printk(KERN_ALERT "%s ioctl = %u\n", __func__, cmd);

	switch (cmd)	{
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_OPEN:
		cmd = IOCTL_VIVO_AUDIO_KTV_OPEN;
		break;
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_CLOSE:
		cmd = IOCTL_VIVO_AUDIO_KTV_CLOSE;
		break;
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG:
		cmd = IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG;
		break;
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK:
		cmd = IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK;
		break;
	default:
		printk(KERN_ERR "%s wrong ioctrl 0x%x\n", __func__, cmd);
		return -ENOTTY;

	}

	return vivo_audio_ktv_dev_ioctl(file, cmd, ioctl_param);

}

static unsigned vivo_audio_ktv_dev_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	PRINTK_AUD_KTV(KERN_ALERT "%s\n", __func__);
	poll_wait(file, &vivo_audio_ktv_wait, wait);

	if (vivo_audio_ktv_data_is_full)	{
		mask |= (POLLIN | POLLRDNORM);
	}
	vivo_audio_ktv_data_is_full = 0;
	return mask;

}

static int vivo_audio_ktv_dev_mmap(struct file *file, struct vm_area_struct *vma)
{

	unsigned long page = (virt_to_phys(vivo_audio_ktv_mmap_buf) >> PAGE_SHIFT);
	unsigned long vmsize = vma->vm_end - vma->vm_start;
	unsigned long size = sizeof(vivo_audio_ktv_mmap_buf);

	printk(KERN_ALERT "%s size = %lu, vmsize = %lu\n", __func__, size, vmsize);

	/* PAGE_SIZE */
	if (remap_pfn_range(vma, vma->vm_start, page, vmsize, PAGE_SHARED)) {
		printk(KERN_ERR "%s faild\n", __func__);
		return -EAGAIN;
	}
	return 0;

}

void vivo_audio_ktv_tx_init(void)
{
	g_vivo_audio_ktv.tx_read_total = 0;
	g_vivo_audio_ktv.rx_write_total = 0;


	g_vivo_audio_ktv.tx_first_cnt = 0; /* for the first time check if data is 5ms period */

	g_vivo_audio_ktv.tx_write_remained = 0;

	g_vivo_audio_ktv.tx_share_buffer_idx = 0;
	g_vivo_audio_ktv.rx_share_buffer_idx = 0;

	g_vivo_audio_ktv.tx_saved_read_idx = 0;
	g_vivo_audio_ktv.rx_saved_write_idx = 2 * VIVO_AUDIO_KTV_PERIOD_SIZE;
	g_vivo_audio_ktv.rx_mixer_flag = 0;
	g_vivo_audio_ktv.tx_kernel_buf_idx = 0;

}

void vivo_audio_ktv_rx_init(void)
{
	g_vivo_audio_ktv.rx_write_total = 0;
	g_vivo_audio_ktv.tx_read_total = 0;
	g_vivo_audio_ktv.rx_mixer_flag = 0;

	g_vivo_audio_ktv.tx_first_cnt = 0;

	g_vivo_audio_ktv.rx_share_buffer_idx = g_vivo_audio_ktv.tx_share_buffer_idx;

	g_vivo_audio_ktv.tx_saved_read_idx = 0;
	g_vivo_audio_ktv.rx_saved_write_idx = 2 * VIVO_AUDIO_KTV_PERIOD_SIZE;

}

/********************************************************************
*  Function: get tx data for ktv algorithm processing, the data will be readed in user
*   space, and processed by ktv algorithm, if data is gether more than one periods
*   VIVO_AUDIO_KTV_PERIOD_SIZE, it will poll to inform the user to takeaway the
*   data.
*  addr_start: the virtual address of kernel buffer for tx(usually DMA buffer)
*  readIdx: the cursor of tx data start to read
*  size: the size of tx data want to read, after that the cursor will move to readIdx+size
*  buffer_size: the DMA buffer size, decide by user process.
*
*******************************************************************/

int32_t vivo_audio_ktv_save_tx_data(uint8_t *addr_start, int32_t readIdx, int32_t size, int32_t buffer_size)
{

	uint32_t *user_buffer_addr;
	uint32_t *kernel_buffer_addr;
	uint32_t *saved_buffer_addr;

	uint32_t frame_count;
	uint32_t actual_size;
	int32_t valueLeft, valueRight;

	int32_t i;
	int32_t kernel_idx;
	int32_t user_idx;
	int32_t save_idx;
	int32_t buffer_frame_size = buffer_size >> 2;

	PRINTK_AUD_KTV(KERN_DEBUG "vivo_audio_ktv_save_tx_data readIdx = 0x%x size = 0x%x.\n", readIdx, size);

   /* add for the interrupt time is not 5ms at the start of the catputre */
	if (g_vivo_audio_ktv.tx_first_cnt == 0 && size >= 0x1E00) {
		return 0;
	 }

	g_vivo_audio_ktv.tx_first_cnt += size;

	 /* ignore the first two periods (10ms) datas,but still need to move tx_kernel_buf_idx */
	 if (g_vivo_audio_ktv.tx_first_cnt <= VIVO_AUDIO_KTV_PERIOD_SIZE * 2) {
		PRINTK_AUD_KTV(KERN_ALERT "vivo_audio_ktv_save_tx_data tx_first_cnt = %lld\n", g_vivo_audio_ktv.tx_first_cnt);
		g_vivo_audio_ktv.tx_kernel_buf_idx = readIdx + size;

		if (g_vivo_audio_ktv.tx_kernel_buf_idx >= buffer_size) {

			g_vivo_audio_ktv.tx_kernel_buf_idx -= buffer_size;

		}

		return 0;
	 }

	if (g_vivo_audio_ktv.tx_kernel_buf_idx != readIdx) {
		if (readIdx + size > g_vivo_audio_ktv.tx_kernel_buf_idx) {
			actual_size = readIdx + size - g_vivo_audio_ktv.tx_kernel_buf_idx;
		} else {
			actual_size = 0;
		}

		if (actual_size >= buffer_size)
			actual_size -= buffer_size;

		if (actual_size >= (buffer_size >> 1)) {
			actual_size = 0;
		}

		if (actual_size == 0) {
			return 0;
		}
	} else {
		actual_size = size;
	}

	if (actual_size > VIVO_AUDIO_KTV_PERIOD_SIZE) {
		PRINTK_AUD_KTV(KERN_DEBUG "%s overflow size = 0x%x, actual_size=%d. \n", __func__, size, actual_size);
	}
	g_vivo_audio_ktv.tx_read_total += actual_size;
	g_vivo_audio_ktv.tx_write_remained += actual_size;


	kernel_buffer_addr = (int32_t *)(addr_start);
	kernel_idx = (g_vivo_audio_ktv.tx_kernel_buf_idx >> 2); /* bitwidth=32bit, 4byte */

	user_buffer_addr = (int32_t *)(g_vivo_audio_ktv.tx_buffer_addr);
	user_idx = (g_vivo_audio_ktv.tx_share_buffer_idx >> 2); /* bitwidth=32bit, 4byte */

	saved_buffer_addr = (int32_t *)(save_processed_buf);
	save_idx = (g_vivo_audio_ktv.tx_saved_read_idx >> 2); /* bitwidth=32bit, 4byte */
	PRINTK_AUD_KTV(KERN_DEBUG "vivo_audio_ktv_save_tx_data kernel_idx = 0x%x user_idx = 0x%x save_idx = 0x%x actual_size = 0x%x, mixer_flag=%d, rx_mixer_flag=%d.\n",
		(kernel_idx << 2), (user_idx << 2), (save_idx << 2), actual_size, g_vivo_audio_ktv.mixer_flag, g_vivo_audio_ktv.rx_mixer_flag);

	frame_count = actual_size >> 3; /* byte to frame(32bit * 2ch) */

	for (i = 0; i < frame_count; i++) {
		user_buffer_addr[user_idx] = kernel_buffer_addr[kernel_idx];

		if (g_vivo_audio_ktv.mixer_flag) {
			valueLeft = saved_buffer_addr[save_idx];
			valueLeft <<= 1;
			if (valueLeft > 0x7fffff)
				valueLeft = 0x7fffff;

			if (valueLeft < -0x800000)
				valueLeft = -0x800000;
			if (g_vivo_audio_ktv.rx_mixer_flag)
				kernel_buffer_addr[kernel_idx] = valueLeft;
			else
				kernel_buffer_addr[kernel_idx] = 0;

		} else {
			valueLeft = kernel_buffer_addr[kernel_idx];
			valueLeft <<= 1;
			if (valueLeft > 0x7fffff)
				valueLeft = 0x7fffff;

			if (valueLeft < -0x800000)
				valueLeft = -0x800000;
			kernel_buffer_addr[kernel_idx] = valueLeft;
		}
		user_idx++;
		kernel_idx++;
		save_idx++;

		user_buffer_addr[user_idx] = kernel_buffer_addr[kernel_idx];

		if (g_vivo_audio_ktv.mixer_flag) {
			valueRight = saved_buffer_addr[save_idx];
			valueRight <<= 1;
			if (valueRight > 0x7fffff)
				valueRight = 0x7fffff;

			if (valueRight < -0x800000)
				valueRight = -0x800000;
			if (g_vivo_audio_ktv.rx_mixer_flag)
				kernel_buffer_addr[kernel_idx] = valueRight;
			else
				kernel_buffer_addr[kernel_idx] = 0;

		} else {
			valueRight = kernel_buffer_addr[kernel_idx];
			valueRight <<= 1;
			if (valueRight > 0x7fffff)
				valueRight = 0x7fffff;

			if (valueRight < -0x800000)
				valueRight = -0x800000;
			kernel_buffer_addr[kernel_idx] = valueRight;
		}

		user_idx++;
		kernel_idx++;
		save_idx++;

		if (kernel_idx >= buffer_frame_size) {
			kernel_idx = 0;
		}

		if (user_idx >= g_vivo_audio_ktv.buffer_frame_size) {
			user_idx = 0;
		}

		if (save_idx >= g_vivo_audio_ktv.buffer_frame_size) {
			save_idx = 0;
		}

	}

	if (g_vivo_audio_ktv.rx_mixer_flag)
		g_vivo_audio_ktv.tx_saved_read_idx = (save_idx << 2);

	g_vivo_audio_ktv.tx_share_buffer_idx = (user_idx << 2);
	g_vivo_audio_ktv.tx_kernel_buf_idx = (kernel_idx << 2);

	/* if data is full, inform the user process to take away the data */
	if (g_vivo_audio_ktv.tx_write_remained >= VIVO_AUDIO_KTV_PERIOD_SIZE) {
		g_vivo_audio_ktv.tx_write_remained -= VIVO_AUDIO_KTV_PERIOD_SIZE;
		*g_vivo_audio_ktv.share_buf_idx_addr = (g_vivo_audio_ktv.tx_share_buffer_idx / VIVO_AUDIO_KTV_PERIOD_SIZE);
		vivo_audio_ktv_data_is_full = 1;
		wake_up(&vivo_audio_ktv_wait);

	}
	return 0;

}

/****************************************************************************************
*  Function: this will mixer tx data to rx, the tx data is the one has been process by ktv algorithm, rx is the
*   original playback data. the mixer is tx_left + rx_left = left_final, tx_right + rx_right = right_final
*  addr_start: the virtual address of rx kernel buffer for playback, the mixerdata will put back the kernel buffer
*  writeIdx: the cursor of data in kernel to start mixer
*  size: the size in byte to mixer
*  buffer_size: the DMA buffer size, decide by user process
*
*********************************************************************************************/
int32_t vivo_audio_ktv_mixer_tx_to_rx(uint8_t *addr_start, int32_t writeIdx, int32_t size, int32_t buffer_size)
{

	ssize_t frame_count;
	int32_t i;
	int32_t txDataLeft, txDataRight, rxDataLeft, rxDataRight;
	int32_t valueLeft, valueRight;

	uint32_t *user_buffer_addr;
	uint32_t *kernel_buffer_addr;
	uint32_t *saved_buffer_addr;

	int32_t kernel_idx;
	int32_t user_idx;
	int32_t save_idx;
	int32_t buffer_frame_size = buffer_size >> 2;

	PRINTK_AUD_KTV(KERN_DEBUG "%s : writeIdx = 0x%x size = 0x%x, mixer_flag=%d, rx_mixer_flag=%d.\n",
		__func__, writeIdx, size, g_vivo_audio_ktv.mixer_flag, g_vivo_audio_ktv.rx_mixer_flag);

	if (g_vivo_audio_ktv.rx_mixer_flag == 0) { /* ignore first 4 periods data(4*5ms) */
		if ((g_vivo_audio_ktv.rx_write_total + 4 * VIVO_AUDIO_KTV_PERIOD_SIZE) > g_vivo_audio_ktv.tx_read_total) {
			return 0;
		} else {
			g_vivo_audio_ktv.rx_mixer_flag = 1;
		}
	} else {
		if (size > VIVO_AUDIO_KTV_PERIOD_SIZE) {
			PRINTK_AUD_KTV("%s overflow size = %d\n", __func__, size);
		}

		if ((g_vivo_audio_ktv.rx_write_total  + size) > g_vivo_audio_ktv.tx_read_total) {
			g_vivo_audio_ktv.rx_mixer_flag = 0;
			return 0;
		}
	}

	if (g_vivo_audio_ktv.tx_read_total - g_vivo_audio_ktv.rx_write_total >= VIVO_AUDIO_KTV_PERIOD_SIZE * 7) {
		PRINTWARN_AUD_KTV("%s overflow need reset\n", __func__);
		g_vivo_audio_ktv.rx_write_total = 0;
		g_vivo_audio_ktv.tx_read_total = 0;
		g_vivo_audio_ktv.rx_mixer_flag = 0;

		g_vivo_audio_ktv.rx_share_buffer_idx = g_vivo_audio_ktv.tx_share_buffer_idx;

		g_vivo_audio_ktv.tx_saved_read_idx = 0;
		g_vivo_audio_ktv.rx_saved_write_idx = 2 * VIVO_AUDIO_KTV_PERIOD_SIZE;
		*g_vivo_audio_ktv.share_buf_idx_addr = 0;
		return 0;
	}
	PRINTK_AUD_KTV(KERN_DEBUG "%s : rx_write_total = %lld tx_read_total = %lld diff = %lld\n", __func__,
		g_vivo_audio_ktv.rx_write_total, g_vivo_audio_ktv.tx_read_total, g_vivo_audio_ktv.tx_read_total - g_vivo_audio_ktv.rx_write_total);

	g_vivo_audio_ktv.rx_write_total += size;

	kernel_buffer_addr = (int32_t *)(addr_start);
	/* kernel_idx = (writeIdx + VIVO_AUDIO_KTV_PERIOD_SIZE * 2) >> 2; */ /* bit_width_sample=32 */
	kernel_idx = writeIdx >> 2; /* bit_width_sample=32 */
	if (kernel_idx >= buffer_frame_size) {
		kernel_idx -= buffer_frame_size;
	}


	user_buffer_addr = (int32_t *)(g_vivo_audio_ktv.rx_buffer_addr);
	user_idx = (g_vivo_audio_ktv.rx_share_buffer_idx >> 2);

	saved_buffer_addr = (int32_t *)(save_processed_buf);
	save_idx = (g_vivo_audio_ktv.rx_saved_write_idx >> 2);
	PRINTK_AUD_KTV(KERN_DEBUG "%s : kernel_idx = 0x%x user_idx = 0x%x save_idx = 0x%x\n", __func__,
		(kernel_idx << 2), (user_idx << 2), (save_idx << 2));

	frame_count = (size >> 3);

	for (i = 0; i < frame_count; i++) {
		txDataLeft = user_buffer_addr[user_idx]; /* the data has been processed by ktv algorithm */
		user_buffer_addr[user_idx] = 0;
		rxDataLeft = kernel_buffer_addr[kernel_idx]; /* the original playback data */

		valueLeft = rxDataLeft  + txDataLeft; /*mix the tx to rx playback data */


		if (valueLeft > 0x7fffff)
			valueLeft = 0x7fffff;

		if (valueLeft < -0x800000)
			valueLeft = -0x800000;

		if (g_vivo_audio_ktv.ears_back)
			kernel_buffer_addr[kernel_idx] = valueLeft; /* put the mix data to playback buffer again, to play them together. */

		valueLeft <<= 2;
		if (valueLeft > 0x7fffff)
			valueLeft = 0x7fffff;

		if (valueLeft < -0x800000)
			valueLeft = -0x800000;

		if (g_vivo_audio_ktv.mixer_flag == 2)
			saved_buffer_addr[save_idx] = txDataLeft;
		else
			saved_buffer_addr[save_idx] = valueLeft;

		user_idx++;
		kernel_idx++;
		save_idx++;

		txDataRight = user_buffer_addr[user_idx];
		user_buffer_addr[user_idx] = 0;
		rxDataRight = kernel_buffer_addr[kernel_idx];


		valueRight = rxDataRight + txDataRight;


		if (valueRight > 0x7fffff)
			valueRight = 0x7fffff;

		if (valueRight < -0x800000)
			valueRight = -0x800000;

		if (g_vivo_audio_ktv.ears_back)
			kernel_buffer_addr[kernel_idx] = valueRight;


		valueRight <<= 2;

		if (valueRight > 0x7fffff)
			valueRight = 0x7fffff;

		if (valueRight < -0x800000)
			valueRight = -0x800000;

		if (g_vivo_audio_ktv.mixer_flag == 2)
			saved_buffer_addr[save_idx] = txDataRight;
		else
			saved_buffer_addr[save_idx] = valueRight;

		user_idx++;
		kernel_idx++;
		save_idx++;

		if (kernel_idx >= buffer_frame_size) {
			kernel_idx = 0;
		}

		if (user_idx >= g_vivo_audio_ktv.buffer_frame_size) {
			user_idx = 0;
		}

		if (save_idx >= g_vivo_audio_ktv.buffer_frame_size) {
			save_idx = 0;
		}

	}

	g_vivo_audio_ktv.rx_saved_write_idx = (save_idx << 2);
	g_vivo_audio_ktv.rx_share_buffer_idx = (user_idx << 2);


	return 0;
}


void vivo_audio_ktv_tx_process(uint8_t *addr_start, int32_t readIdx, int32_t size, int32_t buffer_size)
{
	if (g_vivo_audio_ktv.vivo_audio_ktv_flag)	{
		vivo_audio_ktv_save_tx_data(addr_start, readIdx, size, buffer_size);
	}

}

void vivo_audio_ktv_rx_process(uint8_t *addr_start, int32_t writeIdx, int32_t size, int32_t buffer_size)
{
	if (g_vivo_audio_ktv.vivo_audio_ktv_flag)	{
		vivo_audio_ktv_mixer_tx_to_rx(addr_start, writeIdx, size, buffer_size);
	}

}


void vivo_audio_ktv_register_device(void)
{
	int32_t ret;
	ret = misc_register(&vivo_audio_ktv_dev_device);

	if (ret < 0) {
		printk(KERN_ERR "%s : register failed %d\n", __func__, ret);
		return;
	}

	printk(KERN_ALERT "%s : register sucess\n", __func__);
	return;
}

void vivo_audio_ktv_deregister_device(void)
{
	misc_deregister(&vivo_audio_ktv_dev_device);
}




