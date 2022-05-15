#ifndef _VIVO_AUDIO_KTV_H_
#define _VIVO_AUDIO_KTV_H_

#include <linux/ioctl.h>
#include <linux/types.h>
#include <asm/compat.h>

#define VIVO_AUDIO_KTV

/* #define DEBUG_AUD_KTV */

#ifdef DEBUG_AUD_KTV
#define PRINTK_AUD_KTV(format, args...)  pr_debug(format, ##args)
#else
#define PRINTK_AUD_KTV(format, args...)
#endif

#define PRINTWARN_AUD_KTV(format, args...)  pr_warn(format, ##args)

#define VIVO_AUDIO_KTV_DEV_MAJOR 100
#define VIVO_AUDIO_KTV_DEV_MINOR 15

#define IOCTL_VIVO_AUDIO_KTV_OPEN 			_IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 0, size_t)
#define IOCTL_VIVO_AUDIO_KTV_CLOSE 			_IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 1, size_t)
#define IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG _IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 2, size_t)
#define IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK  _IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 3, size_t)

#define COMPAT_IOCTL_VIVO_AUDIO_KTV_OPEN 			_IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 0, compat_size_t)
#define COMPAT_IOCTL_VIVO_AUDIO_KTV_CLOSE 			_IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 1, compat_size_t)
#define COMPAT_IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG 	_IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 2, compat_size_t)
#define COMPAT_IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK	_IOR(VIVO_AUDIO_KTV_DEV_MAJOR, 3, compat_size_t)


typedef struct vivo_audio_ktv_prv {
	uint8_t *tx_buffer_addr;
	uint8_t *rx_buffer_addr;
	uint64_t *share_buf_idx_addr;
	uint32_t tx_share_buffer_idx;
	uint32_t rx_share_buffer_idx;
	uint32_t tx_saved_read_idx;
	uint32_t rx_saved_write_idx;
	uint32_t tx_kernel_buf_idx;
	uint32_t rx_mixer_flag;
	uint32_t buffer_frame_size;
	uint32_t buffer_size;
	uint32_t tx_write_remained;
	uint64_t tx_first_cnt;
	uint64_t tx_read_total;
	uint64_t rx_write_total;
	uint32_t vivo_audio_ktv_flag;
	uint32_t mixer_flag;
	uint32_t ears_back;
} vivo_audio_ktv_prv_t;

extern void vivo_audio_ktv_register_device(void);
extern void vivo_audio_ktv_deregister_device(void);
extern void vivo_audio_ktv_tx_init(void);
extern void vivo_audio_ktv_rx_init(void);
extern void vivo_audio_ktv_tx_process(uint8_t *addr_start, int32_t readIdx, int32_t size, int32_t buffer_size);
extern void vivo_audio_ktv_rx_process(uint8_t *addr_start, int32_t writeIdx, int32_t size, int32_t buffer_size);
extern void vivo_audio_ktv_set_irq_mcu_counter(void);
#endif
