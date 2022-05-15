# In case the platform does NOT support this type of sensors

ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include

obj-y	+= situation.o
obj-$(CONFIG_MTK_INPKHUB)	+=	inpocket/
obj-$(CONFIG_MTK_STATHUB) 	+= 	stationary/
obj-$(CONFIG_MTK_WAKEHUB)	+=	wake_gesture/
obj-$(CONFIG_MTK_GLGHUB)	+=	glance_gesture/
obj-$(CONFIG_MTK_PICKUPHUB)	+=	pickup_gesture/
obj-$(CONFIG_MTK_ANSWER_CALL_HUB)		+=	answercall/
obj-$(CONFIG_MTK_DEVICE_ORIENTATION_HUB)	+=	device_orientation/
obj-$(CONFIG_MTK_MOTION_DETECT_HUB)			+=	motion_detect/
obj-$(CONFIG_MTK_TILTDETECTHUB)				+=	tilt_detector/
obj-$(CONFIG_MTK_ANGLE_JUDGE_HUB)				+=	angle_judge/
obj-$(CONFIG_MTK_AMD_HUB)				+=	amd/
obj-$(CONFIG_MTK_RAISEUP_DETECT_HUB)				+=	raiseup_detect/
obj-$(CONFIG_MTK_PUTDOWN_DETECT_HUB)				+=	putdown_detect/
obj-$(CONFIG_MTK_ANGLE_DIRECTION_HUB)				+=	angle_direction/
