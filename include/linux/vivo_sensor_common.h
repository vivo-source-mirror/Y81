/*============================================================================
 * @file vivo_sensor_common.h
 *
 * Vivo Sensor Common defination
 *
 * Copyright (c) 2014-2018 vivo. All Rights Reserved.
 */
#ifndef _VIVO_SENSOR_COMMON_
#define _VIVO_SENSOR_COMMON_

/*
* NOTICE:Please keep this command as:
*  A BB
*  A -->  sensor ID, like porximity sensor as 8.refer to Sensor.java
*  BB --> cmds,as you like.
* ALSO NOTICE -- Keep this head file as same as kernel's
*
*/
typedef enum {
    GSENSOR_SELF_TEST                       = 0x100,
    SENSOR_READ_REG                         = 0x101,
    SENSOR_WRITE_REG                        = 0x102,
    GSENSOR_SET_CALI                        = 0x111,
    MAGSENSOR_SELF_TEST                     = 0x200,
    GYROSENSOR_SELF_TEST                    = 0x400,
    SENSOR_COMMAND_CHECK_PS_INT     = 0x800,
    SENSOR_COMMAND_READ_PS_INT      = 0x801,
    SENSOR_COMMAND_GET_PS_DATA_RANGE        = 0x802,
    SENSOR_COMMAND_GET_PS_STATUS            = 0x803,
    SENSOR_COMMAND_READ_PS_REG_DATA         = 0x804,
    SENSOR_COMMAND_WRITE_PS_REG_DATA        = 0x805,
    SENSOR_COMMAND_SET_PS_CALI_DATA         = 0x806,
    SENSOR_COMMAND_SET_PS_ENG_CALI_DATA     = 0x807,
    SENSOR_COMMAND_NOTIFY_PS_THRES_LEVEL    = 0x808,
    SENSOR_COMMAND_GET_PS_PARA_INDEX        = 0x809,
    SENSOR_COMMAND_GET_ALS_PARA_INDEX       = 0x80a
} vivo_sensor_eng_cmd;

#define VIVO_SENSOR_ENG_CMD_ARG_SIZE        2
#endif
