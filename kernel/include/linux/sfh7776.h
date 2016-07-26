/* include/linux/sfh7776.h
 *
 * Copyright (C) 2013 OSRAM, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SFH7776_H
#define __SFH7776_H

#include <linux/ioctl.h>
/*
#define SFH7776_I2C_NAME 	"sfh7776"
#define ALS_INPUT_NAME		"sfh7776_lightsensor"
#define PS_INPUT_NAME 		"sfh7776_proximity"
#define ALS_MISC_NAME		"sfh7776_lightsensor"
#define PS_MISC_NAME		"sfh7776_proximity"
*/
#define SFH7776_I2C_NAME 	"SFH7776"
#define ALS_INPUT_NAME		"light_sensor"
#define PS_INPUT_NAME 		"proximity_sensor"
#define ALS_MISC_NAME		"sfh7776_als"
#define PS_MISC_NAME		"sfh7776_ps"

#define CHIP_ID		0x9
#define SFH7776_SYS_CTRL 			0X40
#define SFH7776_MOD_CTRL 			0X41
#define SFH7776_GAIN_CURRENT_CTRL 		0X42
#define SFH7776_STATUS_REG		 	0X43

#define SFH7776_PDATA_LSB 			0X44
#define SFH7776_PDATA_MSB			0X45
#define SFH7776_VIS_ADATA_LSB 			0X46
#define SFH7776_VIS_ADATA_MSB 			0X47
#define SFH7776_IR_ADATA_LSB 			0X48
#define SFH7776_IR_ADATA_MSB 			0X49

#define SFH7776_INT_SETTING			0X4A

#define SFH7776_PS_TH_LSB			0X4B
#define SFH7776_PS_TH_MSB			0X4C
#define SFH7776_PS_TL_LSB			0X4D
#define SFH7776_PS_TL_MSB			0X4E
#define SFH7776_ALS_VIS_TH_LSB			0X4F
#define	SFH7776_ALS_VIS_TH_MSB			0X50
#define SFH7776_ALS_VIS_TL_LSB			0X51
#define SFH7776_ALS_VIS_TL_MSB			0X52

#define SFH7776_PWR	1
#define ALS_LEVEL_NUM	15

struct sfh7776_platform_data {	
	int (*power)(int, uint8_t); /* power to the chip */
	int intr;
	
	uint8_t als_polling;
	uint8_t ps_polling;
	
	uint8_t als_delay;
	uint8_t ps_delay;
	
	uint16_t als_level_table[ALS_LEVEL_NUM];
	uint16_t als_value_table[ALS_LEVEL_NUM+1];

	uint8_t ps_lt;
	uint8_t ps_ht;
};

#define SFH7776_IOCTL_MAGIC 'c'

#define SFH_IOCTL_ALS_GET_ENABLED		_IOR(SFH7776_IOCTL_MAGIC, 1, int *)
#define SFH_IOCTL_ALS_SET_ENABLED		_IOW(SFH7776_IOCTL_MAGIC, 2, int *)
#define SFH_IOCTL_ALS_GET_DELAY			_IOR(SFH7776_IOCTL_MAGIC, 3, int *)
#define SFH_IOCTL_ALS_SET_DELAY			_IOW(SFH7776_IOCTL_MAGIC, 4, int *)
#define SFH_IOCTL_PS_GET_ENABLED		_IOR(SFH7776_IOCTL_MAGIC, 5, int *)
#define SFH_IOCTL_PS_SET_ENABLED		_IOW(SFH7776_IOCTL_MAGIC, 6, int *)
#define SFH_IOCTL_PS_GET_DELAY			_IOR(SFH7776_IOCTL_MAGIC, 7, int *)
#define SFH_IOCTL_PS_SET_DELAY			_IOW(SFH7776_IOCTL_MAGIC, 8, int *)

#endif
