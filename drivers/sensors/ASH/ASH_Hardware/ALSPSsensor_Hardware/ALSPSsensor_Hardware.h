/* 
 * Copyright (C) 2015 ASUSTek Inc.
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

/*******************************/
/* ALSPS Sensor Hardware Module */
/******************************/
#ifndef __LINUX_ALSPS_HARDWARE_H
#define __LINUX_ALSPS_HARDWARE_H

/****************************/
/* ALSPS Sensor Configuration */
/**************************/
#ifdef CONFIG_TMD2755_FLAG
enum hardware_source {
	ALSPS_hw_source_tmd2755=0,
	ALSPS_hw_source_max,
};
#else
enum hardware_source {
	ALSPS_hw_source_vcnl36866=0,
	ALSPS_hw_source_max,
};
#endif

#include <linux/input/ASH.h>
extern ALSPS_hw* ALSPS_hw_vcnl36866_getHardware(void);
extern ALSPS_hw* ALSPS_hw_vcnl36866_getHardware_2nd(void);
extern ALSPS_hw* ALSPS_hw_cm36686_getHardware(void);
extern ALSPS_hw* ALSPS_hw_ap3045_getHardware(void);
extern ALSPS_hw* ALSPS_hw_tmd2755_getHardware(void);

#endif

