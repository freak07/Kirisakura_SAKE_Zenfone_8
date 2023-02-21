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

 #ifndef __LINUX_IRSENSOR_H
#define __LINUX_IRSENSOR_H

/**
 * @LIGHT_CALVALUE_200LUX_DEFAULT : 
 * @LIGHT_CALVALUE_1000LUX_DEFAULT : The default value of 200/1000 lux calibration, 
 * which are independent of platforms and hardware.
 */
#define LIGHT_CALVALUE_200LUX_DEFAULT	(200)
#define LIGHT_CALVALUE_1000LUX_DEFAULT	(1000)

/**
 * LIGHT_CHANGE_SENSITIVITY - These define the sensitivity of light sensor.
 * @LIGHT_CHANGE_LOW_SENSITIVITY : >= 1000 lux.
 * @LIGHT_CHANGE_MID_SENSITIVITY : < 1000 lux, > 200 lux.
 * @LIGHT_CHANGE_HI_SENSITIVITY : <= 200 lux.
 * @LIGHT_CHANGE_FACTORY_SENSITIVITY : for factory build.
 * @LIGHT_CHANGE_MIN_SENSITIVITY : for dynamic change sensitivity.
 */
#define LIGHT_CHANGE_LOW_SENSITIVITY 		(10)
#define LIGHT_CHANGE_MID_SENSITIVITY 		(5)
#define LIGHT_CHANGE_HI_SENSITIVITY 			(2)
#define LIGHT_CHANGE_FACTORY_SENSITIVITY 	(0)

/**
 * @LIGHT_MAX_LUX : Report 0~20000 Lux.
 */
#define LIGHT_MAX_LUX							(20000)

/**
 * @LIGHT_TURNON_DELAY_TIME : After light sensor turn on 250ms, 
 * driver will cat first correct adc/lux value.
 */
#if defined ASUS_SAKE_PROJECT
#define LIGHT_CALDATA_TRANSFER_RATIO1		(17)
#define LIGHT_CALDATA_TRANSFER_RATIO2		(10)
#define PROXIMITY_POCKET_MODE_RATIO		(30)
#define PROXIMITY_POCKET_MODE_OFFSET_RATIO		(25)
#define PROXIMITY_ADC_PER_OFFSET		(11)
#else
#define PROXIMITY_POCKET_MODE_RATIO		(100)
#endif
#ifdef CONFIG_TMD2755_FLAG
/* AMS proximity autok will change adc value, since autok need time, wait for ams chip */
#define PROXIMITY_CALIBRATION_DELAY		(150)
#define LIGHT_TURNON_DELAY_TIME			(10)
#else
#define PROXIMITY_CALIBRATION_DELAY		(0)
#define LIGHT_TURNON_DELAY_TIME			(10)
#endif

#define PROXIMITY_OIL_ALGO_DEFAULT_FLAG			(false)

#define PROXIMITY_TURNON_DELAY_TIME	(11)

#define PROXIMITY_POLLING_TIME			(1000)
#define LIGHT_POLLING_TIME			(500)

/**
 * LIGHT_LOG_THRESHOLD : We print light sensor log 
 * when the current lux value change over 100 lux from the last lux.
 */
#define LIGHT_LOG_THRESHOLD					(100)
#define LIGHT_LOG_LOW_LUX_THRESHOLD					(20)

/**
 * @ALSPS_DEFAULT_VALUE : Define the default value for driver data.
 */
#define ALSPS_DEFAULT_VALUE				(-1)


#ifdef ONE_PL_CHIP
#define PROXIMITY_INF_ER_DEFAULT     (85)
#define PROXIMITY_THDL_ER_DEFAULT    (134)
#define PROXIMITY_THDH_ER_DEFAULT    (249)
#define PROXIMITY_POCKET_ER_DEFAULT       (4073)
#ifdef CONFIG_TMD2755_FLAG
#define PROXIMITY_POCKET_DEFAULT       (3800)
#else
#define PROXIMITY_POCKET_DEFAULT       (4073)
#endif //CONFIG_TMD2755_FLAG
#define PROXIMITY_NOISE_PERIOD       (11)
#define LIGHT_ER_CALIBRATION_DEFAULT (1526)
/* ASUS BSP +++ Clay: shift lux to mitigate psensor noise when psensor on and lux < offset */
#define LIGHT_LOW_LUX_NOISE_OFFSET     (0)
/* ASUS BSP--- */
/* ASUS BSP+++ Clay: average 5 lux for offset behavior to mitigate the low lux gap */
#define LIGHT_LOW_LUX_AVG_COUNT     (5)
/* ASUS BSP--- */
#else
#define PROXIMITY_INF_ER_DEFAULT     (513)
#define PROXIMITY_THDL_ER_DEFAULT    (607)
#define PROXIMITY_THDH_ER_DEFAULT    (879)
#define PROXIMITY_POCKET_ER_DEFAULT       (4094)
#define PROXIMITY_INF_ER2_DEFAULT     (466)
#define PROXIMITY_THDL_ER2_DEFAULT    (547)
#define PROXIMITY_THDH_ER2_DEFAULT    (788)
#define PROXIMITY_POCKET_ER2_DEFAULT       (4093)
#define PROXIMITY_POCKET_DEFAULT       (4091)
#define PROXIMITY_NOISE_PERIOD       (20)
#define LIGHT_ER_CALIBRATION_DEFAULT (4563)
#define LIGHT_ER2_CALIBRATION_DEFAULT (3372)
/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset +++ */
#define LIGHT_LOW_LUX_NOISE_OFFSET     (30)
/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset --- */
/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap +++ */
#define LIGHT_LOW_LUX_AVG_COUNT     (5)
/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap --- */
#endif
#define PROXIMITY_BASIC_PERIOD       (11)
#define CS_IT_400MS (3)
#define CS_IT_100MS (1)
#define CS_IT_50MS (0)
#endif

