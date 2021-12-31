/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, Focaltech Ltd. All rights reserved.
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

/*****************************************************************************
*
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
// Vodka_BSP Display
#include <drm/drm_zf8.h>

/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/*
 * report key code
 */
// FOD
#define KEY_GESTURE_O                           KEY_O
#define KEY_GESTURE_F                           KEY_F
#define KEY_GESTURE_U                           KEY_U
#define KEY_GESTURE_L                           KEY_L
// Zenmotion
#define KEY_GESTURE_UP                          KEY_UP
#define KEY_GESTURE_E                           KEY_E
#define KEY_GESTURE_M                           KEY_M
#define KEY_GESTURE_W                           KEY_W
#define KEY_GESTURE_S                           KEY_S
#define KEY_GESTURE_V                           KEY_V
#define KEY_GESTURE_Z                           KEY_Z

/*
 * gesture id
 */
#define GESTURE_O                               0x2A
#define GESTURE_F                               0x28
#define GESTURE_U                               0x29
#define GESTURE_L                               0x27

#define GESTURE_UP                              0x22
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x65

#define MUSIC_PAUSE				0x26
#define MUSIC_REWIND				0x51
#define MUSIC_FORWARD				0x52


#define GESTURE_TYPE             "driver/gesture_type"
#define DCLICK                   "driver/dclick"
#define SWIPEUP                  "driver/swipeup"
/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
* gesture_id    - mean which gesture is recognised
* point_num     - points number of this gesture
* coordinate_x  - All gesture point x coordinate
* coordinate_y  - All gesture point y coordinate
* mode          - gesture enable/disable, need enable by host
*               - 1:enable gesture function(default)  0:disable
* active        - gesture work flag,
*                 always set 1 when suspend, set 0 when resume
*/
struct fts_gesture_st {
    u8 gesture_id;
    u8 point_num;
    u16 coordinate_x[FTS_GESTURE_POINTS_MAX];
    u16 coordinate_y[FTS_GESTURE_POINTS_MAX];
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern u8 FTS_gesture_register_D1;
extern u8 FTS_gesture_register_D2;
extern u8 FTS_gesture_register_D5;
extern u8 FTS_gesture_register_D6;
extern u8 FTS_gesture_register_D7;
extern u8 FTS_gesture_register_D8;

extern bool proximityStatus(void);
//extern bool proximitySecStatus(void);
/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct fts_ts_data *ts_data = fts_data;

    mutex_lock(&ts_data->input_dev->mutex);
    fts_read_reg(FTS_REG_GESTURE_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "Gesture Mode:%s\n",
                     ts_data->gesture_mode ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0)=%d\n", val);
    mutex_unlock(&ts_data->input_dev->mutex);

    return count;
}

static ssize_t fts_gesture_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct fts_ts_data *ts_data = fts_data;

    mutex_lock(&ts_data->input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_DEBUG("enable gesture");
        ts_data->gesture_mode = ENABLE;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_DEBUG("disable gesture");
        ts_data->gesture_mode = DISABLE;
    }
    mutex_unlock(&ts_data->input_dev->mutex);

    return count;
}

static ssize_t fts_gesture_buf_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    struct input_dev *input_dev = fts_data->input_dev;
    struct fts_gesture_st *gesture = &fts_gesture_data;

    mutex_lock(&input_dev->mutex);
    count = snprintf(buf, PAGE_SIZE, "Gesture ID:%d\n", gesture->gesture_id);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum:%d\n",
                      gesture->point_num);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Points Buffer:\n");

    /* save point data,max:6 */
    for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i,
                          gesture->coordinate_x[i], gesture->coordinate_y[i]);
        if ((i + 1) % 4 == 0)
            count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_gesture_buf_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

// Touch_BSP +++
static ssize_t switch_game_mode_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp = 0;
	tmp = buf[0] - 48;

	if (tmp == 0) {
		fts_data->game_mode = 0;
		printk("[Focal][Touch] game_mode_disable ! \n");
	} else if (tmp == 1) {
		fts_data->game_mode = 1;
		printk("[Focal][Touch] game_mode_enable ! \n");
	}
	return count;
}

static ssize_t switch_game_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d \n", fts_data->game_mode);
}

int fts_enter_game_mode(int type)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if (ts_data->suspended) {
        FTS_INFO("In suspend mode, skip write i2c.");
        return ret;
    }

    if (type == 0) {	/* Rotation = 0 */
	ret = fts_write_reg(0x8C, 0x00);
	if (ret < 0) {
            FTS_ERROR("write value fail, addr=0x8C, value=0x00");
        }
	ret = fts_write_reg(0x8D, 0x00);
	if (ret < 0) {
            FTS_ERROR("write value fail, addr=0x8D, value=0x00");
        }
    } else if (type == 1) {	/* Rotation = 90 */
	ret = fts_write_reg(0x8C, 0x01);
	if (ret < 0) {
            FTS_ERROR("write value fail, addr=0x8C, value=0x01");
        }
	ret = fts_write_reg(0x8D, 0x30);
	if (ret < 0) {
            FTS_ERROR("write value fail, addr=0x8D, value=0x30");
        }
    } else if (type == 2) {	/* Rotation = 270 */
        ret = fts_write_reg(0x8C, 0x02);
	if (ret < 0) {
            FTS_ERROR("write value fail, addr=0x8C, value=0x02");
        }
	ret = fts_write_reg(0x8D, 0x30);
	if (ret < 0) {
            FTS_ERROR("write value fail, addr=0x8D, value=0x30");
        }
    }

    return ret;
}

static ssize_t rotation_type_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp = 0;
	tmp = buf[0] - 48;

	if (tmp == 0) {
		fts_data->rotation_type = 0;
		fts_enter_game_mode(0);
		printk("[Focal][Touch] game_mode_disable, rotation = 0 ! \n");
	} else if (tmp == 1) {
		fts_data->rotation_type = 1;
		fts_enter_game_mode(1);
		printk("[Focal][Touch] game_mode_enable, rotation = 90 ! \n");
	} else if (tmp == 2) {
		fts_data->rotation_type = 2;
		fts_enter_game_mode(2);
		printk("[Focal][Touch] game_mode_enable, rotation = 270 ! \n");
	}
	return count;
}

static ssize_t rotation_type_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d \n", fts_data->rotation_type);
}
// Touch_BSP ---

/* sysfs gesture node
 *   read example: cat  fts_gesture_mode       ---read gesture mode
 *   write example:echo 1 > fts_gesture_mode   --- write gesture mode to 1
 *
 */
static DEVICE_ATTR(fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show,
                   fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf        --- read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR,
                   fts_gesture_buf_show, fts_gesture_buf_store);
// Touch_BSP +++
// add for game mode
static DEVICE_ATTR(game_mode, S_IRUGO | S_IWUSR, switch_game_mode_show, switch_game_mode_store);
static DEVICE_ATTR(rotation_type, S_IRUGO | S_IWUSR, rotation_type_show, rotation_type_store);

// Touch_BSP ---

static struct attribute *fts_gesture_mode_attrs[] = {
    &dev_attr_fts_gesture_mode.attr,
    &dev_attr_fts_gesture_buf.attr,
    &dev_attr_game_mode.attr,
    &dev_attr_rotation_type.attr,
    NULL,
};

static struct attribute_group fts_gesture_group = {
    .attrs = fts_gesture_mode_attrs,
};

static int fts_create_gesture_sysfs(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_gesture_group);
    if (ret) {
        FTS_ERROR("gesture sys node create fail");
        sysfs_remove_group(&dev->kobj, &fts_gesture_group);
        return ret;
    }

    return 0;
}

void write_fp_xy(struct fts_ts_data *ts_data) 
{
    u8 fp_x_h;
    u8 fp_x_l;
    u8 fp_y_h;
    u8 fp_y_l;
    int ret = 0;
    ret =fts_read_reg(FTS_FOD_X_H_POS_REG, &fp_x_h);
    ret =fts_read_reg(FTS_FOD_X_L_POS_REG, &fp_x_l);
    ret =fts_read_reg(FTS_FOD_Y_H_POS_REG, &fp_y_h);
    ret =fts_read_reg(FTS_FOD_Y_L_POS_REG, &fp_y_l);
    ts_data->fp_x = (u16)(((fp_x_h & 0x0F) << 8) + fp_x_l);
    ts_data->fp_y = (u16)(((fp_y_h & 0x0F) << 8) + fp_y_l);
    FTS_DEBUG("x %d y %d",ts_data->fp_x,ts_data->fp_y);      
}

static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
    int gesture = -1;
    struct fts_ts_data *ts_data = fts_data;
    bool proxy_status = false;
    bool proxy_skip = false;
    unsigned int time_delta;

    FTS_DEBUG("gesture_id:0x%x", gesture_id);

    if (ts_data->phone_call_state == ENABLE) {
	proxy_status = proximityStatus();
	if (proxy_status) {
	    FTS_INFO("Proximity on");
	    return;
	}
    }

    switch (gesture_id) {
    case GESTURE_O:
	if ((ts_data->fp_enable == 1) && (ts_data->fp_report_type != 0)) {
		gesture = KEY_GESTURE_O;
		FTS_INFO("key O");
		fts_data->next_resume_isaod = true;
		write_fp_xy(fts_data);
		proxy_skip = true;
	}
	if ((ts_data->aod_enable == ENABLE) && (ts_data->fp_report_type == 0)) {  // AOD only
		FTS_INFO("key L in aod");
		gesture = KEY_GESTURE_L;
		proxy_skip = false;
	}
        break;
    case GESTURE_F:
	if ((ts_data->fp_enable == 1) && (ts_data->fp_report_type != 0)) {
		// Vodka BSP Display
                zf8_drm_notify(ASUS_NOTIFY_FOD_TOUCHED, 1);
		gesture = KEY_GESTURE_F;
		FTS_INFO("key F");
		fts_data->next_resume_isaod = true;
		write_fp_xy(fts_data);
		proxy_skip = true;
	}
	if ((ts_data->aod_enable == ENABLE) && (ts_data->fp_report_type == 0)) {  // AOD only
		FTS_INFO("key L in aod");
		gesture = KEY_GESTURE_L;
		proxy_skip = false;
	}
	break;
    case GESTURE_U:
	if ((ts_data->fp_enable == 1) && (ts_data->fp_report_type != 0)) {
		gesture = KEY_GESTURE_U;
		FTS_INFO("key U");
		fts_data->next_resume_isaod = false;
		proxy_skip = true;
	}
	if ((ts_data->aod_enable == ENABLE) && (ts_data->fp_report_type == 0)) {  // AOD only
		FTS_INFO("skip report U in aod");
	}
	break;
    case GESTURE_L:
	if ((ts_data->fp_enable == 1) && (ts_data->fp_report_type != 0)) {
		gesture = KEY_GESTURE_L;
		FTS_INFO("key L");
		proxy_skip = true;
	}
	if ((ts_data->aod_enable == ENABLE) && (ts_data->fp_report_type == 0)) {  // AOD only
		FTS_INFO("key L in aod");
		gesture = KEY_GESTURE_L;
		proxy_skip = false;
	}
	break;
// Zenmotion
    case GESTURE_W:
	if (ts_data->gesture_type & 1 << 1) { // W
	gesture = KEY_GESTURE_W;
	FTS_INFO("key W");
	}
	break;
    case GESTURE_S:
        if (ts_data->gesture_type & 1 << 2) { // S
	gesture = KEY_GESTURE_S;
	FTS_INFO("key S");
	}
	break;
    case GESTURE_E:
	if (ts_data->gesture_type & 1 << 3) { // e
	gesture = KEY_GESTURE_E;
	FTS_INFO("key E");
	}
	break;
    case GESTURE_M:
	if (ts_data->gesture_type & 1 << 4) { // M
	gesture = KEY_GESTURE_M;
	FTS_INFO("key M");
	}
	break;
    case GESTURE_Z:
        if (ts_data->gesture_type & 1 << 5) { // Z
	gesture = KEY_GESTURE_Z;
	FTS_INFO("key Z");
	}
	break;
    case GESTURE_V:
	if (ts_data->gesture_type & 1 << 6) { // V
	gesture = KEY_GESTURE_V;
	FTS_INFO("key V");
	}
	break;
    case GESTURE_DOUBLECLICK:
        if (ts_data->dclick_mode == 1) {
	gesture = KEY_POWER;
	FTS_INFO("key double click");
	}
	break;
    case GESTURE_UP:
	if (ts_data->swipeup_mode == 1) {
	gesture = KEY_GESTURE_UP;
	FTS_INFO("key swipe up");
	}
	break;
    case MUSIC_PAUSE:
	if (ts_data->gesture_type & 1 << 7) { // music_control
	gesture = KEY_PAUSE;
	FTS_INFO("key MUSIC_PAUSE");
	}
	break;
    case MUSIC_REWIND:
        if (ts_data->gesture_type & 1 << 7) { // music_control
	gesture = KEY_REWIND;
	FTS_INFO("key MUSIC_REWIND");
	}
	break;
    case MUSIC_FORWARD:
        if (ts_data->gesture_type & 1 << 7) { // music_control
	gesture = KEY_FORWARD;
	FTS_INFO("key MUSIC_FORWARD");
	}
	break;
    default:
        gesture = -1;
        break;
    }

    /* report event key */
    if (gesture != -1) {
        if (proxy_skip != true) {
            proxy_status = proximityStatus();
            if (proxy_status) {
                FTS_INFO("Gesture Code %d not FOD gesture and Proximity on", gesture);
                return;
            }
        }
        if(ts_data->perftime == 1) {
            input_report_key(input_dev, gesture, 1);
            input_sync(input_dev);
            input_report_key(input_dev, gesture, 0);
            input_sync(input_dev);
            time_delta = ktime_ms_delta(ktime_get(), ts_data->irq_received);
            FTS_INFO("Gesture Code = %d , delta_time %llu ms", gesture, time_delta);
        } else {
            input_report_key(input_dev, gesture, 1);
            input_sync(input_dev);
            input_report_key(input_dev, gesture, 0);
            input_sync(input_dev);
            FTS_INFO("Gesture Code = %d", gesture);
        }

    } else {
        FTS_INFO("gesture = %d, do not report key.", gesture);
    }
}

/*****************************************************************************
* Name: fts_gesture_readdata
* Brief: Read information about gesture: enable flag/gesture points..., if ges-
*        ture enable, save gesture points' information, and report to OS.
*        It will be called this function every intrrupt when FTS_GESTURE_EN = 1
*
*        gesture data length: 1(enable) + 1(reserve) + 2(header) + 6 * 4
* Input: ts_data - global struct data
*        data    - gesture data buffer if non-flash, else NULL
* Output:
* Return: 0 - read gesture data successfully, the report data is gesture data
*         1 - tp not in suspend/gesture not enable in TP FW
*         -Exx - error
*****************************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *data)
{
    int ret = 0;
    int i = 0;
    int index = 0;
    u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
    struct input_dev *input_dev = ts_data->input_dev;
    struct fts_gesture_st *gesture = &fts_gesture_data;

    if (!ts_data->suspended || !ts_data->gesture_mode) {
        return 1;
    }

    msleep(10);

    ret = fts_read_reg(FTS_REG_GESTURE_EN, &buf[0]);
    if ((ret < 0) || (buf[0] != ENABLE)) {
        FTS_DEBUG("gesture not enable in fw, don't process gesture");
        return 1;
    }

    buf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
    ret = fts_read(&buf[2], 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
    if (ret < 0) {
        FTS_ERROR("read gesture header data fail");
        return ret;
    }

    /* init variable before read gesture point */
    memset(gesture->coordinate_x, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
    memset(gesture->coordinate_y, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
    gesture->gesture_id = buf[2];
    gesture->point_num = buf[3];
    FTS_DEBUG("gesture_id=%d, point_num=%d",
              gesture->gesture_id, gesture->point_num);

    /* save point data,max:6 */
    for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
        index = 4 * i + 4;
        gesture->coordinate_x[i] = (u16)(((buf[0 + index] & 0x0F) << 8)
                                         + buf[1 + index]);
        gesture->coordinate_y[i] = (u16)(((buf[2 + index] & 0x0F) << 8)
                                         + buf[3 + index]);
    }

    /* report gesture to OS */
    fts_gesture_report(input_dev, gesture->gesture_id);
    return 0;
}

void fts_gesture_recovery(struct fts_ts_data *ts_data)
{
    if (ts_data->gesture_mode && ts_data->suspended && is_enter_gesture_mode(ts_data)) {
        set_gesture_register(ts_data);
        FTS_DEBUG("gesture recovery...");
/*
        fts_write_reg(0xD1, 0xFF);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
*/
    }
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
//    int i = 0;
//    u8 state = 0xFF;
    int gesture_suspend = 0;

    FTS_FUNC_ENTER();

    gesture_suspend = set_gesture_register(ts_data);

/*
    for (i = 0; i < 5; i++) {
        fts_write_reg(0xD1, 0xFF);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }

    if (i >= 5)
        FTS_ERROR("make IC enter into gesture(suspend) fail,state:%x", state);
    else
        FTS_INFO("Enter into gesture(suspend) successfully");
*/

    if (enable_irq_wake(ts_data->irq)) {
        FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
    }
    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_resume(struct fts_ts_data *ts_data)
{
    int i = 0;
    u8 state = 0xFF;

    //FTS_FUNC_ENTER();
    if (disable_irq_wake(ts_data->irq)) {
        FTS_DEBUG("disable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    for (i = 0; i < 5; i++) {
        fts_write_reg(FTS_REG_GESTURE_EN, DISABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == DISABLE)
            break;
    }

    if (i >= 5)
        FTS_ERROR("make IC exit gesture(resume) fail,state:%x", state);
    else
        FTS_INFO("resume from gesture successfully");

    //FTS_FUNC_EXIT();
    return 0;
}

// Touch_BSP +++
int set_gesture_register(struct fts_ts_data *ts_data)
{
	int i = 0;
	int ret = -1; // register set result
	u8 state = 0xFF;

// D1
	u8 fod_bit		= 0x80;
	u8 music_pause_bit	= 0x40;
	u8 dclick_bit		= 0x10;
	u8 swipeup_bit		= 0x04;
// D2
	u8 e_bit		= 0x08;
	u8 m_bit		= 0x04;
	u8 w_bit		= 0x02;
// D5
	u8 s_bit		= 0x40;
// D6
	u8 v_bit		= 0x10;
	u8 music_forward_bit	= 0x04;
	u8 music_rewind_bit	= 0x02;
// D7
	u8 z_bit		= 0x20;

	FTS_gesture_register_D1 = 0x00;
	FTS_gesture_register_D2 = 0x00;
	FTS_gesture_register_D5 = 0x00;
	FTS_gesture_register_D6 = 0x00;
	FTS_gesture_register_D7 = 0x00;

	if ((ts_data->fp_enable) || (ts_data->aod_enable == ENABLE)) {
		FTS_gesture_register_D1 |= fod_bit;
	}
	if (ts_data->dclick_mode == 1) {
		FTS_gesture_register_D1 |= dclick_bit;
	}
	if (ts_data->swipeup_mode == 1) {
		FTS_gesture_register_D1 |= swipeup_bit;
	}
	if (ts_data->music_control == 1) {
		FTS_gesture_register_D1 |= music_pause_bit;
		FTS_gesture_register_D6 |= music_forward_bit;
		FTS_gesture_register_D6 |= music_rewind_bit;
	}
	if (ts_data->gesture_mode_enable == 1) {
		if (ts_data->gesture_type & 1 << 6)
			FTS_gesture_register_D6 |= v_bit;
		if (ts_data->gesture_type & 1 << 5)
			FTS_gesture_register_D7 |= z_bit;
		if (ts_data->gesture_type & 1 << 4)
			FTS_gesture_register_D2 |= m_bit;
		if (ts_data->gesture_type & 1 << 3)
			FTS_gesture_register_D2 |= e_bit;
		if (ts_data->gesture_type & 1 << 2)
			FTS_gesture_register_D5 |= s_bit;
		if (ts_data->gesture_type & 1 << 1)
			FTS_gesture_register_D2 |= w_bit;
	}

	for (i = 0; i < 5; i++) {
		fts_write_reg(0xD1, FTS_gesture_register_D1);
		fts_write_reg(0xD2, FTS_gesture_register_D2);
		fts_write_reg(0xD5, FTS_gesture_register_D5);
		fts_write_reg(0xD6, FTS_gesture_register_D6);
		fts_write_reg(0xD7, FTS_gesture_register_D7);
		fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
		msleep(1);
		fts_read_reg(FTS_REG_GESTURE_EN, &state);
		if (state == ENABLE) {
			ts_data->gesture_mode = ENABLE;
			break;
		}
	}

	if (i >= 5)
		FTS_ERROR("make IC enter into gesture(suspend) fail, state: %#x", state);
	else {
		FTS_INFO("Enter into gesture(suspend) successfully");
		ret = ENABLE;
	}

	FTS_INFO("reg D1:%#2X D2:%#2X D5:%#2X D6:%#2X D7:%#2X",
		FTS_gesture_register_D1, FTS_gesture_register_D2, FTS_gesture_register_D5,
		FTS_gesture_register_D6, FTS_gesture_register_D7);
	return ret;
}

int is_enter_gesture_mode(struct fts_ts_data *ts_data)
{
	int enable_gesture = 0;

	if (ts_data->fp_enable == 1) {
		enable_gesture = 1;
		if (!ts_data->suspended)
		FTS_INFO("FP auth enable, enter gesture mode");
	}

	if (ts_data->gesture_mode_enable == 1) {
		enable_gesture = 1;
		if (!ts_data->suspended)
		FTS_INFO("Zenmotion enable, enter gesture mode");
	}

	if (ts_data->dclick_mode == 1) {
		enable_gesture = 1;
		if (!ts_data->suspended)
		FTS_INFO("Double click enable, enter gesture mode");
	}

	if (ts_data->swipeup_mode == 1) {
		enable_gesture = 1;
		if (!ts_data->suspended)
		FTS_INFO("Swipe up enable, enter gesture mode");
	}

	if (ts_data->aod_enable == ENABLE) {
		enable_gesture = 1;
		FTS_INFO("AOD triggered by touch , enter gesture mode");
	}

	return enable_gesture;
}

static ssize_t asus_gesture_proc_type_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *buff = NULL;
	int offset = 0;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	offset += sprintf(buff, "%#x \n", fts_data->gesture_type);

	ret = simple_read_from_buffer(buf, count, ppos, buff, offset);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_type_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	int tmp = 0;
	u8 gesture_tmp = 0;
	char gesture_buf[16];
	char gesture_type_buf[16] = {'0'};
	char messages[16];
	memset(messages, 0, sizeof(messages));

	if (len > 16)
		len = 16;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	memset(gesture_buf, 0, sizeof(gesture_buf));
	sprintf(gesture_buf, "%s", messages);
	gesture_buf[len] = '\0';
	FTS_DEBUG("fts_gesture_store %s ! length %d", gesture_buf, len);

	memset(gesture_type_buf, 0, sizeof(gesture_type_buf));
	gesture_type_buf[8] = '\0';
	for (tmp = 0; tmp < len; tmp++) {
		gesture_type_buf[tmp] = gesture_buf[len-tmp-1];
	}

	if (gesture_type_buf[0] == '1') {
		fts_data->gesture_mode_enable = 1;
		FTS_INFO("gesture_mode enable !");
	} else
		fts_data->gesture_mode_enable = 0;

	if (gesture_type_buf[7] == '1') {
		fts_data->music_control = 1;
		FTS_INFO("music_control enable !");
	} else
		fts_data->music_control = 0;

	if (fts_data->gesture_mode_enable == 1) {
		for (tmp = 0; tmp < 8; tmp++) {
			if (gesture_type_buf[tmp] == '1') {
				gesture_tmp |= (1 << tmp);
			}
		}
		fts_data->gesture_type = gesture_tmp;
		FTS_INFO("gesture_mode_enable type = %#x !", fts_data->gesture_type);
	} else {
		fts_data->gesture_mode_enable = 0;
		fts_data->music_control = 0;
		fts_data->gesture_type = 0;
		FTS_INFO("gesture mode is disabled.");
	}

	return len;
}

static ssize_t asus_gesture_proc_dclick_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", fts_data->dclick_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_dclick_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		fts_data->dclick_mode = 0;
		printk("[Focal][Touch] dclick_mode_disable ! \n");
	} else {
		fts_data->dclick_mode = 1;
		printk("[Focal][Touch] dclick_mode_enable ! \n");
	}
	return len;
}

static ssize_t asus_gesture_proc_swipeup_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", fts_data->swipeup_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_swipeup_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		fts_data->swipeup_mode = 0;
		printk("[Focal][Touch] swipeup_mode_disable ! \n");
	} else {
		fts_data->swipeup_mode = 1;
		printk("[Focal][Touch] swipeup_mode_enable ! \n");
	}
	return len;
}

static struct file_operations asus_gesture_proc_type_ops = {
	.write = asus_gesture_proc_type_write,
	.read  = asus_gesture_proc_type_read,
};

static struct file_operations asus_gesture_proc_dclick_ops = {
	.write = asus_gesture_proc_dclick_write,
	.read  = asus_gesture_proc_dclick_read,
};

static struct file_operations asus_gesture_proc_swipeup_ops = {
	.write = asus_gesture_proc_swipeup_write,
	.read  = asus_gesture_proc_swipeup_read,
};
// Touch_BSP ---

int fts_gesture_init(struct fts_ts_data *ts_data)
{
    struct input_dev *input_dev = ts_data->input_dev;

    FTS_FUNC_ENTER();

// FOD
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_F);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);

    __set_bit(KEY_GESTURE_O, input_dev->keybit);
    __set_bit(KEY_GESTURE_F, input_dev->keybit);
    __set_bit(KEY_GESTURE_U, input_dev->keybit);
    __set_bit(KEY_GESTURE_L, input_dev->keybit);
// Zenmotion
    input_set_capability(input_dev, EV_KEY, KEY_POWER);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);

    __set_bit(KEY_GESTURE_UP, input_dev->keybit);
    __set_bit(KEY_GESTURE_E, input_dev->keybit);
    __set_bit(KEY_GESTURE_M, input_dev->keybit);
    __set_bit(KEY_GESTURE_W, input_dev->keybit);
    __set_bit(KEY_GESTURE_S, input_dev->keybit);
    __set_bit(KEY_GESTURE_V, input_dev->keybit);
    __set_bit(KEY_GESTURE_Z, input_dev->keybit);
// Media
    input_set_capability(input_dev, EV_KEY, KEY_PAUSE);
    input_set_capability(input_dev, EV_KEY, KEY_REWIND);
    input_set_capability(input_dev, EV_KEY, KEY_FORWARD);

    fts_create_gesture_sysfs(ts_data->dev);

    memset(&fts_gesture_data, 0, sizeof(struct fts_gesture_st));
    ts_data->gesture_mode = FTS_GESTURE_EN;

    proc_create(GESTURE_TYPE, 0666, NULL, &asus_gesture_proc_type_ops);
    proc_create(DCLICK, 0666, NULL, &asus_gesture_proc_dclick_ops);
    proc_create(SWIPEUP, 0666, NULL, &asus_gesture_proc_swipeup_ops);

    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_exit(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);
    FTS_FUNC_EXIT();
    return 0;
}
