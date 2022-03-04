/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
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

/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define GESTURE_UP 0x22
#define GESTURE_DOUBLECLICK 0x24
#define GESTURE_W 0x31
#define GESTURE_M 0x32
#define GESTURE_E 0x33
#define GESTURE_S 0x46
#define GESTURE_V 0x54

#if defined ASUS_SAKE_PROJECT
#define KEY_LAST_USED BTN_TRIGGER_HAPPY40
#define KEY_GESTURE_UP (KEY_LAST_USED + 1)
#define KEY_GESTURE_E (KEY_LAST_USED + 2)
#define KEY_GESTURE_M (KEY_LAST_USED + 3)
#define KEY_GESTURE_W (KEY_LAST_USED + 4)
#define KEY_GESTURE_S (KEY_LAST_USED + 5)
#define KEY_GESTURE_V (KEY_LAST_USED + 6)
#define KEY_GESTURE_Z (KEY_LAST_USED + 7)

#define KEY_GESTURE_PAUSE (KEY_LAST_USED + 8)
#define KEY_GESTURE_REWIND (KEY_LAST_USED + 9)
#define KEY_GESTURE_FORWARD (KEY_LAST_USED + 10)

#define GESTURE_MUSIC_PAUSE 0x26
#define GESTURE_MUSIC_REWIND 0x51
#define GESTURE_MUSIC_FORWARD 0x52
#define GESTURE_Z 0x65

#define GESTURE_FOD_PRESS 0x28
#define GESTURE_FOD_PARTIAL_PRESS 0x2A
#define GESTURE_FOD_UNPRESS 0x29
#else
#define KEY_GESTURE_UP KEY_UP
#define KEY_GESTURE_E KEY_E
#define KEY_GESTURE_M KEY_M
#define KEY_GESTURE_W KEY_W
#define KEY_GESTURE_S KEY_S
#define KEY_GESTURE_V KEY_V
#define KEY_GESTURE_Z KEY_Z

#define KEY_GESTURE_LEFT KEY_LEFT
#define KEY_GESTURE_RIGHT KEY_RIGHT
#define KEY_GESTURE_DOWN KEY_DOWN
#define KEY_GESTURE_L KEY_L
#define KEY_GESTURE_O KEY_O
#define KEY_GESTURE_C KEY_C

#define GESTURE_LEFT 0x20
#define GESTURE_RIGHT 0x21
#define GESTURE_DOWN 0x23
#define GESTURE_L 0x44
#define GESTURE_O 0x30
#define GESTURE_Z 0x41
#define GESTURE_C 0x34
#endif

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
#if !defined ASUS_SAKE_PROJECT
	u16 coordinate_x[FTS_GESTURE_POINTS_MAX];
	u16 coordinate_y[FTS_GESTURE_POINTS_MAX];
#endif
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
#if defined ASUS_SAKE_PROJECT
struct gesture_type_reg {
	unsigned int index;
	unsigned int nr;
};

static const char *gestures_names[] = {
	[GESTURE_TYPE_UP] = "up",
	[GESTURE_TYPE_DOUBLECLICK] = "double_click",
	[GESTURE_TYPE_PAUSE] = "pause",
	[GESTURE_TYPE_FOD] = "fod",
	[GESTURE_TYPE_W] = "w",
	[GESTURE_TYPE_M] = "m",
	[GESTURE_TYPE_E] = "e",
	[GESTURE_TYPE_S] = "s",
	[GESTURE_TYPE_REWIND] = "rewind",
	[GESTURE_TYPE_FORWARD] = "forward",
	[GESTURE_TYPE_V] = "v",
	[GESTURE_TYPE_Z] = "z",
};

static const struct gesture_type_reg gesture_types_reg[] = {
	[GESTURE_TYPE_UP] = { 0, 2 },
	[GESTURE_TYPE_DOUBLECLICK] = { 0, 4 },
	[GESTURE_TYPE_PAUSE] = { 0, 6 },
	[GESTURE_TYPE_FOD] = { 0, 7 },

	[GESTURE_TYPE_W] = { 1, 1 },
	[GESTURE_TYPE_M] = { 1, 2 },
	[GESTURE_TYPE_E] = { 1, 3 },

	[GESTURE_TYPE_S] = { 2, 6 },

	[GESTURE_TYPE_REWIND] = { 3, 1 },
	[GESTURE_TYPE_FORWARD] = { 3, 2 },
	[GESTURE_TYPE_V] = { 3, 4 },

	[GESTURE_TYPE_Z] = { 4, 5 },
};

static const u8 gesture_regs[] = { 0xD1, 0xD2, 0xD5, 0xD6, 0xD7 };

static void fts_gesture_apply(struct fts_ts_data *ts_data)
{
	unsigned int i;

	for (i = 0; i < sizeof(ts_data->gesture_data); i++)
		fts_write_reg(gesture_regs[i], ts_data->gesture_data[i]);
}

static void fts_gesture_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data =
		container_of(work, struct fts_ts_data, gesture_work);
	bool suspended = ts_data->suspended;
	bool gesture_mode = false;
	unsigned int i;

	for (i = 0; i < GESTURE_TYPE_MAX; i++) {
		const struct gesture_type_reg *reg = &gesture_types_reg[i];
		u8 mask = BIT(reg->nr);

		if (ts_data->enabled_gestures[i]) {
			ts_data->gesture_data[reg->index] |= mask;
			gesture_mode = true;
		} else {
			ts_data->gesture_data[reg->index] &= ~mask;
		}
	}

	if (suspended)
		fts_ts_resume(ts_data->dev);
	ts_data->gesture_mode = gesture_mode;
	if (suspended)
		fts_ts_suspend(ts_data->dev);
}

void fts_gesture_set(struct fts_ts_data *ts_data, enum gesture_type type,
		     bool enabled)
{
	if (ts_data->enabled_gestures[type] == enabled)
		return;

	ts_data->enabled_gestures[type] = enabled;

	queue_work(ts_data->ts_workqueue, &ts_data->gesture_work);
}

static ssize_t fts_gestures_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data = fts_data;
	unsigned int i;
	int count = 0;

	for (i = 0; i < GESTURE_TYPE_MAX; i++)
		count += snprintf(buf + count, PAGE_SIZE, "%s=%u\n",
				  gestures_names[i],
				  ts_data->enabled_gestures[i]);

	return count;
}

static ssize_t fts_gestures_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	const char *start = buf;
	const char *end;
	const char *eq;
	unsigned int i;

	eq = strnchr(start, buf + count - start, '=');
	if (!eq) {
		FTS_ERROR("invalid format, failed to find =");
		return -EINVAL;
	}

	end = strnchr(eq + 1, buf + count - eq - 1, '\n');
	if (!end) {
		FTS_ERROR("invalid format, failed to find newline");
		return -EINVAL;
	}

	if (eq + 1 == end) {
		FTS_ERROR("invalid format, nothing after =");
		return -EINVAL;
	}

	for (i = 0; i < GESTURE_TYPE_MAX; i++) {
		const char *gesture_name = gestures_names[i];

		if (!gesture_name) {
			FTS_ERROR("missing name for gesture %u", i);
			continue;
		}

		if (!strncmp(start, gesture_name, eq - start))
			break;
	}

	if (i == GESTURE_TYPE_MAX) {
		FTS_ERROR("unknown gesture name %s", start);
		return -EINVAL;
	}

	fts_gesture_set(fts_data, i, eq[1] != '0');

	return count;
}

static ssize_t fts_fod_pressed_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fts_data->fod_pressed);
}

static DEVICE_ATTR(fts_gestures, S_IRUGO | S_IWUSR, fts_gestures_show,
		   fts_gestures_store);
static DEVICE_ATTR(fts_fod_pressed, S_IRUGO, fts_fod_pressed_show, NULL);
#else
static ssize_t fts_gesture_show(struct device *dev,
				struct device_attribute *attr, char *buf)
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

static ssize_t fts_gesture_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
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

static ssize_t fts_gesture_buf_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i = 0;
	struct input_dev *input_dev = fts_data->input_dev;
	struct fts_gesture_st *gesture = &fts_gesture_data;

	mutex_lock(&input_dev->mutex);
	count = snprintf(buf, PAGE_SIZE, "Gesture ID:%d\n",
			 gesture->gesture_id);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum:%d\n",
			  gesture->point_num);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture Points Buffer:\n");

	/* save point data,max:6 */
	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i,
				  gesture->coordinate_x[i],
				  gesture->coordinate_y[i]);
		if ((i + 1) % 4 == 0)
			count += snprintf(buf + count, PAGE_SIZE, "\n");
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
	mutex_unlock(&input_dev->mutex);

	return count;
}

static ssize_t fts_gesture_buf_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	return -EPERM;
}

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
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR, fts_gesture_buf_show,
		   fts_gesture_buf_store);
#endif

static struct attribute *fts_gesture_mode_attrs[] = {
#if defined ASUS_SAKE_PROJECT
	&dev_attr_fts_gestures.attr,
	&dev_attr_fts_fod_pressed.attr,
#else
	&dev_attr_fts_gesture_mode.attr,
	&dev_attr_fts_gesture_buf.attr,
#endif
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

static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
	int gesture;

	FTS_DEBUG("gesture_id:0x%x", gesture_id);
	switch (gesture_id) {
#if !defined ASUS_SAKE_PROJECT
	case GESTURE_LEFT:
		gesture = KEY_GESTURE_LEFT;
		break;
	case GESTURE_RIGHT:
		gesture = KEY_GESTURE_RIGHT;
		break;
#endif
	case GESTURE_UP:
		gesture = KEY_GESTURE_UP;
		break;
#if !defined ASUS_SAKE_PROJECT
	case GESTURE_DOWN:
		gesture = KEY_GESTURE_DOWN;
		break;
#endif
	case GESTURE_DOUBLECLICK:
		gesture = KEY_WAKEUP;
		break;
#if !defined ASUS_SAKE_PROJECT
	case GESTURE_O:
		gesture = KEY_GESTURE_O;
		break;
#endif
	case GESTURE_W:
		gesture = KEY_GESTURE_W;
		break;
	case GESTURE_M:
		gesture = KEY_GESTURE_M;
		break;
	case GESTURE_E:
		gesture = KEY_GESTURE_E;
		break;
#if !defined ASUS_SAKE_PROJECT
	case GESTURE_L:
		gesture = KEY_GESTURE_L;
		break;
#endif
	case GESTURE_S:
		gesture = KEY_GESTURE_S;
		break;
	case GESTURE_V:
		gesture = KEY_GESTURE_V;
		break;
	case GESTURE_Z:
		gesture = KEY_GESTURE_Z;
		break;
#if !defined ASUS_SAKE_PROJECT
	case GESTURE_C:
		gesture = KEY_GESTURE_C;
		break;
#endif
#if defined ASUS_SAKE_PROJECT
	case GESTURE_MUSIC_PAUSE:
		gesture = KEY_GESTURE_PAUSE;
		break;
	case GESTURE_MUSIC_REWIND:
		gesture = KEY_GESTURE_REWIND;
		break;
	case GESTURE_MUSIC_FORWARD:
		gesture = KEY_GESTURE_FORWARD;
		break;
#endif
	default:
		gesture = -1;
		break;
	}
	/* report event key */
	if (gesture != -1) {
		FTS_DEBUG("Gesture Code=%d", gesture);
		input_report_key(input_dev, gesture, 1);
		input_sync(input_dev);
		input_report_key(input_dev, gesture, 0);
		input_sync(input_dev);
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
#if !defined ASUS_SAKE_PROJECT
	int i = 0;
	int index = 0;
#endif
	u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
	struct input_dev *input_dev = ts_data->input_dev;
	struct fts_gesture_st *gesture = &fts_gesture_data;

	if (!ts_data->suspended || !ts_data->gesture_mode) {
		return 1;
	}

#if defined ASUS_SAKE_PROJECT
	ret = fts_read_reg(FTS_REG_GESTURE_EN, &buf[0]);
	if ((ret < 0) || (buf[0] != ENABLE)) {
		FTS_DEBUG("gesture not enable in fw, don't process gesture %d",
			  buf[0]);
		return 1;
	}

	buf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
	ret = fts_read(&buf[2], 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
	if (ret < 0) {
		FTS_ERROR("read gesture header data fail");
		return ret;
	}
#else
	if (!data) {
		FTS_ERROR("gesture data buffer is null");
		ret = -EINVAL;
		return ret;
	}

	memcpy(buf, data, FTS_GESTURE_DATA_LEN);
	if (buf[0] != ENABLE) {
		FTS_DEBUG("gesture not enable in fw, don't process gesture");
		return 1;
	}
#endif

#if !defined ASUS_SAKE_PROJECT
	/* init variable before read gesture point */
	memset(gesture->coordinate_x, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
	memset(gesture->coordinate_y, 0, FTS_GESTURE_POINTS_MAX * sizeof(u16));
#endif
	gesture->gesture_id = buf[2];
	gesture->point_num = buf[3];
	FTS_DEBUG("gesture_id=%d, point_num=%d", gesture->gesture_id,
		  gesture->point_num);

#if !defined ASUS_SAKE_PROJECT
	/* save point data,max:6 */
	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		index = 4 * i + 4;
		gesture->coordinate_x[i] =
			(u16)(((buf[0 + index] & 0x0F) << 8) + buf[1 + index]);
		gesture->coordinate_y[i] =
			(u16)(((buf[2 + index] & 0x0F) << 8) + buf[3 + index]);
	}
#endif

#if defined ASUS_SAKE_PROJECT
	switch (gesture->gesture_id) {
	case GESTURE_FOD_PRESS:
	case GESTURE_FOD_PARTIAL_PRESS:
		if (gesture->point_num >= 1) {
			ts_data->fp_x = ((buf[4] & 0x0F) << 8) + buf[5];
			ts_data->fp_y = ((buf[6] & 0x0F) << 8) + buf[7];
		}

		ts_data->fod_pressed = true;

		sysfs_notify(&ts_data->dev->kobj, NULL, "fts_fod_pressed");
		return 0;
	case GESTURE_FOD_UNPRESS:
		ts_data->fod_pressed = false;
		return 0;
	}
#endif

	/* report gesture to OS */
	fts_gesture_report(input_dev, gesture->gesture_id);
	return 0;
}

void fts_gesture_recovery(struct fts_ts_data *ts_data)
{
	if (ts_data->gesture_mode && ts_data->suspended) {
		FTS_DEBUG("gesture recovery...");
#if defined ASUS_SAKE_PROJECT
		fts_gesture_apply(ts_data);
#else
		fts_write_reg(0xD1, 0xFF);
		fts_write_reg(0xD2, 0xFF);
		fts_write_reg(0xD5, 0xFF);
		fts_write_reg(0xD6, 0xFF);
		fts_write_reg(0xD7, 0xFF);
		fts_write_reg(0xD8, 0xFF);
		fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
#endif
	}
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
	int i = 0;
	u8 state = 0xFF;

	FTS_FUNC_ENTER();
	if (enable_irq_wake(ts_data->irq)) {
		FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
	}

	for (i = 0; i < 5; i++) {
#if defined ASUS_SAKE_PROJECT
		fts_gesture_apply(ts_data);
#else
		fts_write_reg(0xD1, 0xFF);
		fts_write_reg(0xD2, 0xFF);
		fts_write_reg(0xD5, 0xFF);
		fts_write_reg(0xD6, 0xFF);
		fts_write_reg(0xD7, 0xFF);
		fts_write_reg(0xD8, 0xFF);
#endif

		fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
		msleep(1);
		fts_read_reg(FTS_REG_GESTURE_EN, &state);
		if (state == ENABLE)
			break;
	}

	if (i >= 5)
		FTS_ERROR("make IC enter into gesture(suspend) fail,state:%x",
			  state);
	else
		FTS_INFO("Enter into gesture(suspend) successfully");

	FTS_FUNC_EXIT();
	return 0;
}

int fts_gesture_resume(struct fts_ts_data *ts_data)
{
	int i = 0;
	u8 state = 0xFF;

	FTS_FUNC_ENTER();
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

	FTS_FUNC_EXIT();
	return 0;
}

int fts_gesture_init(struct fts_ts_data *ts_data)
{
	struct input_dev *input_dev = ts_data->input_dev;

	FTS_FUNC_ENTER();
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
#if !defined ASUS_SAKE_PROJECT
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
#endif
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
#if !defined ASUS_SAKE_PROJECT
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
#endif
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
#if !defined ASUS_SAKE_PROJECT
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);
#endif
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_PAUSE);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_REWIND);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_FORWARD);

#if defined ASUS_SAKE_PROJECT
	INIT_WORK(&ts_data->gesture_work, fts_gesture_work);
#endif

	fts_create_gesture_sysfs(ts_data->dev);

	memset(&fts_gesture_data, 0, sizeof(struct fts_gesture_st));
	ts_data->gesture_mode = FTS_GESTURE_EN;

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
