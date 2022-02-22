/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
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

/************************************************************************
*
* File Name: focaltech_i2c.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-04
*
* Abstract: i2c communication with TP
*
* Version: v1.0
*
* Revision History:
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define I2C_RETRY_NUMBER 3
#define I2C_BUF_LENGTH 256

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/*****************************************************************************
* functions body
*****************************************************************************/
int fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	struct fts_ts_data *ts_data = fts_data;
	struct i2c_msg msg_list[2] = {
		{
			.addr = ts_data->client->addr,
			.len = cmdlen,
			.buf = ts_data->bus_tx_buf,
		},
		{
			.addr = ts_data->client->addr,
			.flags = I2C_M_RD,
			.len = datalen,
			.buf = ts_data->bus_rx_buf,
		},
	};
	struct i2c_msg *msg = NULL;
	int msg_num = 0;
	int ret = 0;
	int i = 0;

	/* must have data when read */
	if (!data || !datalen || datalen > I2C_BUF_LENGTH ||
	    cmdlen > I2C_BUF_LENGTH) {
		FTS_ERROR(
			"fts_data/client/cmdlen(%d)/data/datalen(%d) is invalid",
			cmdlen, datalen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	if (cmd && cmdlen) {
		memcpy(ts_data->bus_tx_buf, cmd, cmdlen);
		msg = &msg_list[0];
		msg_num = 2;
	} else {
		msg = &msg_list[1];
		msg_num = 1;
	}

	for (i = 0; i < I2C_RETRY_NUMBER; i++) {
		ret = i2c_transfer(ts_data->client->adapter, msg, msg_num);
		if (ret < 0) {
			FTS_ERROR("i2c_transfer(read) fail,ret:%d", ret);
		} else {
			memcpy(data, ts_data->bus_rx_buf, datalen);
			break;
		}
	}
	mutex_unlock(&ts_data->bus_lock);

	return ret;
}

int fts_write(u8 *writebuf, u32 writelen)
{
	struct fts_ts_data *ts_data = fts_data;
	struct i2c_msg msgs = {
		.addr = ts_data->client->addr,
		.len = writelen,
		.buf = ts_data->bus_tx_buf,
	};
	int ret = 0;
	int i = 0;

	if (!writebuf || !writelen || writelen > I2C_BUF_LENGTH) {
		FTS_ERROR("fts_data/client/data/datalen(%d) is invalid",
			  writelen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	memcpy(ts_data->bus_tx_buf, writebuf, writelen);
	for (i = 0; i < I2C_RETRY_NUMBER; i++) {
		ret = i2c_transfer(ts_data->client->adapter, &msgs, 1);
		if (ret < 0) {
			FTS_ERROR("i2c_transfer(write) fail,ret:%d", ret);
		} else {
			break;
		}
	}
	mutex_unlock(&ts_data->bus_lock);

	return ret;
}

int fts_read_reg(u8 addr, u8 *value)
{
	return fts_read(&addr, 1, value, 1);
}

int fts_write_reg(u8 addr, u8 value)
{
	u8 buf[2] = { addr, value };

	return fts_write(buf, sizeof(buf));
}

int fts_bus_init(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	ts_data->bus_tx_buf = kzalloc(I2C_BUF_LENGTH, GFP_KERNEL);
	if (NULL == ts_data->bus_tx_buf) {
		FTS_ERROR("failed to allocate memory for bus_tx_buf");
		return -ENOMEM;
	}

	ts_data->bus_rx_buf = kzalloc(I2C_BUF_LENGTH, GFP_KERNEL);
	if (NULL == ts_data->bus_rx_buf) {
		FTS_ERROR("failed to allocate memory for bus_rx_buf");
		return -ENOMEM;
	}
	FTS_FUNC_EXIT();
	return 0;
}

int fts_bus_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	if (ts_data && ts_data->bus_tx_buf) {
		kfree(ts_data->bus_tx_buf);
		ts_data->bus_tx_buf = NULL;
	}

	if (ts_data && ts_data->bus_rx_buf) {
		kfree(ts_data->bus_rx_buf);
		ts_data->bus_rx_buf = NULL;
	}
	FTS_FUNC_EXIT();
	return 0;
}
