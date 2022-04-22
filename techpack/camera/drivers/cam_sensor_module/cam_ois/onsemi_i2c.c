#include "onsemi_i2c.h"
#include "utils.h"
#include "onsemi_interface.h"

#undef  pr_fmt
#define pr_fmt(fmt) "OIS-I2C %s(): " fmt, __func__
extern struct mutex g_dualoisMutex;
int onsemi_read_byte(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint32_t* reg_data)
{
	int rc = 0;
	int i;

	for(i=0;i<3;i++)
	{
		mutex_lock(&g_dualoisMutex);
		rc = camera_io_dev_read(&(ctrl->io_master_info),
									reg_addr,
									reg_data,
									CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
									CAMERA_SENSOR_I2C_TYPE_BYTE);//data_type
		mutex_unlock(&g_dualoisMutex);
		if(rc == 0 || rc == -110)
		{
			break;
		}
		else
		{
			pr_err("read reg 0x%04x failed, rc = %d, retry_count = %d\n",reg_addr,rc,i);
			delay_ms(2);
		}
	}

	if( i != 0 )
	{
		if(rc == 0)
		{
			pr_info("read reg 0x%04x OK after retried %d times\n",reg_addr,i);
		}
		else
		{
			pr_info("read reg 0x%04x FAILED after retried %d times\n",reg_addr,i);
		}
	}
	if(rc == -110)
	{
		pr_info("read reg 0x%04x FAILED due to cci timeout!\n",reg_addr);
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}

int onsemi_read_word(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint32_t* reg_data)
{
	int rc = 0;
	int i;

	for(i=0;i<3;i++)
	{
		mutex_lock(&g_dualoisMutex);
		rc = camera_io_dev_read(&(ctrl->io_master_info),
									reg_addr,
									reg_data,
									CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
									CAMERA_SENSOR_I2C_TYPE_WORD);//data_type
		mutex_unlock(&g_dualoisMutex);
		if(rc == 0 || rc == -110)
		{
			break;
		}
		else
		{
			pr_err("read reg 0x%04x failed, rc = %d, retry_count = %d\n",reg_addr,rc,i);
			delay_ms(2);
		}
	}

	if( i != 0 )
	{
		if(rc == 0)
		{
			pr_info("read reg 0x%04x OK after retried %d times\n",reg_addr,i);
		}
		else
		{
			pr_info("read reg 0x%04x FAILED after retried %d times\n",reg_addr,i);
		}
	}
	if(rc == -110)
	{
		pr_info("read reg 0x%04x FAILED due to cci timeout!\n",reg_addr);
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}
int onsemi_read_dword(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint32_t* reg_data)
{
	int rc = 0;
	int i;

	for(i=0;i<3;i++)
	{
		mutex_lock(&g_dualoisMutex);
		rc = camera_io_dev_read(&(ctrl->io_master_info),
									reg_addr,
									reg_data,
									CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
									CAMERA_SENSOR_I2C_TYPE_DWORD);//data_type
		mutex_unlock(&g_dualoisMutex);
		if(rc == 0 || rc == -110)
		{
			break;
		}
		else
		{
			pr_err("read reg 0x%04x failed, rc = %d, retry_count = %d\n",reg_addr,rc,i);
			delay_ms(2);
		}
	}

	if( i != 0 )
	{
		if(rc == 0)
		{
			pr_info("read reg 0x%04x OK after retried %d times\n",reg_addr,i);
		}
		else
		{
			pr_info("read reg 0x%04x FAILED after retried %d times\n",reg_addr,i);
		}
	}
	if(rc == -110)
	{
		pr_info("read reg 0x%04x FAILED due to cci timeout!\n",reg_addr);
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}

int onsemi_read_seq_bytes(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint8_t* reg_data, uint32_t size)
{
	int rc = 0;

	if(size > I2C_REG_DATA_MAX)
	{
		pr_err("read size %d too large, max seq size is %d\n",size,I2C_REG_DATA_MAX);
		return -1;
	}
	mutex_lock(&g_dualoisMutex);
	rc = camera_io_dev_read_seq(&(ctrl->io_master_info),
									reg_addr,
									reg_data,
									CAMERA_SENSOR_I2C_TYPE_WORD,
									CAMERA_SENSOR_I2C_TYPE_BYTE,
									size);
	mutex_unlock(&g_dualoisMutex);
	if(rc < 0)
	{
		pr_err("seq read %d bytes from reg 0x%04x failed, rc = %d\n",size,reg_addr,rc);
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}

int onsemi_poll_byte(struct cam_ois_ctrl_t * ctrl, uint32_t reg_addr, uint16_t reg_data, uint32_t delay_ms)
{
	int rc = 0;
	uint32_t varify_val;
	mutex_lock(&g_dualoisMutex);
	rc = camera_io_dev_poll(&(ctrl->io_master_info),
							reg_addr,
							reg_data,
							0,//mask, just lowest byte used
							CAMERA_SENSOR_I2C_TYPE_BYTE,//data type
							CAMERA_SENSOR_I2C_TYPE_WORD,//addr type
							delay_ms);
	mutex_unlock(&g_dualoisMutex);
	if(rc != 0)
	{
		pr_err("poll 0x%x from reg 0x%04x delay %d ms failed, rc = %d\n",reg_data,reg_addr,delay_ms,rc);
		rc = onsemi_read_byte(ctrl,reg_addr,&varify_val);
		if(rc < 0)
		{
			//
		}
		else
		{
			pr_info("reg 0x%04x value now is 0x%x actually\n",reg_addr,varify_val);
		}
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}

int onsemi_poll_word(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint16_t reg_data, uint32_t delay_ms)
{
	int rc = 0;
	uint32_t varify_val;
	mutex_lock(&g_dualoisMutex);
	rc = camera_io_dev_poll(&(ctrl->io_master_info),
							reg_addr,
							reg_data,
							0,//mask, just lowest byte used
							CAMERA_SENSOR_I2C_TYPE_WORD,//data type
							CAMERA_SENSOR_I2C_TYPE_WORD,//addr type
							delay_ms);
	mutex_unlock(&g_dualoisMutex);
	if(rc != 0)
	{
		pr_err("poll 0x%x from reg 0x%04x delay %d ms failed, rc = %d\n",reg_data,reg_addr,delay_ms,rc);
		rc = onsemi_read_word(ctrl,reg_addr,&varify_val);
		if(rc < 0)
		{
			//
		}
		else
		{
			pr_info("reg 0x%04x value now is 0x%x actually\n",reg_addr,varify_val);
		}
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}

int onsemi_write_byte(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint32_t reg_data)
{
	int rc = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data;

	data.reg_addr = reg_addr;
	data.reg_data = reg_data;
	data.delay = 0;
	data.data_mask = 0;//how to use? not used so far

	write_setting.reg_setting = &data;
	write_setting.size = 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	mutex_lock(&g_dualoisMutex);
	rc = camera_io_dev_write(&(ctrl->io_master_info),&write_setting);
	mutex_unlock(&g_dualoisMutex);
	if(rc < 0)
	{
		pr_err("write 0x%x to reg 0x%x failed! rc = %d",
						data.reg_data,data.reg_addr,rc);
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}
int onsemi_write_word(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint32_t reg_data)
{
	int rc = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data;

	data.reg_addr = reg_addr;
	data.reg_data = reg_data;
	data.delay = 0;
	data.data_mask = 0;//how to use?

	write_setting.reg_setting = &data;
	write_setting.size = 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.delay = 0;
	mutex_lock(&g_dualoisMutex);
	rc = camera_io_dev_write(&(ctrl->io_master_info),&write_setting);
	mutex_unlock(&g_dualoisMutex);
	if(rc < 0)
	{
		pr_err("write 0x%x to reg 0x%x failed! rc = %d",
						data.reg_data,data.reg_addr,rc);
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}

int onsemi_write_dword(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint32_t reg_data)
{
	int rc = 0;

#if 1
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data;

	data.reg_addr = reg_addr;
	data.reg_data = reg_data;
	data.delay = 0;
	data.data_mask = 0;//how to use?

	write_setting.reg_setting = &data;
	write_setting.size = 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	write_setting.delay = 0;
	mutex_lock(&g_dualoisMutex);
	rc = camera_io_dev_write(&(ctrl->io_master_info),&write_setting);
	mutex_unlock(&g_dualoisMutex);
	if(rc < 0)
	{
		pr_err("write 0x%x to reg 0x%x failed! rc = %d",
						data.reg_data,data.reg_addr,rc);
	}
#else
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data[4];
	int i;

	for(i=0;i<4;i++)
	{
		data[i].reg_addr = reg_addr;
		data[i].delay = 0;
		data[i].data_mask = 0;//how to use?
	}
	//MSB in lower address
	data[0].reg_data = (uint8_t)((reg_data & 0xFF000000) >> 24);
	data[1].reg_data = (uint8_t)((reg_data & 0x00FF0000) >> 16);
	data[2].reg_data = (uint8_t)((reg_data & 0x0000FF00) >> 8);
	data[3].reg_data = (uint8_t)(reg_data & 0x000000FF);

	write_setting.reg_setting = data;
	write_setting.size = 4;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;

	rc = camera_io_dev_write_continuous(&(ctrl->io_master_info),
										&write_setting,
										1);//1 burst, 0 sequence
	if(rc < 0)
	{
		pr_err("write 0x%02x 0x%02x 0x%02x 0x%02x to reg 0x%x failed! rc = %d",
						data[0].reg_data,data[1].reg_data,data[2].reg_data,data[3].reg_data,data[0].reg_addr,rc);
	}
#endif
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	return rc;
}
int onsemi_write_seq_bytes(struct cam_ois_ctrl_t * ctrl,uint32_t reg_addr, uint8_t* reg_data,uint32_t size)
{
	int rc = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array* data = NULL;
	int i;

	data = kzalloc(sizeof(struct cam_sensor_i2c_reg_array)*size,GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for(i=0;i<size;i++)
	{
		data[i].reg_addr = reg_addr;
		data[i].reg_data = reg_data[i];
		data[i].delay = 0;
		data[i].data_mask = 0;
	}

	write_setting.reg_setting = data;
	write_setting.size = size;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	mutex_lock(&g_dualoisMutex);
	rc = camera_io_dev_write_continuous(&(ctrl->io_master_info),
										&write_setting,
										1);//burst
	mutex_unlock(&g_dualoisMutex);
	if(rc < 0)
	{
		pr_err("seq write %d seq bytes to reg 0x%04x failed, rc = %d\n",size,reg_addr,rc);
	}
	ctrl->cci_status = rc; //ASUS_BSP Byron for record cci_status
	kfree(data);

	return rc;
}
