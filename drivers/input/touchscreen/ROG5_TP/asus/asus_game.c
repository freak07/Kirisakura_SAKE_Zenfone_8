#include "focaltech_core.h"
#include "asus_tp.h"

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
bool game_mode_active = false;
int touch_figer_slot[TOTAL_GAME_USED_SLOT] = {0};
u8 slidling_sen = 0x0,sliding_acy = 0x0, touch_acy = 0x0;
u8 Rcoefleft = 0x0A, RcoefRight=0x0A;
int pre_angle = 0;
/*****************************************************************************
* 1.Static function prototypes
*******************************************************************************/
static u16 shex_to_u16(const char *hex_buf, int size)
{
    int i;
    int base = 1;
    int value = 0;
    char single;

    for (i = size - 1; i >= 0; i--) {
        single = hex_buf[i];

        if ((single >= '0') && (single <= '9')) {
            value += (single - '0') * base;
        } else if ((single >= 'a') && (single <= 'z')) {
            value += (single - 'a' + 10) * base;
        } else if ((single >= 'A') && (single <= 'Z')) {
            value += (single - 'A' + 10) * base;
        } else {
            return -EINVAL;
        }
		base *= 10;
    }

    return value;
}

void reconfig_game_reg(bool reconfig) {
    int ret1 = 0 , ret2 = 0, ret3 = 0, i = 0;
    u8 value = 0x0;
    
    FTS_INFO("Reconfig game reg");
    set_rotation_mode();
//    rise_report_rate (true);
    mutex_lock(&fts_data->reg_lock);
    for (i = 0; i < 6; i++) {
	ret1 = fts_write_reg(FTS_REG_SLIDING_SENSITIVITY, slidling_sen);
	msleep(20);
	fts_read_reg(FTS_REG_SLIDING_SENSITIVITY, &value);
	if (value == slidling_sen){
//	    FTS_DEBUG("set FTS_REG_SLIDING_SENSITIVITY 0x%x success",slidling_sen);
	    break;
	} else {
	    FTS_DEBUG("set FTS_REG_SLIDING_SENSITIVITY 0x%x failed,retry count %d",value, i);
	    msleep(20);
	}
    }
	    
    for (i = 0; i < 6; i++) {
	ret2 = fts_write_reg(FTS_REG_SLIDING_PRECISION, sliding_acy);
	msleep(20);
	fts_read_reg(FTS_REG_SLIDING_PRECISION, &value);
	if (value == sliding_acy){
//	    FTS_DEBUG("set FTS_REG_SLIDING_PRECISION 0x%x success",sliding_acy);
	    break;
	} else {
	    FTS_DEBUG("set FTS_REG_SLIDING_PRECISION 0x%x failed,retry count %d",value, i);
	    msleep(20);
	}
    }

    for (i = 0; i < 6; i++) {
	ret3 = fts_write_reg(FTS_REG_TOUCH_SENSITIVITY, touch_acy);
	msleep(20);
	fts_read_reg(FTS_REG_TOUCH_SENSITIVITY, &value);
	if (value == touch_acy){
//	    FTS_DEBUG("set FTS_REG_TOUCH_SENSITIVITY 0x%x success",touch_acy);
	    break;
	} else {
	    FTS_DEBUG("set FTS_REG_TOUCH_SENSITIVITY 0x%x failed,retry count %d",value, i);
	    msleep(20);
	}
    }
    mutex_unlock(&fts_data->reg_lock);

}

int set_rotation_mode()
{
    int ret1 = 0, ret2 = 0, ret3 = 0;
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;
    u8 l_val = 0 , r_val = 0;
    u8 mode = 0;
    int i =0;
    if (ts_data->game_mode == ENABLE) {
        FTS_INFO("Game mode enable with rotation angle %d",ts_data->rotation_angle);
	switch (ts_data->rotation_angle) {
	    case ANGLE_90:
	        mutex_lock(&fts_data->reg_lock);
		for (i = 0; i < 6; i++) {
		    ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x01);
		    msleep(20);
		    ret2 = fts_write_reg(FTS_REG_EDGEPALM_LEFT, Rcoefleft);
		    msleep(20);		
		    ret3 = fts_write_reg(FTS_REG_EDGEPALM_RIGHT, RcoefRight);
		    msleep(20);

		    fts_read_reg(FTS_REG_EDGEPALM_MODE_EN, &mode);
		    if (mode!=0x01) {
		      FTS_INFO("Game mode enable,set rotation reg to 90 fail , retry %d",i);
		      msleep(20);
		      ret = -1;
		    } else {
		      game_mode_active = true;
		      FTS_INFO("Game mode enable,set rotation reg to 90 success");
		      break;
		    }
		}
		mutex_unlock(&fts_data->reg_lock);
	      break;
	    case ANGLE_270:
	        mutex_lock(&fts_data->reg_lock);
		for (i = 0; i < 6; i++) {
		    ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x02);
		    msleep(20);
		    ret2 = fts_write_reg(FTS_REG_EDGEPALM_LEFT, Rcoefleft);
		    msleep(20);
		    ret3 = fts_write_reg(FTS_REG_EDGEPALM_RIGHT, RcoefRight);
		    msleep(20);
		    fts_read_reg(FTS_REG_EDGEPALM_MODE_EN, &mode);		
		    if (mode!=0x02) {
			FTS_INFO("Game mode enable,set rotation reg to 270 fail ");
			ret = -1;
			msleep(50);
		    } else {
			game_mode_active =true;
			FTS_INFO("Game mode enable,set rotation reg to 270 success");
			break;
		    }
		}
		mutex_unlock(&fts_data->reg_lock);
	      break;
	    case ANGLE_0:
	      if (game_mode_active) {
		  ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x00);
		  ret2 = fts_write_reg(FTS_REG_EDGEPALM_LEFT, 0);
		  ret3 = fts_write_reg(FTS_REG_EDGEPALM_RIGHT, 0);
		  if (ret1 < 0 || ret2 < 0) {
		      FTS_INFO("Game mode enable,set rotation reg to 0 fail ");
		      ret = -1;
		  } else 
		      game_mode_active =false;
	      }
	}
	fts_read_reg(FTS_REG_EDGEPALM_LEFT, &l_val);
	fts_read_reg(FTS_REG_EDGEPALM_RIGHT, &r_val);
	FTS_INFO("Game mode enable,set rotation reg to %d , palm range left %x right %x",ts_data->rotation_angle,l_val,r_val);	
    } else {
	    ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x00);
	    if (ret1 < 0 || ret2 < 0) {
		FTS_INFO("Game mode Exit ,set rotation reg to 0 fail");
		ret = -1;
	    } else {
		FTS_INFO("Game mode Exit ,set rotation reg to 0");
		game_mode_active =false;
	    }
    }
    
    return ret;
}

void rise_report_rate (bool rise) {
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0 , i = 0;
    u8 rate = 0;
    if (rise) {
	mutex_lock(&fts_data->reg_lock);      
	for (i = 0; i < 6; i++) {
	    ret = fts_write_reg(FTS_REG_REPORT_RATE, 0x1C);
	    msleep(20);
	    ret = fts_read_reg(FTS_REG_REPORT_RATE, &rate);
	    if (rate!=0x1C){
	      FTS_DEBUG("Rise report rate to 300Hz fail rate 0x%X , retry %d",rate, i);
	      msleep(20);
	    } else {
      	      ts_data->report_rate = 300;
	      FTS_DEBUG("Rise report rate to 300Hz rate %X",rate);
	      break;
	    }
	}
	mutex_unlock(&fts_data->reg_lock);
    } else {
        ret = fts_write_reg(FTS_REG_REPORT_RATE, 0x00);
	msleep(10);
	fts_read_reg(FTS_REG_REPORT_RATE, &rate);
	if (rate != 0x1C){
	  ts_data->report_rate = 120;
          FTS_DEBUG("Decrease report rate to 120Hz");
	}
    }
      
}
void ATR_touch(int id,int action, int x, int y, int random)
{
	static int random_x = -5, random_y = -5, random_major = -5;
	struct input_dev *input_dev = fts_data->input_dev;
	int first_empty_slot = -1;
	int i;

	FTS_INFO("keymapping ATR_touch  id=%d, action=%d, x=%d, y=%d", id, action,  x,  y);
	mutex_lock(&input_dev->mutex);
	if(action) //press, find first slot or find last slot;
	{
		for(i = TOTAL_GAME_USED_SLOT -1; i >= 0 ; i--)
		{
			if(first_empty_slot == -1 && touch_figer_slot[i] == 0) //find first empty slot
				first_empty_slot = i;
			if(touch_figer_slot[i] == (id + 1)) //if the last id has been pressed, keep reporting same slot
				first_empty_slot = i;
		}
		FTS_INFO("keymapping ATR_touch press found slot %d", first_empty_slot);
		if(first_empty_slot != -1) // found an available slot
		{
			if(touch_figer_slot[first_empty_slot] ==0)
				FTS_INFO("keymapping report %d down x=%d ,y=%d ",first_empty_slot,x,y);
			
			FTS_INFO("slot %d", first_empty_slot+10);
			input_mt_slot(input_dev, first_empty_slot + 10);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
			if (fts_data->realtime == 1) { 
			    input_set_timestamp(input_dev, fts_data->atr_received);
			}
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0x09 + random_major);

			if(!random)
			{
				input_report_abs(input_dev, ABS_MT_POSITION_X, x + random_x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, y + random_y);
			} else {
				input_report_abs(input_dev, ABS_MT_POSITION_X, x );
				input_report_abs(input_dev, ABS_MT_POSITION_Y, y );
			}

			if(!fts_data->finger_press){
				FTS_INFO("no touch BTN down , atr touch down");
				input_report_key(input_dev, BTN_TOUCH, 1);
			} else {
				FTS_INFO("touch BTN down long pressed , ignore atr touch down");
			}
			input_sync(input_dev);

			touch_figer_slot[first_empty_slot] = id + 1; // save finger id in slot 			
			fts_data->atr_press = true;

		}
	} 
	else //release
	{
		for(i = TOTAL_GAME_USED_SLOT -1; i >= 0 ; i--)
		{
			if(touch_figer_slot[i] == (id + 1)) //find the released slot
			{
				first_empty_slot = i;
				break;
			}
		}

		FTS_INFO("keymapping  release slot %d", first_empty_slot);
		if(first_empty_slot >= 0)
		{
			input_mt_slot(input_dev, first_empty_slot + 10);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
			input_sync(input_dev);
			touch_figer_slot[first_empty_slot] = 0;
		}
	}

	for(i = TOTAL_GAME_USED_SLOT -1; i >= 0 ; i--)
	{
	    if (touch_figer_slot[i] != 0) //find the released slot
		break;
	}   
	if(i < 0) // all button up
	{
		fts_data->atr_press = false;
		if(!fts_data->finger_press)
		{
		    FTS_INFO("no touch and atr BTN down, all BTN up");
		    input_report_key(input_dev, BTN_TOUCH, 0);
		    input_sync(input_dev);
		}
	}
	
	random_x += 1; if(random_x > 5) random_x = -5;
	random_y += 1; if(random_y > 5) random_y = -5;
	random_major += 1; if(random_major > 5) random_major = -5;

	mutex_unlock(&input_dev->mutex);
}

/*
 * attribute functions
 */
static ssize_t keymapping_touch_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
    bool stat = 0;
    if (fts_data->atr_press)
	stat = true;
    else
	stat = false;

    return sprintf(buf, "%d", stat);
}

static ssize_t keymapping_touch_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int id = 0, action = 0, x = 0, y = 0, random = 0, minus = 0;
	FTS_INFO("keymapping cmd buf: %s len=%d", buf, count);
	
	if ((count > 16) || (count < 14)){
	    FTS_INFO("Invalid cmd buffer %d", count);
	    return -EINVAL;
	}
	
	if (fts_data->realtime == 1)
	    fts_data->atr_received =  ktime_get();
	
	if (count == 14) {
		id = buf[0] - '0';
		action = buf[1] - '0';
		random = buf[2] - '0';
		
		minus = buf[3];
		x =  shex_to_u16(buf + 4, 4);
		if(minus == '-')
			x = -x;

		minus = buf[8];
		y =  shex_to_u16(buf + 9, 4);
		if(minus == '-')
			y = -y;
	} else if (count == 15) {
		id = shex_to_u16(buf, 2);
		action = buf[2] - '0';
		random = buf[3] - '0';

		minus = buf[4];
		x =  shex_to_u16(buf + 5, 4);
		if(minus == '-')
			x = -x;

		minus = buf[9];
		y =  shex_to_u16(buf + 10, 4);
		if(minus == '-')
			y = -y;
	}
	
	FTS_INFO("keymapping ID=%d ACTION=%d X=%d Y=%d RANDOM=%d", id, action, x, y, random);
	ATR_touch(id, action, x, y, random);

	return count;
}

static ssize_t fts_game_mode_show (struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    
    count = snprintf(buf + count, PAGE_SIZE, "Game Mode:%s\n",
                     ts_data->game_mode ? "On" : "Off");
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_game_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct fts_ts_data *ts_data = fts_data;
    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->game_mode) {
            FTS_DEBUG("enter game mode");
            ts_data->game_mode = ENABLE;
//	    rise_report_rate(true);
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->game_mode) {
            FTS_DEBUG("exit game mode");
//	    rise_report_rate(false);
            ts_data->game_mode = DISABLE;
        }
    }

    FTS_DEBUG("game mode:%d", ts_data->game_mode);
    return count;
}

static ssize_t fts_rotation_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d \n", fts_data->rotation_angle);
}

static ssize_t fts_rotation_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        bool reconfig = false;
	if (buf[0] == '1') {
	  if (pre_angle != 1)
	      reconfig = true;	  
	  fts_data->rotation_angle = ANGLE_90;
	  pre_angle = 1;
	} else if (buf[0] == '2') {
  	  if (pre_angle != 2)
	      reconfig = true;	  
	  fts_data->rotation_angle = ANGLE_270;
	  pre_angle = 2;
	} else if (buf[0] == '0') {
  	  if (pre_angle != 0)
	      reconfig = true;	  
	  fts_data->rotation_angle = ANGLE_0;
	  pre_angle = 0;
	}
	
	if (reconfig)
	    set_rotation_mode();
	
	return count;
}

static ssize_t game_settings_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 sli_sen = 0 , sli_acy = 0,tp_acy = 0;
    
    fts_read_reg(FTS_REG_SLIDING_SENSITIVITY, &sli_sen);
    fts_read_reg(FTS_REG_SLIDING_PRECISION, &sli_acy);
    fts_read_reg(FTS_REG_TOUCH_SENSITIVITY, &tp_acy);

  return sprintf(buf, "%03X.%03X.%03X\n", slidling_sen,sliding_acy,tp_acy);
}

static ssize_t game_settings_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char game_settings[29];
	int ret1,ret2,ret3;
	u8 value = 0x0;
	int i = 0;

	memset(game_settings, 0, sizeof(game_settings));
	sprintf(game_settings, "%s", buf);
	game_settings[count-1] = '\0';
	
	FTS_INFO("game_settings %s count %d ",game_settings,count);
	
	if(count > 13){
            return -EINVAL;
	}
	
	slidling_sen = (u16)shex_to_u16(game_settings +0, 3);
	sliding_acy = (u16)shex_to_u16(game_settings +4, 3);
	touch_acy = (u16)shex_to_u16(game_settings +8, 3);

	FTS_INFO("slidling_sen 0x%03X, sliding_acy 0x%03X touch_acy 0x%03X",
		slidling_sen,sliding_acy,touch_acy);
	if (!fts_data->suspended) {	  
	    mutex_lock(&fts_data->reg_lock);
	    for (i = 0; i < 6; i++) {
		ret1 = fts_write_reg(FTS_REG_SLIDING_SENSITIVITY, slidling_sen);
		msleep(20);
		fts_read_reg(FTS_REG_SLIDING_SENSITIVITY, &value);
		if (value == slidling_sen){
//		    FTS_DEBUG("set FTS_REG_SLIDING_SENSITIVITY 0x%x success",slidling_sen);
		    break;
		} else {
		    FTS_DEBUG("set FTS_REG_SLIDING_SENSITIVITY 0x%x failed,retry count %d",value, i);
		    msleep(20);
		}
	    }
	    
    	    for (i = 0; i < 6; i++) {
		ret2 = fts_write_reg(FTS_REG_SLIDING_PRECISION, sliding_acy);
		msleep(20);
		fts_read_reg(FTS_REG_SLIDING_PRECISION, &value);
		if (value == sliding_acy){
//		    FTS_DEBUG("set FTS_REG_SLIDING_PRECISION 0x%x success",sliding_acy);
		    break;
		} else {
		    FTS_DEBUG("set FTS_REG_SLIDING_PRECISION 0x%x failed,retry count %d",value, i);
		    msleep(20);
		}
	    }

    	    for (i = 0; i < 6; i++) {
		ret3 = fts_write_reg(FTS_REG_TOUCH_SENSITIVITY, touch_acy);
		msleep(20);
		fts_read_reg(FTS_REG_TOUCH_SENSITIVITY, &value);
		if (value == touch_acy){
//		    FTS_DEBUG("set FTS_REG_TOUCH_SENSITIVITY 0x%x success",touch_acy);
		    break;
		} else {
		    FTS_DEBUG("set FTS_REG_TOUCH_SENSITIVITY 0x%x failed,retry count %d",value, i);
		    msleep(20);
		}
	    }
	    mutex_unlock(&fts_data->reg_lock);
	}
	return count;
}

static ssize_t rise_report_rate_show (struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 rate = 0;
    int report_rate = 0;
    fts_read_reg(FTS_REG_REPORT_RATE, &rate);
    
    if (rate == 0x1c)
        report_rate = 300;
    else
        report_rate = 120;
    count = snprintf(buf + count, PAGE_SIZE, "%d\n",
                     report_rate);

    return count;
}

static ssize_t rise_report_rate_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{  
    if (FTS_SYSFS_ECHO_ON(buf)) {
        rise_report_rate(true);
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
	rise_report_rate(false);
    }

    FTS_DEBUG("Report rate set to:%d", fts_data->report_rate);
    return count;
}


static ssize_t edge_settings_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 l_val = 0 , r_val = 0;
    
    fts_read_reg(FTS_REG_EDGEPALM_LEFT, &l_val);
    fts_read_reg(FTS_REG_EDGEPALM_RIGHT, &r_val);
    FTS_INFO("palm range left %x right %x",l_val,r_val);	
    
    return sprintf(buf, "%03d.%03d\n", l_val,r_val);
}

static ssize_t edge_settings_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char edge_settings[10];
	int ret;
	u8 rate = 0;
	
	memset(edge_settings, 0, sizeof(edge_settings));
	sprintf(edge_settings, "%s", buf);
	edge_settings[count-1] = '\0';
	
	FTS_INFO("edge_settings %s count %d ",edge_settings,count);
	
	if(count > 9){
            return -EINVAL;
	}
	
	Rcoefleft = (u16)shex_to_u16(edge_settings +0, 3);
	RcoefRight = (u16)shex_to_u16(edge_settings +4, 3);
	
	FTS_INFO("Rcoefleft 0x%X,RcoefRight 0x%X",Rcoefleft,RcoefRight);
	if (!fts_data->suspended) {
	    if (Rcoefleft > 10 || RcoefRight > 10) {
		ret = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x00);
		FTS_INFO("edge settings larger more than 10 , disable edge palm");
		fts_read_reg(FTS_REG_REPORT_RATE, &rate);
/*		if (rate != 0x1c && (fts_data->game_mode == ENABLE))
		  rise_report_rate(true);
*/
		return count;
	    }
	
	    ret = set_rotation_mode();
/*	    fts_read_reg(FTS_REG_REPORT_RATE, &rate);
	    if (rate != 0x1c)
	      rise_report_rate(true);
*/
	}

	return count;
}


static DEVICE_ATTR(keymapping_touch, S_IRUGO | S_IWUSR, keymapping_touch_show, keymapping_touch_store);
static DEVICE_ATTR(fts_game_mode, S_IRUGO | S_IWUSR, fts_game_mode_show, fts_game_mode_store);
static DEVICE_ATTR(fts_rotation_mode, S_IRUGO | S_IWUSR, fts_rotation_mode_show, fts_rotation_mode_store);
static DEVICE_ATTR(game_settings, S_IRUGO | S_IWUSR, game_settings_show, game_settings_store);
static DEVICE_ATTR(rise_report_rate, S_IRUGO | S_IWUSR, rise_report_rate_show, rise_report_rate_store);
static DEVICE_ATTR(edge_settings, S_IRUGO | S_IWUSR, edge_settings_show, edge_settings_store);
/* add your attr in here*/
static struct attribute *fts_attributes[] = {
    &dev_attr_keymapping_touch.attr,
    &dev_attr_fts_game_mode.attr,
    &dev_attr_fts_rotation_mode.attr,
    &dev_attr_game_settings.attr,
    &dev_attr_rise_report_rate.attr,
    &dev_attr_edge_settings.attr,
    NULL
};

static struct attribute_group asus_game_attribute_group = {
    .attrs = fts_attributes
};

void asus_game_recovery(struct fts_ts_data *ts_data) 
{
    if (ts_data->game_mode == ENABLE) {
      FTS_INFO("Game mode recovery");
    }
  
}

int asus_game_create_sysfs(struct fts_ts_data *ts_data)
{
    int ret = 0;

    ret = sysfs_create_group(&ts_data->dev->kobj, &asus_game_attribute_group);
    if (ret) {
        FTS_ERROR("[EX]: asus_create_group() failed!!");
        sysfs_remove_group(&ts_data->dev->kobj, &asus_game_attribute_group);
        return -ENOMEM;
    } else {
        FTS_INFO("[EX]: asus_create_group() succeeded!!");
    }
    
    ts_data->game_mode = DISABLE;
    ts_data->rotation_angle = 0;
    ts_data->report_rate = 120;
    mutex_init(&ts_data->reg_lock);
    return ret;
}

int asus_game_remove_sysfs(struct fts_ts_data *ts_data)
{
    sysfs_remove_group(&ts_data->dev->kobj, &asus_game_attribute_group);
    return 0;
}