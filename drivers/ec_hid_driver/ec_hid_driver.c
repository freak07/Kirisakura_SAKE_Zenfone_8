/* Copyright (c) 2019, The Linux Foundation. All rights reserved.
 * Edit by ASUS Deeo, deeo_ho@asus.com
 * V10
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
//#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/hidraw.h>
#include <linux/usb.h>
#include <linux/time.h>
#include <linux/iio/consumer.h>

#include <dt-bindings/iio/qcom,spmi-vadc.h>
#include <dt-bindings/iio/qcom,spmi-adc7-pm8350.h>
#include "../thermal/qcom/adc-tm.h"

#include "ec_hid_driver.h"
#include "ec_comm.h"

extern int32_t adc_tm7_channel_measure(struct adc_tm_chip *chip,struct adc_tm_param *param);
extern struct adc_tm_chip *get_adc_tm(struct device *dev,const char *name);
void ec_hid_uevent(void);
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state);

struct iio_channel *asus_pogo_vadc_chan;
struct adc_tm_chip *adc_tm_dev_pogo;
struct adc_tm_param adc_tm_param_pogo;

struct mutex ec_i2c_func_mutex;
EXPORT_SYMBOL(ec_i2c_func_mutex);

struct blocking_notifier_head ec_hid_event_header;
EXPORT_SYMBOL(ec_hid_event_header);

struct ec_i2c_platform_data *ec_i2c_data;
EXPORT_SYMBOL(ec_i2c_data);

struct ec_check_int_interface ec_check_int;
EXPORT_SYMBOL(ec_check_int);

struct ec_set_gpio_interface ec_set_gpio;
EXPORT_SYMBOL(ec_set_gpio);

struct ec_get_gpio_interface ec_get_gpio;
EXPORT_SYMBOL(ec_get_gpio);

struct ec_battery_interface ec_battery_func;
EXPORT_SYMBOL(ec_battery_func);

struct ec_set_dp_display_interface ec_set_dp_display;
EXPORT_SYMBOL(ec_set_dp_display);

struct ec_porta_cc_interface ec_porta_cc;
EXPORT_SYMBOL(ec_porta_cc);

struct ec_fw_ver_interface ec_fw_ver;
EXPORT_SYMBOL(ec_fw_ver);

extern void usb_enable_autosuspend(struct usb_device *);
extern void usb_disable_autosuspend(struct usb_device *);

/* ASUS BSP DP +++ */
extern char *get_last_backlight_value(void);
extern int asus_current_fps;
extern bool g_Charger_mode;
extern bool g_is_new_station;
extern bool g_skip_ss_lanes;
/* ASUS BSP DP --- */

//extern bool g_station_sleep;
//extern int lid_status;
//extern bool g_station_dp_disconnect;

// For HID wait for completion
struct completion hid_state;
EXPORT_SYMBOL(hid_state);

u8 gEC_init=0;
EXPORT_SYMBOL(gEC_init);

int is_porta_cc_locked = 0;
EXPORT_SYMBOL(is_porta_cc_locked);

static u8 pogo_mutex_state = 0;
int g_pogo_id_voltage = 0;

int is_ec_has_removed = 255;
EXPORT_SYMBOL(is_ec_has_removed);

int pogo_sync_key = 0;
EXPORT_SYMBOL(pogo_sync_key);

bool ENE_upgrade_mode;
EXPORT_SYMBOL(ENE_upgrade_mode);

int ec_i2c_is_suspend = 0;
EXPORT_SYMBOL(ec_i2c_is_suspend);

//int ec_i2c_ultra_mode = 0;
//EXPORT_SYMBOL(ec_i2c_ultra_mode);

int ec_i2c_driver_state = 0;
EXPORT_SYMBOL(ec_i2c_driver_state);

// Project ROG3 default use I2S audio
int audio_switch = 1;
EXPORT_SYMBOL(audio_switch);

int Station_HWID = 0;
EXPORT_SYMBOL(Station_HWID);

/*
 * 	gDongleEvent : only for Station ( gDongleType == 2 )
 *
 * 	0 	: Normal mode
 * 	1 	: Upgrade mode
 * 	2 	: Low Battery mode
 * 	3 	: ShutDown & Virtual remove mode
 */
uint8_t gDongleEvent=DongleEvent_Normal_mode;
EXPORT_SYMBOL(gDongleEvent);

/*
 * 	gDongleType
 * 	- 1: Error
 * 	0 	: No Insert
 * 	1 	: InBox5
 *  2   : ERROR
 *  3   : Other
 * 255  : Default status
 */
uint8_t gDongleType=0;
EXPORT_SYMBOL(gDongleType);

/*
 * 	gPanelStatusForHdcpWork
 * 	0 	: Phone panel off
 * 	1 	: Phone panel on
 */
uint8_t gPanelStatusForHdcpWork=0;
EXPORT_SYMBOL(gPanelStatusForHdcpWork);

static const unsigned int asus_dongle_cable[] = {
	EXTCON_NONE,
};

struct extcon_dev *extcon_dongle;

int ec_hid_event_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ec_hid_event_header,nb);
}
EXPORT_SYMBOL(ec_hid_event_register);


int ec_hid_event_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ec_hid_event_header,nb);
}
EXPORT_SYMBOL(ec_hid_event_unregister);

/*
int hid_to_get_ec_fw_ver(void)
{
	int ret = 0;

	mutex_lock(&ec_i2c_func_mutex);

	if(ec_fw_ver.i2c_get_ec_fw_ver == NULL)
	{
		mutex_unlock(&ec_i2c_func_mutex);
		printk("[EC_HID] ec_fw_ver.i2c_get_ec_fw_ver == NULL \n");
		ret = -1;
		return ret;
	}

	mutex_unlock(&ec_i2c_func_mutex);
	ret = ec_fw_ver.i2c_get_ec_fw_ver();

	return ret;
}
EXPORT_SYMBOL(hid_to_get_ec_fw_ver);

int hid_to_set_ultra_power_mode(u8 type)  // 1:In, 0:Out
{
	int ret = 0;

	mutex_lock(&ec_i2c_func_mutex);

	printk("[EC_HID] hid_to_set_ultra_power_mode: type %d, ec_i2c_ultra_mode %d\n", type, ec_i2c_ultra_mode);
	if(ec_i2c_is_suspend == 1)
	{
		printk("[EC_HID] : ec i2c does not resume ,set_ultra_power_mode later\n");
		mutex_unlock(&ec_i2c_func_mutex);
		return ret;
	}

	if(ec_battery_func.i2c_set_ultra_low_power_mode == NULL)
	{
		mutex_unlock(&ec_i2c_func_mutex);
		printk("[EC_HID] i2c_set_ultra_low_power_mode == NULL\n");
		ret = -1;
		return ret;
	}

	mutex_unlock(&ec_i2c_func_mutex);
	ret = ec_battery_func.i2c_set_ultra_low_power_mode(type);
	
	return ret;
}
EXPORT_SYMBOL(hid_to_set_ultra_power_mode);

static int hid_to_set_phone_panel_state(u8 type)  // 1:On, 0:Off
{
	int ret = 0;

	mutex_lock(&ec_i2c_func_mutex);

	printk("[EC_HID] hid_to_set_phone_panel_state: type %d\n", type);

	if(ec_battery_func.i2c_set_phone_panel_state == NULL)
	{
		mutex_unlock(&ec_i2c_func_mutex);
		printk("[EC_HID] i2c_set_phone_panel_state == NULL\n");
		ret = -1;
		return ret;
	}

	mutex_unlock(&ec_i2c_func_mutex);
	ret = ec_battery_func.i2c_set_phone_panel_state(type);

	return ret;
}
//EXPORT_SYMBOL(hid_to_set_phone_panel_state);
*/

/*
int ec_hid_get_porta_cc_state(int *state)
{
	int ret = 0;

	mutex_lock(&ec_i2c_func_mutex);	
	if(ec_porta_cc.ec_i2c_get_porta_cc_state == NULL)
	{
		mutex_unlock(&ec_i2c_func_mutex);
		printk("[EC_HID] ec_porta_cc.ec_i2c_get_porta_cc_state == NULL \n");
		ret = -1;
		return ret;
	}

	mutex_unlock(&ec_i2c_func_mutex);
	ret = ec_porta_cc.ec_i2c_get_porta_cc_state(state);

	return ret;

}
*/

static ssize_t pogo_id_voltage_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[EC_HID] pogo_id_voltage_show : %d\n", g_pogo_id_voltage);
	return sprintf(buf, "%d\n", g_pogo_id_voltage);

}

static ssize_t is_ec_has_removed_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);

	printk("[EC_HID] is_ec_has_removed_store : %d\n", val);

	if(val)
		is_ec_has_removed = 1;
	else
		is_ec_has_removed = 0;
	
	return count;
}

static ssize_t is_ec_has_removed_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[EC_HID] is_ec_has_removed_show : %d\n", is_ec_has_removed);
	return sprintf(buf, "%d\n", is_ec_has_removed);

}
/*
static ssize_t pogo_sync_key_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[EC_HID] pogo_sync_key_show : %d\n", pogo_sync_key);
	return sprintf(buf, "%d\n", pogo_sync_key);

}


static ssize_t pogo_sync_key_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val = 255;
	int state = 255;
	int ret = 0;
	sscanf(data, "%d", &val);

	printk("[EC_HID] pogo_sync_key_store : %d\n", val);

	if(val == 0){

		ret = ec_hid_get_porta_cc_state(&state);
		if(ret < 0) {
			printk("[EC_HID] pogo_sync_key_store EC I2C Driver has been removed!!\n");
			is_porta_cc_locked = 0;
			pogo_sync_key = 0;
			return count;
		}
		printk("[EC_HID] pogo_sync_key_store start recheck latch key states is %d\n",state);

		if (1 == is_porta_cc_locked && state == 1) {
			printk("[EC_HID] : pogo_sync_key_store has locked \n");
			pogo_sync_key = 0;
		}
		else if (0 == is_porta_cc_locked && state == 0){
			printk("[EC_HID] : pogo_sync_key_store has unlocked \n");
			pogo_sync_key = 0;
		} else if(0 == is_porta_cc_locked && state == 1){
			printk("[EC_HID] : 0 == is_porta_cc_locked && state == 1 \n");
			pogo_sync_key = 1;
			gDongleType = Dongle_Station2;
			blocking_notifier_call_chain(&ec_hid_event_header,gDongleType,NULL);
			ec_hid_uevent();
			is_porta_cc_locked = 1;
		} else if(1 == is_porta_cc_locked && state == 0){
			printk("[EC_HID] : 1 == is_porta_cc_locked && state == 0 \n");
			pogo_sync_key = 1;
			gDongleType = 200;
			blocking_notifier_call_chain(&ec_hid_event_header,gDongleType,NULL);
			ec_hid_uevent();
			is_porta_cc_locked = 0;
			g_skip_ss_lanes = false; // ASUS BSP DP +++
		} else{
			printk("[EC_HID] : EC I2C Driver has been removed, fore unlock pogo_sync_key!!\n");
			is_porta_cc_locked = 0;
			pogo_sync_key = 0;
		}
	}
	
	return count;
}
*/

static ssize_t sync_state_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val == 0){
		ASUSEvtlog("[EC_HID] asus_extcon_set_state_sync : %d\n", val);
		printk("[EC_HID][EXTCON] extcon_dongle->state : %d, val : %d\n", extcon_dongle->state, val);
		asus_extcon_set_state_sync(extcon_dongle, val);

		pogo_mutex_state = 0;
		printk("[EC_HID] pogo_sema up!!! %d\n", val);
		up(&g_hid_data->pogo_sema);
	}else if ((val > 0 && val <= 4) || val == 7 || (val >= 11 && val <= 15)){
		ASUSEvtlog("[EC_HID] asus_extcon_set_state_sync : %d\n", val+5);
		printk("[EC_HID][EXTCON] extcon_dongle->state : %d, val : %d\n", extcon_dongle->state, (val+5));
		asus_extcon_set_state_sync(extcon_dongle, (val+5));

		pogo_mutex_state = 0;

		if((val != 11)  || (val != 13)) {
			printk("[EC_HID] pogo_sema up!!! %d\n", val);
			up(&g_hid_data->pogo_sema);
		}
	}else {
		printk("[EC_HID] Wrong value %d, Do not sync state!!!\n", val);

		pogo_mutex_state = 0;
		printk("[EC_HID] pogo_sema up!!! %d\n", val);
		up(&g_hid_data->pogo_sema);
	}

	return count;
}

void control_pogo_det(enum asus_dongle_type type){

	switch(type){
		case Dongle_NO_INSERT:
			gpio_set_value(g_hid_data->pogo_sleep, 0);
			msleep(5);
			gpio_set_value(g_hid_data->pogo_det, 0); 
		break;
		case Dongle_INBOX5:
			gpio_set_value(g_hid_data->pogo_sleep, audio_switch);
			msleep(5);
			gpio_set_value(g_hid_data->pogo_det, 1);
		break;
		case Dongle_ERROR:
			gpio_set_value(g_hid_data->pogo_det, 0); 
		break;
		case Dongle_Others:
			gpio_set_value(g_hid_data->pogo_det, 0); 
		break;
		default:
			gpio_set_value(g_hid_data->pogo_det, 0);
	}
}

void ec_hid_uevent(void){
	
	u8 type;
	int retry_times = 0;
//	struct timespec uptime;

//	get_monotonic_boottime(&uptime);
	type = gDongleType;
	gDongleEvent = DongleEvent_Normal_mode;

	// before boot time < 15s, Do not send uevent
//	if ((unsigned long) uptime.tv_sec < 8){
//		printk("[EC_HID] boot time too early\n");
//		printk("[EC_HID] gDongleType %d, gDongleEvent %d.\n", gDongleType, gDongleEvent);
//	}

	if (type == Dongle_default_status){
		printk("[EC_HID] type = 255, fake dongle type\n");
		return;
	}

	if(type == 100 || type == 200) {
		printk("[EC_HID] type : %d, pogo_mutex_state : %d \n", type, pogo_mutex_state);

		if(type == 100 && ec_i2c_driver_state == 1){
			while(ec_i2c_driver_state && retry_times < 6)
			{
				printk("[EC_HID] ec i2c driver has not removed, wait for it!!\n");
				msleep(100);
				retry_times ++;
			}

			printk("[EC_HID] : type:ec_i2c_driver_state:retry_times %d:%d:%d!\n",type,ec_i2c_driver_state,retry_times);
			retry_times = 0;
		}

		ec_i2c_driver_state = 0;
		
		kobject_uevent(&g_hid_data->dev->kobj, KOBJ_CHANGE);
	}else{
		if (pogo_mutex_state && type == Dongle_NO_INSERT){
			printk("[EC_HID] type : %d, pogo_mutex_state : %d, force unlock!!\n", type, pogo_mutex_state);
			pogo_mutex_state = 0;
			printk("[EC_HID] pogo_sema up, %d!!!\n", type);
			up(&g_hid_data->pogo_sema);
		}
		
		down(&g_hid_data->pogo_sema);
		printk("[EC_HID] pogo_sema down, %d!!!\n", type);
		pogo_mutex_state = 1;

		kobject_uevent(&g_hid_data->dev->kobj, KOBJ_CHANGE);
		ASUSEvtlog("[EC_HID] gDongleEvent : %d, previous_event %d\n", gDongleEvent, g_hid_data->previous_event);
		g_hid_data->previous_event = gDongleEvent;
	}
}
EXPORT_SYMBOL(ec_hid_uevent);

int dongle_type_detect(enum asus_dongle_type *type)
{
	int retval = 0 ,pogo_id_voltage = 0;
	struct iio_channel *pogo_id_adc_chan;

	pogo_id_adc_chan = iio_channel_get(g_hid_data->dev,"asus_pogo_vadc");
	if (IS_ERR_OR_NULL(pogo_id_adc_chan)) {
		retval = PTR_ERR(pogo_id_adc_chan);
		printk("[EC_HID] iio_channel_get fail.\n");
		
		return retval;
	}
	
	mutex_lock( &g_hid_data->pogo_id_mutex);

	retval = iio_read_channel_processed(pogo_id_adc_chan,&pogo_id_voltage);
	if(retval < 0)
		printk("[EC_HID] iio_read_channel_processed fail.\n");
		
	mutex_unlock(&g_hid_data->pogo_id_mutex);

	printk("[EC_HID] pogo_id_adc is %d.\n",pogo_id_voltage);
	
	g_pogo_id_voltage = pogo_id_voltage;

	if(pogo_id_voltage >= 1700000)
	{
		*type = Dongle_NO_INSERT;
	}
	else if( 620000 <= pogo_id_voltage && pogo_id_voltage <= 780000) 
	{
		*type = Dongle_INBOX5;
	}
	else if(pogo_id_voltage <= 100000)
	{
		*type = Dongle_ERROR;
	}
	else
	{
		*type = Dongle_Others;
	}

	gDongleType = *type;

	printk("[EC_HID] gDongleType : %d  type : %d !\n", gDongleType, *type);

	return retval;
}

void update_POGO_ID_ADC_Threshold(enum asus_dongle_type type)
{
	int rc;
	
	switch(type){
		case Dongle_NO_INSERT:
		adc_tm_param_pogo.low_thr = 1000000;
		adc_tm_param_pogo.high_thr = 2000000;
		adc_tm_param_pogo.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		msleep(20);
		rc = adc_tm7_channel_measure(adc_tm_dev_pogo,&adc_tm_param_pogo);
		if(rc)
			printk("[EC_HID] : adc_tm7_channel_measure ERROR\n");
		break;
		case Dongle_INBOX5:
		adc_tm_param_pogo.low_thr = 100000;
		adc_tm_param_pogo.high_thr = 1700000;
		adc_tm_param_pogo.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		msleep(20);
		rc = adc_tm7_channel_measure(adc_tm_dev_pogo,&adc_tm_param_pogo);
		if(rc)
			printk("[EC_HID] : adc_tm7_channel_measure ERROR\n");
		break;
		case Dongle_ERROR:
			adc_tm_param_pogo.low_thr = 0;
			adc_tm_param_pogo.high_thr = 100000;
			adc_tm_param_pogo.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
			msleep(20);
			rc = adc_tm7_channel_measure(adc_tm_dev_pogo,&adc_tm_param_pogo);
			if(rc)
				printk("[EC_HID] : adc_tm7_channel_measure ERROR\n");
		break;
		case Dongle_Others:
			adc_tm_param_pogo.low_thr = 100000;
			adc_tm_param_pogo.high_thr = 1700000;
			adc_tm_param_pogo.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
			msleep(20);
			rc = adc_tm7_channel_measure(adc_tm_dev_pogo,&adc_tm_param_pogo);
			if(rc)
				printk("[EC_HID] : adc_tm7_channel_measure ERROR\n");
		break;
		default:
		adc_tm_param_pogo.low_thr = 1000000;
		adc_tm_param_pogo.high_thr = 2000000;
		adc_tm_param_pogo.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		msleep(20);
		rc = adc_tm7_channel_measure(adc_tm_dev_pogo,&adc_tm_param_pogo);
		if(rc)
			printk("[EC_HID] : adc_tm7_channel_measure ERROR\n");
	}
}

static void qpnp_pogo_id_adc_notification(enum adc_tm_state state,void *ctx)
{
	enum asus_dongle_type type = 255;
	
	printk("[EC_HID] : state is %d!\n",state);
	dongle_type_detect(&type);
	
	if(type == Dongle_ERROR|| type == Dongle_Others)
	{
		msleep(100);
		dongle_type_detect(&type);
	}
	
	update_POGO_ID_ADC_Threshold(type);
	
	control_pogo_det(type);

	blocking_notifier_call_chain(&ec_hid_event_header,gDongleType,NULL);
	
	ec_hid_uevent();
}

int asus_wait4hid (void)
{
	int rc = 0;
/* no need to wait
    if (gDongleType != Dongle_Station2){
		printk("[EC_HID] Dongle remove, no need to wait\n");
		return -9;
	}

	if (!wait_for_completion_timeout(&hid_state, msecs_to_jiffies(3000))) {
		rc = -EINVAL;
		printk("[EC_HID] wait for HID complete timeout\n");
	}
    
    if (gDongleType != Dongle_Station2){
		printk("[EC_HID] Dongle remove, no need to wait\n");
		return -9;
	}
*/
    return rc;
}
EXPORT_SYMBOL_GPL(asus_wait4hid);

int ec_mutex_lock(char *name)
{
	mutex_lock(&g_hid_data->report_mutex);
	printk("[EC_HID] ec_mutex_lock : %s\n", name);

    return 0;
}
EXPORT_SYMBOL_GPL(ec_mutex_lock);

int ec_mutex_unlock(char *name)
{
	printk("[EC_HID] ec_mutex_unlock : %s\n", name);
	mutex_unlock(&g_hid_data->report_mutex);

    return 0;
}
EXPORT_SYMBOL_GPL(ec_mutex_unlock);

static ssize_t gDongleType_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int retval;
	enum asus_dongle_type type = Dongle_default_status;

	retval = dongle_type_detect(&type);
	
	printk("[EC_HID] gDongleType_show : %d\n", gDongleType);
	return sprintf(buf, "%d\n", gDongleType);
}

static ssize_t gDongleType_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);

	printk("[EC_HID] gDongleType_store : %d\n", val);

	switch(val) {
		case Dongle_NO_INSERT:
			gDongleType = Dongle_NO_INSERT;
		break;
		case Dongle_INBOX5:
			gDongleType = Dongle_INBOX5;
		break;
		case Dongle_ERROR:
			gDongleType = Dongle_ERROR;
		break;
		case Dongle_Others:
			gDongleType = Dongle_Others;
		break;
		default:
			printk("[EC_HID] NO Recognize Dongle Type!!! Set gDongleType as Dongle_Others!\n");
			gDongleType = Dongle_Others;
			break;
	}

	if (pogo_mutex_state)
	{
		printk("[EC_HID] pogo_mutex_state : %d, skip send uevent.\n", pogo_mutex_state);
		return count;
	}

	down(&g_hid_data->pogo_sema);
	printk("[EC_HID] pogo_sema down!!! %d\n", val);
	pogo_mutex_state = 1;

	ec_hid_uevent();
	//kobject_uevent(&g_hid_data->dev->kobj, KOBJ_CHANGE);
	return count;
}

static ssize_t lock_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[EC_HID] lock_show : %d\n", g_hid_data->lock);

	return snprintf(buf, PAGE_SIZE,"%d\n", g_hid_data->lock);
}

static ssize_t lock_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		printk("[EC_HID] Lock\n");
		g_hid_data->lock = true;
	}else {
		printk("[EC_HID] Unlock\n");
		g_hid_data->lock = false;
	}

	return count;
}

static ssize_t pogo_mutex_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
//		if (pogo_mutex_state) {
		if(0) {
			printk("[EC_HID] pogo_mutex is already lock\n");
		} else {
//			mutex_lock(&g_hid_data->pogo_mutex);
//			printk("[EC_HID] force mutex_lock pogo_mutex\n");
			down(&g_hid_data->pogo_sema);
			printk("[EC_HID] pogo_sema down!!!\n");

			pogo_mutex_state = 1;
		}
	}else {
		//if (pogo_mutex_state) {
		if(1) {
			pogo_mutex_state = 0;
			//printk("[EC_HID] force mutex_unlock pogo_mutex\n");
			//mutex_unlock(&g_hid_data->pogo_mutex);
			printk("[EC_HID] pogo_sema up!!!\n");
			up(&g_hid_data->pogo_sema);
		} else {
			printk("[EC_HID] pogo_mutex is already unlock\n");
		}
	}

	return count;
}

static ssize_t pogo_mutex_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[EC_HID] pogo_mutex_show : %d\n", pogo_mutex_state);

	return snprintf(buf, PAGE_SIZE,"%d\n", pogo_mutex_state);
}

static ssize_t block_input_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE,"0x%x\n", block_hid_input);
}

static ssize_t block_input_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		block_hid_input = true;
	}else {
		block_hid_input = false;
	}

	return count;
}

static ssize_t gDongleEvent_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[EC_HID] gDongleEvent_show : %d\n", gDongleEvent);
	return sprintf(buf, "%d\n", gDongleEvent);
}

static ssize_t gDongleEvent_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);

	printk("[EC_HID] gDongleEvent_store : %d\n", val);
	gDongleEvent = val;

	return count;
}

static ssize_t pogo_detect_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int val =  0;
	
	val = gpio_get_value(g_hid_data->pogo_det);
	
	printk("[EC_HID] pogo_detect_show : %d\n", val);
	
	return sprintf(buf, "%d\n", val);
}

static ssize_t pogo_detect_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val, retval;
	sscanf(data, "%d", &val);

	printk("[EC_HID] pogo_detect_store : %d\n", val);

	retval = gpio_direction_output(g_hid_data->pogo_det, val);
	if (retval)
		printk("[EC_HID] pogo_det output high, err %d\n", retval);

//	if(val)
//		gpio_set_value(g_hid_data->pogo_det, 1);
//	else
//		gpio_set_value(g_hid_data->pogo_det, 0);
	
	return count;
}


static ssize_t usb2_mux2_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int val =  0;
	
	val = gpio_get_value(g_hid_data->usb2_mux2_en);
	
	printk("[EC_HID] usb2_mux2_show : %d\n", val);
	
	return sprintf(buf, "%d\n", val);
}

static ssize_t usb2_mux2_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val, retval;
	sscanf(data, "%d", &val);

	printk("[EC_HID] usb2_mux2_store : %d\n", val);

	retval = gpio_direction_output(g_hid_data->usb2_mux2_en, val);
	if (retval)
		printk("[EC_HID] pogo_det output high, err %d\n", retval);

//	if(val)
//		gpio_set_value(g_hid_data->usb2_mux2_en, 1);
//	else
//		gpio_set_value(g_hid_data->usb2_mux2_en, 0);
	
	return count;
}


static ssize_t pogo_sleep_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int val =  0;
	
	val = gpio_get_value(g_hid_data->pogo_sleep);
	
	printk("[EC_HID] pogo_sleep_show : %d\n", val);
	
	return sprintf(buf, "%d\n", val);
}

static ssize_t pogo_sleep_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);

	printk("[EC_HID] pogo_sleep_store : %d\n", val);

	if(val)
		gpio_set_value(g_hid_data->pogo_sleep, 1);
	else
		gpio_set_value(g_hid_data->pogo_sleep, 0);
	
	return count;
}

static ssize_t pogo_fan_on_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int val =  0;
	
	val = gpio_get_value(g_hid_data->pogo_fan_on);
	
	printk("[EC_HID] pogo_fan_on_show : %d\n", val);
	
	return sprintf(buf, "%d\n", val);
}

static ssize_t pogo_fan_on_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);

	printk("[EC_HID] pogo_fan_on_store : %d\n", val);

	if(val)
		gpio_set_value(g_hid_data->pogo_fan_on, 1);
	else
		gpio_set_value(g_hid_data->pogo_fan_on, 0);
	
	return count;
}

static ssize_t pogo_aura_en_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int val =  0;
	
	val = gpio_get_value(g_hid_data->pogo_aura_en);
	
	printk("[EC_HID] pogo_aura_en_show : %d\n", val);
	
	return sprintf(buf, "%d\n", val);
}

static ssize_t pogo_aura_en_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);

	printk("[EC_HID] pogo_aura_en_store : %d\n", val);

	if(val)
		gpio_set_value(g_hid_data->pogo_aura_en, 1);
	else
		gpio_set_value(g_hid_data->pogo_aura_en, 0);
	
	return count;
}

static ssize_t pogo_temp_intr_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int val =  0;
	
	val = gpio_get_value(g_hid_data->pogo_temp_intr);
	
	printk("[EC_HID] pogo_temp_intr_show : %d\n", val);
	
	return sprintf(buf, "%d\n", val);
}

static ssize_t pogo_temp_intr_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);

	printk("[EC_HID] pogo_temp_intr_store : %d\n", val);

	if(val)
		gpio_set_value(g_hid_data->pogo_temp_intr, 1);
	else
		gpio_set_value(g_hid_data->pogo_temp_intr, 0);
	
	return count;
}

static int start_ec_adc_measure(void)
{
	int rc = 0;

	if(adc_tm_dev_pogo == NULL)
	{
		printk("[EC_HID] : adc_tm_dev_pogo First in \n");
		adc_tm_dev_pogo = get_adc_tm(g_hid_data->dev,"pogo-id");
		if(IS_ERR(adc_tm_dev_pogo)) {
			printk("[EC_HID] get_adc_tm error\n");
			return -1;
		}

		adc_tm_param_pogo.low_thr = 1000000;
		adc_tm_param_pogo.high_thr = 2000000;
		adc_tm_param_pogo.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		adc_tm_param_pogo.channel = PM8350_ADC7_AMUX_THM4;
		adc_tm_param_pogo.btm_ctx = g_hid_data->dev;
		adc_tm_param_pogo.threshold_notification = qpnp_pogo_id_adc_notification;
		msleep(20);
		rc = adc_tm7_channel_measure(adc_tm_dev_pogo,&adc_tm_param_pogo);
		if(rc < 0)
			printk("[EC_HID] : adc_tm7_channel_measure ERROR\n");
	}
	
	return rc;
}

static ssize_t start_ec_adc_measure_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE,"0x%x\n", is_ec_adc_tm7_run);
}

static ssize_t start_ec_adc_measure_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val == 1) 
	{
		ret = start_ec_adc_measure();
		
		if(ret < 0)
		{
			printk("[EC_HID] start_ec_adc_measure error\n");
			is_ec_adc_tm7_run = false;
		}
		else
		{
			printk("[EC_HID] start_ec_adc_measure secuess\n");
			is_ec_adc_tm7_run = true;
		}
	}
	else 
	{
		printk("[EC_HID] start_ec_adc_measure_store code != 1, error code\n");
		is_ec_adc_tm7_run = false;
	}

	return count;
}

static DEVICE_ATTR(gDongleType, S_IRUGO | S_IWUSR, gDongleType_show, gDongleType_store);
static DEVICE_ATTR(lock, S_IRUGO | S_IWUSR, lock_show, lock_store);
static DEVICE_ATTR(pogo_mutex, S_IRUGO | S_IWUSR, pogo_mutex_show, pogo_mutex_store);
static DEVICE_ATTR(block_input, S_IRUGO | S_IWUSR, block_input_show, block_input_store);
static DEVICE_ATTR(start_ec_adc_tm7, S_IRUGO | S_IWUSR, start_ec_adc_measure_show, start_ec_adc_measure_store);
static DEVICE_ATTR(sync_state, S_IRUGO | S_IWUSR, NULL, sync_state_store);
static DEVICE_ATTR(gDongleEvent, S_IRUGO | S_IWUSR, gDongleEvent_show, gDongleEvent_store);
static DEVICE_ATTR(pogo_sleep, S_IRUGO | S_IWUSR, pogo_sleep_show, pogo_sleep_store);
static DEVICE_ATTR(pogo_detect, S_IRUGO | S_IWUSR, pogo_detect_show, pogo_detect_store);
static DEVICE_ATTR(usb2_mux2, S_IRUGO | S_IWUSR, usb2_mux2_show, usb2_mux2_store);
static DEVICE_ATTR(pogo_fan_on, S_IRUGO | S_IWUSR, pogo_fan_on_show, pogo_fan_on_store);
static DEVICE_ATTR(pogo_aura_en, S_IRUGO | S_IWUSR, pogo_aura_en_show, pogo_aura_en_store);
static DEVICE_ATTR(pogo_temp_intr, S_IRUGO | S_IWUSR, pogo_temp_intr_show, pogo_temp_intr_store);
static DEVICE_ATTR(pogo_id_voltage, S_IRUGO | S_IWUSR, pogo_id_voltage_show, NULL);
//static DEVICE_ATTR(pogo_sync_key, S_IRUGO | S_IWUSR, pogo_sync_key_show, pogo_sync_key_store);
static DEVICE_ATTR(is_ec_has_removed, S_IRUGO | S_IWUSR, is_ec_has_removed_show, is_ec_has_removed_store);

static struct attribute *ec_hid_attrs[] = {
	&dev_attr_gDongleType.attr,
	&dev_attr_lock.attr,
	&dev_attr_pogo_mutex.attr,
	&dev_attr_block_input.attr,
	&dev_attr_start_ec_adc_tm7.attr,
	&dev_attr_sync_state.attr,
	&dev_attr_gDongleEvent.attr,
	&dev_attr_pogo_sleep.attr,
	&dev_attr_pogo_detect.attr,
	&dev_attr_usb2_mux2.attr,
	&dev_attr_pogo_fan_on.attr,
	&dev_attr_pogo_aura_en.attr,
	&dev_attr_pogo_temp_intr.attr,
	&dev_attr_pogo_id_voltage.attr,
//	&dev_attr_pogo_sync_key.attr,
	&dev_attr_is_ec_has_removed.attr,
	NULL
};

const struct attribute_group ec_hid_group = {
	.attrs = ec_hid_attrs,
};

int ec_hid_display_notifier_call(struct notifier_block *self, unsigned long event, void *data)
{
	struct ec_hid_data *p_data = container_of(self, struct ec_hid_data, display_notifier);
	struct drm_panel_notifier *evdata = data;
	int blank;

	if (!evdata)
		return 0;

	if (gDongleType == 2){
		asus_wait4hid();
	}

	if (event != DRM_PANEL_EVENT_BLANK)
		return 0;

	if (evdata->data && event == DRM_PANEL_EVENT_BLANK && p_data) {
		blank = *(int *) (evdata->data);

		switch (blank) {
		case DRM_PANEL_BLANK_POWERDOWN:
			// panel is power down notify
//			printk("[EC_HID] Panel Off: Power Saving %d, DP Disconnect %d, Lid status %d\n", g_station_sleep, g_station_dp_disconnect, lid_status);
//			hid_switch_usb_autosuspend(true);
//			//block_hid_input = true;
//			hid_to_set_phone_panel_state(0);

//			if (g_station_sleep && g_station_dp_disconnect && lid_status && !ec_i2c_ultra_mode){
//				hid_to_set_ultra_power_mode(1);
//			}

			//dwc3_station_uevent(2);
		break;

		case DRM_PANEL_BLANK_UNBLANK:
			// panel is power on notify
//			printk("[EC_HID] Panel On: Ultra Low Power mode %d\n", ec_i2c_ultra_mode);
//			hid_switch_usb_autosuspend(false);
//			//block_hid_input = false;
//			hid_to_set_phone_panel_state(1);

//			if (ec_i2c_ultra_mode == 1)
//				hid_to_set_ultra_power_mode(0);
//			//dwc3_station_uevent(1);
		break;

		default:
		break;
		}
	}

	return NOTIFY_OK;
}

void inbox_disconnect_notifier(void)
{
	enum asus_dongle_type type = Dongle_default_status;
	
	gDongleType = Dongle_NO_INSERT;
	ec_hid_uevent();
	
	dongle_type_detect(&type);
	ec_hid_uevent();
}
EXPORT_SYMBOL_GPL(inbox_disconnect_notifier);

static int ec_hid_parse_dt(struct device *dev, struct ec_hid_data *ec_hid_device){
	struct device_node *np = dev->of_node;
	int retval=0;

/*
	ec_hid_device->pogo_sleep = of_get_named_gpio_flags(np, "dongle,pogo-sleep", 0, NULL);
	printk("[EC_HID] pogo_sleep : %d\n", ec_hid_device->pogo_sleep);
*/
	
	// Get the pinctrl node
/*
	ec_hid_device->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(ec_hid_device->pinctrl)) {
	     dev_err(dev, "%s: Failed to get pinctrl\n", __func__);
	     return -1;
	}

	printk("[EC_HID] Get default state\n");
	ec_hid_device->pins_default = pinctrl_lookup_state(ec_hid_device->pinctrl, "default");
	if (IS_ERR_OR_NULL(ec_hid_device->pins_default)) {
		dev_err(dev, "%s: Failed to get pinctrl state default\n", __func__);
	}
	printk("[EC_HID] set the default state\n");
	retval = pinctrl_select_state(ec_hid_device->pinctrl, ec_hid_device->pins_default);
	if (retval)
		dev_err(dev, "%s: pinctrl_select_state retval:%d\n", __func__, retval);
*/

/*
	// Set POGO_SLEEP
	if ( gpio_is_valid(ec_hid_device->pogo_sleep) ) {
		printk("[EC_HID] Request pogo_sleep config.\n");
		retval = gpio_request(ec_hid_device->pogo_sleep, "pogo_sleep");
		if (retval)
			printk("[EC_HID] pogo_sleep gpio_request, err %d\n", retval);

		printk("[EC_HID] pogo_sleep default on.\n");
		retval = gpio_direction_output(ec_hid_device->pogo_sleep, 0);
		if (retval)
			printk("[EC_HID] pogo_sleep output low, err %d\n", retval);

		gpio_set_value(ec_hid_device->pogo_sleep, 0);
	}
*/
	// Set POGO_DET
	ec_hid_device->pogo_det = of_get_named_gpio(np, "POGO_DET", 0);
	if ( gpio_is_valid(ec_hid_device->pogo_det) ) {
		printk("[EC_HID] Request pogo_det config.\n");
		retval = gpio_request(ec_hid_device->pogo_det, "POGO_DET");
		if (retval)
			printk("[EC_HID] pogo_det gpio_request, err %d\n", retval);

		printk("[EC_HID] pogo_det default off.\n");
		retval = gpio_direction_output(ec_hid_device->pogo_det, 0);
		if (retval)
			printk("[EC_HID] pogo_det output high, err %d\n", retval);

		//gpio_set_value(ec_hid_device->pogo_det, 0);
	}

	//set USB2_MUX2_EN
	ec_hid_device->usb2_mux2_en = of_get_named_gpio(np, "USB2_MUX2_EN", 0);
	if ( gpio_is_valid(ec_hid_device->usb2_mux2_en) ) {
		printk("[EC_HID] Request usb2_mux2_en config.\n");
		retval = gpio_request(ec_hid_device->usb2_mux2_en, "USB2_MUX2_EN");
		if (retval)
			printk("[EC_HID] usb2_mux2_en gpio_request, err %d\n", retval);

		printk("[EC_HID] usb2_mux2_en default low (usb).\n");
		retval = gpio_direction_output(ec_hid_device->usb2_mux2_en, 0);
		if (retval)
			printk("[EC_HID] usb2_mux2_en output high, err %d\n", retval);

		//gpio_set_value(ec_hid_device->usb2_mux2_en, 0);
	}

	return 0;
}

static int ec_hid_probe(struct platform_device *pdev)
{
	int status = 0;
	struct device *dev = &pdev->dev;
	struct ec_hid_data *ec_hid_device;
	int retval;
	enum asus_dongle_type type = Dongle_default_status;

	printk("[EC_HID] ec_hid_probe.\n");
	ec_hid_device = kzalloc(sizeof(*ec_hid_device), GFP_KERNEL);
	if (ec_hid_device == NULL) {
		printk("[EC_HID] alloc EC_HID data fail.\r\n");
		goto kmalloc_failed;
	}

	ec_hid_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(ec_hid_class)) {
		printk("[EC_HID] ec_hid_probe: class_create() is failed - unregister chrdev.\n");
		goto class_create_failed;
	}
	
	dev = device_create(ec_hid_class, &pdev->dev,
			    ec_hid_device->devt, ec_hid_device, "dongle");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	printk("[EC_HID] ec_hid_probe: device_create() status %d\n", status);

	status = sysfs_create_group(&pdev->dev.kobj, &ec_hid_group);
	printk("[EC_HID] ec_hid_probe: sysfs_create_group() status %d\n", status);

	extcon_dongle = extcon_dev_allocate(asus_dongle_cable);
	if (IS_ERR(extcon_dongle)) {
		status = PTR_ERR(extcon_dongle);
		printk("[EC_HID] failed to allocate ASUS dongle_type device status %d\n", status);
	}
	
	extcon_dongle->fnode_name = "dock";
	
	status = extcon_dev_register(extcon_dongle);
	if (status < 0) {
		printk("[EC_HID] failed to register ASUS dongle_type device status %d\n", status);
	}

// Parse platform data from dtsi
	retval = ec_hid_parse_dt(&pdev->dev, ec_hid_device);
	if (retval) {
		printk("[EC_HID] ec_hid_parse_dt get fail !!!\n");
		goto skip_pinctrl;
	}

//Register drm panel notifier
	drm_registered = false;
	ec_hid_device->display_notifier.notifier_call = ec_hid_display_notifier_call;

	mutex_init(&ec_hid_device->report_mutex);
	mutex_init(&ec_hid_device->pogo_id_mutex);
	mutex_init(&ec_i2c_func_mutex);
    sema_init(&ec_hid_device->pogo_sema, 1);

	ec_hid_device->lock = false;
	ec_hid_device->dev = &pdev->dev;
	g_hid_data = ec_hid_device;

	init_completion(&hid_state);

	block_hid_input = false;

	if(type == Dongle_default_status)
		retval = dongle_type_detect(&type);
/*
	asus_pogo_vadc_chan = iio_channel_get(g_hid_data->dev,"asus_pogo_vadc");
	if (IS_ERR_OR_NULL(asus_pogo_vadc_chan)) {
		retval = PTR_ERR(asus_pogo_vadc_chan);
		printk("[EC_HID] iio_channel_get fail.\n");
	}
*/
// Project ROG3 default use I2S audio

	audio_switch = 1;

	return 0;

skip_pinctrl:
class_create_failed:
kmalloc_failed:
	return -1;
}

static int ec_hid_remove(struct platform_device *pdev)
{
	printk("[EC_HID] ec_hid_remove.\n");

	return 0;
}

int ec_hid_suspend(struct device *dev)
{
	return 0;
}

int ec_hid_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops ec_hid_pm_ops = {
	.suspend	= ec_hid_suspend,
	.resume	= ec_hid_resume,
};

static struct of_device_id dongle_match_table[] = {
	{ .compatible = "asus:ec_hid",},
	{ },
};

static struct platform_driver dongle_hid_driver = {
	.driver = {
		.name = "ec_hid",
		.owner = THIS_MODULE,
		.pm	= &ec_hid_pm_ops,
		.of_match_table = dongle_match_table,
	},
	.probe         	= ec_hid_probe,
	.remove			= ec_hid_remove,
};

static int __init dongle_hid_init(void)
{
	int ret;

	ret = platform_driver_register(&dongle_hid_driver);
	if (ret != 0) {
		printk("[EC_HID] dongle_hid_init fail, Error : %d\n", ret);
	}
	
	return ret;
}
module_init(dongle_hid_init);

static void __exit dongle_hid_exit(void)
{
	platform_driver_unregister(&dongle_hid_driver);
}
module_exit(dongle_hid_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("ROG dongle EC HID driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:ec_hid");
