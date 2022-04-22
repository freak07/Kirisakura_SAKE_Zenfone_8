/*
 * Copyright (c) 2019-2020, The ASUS Company. All rights reserved.
 */

#define pr_fmt(fmt) "BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/soc/qcom/battery_charger.h>

#include <linux/of_gpio.h>
//ASUS BSP +++
#include "fg-core.h"
#include "asus_battery_charger_ZF8.h"
#include <drm/drm_panel.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>
#include <linux/rtc.h>
//ASUS BSP ---

#include "../../thermal/qcom/adc-tm.h"
#include <dt-bindings/iio/qcom,spmi-vadc.h>
#include <dt-bindings/iio/qcom,spmi-adc7-pm8350.h>
#include <dt-bindings/iio/qcom,spmi-adc7-pm8350b.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/delay.h>

bool g_once_usb_thermal = false;
bool g_asuslib_init = false;
bool g_cos_over_full_flag = false;
volatile int g_ultra_cos_spec_time = 2880;
int  g_charger_mode_full_time = 0;

struct iio_channel *usb_conn_temp_vadc_chan;
static int asus_usb_online = 0;
static struct wakeup_source *slowchg_ws;
static int virtual_thermal = 95;
static bool VID_changed = false;

//[+++] Add the external function
extern int battery_chg_write(struct battery_chg_dev *bcdev, void *data, int len);
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state);
extern int asus_extcon_get_state(struct extcon_dev *edev);
extern bool g_Charger_mode;
//[---] Add the external function

extern void qti_charge_notify_device_charge(void);
extern void qti_charge_notify_device_not_charge(void);

#if defined ASUS_VODKA_PROJECT
void tight_camera_motor(void);
#endif

enum {
    QTI_POWER_SUPPLY_USB_TYPE_HVDCP = 0x80,
    QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3,
    QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5,
};

static const char * const power_supply_usb_type_text[] = {
    "Unknown", "USB", "DCP", "CDP", "ACA", "C",
    "PD", "PD_DRP", "PD_PPS", "BrickID"
};
/* Custom usb_type definitions */
static const char * const qc_power_supply_usb_type_text[] = {
    "HVDCP", "HVDCP_3", "HVDCP_3P5"
};

//ASUS_BSP +++ add to printk the WIFI hotspot & QXDM UTS event
bool g_qxdm_en = false;
bool g_wifi_hs_en = false;
//ASUS_BSP --- add to printk the WIFI hotspot & QXDM UTS event

//ASUS_BSP battery safety upgrade +++
#define CYCLE_COUNT_DATA_MAGIC  0x85
#define CYCLE_COUNT_FILE_NAME   "/batinfo/.bs"
#define BAT_PERCENT_FILE_NAME   "/batinfo/Batpercentage"
#define BAT_SAFETY_FILE_NAME   "/batinfo/bat_safety"
#define CYCLE_COUNT_SD_FILE_NAME   "/sdcard/.bs"
#define BAT_PERCENT_SD_FILE_NAME   "/sdcard/Batpercentage"
#define BAT_CYCLE_SD_FILE_NAME   "/sdcard/Batcyclecount"
#define CYCLE_COUNT_DATA_OFFSET  0x0
#define FILE_OP_READ   0
#define FILE_OP_WRITE   1
#define BATTERY_SAFETY_UPGRADE_TIME 1*60

static bool g_cyclecount_initialized = false;
// extern bool rtc_probe_done;
struct bat_safety_condition{
    unsigned long condition1_battery_time;
    unsigned long condition2_battery_time;
    unsigned long condition3_battery_time;
    unsigned long condition4_battery_time;
    int condition1_cycle_count;
    int condition2_cycle_count;
    unsigned long condition1_temp_vol_time;
    unsigned long condition2_temp_vol_time;
    unsigned long condition1_temp_time;
    unsigned long condition2_temp_time;
    unsigned long condition1_vol_time;
    unsigned long condition2_vol_time;
};

static struct bat_safety_condition safety_cond;

static struct CYCLE_COUNT_DATA g_cycle_count_data = {
    .magic = CYCLE_COUNT_DATA_MAGIC,
    .cycle_count=0,
    .battery_total_time = 0,
    .high_vol_total_time = 0,
    .high_temp_total_time = 0,
    .high_temp_vol_time = 0,
    .reload_condition = 0
};
struct delayed_work battery_safety_work;
struct delayed_work asus_min_check_work;

#if defined ASUS_VODKA_PROJECT
#define INIT_FV 4360
#else
#define INIT_FV 4450
#endif
//ASUS_BSP battery safety upgrade ---

//ASUS_BS battery health upgrade +++
#define BATTERY_HEALTH_UPGRADE_TIME 1 //ASUS_BS battery health upgrade
#define BATTERY_METADATA_UPGRADE_TIME 60 //ASUS_BS battery health upgrade
#define BAT_HEALTH_DATA_OFFSET  0x0
#define BAT_HEALTH_DATA_MAGIC  0x86
#define BAT_HEALTH_DATA_BACKUP_MAGIC 0x87
#define BAT_HEALTH_DATA_FILE_NAME   "/batinfo/bat_health"
#define BAT_HEALTH_DATA_SD_FILE_NAME   "/batinfo/.bh"
#define BAT_HEALTH_START_LEVEL 70
#define BAT_HEALTH_END_LEVEL 100
static bool g_bathealth_initialized = false;
static bool g_bathealth_trigger = false;
static bool g_last_bathealth_trigger = false;
static bool g_health_debug_enable = true;
static bool g_health_upgrade_enable = true;
static int g_health_upgrade_index = 0;
static int g_health_upgrade_start_level = BAT_HEALTH_START_LEVEL;
static int g_health_upgrade_end_level = BAT_HEALTH_END_LEVEL;
static int g_health_upgrade_upgrade_time = BATTERY_HEALTH_UPGRADE_TIME;
static int g_bat_health_avg;

#if defined ASUS_VODKA_PROJECT
#define ZF8_DESIGNED_CAPACITY 4750 //mAh //Design Capcaity *0.95 = 4750
#else
#define ZF8_DESIGNED_CAPACITY 3800 //mAh //Design Capcaity *0.95 = 3800
#endif

extern bool no_input_suspend_flag;

static struct BAT_HEALTH_DATA g_bat_health_data = {
    .magic = BAT_HEALTH_DATA_MAGIC,
    .bat_health = 0,
    .charge_counter_begin = 0,
    .charge_counter_end = 0
};
static struct BAT_HEALTH_DATA_BACKUP g_bat_health_data_backup[BAT_HEALTH_NUMBER_MAX] = {
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0},
    {"", 0}
};
struct delayed_work battery_health_work;
//ASUS_BS battery health upgrade ---

//ASUS_BSP +++ LiJen add to printk the WIFI hotspot & QXDM UTS event
#include <linux/proc_fs.h>
#define uts_status_PROC_FILE	"driver/UTSstatus"
static struct proc_dir_entry *uts_status_proc_file;
static int uts_status_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "WIFIHS:%d, QXDM:%d\n", g_wifi_hs_en, g_qxdm_en);
	return 0;
}

static ssize_t uts_status_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val;
	char messages[8]="";

	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);

	switch (val) {
	case 0:
		printk("%s: WIFI Hotspot disable\n");
		g_wifi_hs_en = false;
		break;
	case 1:
		printk("%s: WIFI Hotspot enable\n");
		g_wifi_hs_en = true;
		break;
	case 2:
		printk("%s: QXDM disable\n");
		g_qxdm_en = false;
		break; 
	case 3:
		printk("%s: QXDM enable\n");
		g_qxdm_en = true;
		break;       
	default:
		printk("%s: Invalid mode\n");
		break;
	}
    
	return len;
}

static int uts_status_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, uts_status_proc_read, NULL);
}

static const struct file_operations uts_status_fops = {
	.owner = THIS_MODULE,
    .open = uts_status_proc_open,
    .read = seq_read,
	.write = uts_status_proc_write,
    .release = single_release,
};

void static create_uts_status_proc_file(void)
{
	uts_status_proc_file = proc_create(uts_status_PROC_FILE, 0666, NULL, &uts_status_fops);

    if (uts_status_proc_file) {
		printk("create_uts_status_proc_file sucessed!\n");
    } else {
	    printk("create_uts_status_proc_file failed!\n");
    }
}
//ASUS_BSP --- LiJen add to printk the WIFI hotspot & QXDM UTS event

#if defined ASUS_VODKA_PROJECT
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
void tight_camera_motor(void)
{
	int filep;
	mm_segment_t old_fs;
	char buf[10];

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filep= ksys_open("/proc/driver/motor_auto", O_RDWR | O_SYNC, 0666);

	if(filep < 0) {
		pr_err("open /proc/driver/motor_auto err! error code:%d\n", filep);
	}
   else
   {
        pr_err("open motor_auto success!\n");
	}
	
	sprintf(buf,"%s", "242");
	
	ksys_write(filep, buf, strlen(buf));
	ksys_sync();
	ksys_close(filep);
	set_fs(old_fs);
}
#endif

static const char *get_usb_type_name(u32 usb_type)
{
    u32 i;

    if (usb_type >= QTI_POWER_SUPPLY_USB_TYPE_HVDCP &&
        usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5) {
        for (i = 0; i < ARRAY_SIZE(qc_power_supply_usb_type_text);
             i++) {
            if (i == (usb_type - QTI_POWER_SUPPLY_USB_TYPE_HVDCP))
                return qc_power_supply_usb_type_text[i];
        }
        return "Unknown";
    }

    for (i = 0; i < ARRAY_SIZE(power_supply_usb_type_text); i++) {
        if (i == usb_type)
            return power_supply_usb_type_text[i];
    }

    return "Unknown";
}

//Panel Check +++
static struct notifier_block fb_notif;
static struct drm_panel *active_panel;
struct delayed_work asus_set_panelonoff_current_work;
int g_drm_blank = 0;

int asus_set_panelonoff_charging_current_limit(u32 panelOn)
{
    int rc;
    u32 tmp = panelOn;

    pr_err("panelOn= 0x%x\n", panelOn);
    ChgPD_Info.panel_status = panelOn;
    rc = oem_prop_write(BATTMAN_OEM_Panel_Check, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_Panel_Check rc=%d\n", rc);
        return rc;
    }
    return 0;
}

static int drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        pr_err("find drm_panel count(%d) fail", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            pr_err("find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

    pr_err("no find drm_panel");

    return -ENODEV;
}

void asus_set_panelonoff_current_worker(struct work_struct *work)
{
    if (g_drm_blank == DRM_PANEL_BLANK_UNBLANK) {
        asus_set_panelonoff_charging_current_limit(true);
    } else if (g_drm_blank == DRM_PANEL_BLANK_POWERDOWN) {
        asus_set_panelonoff_charging_current_limit(false);
    }
}

static int drm_notifier_callback(struct notifier_block *self,
            unsigned long event, void *data)
{
    struct drm_panel_notifier *evdata = data;
    int *blank = NULL;

    if (!evdata) {
        printk("[BAT][CHG]drm_notifier_callback: evdata is null");
        return 0;
    }

    if (!((event == DRM_PANEL_EARLY_EVENT_BLANK)
        || (event == DRM_PANEL_EVENT_BLANK))) {
        pr_err("event(%lu) do not need process", event);
        return 0;
    }

    blank = evdata->data;
    g_drm_blank = *blank;

    switch (*blank) {
    case DRM_PANEL_BLANK_UNBLANK:
        printk("[BAT][CHG] DRM_PANEL_BLANK_UNBLANK,Display on");
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            //pr_debug("resume: event = %lu, not care", event);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            printk("[BAT][CHG] asus_set_panelonoff_charging_current_limit = true");
            schedule_delayed_work(&asus_set_panelonoff_current_work, 0);
        }
        break;
    case DRM_PANEL_BLANK_POWERDOWN:
        printk("[BAT][CHG] DRM_PANEL_BLANK_POWERDOWN,Display off");
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            ;
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            printk("[BAT][CHG] asus_set_panelonoff_charging_current_limit = false");
            schedule_delayed_work(&asus_set_panelonoff_current_work, 0);
        }
        break;
    case DRM_PANEL_BLANK_LP:
        printk("[BAT][CHG] DRM_PANEL_BLANK_LP,Display resume into LP1/LP2");
        break;
    case DRM_PANEL_BLANK_FPS_CHANGE:
        break;
    default:
        break;
    }

    return 0;
}

void RegisterDRMCallback()
{
    int ret = 0;

    pr_err("[BAT][CHG] RegisterDRMCallback");
    ret = drm_check_dt(g_bcdev->dev->of_node);
    if (ret) {
        pr_err("[BAT][CHG] parse drm-panel fail");
    }

    fb_notif.notifier_call = drm_notifier_callback;

    if (active_panel) {
        pr_err("[BAT][CHG] RegisterDRMCallback: registering fb notification");
        ret = drm_panel_notifier_register(active_panel, &fb_notif);
        if (ret)
            pr_err("[BAT][CHG] drm_panel_notifier_register fail: %d", ret);
    }

    return;
}
//Panel Check ---

static int read_property_id(struct battery_chg_dev *bcdev,
            struct psy_state *pst, u32 prop_id)
{
    struct battery_charger_req_msg req_msg = { { 0 } };

    req_msg.property_id = prop_id;
    req_msg.battery_id = 0;
    req_msg.value = 0;
    req_msg.hdr.owner = MSG_OWNER_BC;
    req_msg.hdr.type = MSG_TYPE_REQ_RESP;
    req_msg.hdr.opcode = pst->opcode_get;

    pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
        req_msg.property_id);

    return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

ssize_t oem_prop_read(enum battman_oem_property prop, size_t count)
{
    struct battman_oem_read_buffer_req_msg req_msg = { { 0 } };
    int rc;

    req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
    req_msg.hdr.type = MSG_TYPE_REQ_RESP;
    req_msg.hdr.opcode = OEM_OPCODE_READ_BUFFER;
    req_msg.oem_property_id = prop;
    req_msg.data_size = count;

    rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
    if (rc < 0) {
        pr_err("Failed to read buffer rc=%d\n", rc);
        return rc;
    }

    return count;
}

ssize_t oem_prop_write(enum battman_oem_property prop,
                    u32 *buf, size_t count)
{
    struct battman_oem_write_buffer_req_msg req_msg = { { 0 } };
    int rc;

    req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
    req_msg.hdr.type = MSG_TYPE_REQ_RESP;
    req_msg.hdr.opcode = OEM_OPCODE_WRITE_BUFFER;
    req_msg.oem_property_id = prop;
    memcpy(req_msg.data_buffer, buf, sizeof(u32)*count);
    req_msg.data_size = count;

    if (g_bcdev == NULL) {
        pr_err("g_bcdev is null\n");
        return -1;
    }
    rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
    if (rc < 0) {
        pr_err("Failed to write buffer rc=%d\n", rc);
        return rc;
    }

    return count;
}

int asus_get_Batt_ID(void)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_BATT_ID, 1);
    if (rc < 0) {
        pr_err("Failed to get BattID rc=%d\n", rc);
        return rc;
    }
    return 0;
}

//[+++] Addd the interface for accessing the BATTERY power supply
static ssize_t asus_get_FG_SoC_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    union power_supply_propval prop = {};
    int bat_cap, rc = 0;

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CAPACITY, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery SOC, rc=%d\n", rc);
        return rc;
    }
    bat_cap = prop.intval;
    printk(KERN_ERR "%s. BAT_SOC : %d", __func__, bat_cap);

    return scnprintf(buf, PAGE_SIZE, "%d\n", bat_cap);
}
static CLASS_ATTR_RO(asus_get_FG_SoC);
//[---] Addd the interface for accessing the BATTERY power supply

//[+++] Add the interface for accesing the inforamtion of ChargerPD on ADSP
static ssize_t asus_get_PlatformID_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_ADSP_PLATFORM_ID, 1);
    if (rc < 0) {
        pr_err("Failed to get PlatformID rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.PlatformID);
}
static CLASS_ATTR_RO(asus_get_PlatformID);

static ssize_t asus_get_BattID_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = asus_get_Batt_ID();
    if (rc < 0) {
        pr_err("Failed to get BattID rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.BATT_ID);
}
static CLASS_ATTR_RO(asus_get_BattID);

static ssize_t get_usb_type_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    //struct power_supply *psy;
    struct psy_state *pst;
    int rc = 0 , val = 0;

    rc = oem_prop_read(BATTMAN_OEM_AdapterVID, 1);
    if (rc < 0) {
        pr_err("Failed to get CHG_LIMIT_EN rc=%d\n", rc);
        return rc;
    }

    if(ChgPD_Info.AdapterVID == 0xB05){
        return scnprintf(buf, PAGE_SIZE, "PD_ASUS_PPS_30W_2A\n");
    }

    if (g_bcdev == NULL)
        return -1;
    CHG_DBG("%s\n", __func__);
    pst = &g_bcdev->psy_list[PSY_TYPE_USB];
    rc = read_property_id(g_bcdev, pst, 11);//11:USB_REAL_TYPE
    if (!rc) {
        val = pst->prop[11];//11:USB_REAL_TYPE
        CHG_DBG("%s. val : %d\n", __func__, val);
    }

    return scnprintf(buf, PAGE_SIZE, "%s\n", get_usb_type_name(val));
}
static CLASS_ATTR_RO(get_usb_type);

static ssize_t charger_limit_en_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;

    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. enable : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHG_LIMIT_EN, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set CHG_LIMIT_EN rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t charger_limit_en_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHG_LIMIT_EN, 1);
    if (rc < 0) {
        pr_err("Failed to get CHG_LIMIT_EN rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.chg_limit_en);
}
static CLASS_ATTR_RW(charger_limit_en);

static ssize_t charger_limit_cap_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;

    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. cap : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHG_LIMIT_CAP, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set CHG_LIMIT_CAP rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t charger_limit_cap_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHG_LIMIT_CAP, 1);
    if (rc < 0) {
        pr_err("Failed to get CHG_LIMIT_CAP rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.chg_limit_cap);
}
static CLASS_ATTR_RW(charger_limit_cap);

static ssize_t usbin_suspend_en_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    rc = oem_prop_write(BATTMAN_OEM_USBIN_SUSPEND, &tmp, 1);

    pr_err("%s. enable : %d", __func__, tmp);
    if (rc < 0) {
        pr_err("Failed to set USBIN_SUSPEND_EN rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t usbin_suspend_en_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_USBIN_SUSPEND, 1);
    if (rc < 0) {
        pr_err("Failed to get USBIN_SUSPEND_EN rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.usbin_suspend_en);
}
static CLASS_ATTR_RW(usbin_suspend_en);

static ssize_t charging_suspend_en_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. enable : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHARGING_SUSPNED, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set CHARGING_SUSPEND_EN rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t charging_suspend_en_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHARGING_SUSPNED, 1);
    if (rc < 0) {
        pr_err("Failed to get CHARGING_SUSPEND_EN rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.charging_suspend_en);
}
static CLASS_ATTR_RW(charging_suspend_en);

static ssize_t get_ChgPD_FW_Ver_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHGPD_FW_VER, 32);
    if (rc < 0) {
        pr_err("%s. Failed to get ChgPD_FW_Ver rc=%d\n", __func__, rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%s\n", ChgPD_Info.ChgPD_FW);
}
static CLASS_ATTR_RO(get_ChgPD_FW_Ver);

static ssize_t asus_get_fw_version_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_FW_VERSION, 1);
    if (rc < 0) {
        pr_err("Failed to get FW_version rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "0x00%x\n", ChgPD_Info.firmware_version);
}
static CLASS_ATTR_RO(asus_get_fw_version);

static ssize_t asus_get_batt_temp_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_BATT_TEMP, 1);
    if (rc < 0) {
        pr_err("Failed to get Batt_temp rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.batt_temp);
}
static CLASS_ATTR_RO(asus_get_batt_temp);

static ssize_t enter_ship_mode_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
	int tmp;
    bool ship_en;
    tmp = simple_strtol(buf, NULL, 10);
    ship_en = tmp;

    if (ship_en == 0) {
        CHG_DBG_E("%s. NO action for SHIP mode\n", __func__);
        return count;
    }

    CHG_DBG_E("%s. write enter_ship_mode %d \n", __func__,ship_en);

#if defined ASUS_VODKA_PROJECT
	tight_camera_motor();
#endif

	schedule_delayed_work(&g_bcdev->enter_ship_work, 0);

    return count;
}

static ssize_t enter_ship_mode_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(enter_ship_mode);


static ssize_t write_pm8350b_register_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
	int tmp[2];
	int rc;
	char *first;
	char *second;
	char *write_data ;
	char messages[10];

	pr_err("write_pm8350b_register_store %s count %d ",buf,count);
	
	messages[0] = *buf;
	messages[1] = *(buf+1);
	messages[2] = *(buf+2);
	messages[3] = *(buf+3);
	messages[4] = ' ';
	messages[5] = *(buf+5);
	messages[6] = *(buf+6);
	messages[7] = '\0';

	write_data = messages;

	first = strsep(&write_data," ");	
	second	= write_data;
	rc = kstrtoint(first,16,&tmp[0]);//字符串转整形 10:十进制
	if(rc)	
	{
		pr_err("error in kstrtoint");
		return -1;
	}
	
	rc = kstrtoint(second,16,&tmp[1]);
	if(rc)
	{
		return -1;
	}
    
	pr_err("address %x , data %x \n", tmp[0] , tmp[1]);
    
	rc = oem_prop_write(BATTMAN_OEM_Write_PM8350B_Register, tmp, 2);
	if (rc < 0) {
			pr_err("Failed to set BATTMAN_OEM_Panel_Check rc=%d\n", rc);
			return rc;
	}
	return count;
}
static ssize_t write_pm8350b_register_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(write_pm8350b_register);

static ssize_t once_usb_thermal_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	if (g_once_usb_thermal)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "PASS\n");
}
static CLASS_ATTR_RO(once_usb_thermal);

static ssize_t pm8350b_icl_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. set BATTMAN_OEM_PM8350B_ICL : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_PM8350B_ICL, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_PM8350B_ICL rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t pm8350b_icl_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_PM8350B_ICL, 1);
    if (rc < 0) {
        pr_err("Failed to get BATTMAN_OEM_PM8350B_ICL rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.pm8350b_icl);
}
static CLASS_ATTR_RW(pm8350b_icl);

static ssize_t smb1396_icl_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. set BATTMAN_OEM_SMB1396_ICL : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_SMB1396_ICL, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_SMB1396_ICL rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t smb1396_icl_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_SMB1396_ICL, 1);
    if (rc < 0) {
        pr_err("Failed to get BATTMAN_OEM_SMB1396_ICL rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.smb1396_icl);
}
static CLASS_ATTR_RW(smb1396_icl);

static ssize_t batt_FCC_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. set BATTMAN_OEM_FCC : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_FCC, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_FCC rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t batt_FCC_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_FCC, 1);
    if (rc < 0) {
        pr_err("Failed to get BATTMAN_OEM_FCC rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.batt_fcc);
}
static CLASS_ATTR_RW(batt_FCC);

static ssize_t smartchg_slow_charging_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    u32 tmp;

    sscanf(buf, "%d", &tmp);
    ChgPD_Info.slow_chglimit = tmp;

    CHG_DBG("%s. slow charging : %d", __func__, tmp);
    if(asus_usb_online){
        cancel_delayed_work_sync(&asus_slow_charging_work);
        schedule_delayed_work(&asus_slow_charging_work, 0 * HZ);
        __pm_wakeup_event(slowchg_ws, 60 * 1000);
    }

    return count;
}

static ssize_t smartchg_slow_charging_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.slow_chglimit);
}
static CLASS_ATTR_RW(smartchg_slow_charging);

static ssize_t set_debugmask_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = (u32) simple_strtol(buf, NULL, 16);

    CHG_DBG("%s. set BATTMAN_OEM_DEBUG_MASK : 0x%x", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_DEBUG_MASK, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_DEBUG_MASK rc=%d\n", rc);
        return rc;
    }

    return count;
}
static CLASS_ATTR_WO(set_debugmask);

static ssize_t smb_setting_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. set BATTMAN_OEM_SMB_Setting : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_SMB_Setting, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_SMB_Setting rc=%d\n", rc);
        return rc;
    }

    return count;
}
static CLASS_ATTR_WO(smb_setting);

static ssize_t demo_app_status_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);
    ChgPD_Info.demo_app_status = tmp;

    tmp = ChgPD_Info.ultra_bat_life|ChgPD_Info.demo_app_status;
    CHG_DBG("%s. set BATTMAN_OEM_Batt_Protection : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_Batt_Protection, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_Batt_Protection rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t demo_app_status_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ChgPD_Info.demo_app_status);
}
static CLASS_ATTR_RW(demo_app_status);

static ssize_t ultra_bat_life_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);
    ChgPD_Info.ultra_bat_life = tmp;

    tmp = ChgPD_Info.ultra_bat_life||ChgPD_Info.demo_app_status;
    CHG_DBG("%s. set BATTMAN_OEM_Batt_Protection : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_Batt_Protection, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_Batt_Protection rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t ultra_bat_life_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.ultra_bat_life);
}
static CLASS_ATTR_RW(ultra_bat_life);

static ssize_t chg_disable_jeita_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);
    ChgPD_Info.chg_disable_jeita = tmp;

    CHG_DBG_E("%s. set BATTMAN_OEM_CHG_Disable_Jeita : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHG_Disable_Jeita, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_CHG_Disable_Jeita rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t chg_disable_jeita_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.chg_disable_jeita);
}
static CLASS_ATTR_RW(chg_disable_jeita);

static ssize_t virtual_thermal_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    virtual_thermal = tmp;

    return count;
}

static ssize_t virtual_thermal_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", virtual_thermal);
}
static CLASS_ATTR_RW(virtual_thermal);

static ssize_t set_virtualthermal_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int mask;
    mask = simple_strtol(buf, NULL, 16);
    ChgPD_Info.thermel_threshold = mask;
    CHG_DBG_E("%s thermel threshold=%d", __func__, mask);

    return count;
}

static ssize_t set_virtualthermal_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_virtualthermal);

static ssize_t boot_completed_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    ChgPD_Info.boot_completed = tmp;

    cancel_delayed_work_sync(&asus_set_qc_state_work);
    schedule_delayed_work(&asus_set_qc_state_work, msecs_to_jiffies(5000));

    return count;
}

static ssize_t boot_completed_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.boot_completed);
}
static CLASS_ATTR_RW(boot_completed);

static ssize_t in_call_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);
    ChgPD_Info.in_call = tmp;

    CHG_DBG_E("%s. set BATTMAN_OEM_In_Call : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_In_Call, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_In_Call rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t in_call_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.in_call);
}
static CLASS_ATTR_RW(in_call);

static struct attribute *asuslib_class_attrs[] = {
    &class_attr_asus_get_FG_SoC.attr,
    &class_attr_asus_get_PlatformID.attr,
    &class_attr_asus_get_BattID.attr,
    &class_attr_get_usb_type.attr,
    &class_attr_charger_limit_en.attr,
    &class_attr_charger_limit_cap.attr,
    &class_attr_usbin_suspend_en.attr,
    &class_attr_charging_suspend_en.attr,
    &class_attr_get_ChgPD_FW_Ver.attr,
    &class_attr_asus_get_fw_version.attr,
    &class_attr_asus_get_batt_temp.attr,
    &class_attr_enter_ship_mode.attr,
    &class_attr_once_usb_thermal.attr,
    &class_attr_pm8350b_icl.attr,
    &class_attr_smb1396_icl.attr,
    &class_attr_batt_FCC.attr,
    &class_attr_set_debugmask.attr,
    &class_attr_smb_setting.attr,
    &class_attr_write_pm8350b_register.attr,
    &class_attr_smartchg_slow_charging.attr,
    &class_attr_demo_app_status.attr,
    &class_attr_ultra_bat_life.attr,
    &class_attr_chg_disable_jeita.attr,
    &class_attr_virtual_thermal.attr,
    &class_attr_set_virtualthermal.attr,
    &class_attr_boot_completed.attr,
    &class_attr_in_call.attr,
    NULL,
};
ATTRIBUTE_GROUPS(asuslib_class);

struct class asuslib_class = {
    .name = "asuslib",
    .class_groups = asuslib_class_groups,
};

int g_temp_Triger = 70000;
int g_temp_Release = 60000;

void asus_usb_thermal_worker(struct work_struct *work)
{
    int rc;
    u32 tmp;
    int conn_temp;

    rc = iio_read_channel_processed(usb_conn_temp_vadc_chan, &conn_temp);
    if (rc < 0)
        CHG_DBG_E("%s: iio_read_channel_processed fail\n", __func__);
    else
        CHG_DBG_E("%s: usb_conn_temp = %d\n", __func__, conn_temp);

    if (conn_temp > g_temp_Triger) {
        if(ChgPD_Info.usb_present){
            tmp = THERMAL_ALERT_WITH_AC;
            CHG_DBG("%s. set BATTMAN_OEM_THERMAL_ALERT : %d", __func__, tmp);
            rc = oem_prop_write(BATTMAN_OEM_THERMAL_ALERT, &tmp, 1);
            if (rc < 0) {
                pr_err("Failed to set BATTMAN_OEM_THERMAL_ALERT rc=%d\n", rc);
            }
            asus_extcon_set_state_sync(thermal_extcon, THERMAL_ALERT_WITH_AC);
        }else{
            tmp = THERMAL_ALERT_NO_AC;
            CHG_DBG("%s. set BATTMAN_OEM_THERMAL_ALERT : %d", __func__, tmp);
            rc = oem_prop_write(BATTMAN_OEM_THERMAL_ALERT, &tmp, 1);
            if (rc < 0) {
                pr_err("Failed to set BATTMAN_OEM_THERMAL_ALERT rc=%d\n", rc);
            }
            asus_extcon_set_state_sync(thermal_extcon, THERMAL_ALERT_NO_AC);
        }

        g_once_usb_thermal = true;
        CHG_DBG("conn_temp(%d) >= 700, usb thermal alert\n", conn_temp);
    }else if(!ChgPD_Info.usb_present && conn_temp < g_temp_Release){
        tmp = THERMAL_ALERT_NONE;
        CHG_DBG("%s. set BATTMAN_OEM_THERMAL_ALERT : %d", __func__, tmp);
        rc = oem_prop_write(BATTMAN_OEM_THERMAL_ALERT, &tmp, 1);
        if (rc < 0) {
            pr_err("Failed to set BATTMAN_OEM_THERMAL_ALERT rc=%d\n", rc);
        }
        g_once_usb_thermal = false;

        asus_extcon_set_state_sync(thermal_extcon, THERMAL_ALERT_NONE);
        CHG_DBG("conn_temp(%d) <= 600, disable usb suspend\n", conn_temp);
    }

    CHG_DBG("conn_temp(%d), usb(%d), otg(%d), usb_connector = %d\n",
            conn_temp, ChgPD_Info.usb_present, ChgPD_Info.otg_enable, asus_extcon_get_state(thermal_extcon));

    schedule_delayed_work(&g_bcdev->asus_usb_thermal_work, msecs_to_jiffies(60000));
}

void asus_thermal_policy_worker(struct work_struct *work)
{
    int rc;
    u32 tmp;

    tmp = ChgPD_Info.thermel_threshold;
    CHG_DBG("%s. set BATTMAN_OEM_THERMAL_THRESHOLD : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_THERMAL_THRESHOLD, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_THERMAL_THRESHOLD rc=%d\n", rc);
    }

    schedule_delayed_work(&asus_thermal_policy_work, 10 * HZ);
}

int asus_init_power_supply_prop(void) {

    // Initialize the power supply for usb properties
    if (!qti_phy_usb)
        qti_phy_usb = power_supply_get_by_name("usb");

    if (!qti_phy_usb) {
        pr_err("Failed to get usb power supply, rc=%d\n");
        return -ENODEV;
    }

    // Initialize the power supply for battery properties
    if (!qti_phy_bat)
        qti_phy_bat = power_supply_get_by_name("battery");

    if (!qti_phy_bat) {
        pr_err("Failed to get battery power supply, rc=%d\n");
        return -ENODEV;
    }
    return 0;
};

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
                size_t len)
{
    struct evtlog_context_resp_msg3 *evtlog_msg;
    struct oem_enable_change_msg *enable_change_msg;
    struct oem_set_Charger_Type_resp *Update_charger_type_msg;
    struct asus_notify_work_event_msg *work_event_msg;
    struct oem_asus_adaptervid_msg *adaptervid_msg;
    struct oem_jeita_cc_state_msg *jeita_cc_state_msg;
    struct pmic_glink_hdr *hdr = data;
    int rc;
    static int pre_chg_type = 0;

    switch(hdr->opcode) {
    case OEM_ASUS_EVTLOG_IND:
        if (len == sizeof(*evtlog_msg)) {
            evtlog_msg = data;
            pr_err("[adsp] evtlog= %s\n", evtlog_msg->buf);
        }
        break;
    case OEM_PD_EVTLOG_IND:
        if (len == sizeof(*evtlog_msg)) {
            evtlog_msg = data;
            pr_err("[PD] %s\n", evtlog_msg->buf);
        }
        break;
    case OEM_SET_OTG_WA:
        if (len == sizeof(*enable_change_msg)) {
            enable_change_msg = data;
            CHG_DBG("%s OEM_SET_OTG_WA. enable : %d, HWID : %d\n", __func__, enable_change_msg->enable, g_ASUS_hwID);
            ChgPD_Info.otg_enable = enable_change_msg->enable;
            if(g_ASUS_hwID <= HW_REV_EVB2) {
                if (gpio_is_valid(POGO_OTG_GPIO)) {
                    rc = gpio_direction_output(POGO_OTG_GPIO, enable_change_msg->enable);
                    if (rc)
                        pr_err("%s. Failed to control POGO_OTG_EN\n", __func__);
                } else {
                    CHG_DBG_E("%s. POGO_OTG_GPIO is invalid\n", __func__);
                }
            }

            if(g_ASUS_hwID >= HW_REV_ER) {
                if (gpio_is_valid(OTG_LOAD_SWITCH_GPIO)) {
                    rc = gpio_direction_output(OTG_LOAD_SWITCH_GPIO, enable_change_msg->enable);
                    if (rc)
                        pr_err("%s. Failed to control OTG_Load_Switch\n", __func__);
                } else {
                    CHG_DBG_E("%s. OTG_LOAD_SWITCH_GPIO is invalid\n", __func__);
                }
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_OTG_WA\n",
                len);
        }
        break;
    case OEM_USB_PRESENT:
        if (len == sizeof(*enable_change_msg)) {
            enable_change_msg = data;
            CHG_DBG("%s OEM_USB_PRESENT enable : %d\n", __func__, enable_change_msg->enable);
            ChgPD_Info.usb_present = enable_change_msg->enable;
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_OTG_WA\n",
                len);
        }
        break;
    case OEM_SET_CHARGER_TYPE_CHANGE:
        if (len == sizeof(*Update_charger_type_msg)) {
            Update_charger_type_msg = data;
            CHG_DBG("%s OEM_SET_CHARGER_TYPE_CHANGE. new type : %d, old type : %d\n", __func__, Update_charger_type_msg->charger_type, pre_chg_type);
            if (Update_charger_type_msg->charger_type != pre_chg_type) {
                switch (Update_charger_type_msg->charger_type) {
                case ASUS_CHARGER_TYPE_LEVEL0:
                    g_SWITCH_LEVEL = SWITCH_LEVEL0_DEFAULT;
                break;
                case ASUS_CHARGER_TYPE_LEVEL1:
                    g_SWITCH_LEVEL = SWITCH_LEVEL2_QUICK_CHARGING;
                break;
                case ASUS_CHARGER_TYPE_LEVEL2:
                    g_SWITCH_LEVEL = SWITCH_LEVEL3_QUICK_CHARGING;
                break;
                case ASUS_CHARGER_TYPE_LEVEL3:
                    g_SWITCH_LEVEL = SWITCH_LEVEL4_QUICK_CHARGING;
                break;
                default:
                    g_SWITCH_LEVEL = SWITCH_LEVEL0_DEFAULT;
                break;
                }
                if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && qti_phy_bat)
                    power_supply_changed(qti_phy_bat);
                cancel_delayed_work_sync(&asus_set_qc_state_work);
                if(g_Charger_mode){
                    schedule_delayed_work(&asus_set_qc_state_work, msecs_to_jiffies(0));
                }else{
                    schedule_delayed_work(&asus_set_qc_state_work, msecs_to_jiffies(100));
                }
            }
            pre_chg_type = Update_charger_type_msg->charger_type;
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_CHARGER_TYPE_CHANGE\n",
                len);
        }
        break;
    case OEM_ASUS_WORK_EVENT_REQ:
        if (len == sizeof(*work_event_msg)) {
            work_event_msg = data;
            CHG_DBG_E("%s OEM_ASUS_WORK_EVENT_REQ. work=%d, enable=%d\n", __func__, work_event_msg->work, work_event_msg->data_buffer[0]);
            if(work_event_msg->work == WORK_JEITA_PRECHG){
                if(work_event_msg->data_buffer[0] == 1){
                    cancel_delayed_work_sync(&asus_jeita_prechg_work);
                    schedule_delayed_work(&asus_jeita_prechg_work, 0);
                }else{
                    cancel_delayed_work_sync(&asus_jeita_prechg_work);
                }
            }else if(work_event_msg->work == WORK_JEITA_CC){
                if(work_event_msg->data_buffer[0] == 1){
                    cancel_delayed_work_sync(&asus_jeita_cc_work);
                    schedule_delayed_work(&asus_jeita_cc_work, 5 * HZ);
                }else{
                    cancel_delayed_work_sync(&asus_jeita_cc_work);
                }
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_ASUS_WORK_EVENT_REQ\n",
                len);
        }
        break;
    case OEM_ASUS_AdapterVID_REQ:
        if (len == sizeof(*adaptervid_msg)) {
            adaptervid_msg = data;
            CHG_DBG("%s AdapterVID enable : %d\n", __func__, adaptervid_msg->VID);
            ChgPD_Info.AdapterVID = adaptervid_msg->VID;
            asus_extcon_set_state_sync(adaptervid_extcon, ChgPD_Info.AdapterVID);
        } else {
            pr_err("Incorrect response length %zu for OEM_ASUS_AdapterVID_REQ\n",
                len);
        }
        break;
    case OEM_JEITA_CC_STATE_REQ:
        if (len == sizeof(*jeita_cc_state_msg)) {
            jeita_cc_state_msg = data;
            CHG_DBG("%s jeita cc state : %d\n", __func__, jeita_cc_state_msg->state);
            ChgPD_Info.jeita_cc_state = jeita_cc_state_msg->state;
        } else {
            pr_err("Incorrect response length %zu for OEM_JEITA_CC_STATE_REQ\n",
                len);
        }
        break;
    default:
        pr_err("Unknown opcode: %u\n", hdr->opcode);
        break;
    }

}

static void handle_message(struct battery_chg_dev *bcdev, void *data,
                size_t len)
{
    struct battman_oem_read_buffer_resp_msg *oem_read_buffer_resp_msg;
    struct battman_oem_write_buffer_resp_msg *oem_write_buffer_resp_msg;

    struct pmic_glink_hdr *hdr = data;
    bool ack_set = false;
    
    switch (hdr->opcode) {
    case OEM_OPCODE_READ_BUFFER:
        if (len == sizeof(*oem_read_buffer_resp_msg)) {
            oem_read_buffer_resp_msg = data;
            switch (oem_read_buffer_resp_msg->oem_property_id) {
            case BATTMAN_OEM_ADSP_PLATFORM_ID:
                ChgPD_Info.PlatformID = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_BATT_ID:
                ChgPD_Info.BATT_ID = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                if(ChgPD_Info.BATT_ID < 51000*1.15 && ChgPD_Info.BATT_ID > 51000*0.85)
                    asus_extcon_set_state_sync(bat_id_extcon, 1);
                else if(ChgPD_Info.BATT_ID < 100000*1.15 && ChgPD_Info.BATT_ID > 100000*0.85)
                    asus_extcon_set_state_sync(bat_id_extcon, 1);
                else
                    asus_extcon_set_state_sync(bat_id_extcon, 0);
                break;
            case BATTMAN_OEM_CHG_LIMIT_EN:
                CHG_DBG("%s BATTMAN_OEM_CHG_LIMIT_EN successfully\n", __func__);
                ChgPD_Info.chg_limit_en = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_CHG_LIMIT_CAP:
                CHG_DBG("%s BATTMAN_OEM_CHG_LIMIT_CAP successfully\n", __func__);
                ChgPD_Info.chg_limit_cap = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_USBIN_SUSPEND:
                ChgPD_Info.usbin_suspend_en = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_CHARGING_SUSPNED:
                CHG_DBG("%s BATTMAN_OEM_CHARGING_SUSPNED successfully\n", __func__);
                ChgPD_Info.charging_suspend_en = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_CHGPD_FW_VER:
                CHG_DBG("%s. ChgPD_FW : %s\n", __func__, (char*)oem_read_buffer_resp_msg->data_buffer);
                strcpy(ChgPD_Info.ChgPD_FW, (char*)oem_read_buffer_resp_msg->data_buffer);
                ack_set = true;
                break;
            case BATTMAN_OEM_FW_VERSION:
                ChgPD_Info.firmware_version = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_BATT_TEMP:
                ChgPD_Info.batt_temp = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_PM8350B_ICL:
                ChgPD_Info.pm8350b_icl = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_SMB1396_ICL:
                ChgPD_Info.smb1396_icl = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_FCC:
                ChgPD_Info.batt_fcc = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_AdapterVID:
                ChgPD_Info.AdapterVID = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            default:
                ack_set = true;
                pr_err("Unknown property_id: %u\n", oem_read_buffer_resp_msg->oem_property_id);
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_OPCODE_READ_BUFFER\n", len);
        }
        break;
    case OEM_OPCODE_WRITE_BUFFER:
        if (len == sizeof(*oem_write_buffer_resp_msg)) {
            oem_write_buffer_resp_msg = data;
            switch (oem_write_buffer_resp_msg->oem_property_id) {
            case BATTMAN_OEM_CHG_LIMIT_EN:
            case BATTMAN_OEM_CHG_LIMIT_CAP:
            case BATTMAN_OEM_USBIN_SUSPEND:
            case BATTMAN_OEM_CHARGING_SUSPNED:
            case BATTMAN_OEM_PM8350B_ICL:
            case BATTMAN_OEM_SMB1396_ICL:
            case BATTMAN_OEM_FCC:
            case BATTMAN_OEM_DEBUG_MASK:
            case BATTMAN_OEM_SMB_Setting:
            case BATTMAN_OEM_Panel_Check:
            case BATTMAN_OEM_WORK_EVENT:
            case BATTMAN_OEM_THERMAL_ALERT:
            case BATTMAN_OEM_Write_PM8350B_Register:
            case BATTMAN_OEM_Slow_Chg:
            case BATTMAN_OEM_Batt_Protection:
            case BATTMAN_OEM_CHG_Disable_Jeita:
            case BATTMAN_OEM_CHG_MODE:
            case BATTMAN_OEM_THERMAL_THRESHOLD:
            case BATTMAN_OEM_THERMAL_SENSOR:
            case BATTMAN_OEM_FV:
            case BATTMAN_OEM_In_Call:
                CHG_DBG("%s set property:%d successfully\n", __func__, oem_write_buffer_resp_msg->oem_property_id);
                ack_set = true;
                break;
            default:
                ack_set = true;
                pr_err("Unknown property_id: %u\n", oem_write_buffer_resp_msg->oem_property_id);
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_OPCODE_READ_BUFFER\n", len);
        }
        break;
    default:
        pr_err("Unknown opcode: %u\n", hdr->opcode);
        ack_set = true;
        break;
    }

    if (ack_set)
        complete(&bcdev->ack);
}

static int asusBC_msg_cb(void *priv, void *data, size_t len)
{
    struct pmic_glink_hdr *hdr = data;

    // pr_err("owner: %u type: %u opcode: %u len: %zu\n", hdr->owner, hdr->type, hdr->opcode, len);

    if (hdr->owner == PMIC_GLINK_MSG_OWNER_OEM) {
        if (hdr->type == MSG_TYPE_NOTIFY)
            handle_notification(g_bcdev, data, len);
        else
            handle_message(g_bcdev, data, len);
    }
    return 0;
}

static void asusBC_state_cb(void *priv, enum pmic_glink_state state)
{
    pr_err("Enter asusBC_state_cb\n");
}

static char *charging_stats[] = {
	"UNKNOWN",
	"CHARGING",
	"DISCHARGING",
	"NOT_CHARGING",
	"FULL"
};
char *health_type[] = {
	"UNKNOWN",
	"GOOD",
	"OVERHEAT",
	"DEAD",
	"OVERVOLTAGE",
	"UNSPEC_FAILURE",
	"COLD",
	"WATCHDOG_TIMER_EXPIRE",
	"SAFETY_TIMER_EXPIRE",
	"OVERCURRENT",
	"CALIBRATION_REQUIRED",
	"WARM",
	"COOL",
	"HOT"
};
static struct timespec64   g_last_print_time;
static void print_battery_status(void) {
	union power_supply_propval prop = {};
	char battInfo[256];
    int bat_cap, fcc,bat_vol,bat_cur,bat_temp,charge_status,bat_health,rc = 0;
    char UTSInfo[256]; //ASUS_BSP add to printk the WIFI hotspot & QXDM UTS event

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CAPACITY, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery SOC, rc=%d\n", rc);
    }
    bat_cap = prop.intval;

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery full design, rc=%d\n", rc);
    }
    fcc = prop.intval;
    
    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery vol , rc=%d\n", rc);
    }
    bat_vol = prop.intval;
    
    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery current , rc=%d\n", rc);
    }
    bat_cur= prop.intval;
    
    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_TEMP, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery temp , rc=%d\n", rc);
    }
    bat_temp= prop.intval;
    
    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_STATUS, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery status , rc=%d\n", rc);
    }
    charge_status= prop.intval;

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_HEALTH, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery health , rc=%d\n", rc);
    }
    bat_health= prop.intval;

	snprintf(battInfo, sizeof(battInfo), "report Capacity ==>%d, FCC:%dmAh, BMS:%d, V:%dmV, Cur:%dmA, ",
			bat_cap,
			fcc/1000,
			bat_cap,
			bat_vol/1000,
			bat_cur/1000);
	snprintf(battInfo, sizeof(battInfo), "%sTemp:%d.%dC, BATID:%d, CHG_Status:%d(%s), BAT_HEALTH:%s \n",
			battInfo,
			bat_temp/10,
			bat_temp%10,
			ChgPD_Info.BATT_ID,
			charge_status,
			charging_stats[charge_status],
			health_type[bat_health]);

	ASUSEvtlog("[BAT][Ser]%s", battInfo);
	
	//ASUS_BSP +++ add to printk the WIFI hotspot & QXDM UTS event
	snprintf(UTSInfo, sizeof(UTSInfo), "WIFI_HS=%d, QXDM=%d", g_wifi_hs_en, g_qxdm_en);
	ASUSEvtlog("[UTS][Status]%s", UTSInfo);
	//ASUS_BSP --- add to printk the WIFI hotspot & QXDM UTS event
	
	ktime_get_coarse_real_ts64(&g_last_print_time);
	schedule_delayed_work(&g_bcdev->update_gauge_status_work, 180*HZ);
}

void static update_gauge_status_worker(struct work_struct *dat)
{
	print_battery_status();
}

int asus_chg_resume(struct device *dev)
{
    struct timespec64 mtNow;

    ktime_get_coarse_real_ts64(&mtNow);
    if (mtNow.tv_sec - g_last_print_time.tv_sec >= 180) {
			cancel_delayed_work_sync(&g_bcdev->update_gauge_status_work);
            schedule_delayed_work(&g_bcdev->update_gauge_status_work, 0);
    }
	return 0;
}

//ASUS BSP : Show "+" on charging icon +++
void asus_set_qc_state_worker(struct work_struct *work)
{
	int rc;

    rc = oem_prop_read(BATTMAN_OEM_AdapterVID, 1);
    if (rc < 0)
    {
        pr_err("Failed to get CHG_LIMIT_EN rc=%d\n", rc);
    }

    CHG_DBG("%s: VID=%d, level=%d, boot=%d\n", __func__, ChgPD_Info.AdapterVID, g_SWITCH_LEVEL, ChgPD_Info.boot_completed);
    if(ChgPD_Info.AdapterVID == 0xB05 && !VID_changed &&
        g_SWITCH_LEVEL == SWITCH_LEVEL3_QUICK_CHARGING && ChgPD_Info.boot_completed){
        CHG_DBG_E("%s:  report 30W pps \n", __func__);
        VID_changed = true;
        asus_extcon_set_state_sync(quickchg_extcon, 101);//ASUS 30W
        mdelay(10);
    }

    asus_extcon_set_state_sync(quickchg_extcon, g_SWITCH_LEVEL);

    CHG_DBG_E("%s: switch : %d, VID_changed=%d\n", __func__, asus_extcon_get_state(quickchg_extcon), VID_changed);
}

static int pre_batt_status = -1;
void set_qc_stat(int status)
{
    if(!g_asuslib_init) return;
    if(status == pre_batt_status) return;

    CHG_DBG_E("%s: status: %d\n", __func__, status);
    pre_batt_status = status;
    switch (status) {
    //"qc" stat happends in charger mode only, refer to smblib_get_prop_batt_status
    case POWER_SUPPLY_STATUS_CHARGING:
    case POWER_SUPPLY_STATUS_NOT_CHARGING:
    case POWER_SUPPLY_STATUS_FULL:
        cancel_delayed_work_sync(&asus_set_qc_state_work);
        if(g_Charger_mode){
            schedule_delayed_work(&asus_set_qc_state_work, msecs_to_jiffies(0));
        }else{
            schedule_delayed_work(&asus_set_qc_state_work, msecs_to_jiffies(2000));
        }
        break;
    default:
        cancel_delayed_work_sync(&asus_set_qc_state_work);
        asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL0_DEFAULT);
        break;
    }
}
//ASUS BSP : Show "+" on charging icon ---

void static enter_ship_mode_worker(struct work_struct *dat)
{
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	int rc,usb_online;
    union power_supply_propval prop = {};

    msg.hdr.owner = MSG_OWNER_BC;
    msg.hdr.type = MSG_TYPE_REQ_RESP;
    msg.hdr.opcode = 0x36;// = BC_SHIP_MODE_REQ_SET
    msg.ship_mode_type = 0;// = SHIP_MODE_PMIC

	while(1)
	{
		rc = power_supply_get_property(qti_phy_usb,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		if (rc < 0) {
			pr_err("Failed to get usb  online, rc=%d\n", rc);
		}
		usb_online = prop.intval;
		if (usb_online == 0 )
			break;
		mdelay(1000);
	}

	CHG_DBG_E("%s. usb plug out ,begin to set shoip mode\n", __func__);
    rc = battery_chg_write(g_bcdev, &msg, sizeof(msg));
    if (rc < 0)
        pr_err("%s. Failed to write SHIP mode: %d\n", rc);
}

static void asus_jeita_rule_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = WORK_JEITA_RULE;
    printk(KERN_ERR "[BAT][CHG]%s set BATTMAN_OEM_WORK_EVENT : WORK_JEITA_RULE", __func__);
    rc = oem_prop_write(BATTMAN_OEM_WORK_EVENT, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_WORK_EVENT WORK_JEITA_RULE rc=%d\n", rc);
    }

    schedule_delayed_work(&asus_jeita_rule_work, 60 * HZ);
    if(ChgPD_Info.jeita_cc_state > JETA_NONE && ChgPD_Info.jeita_cc_state < JETA_CV){
        __pm_wakeup_event(slowchg_ws, 60 * 1000);
    }
}

static void asus_jeita_prechg_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = WORK_JEITA_PRECHG;
    printk(KERN_ERR "[BAT][CHG]%s set BATTMAN_OEM_WORK_EVENT : WORK_JEITA_PRECHG", __func__);
    rc = oem_prop_write(BATTMAN_OEM_WORK_EVENT, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_WORK_EVENT WORK_JEITA_PRECHG rc=%d\n", rc);
    }

    schedule_delayed_work(&asus_jeita_prechg_work, HZ);
}

static void asus_jeita_cc_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = WORK_JEITA_CC;
    printk(KERN_ERR "[BAT][CHG]%s set BATTMAN_OEM_WORK_EVENT : WORK_JEITA_CC", __func__);
    rc = oem_prop_write(BATTMAN_OEM_WORK_EVENT, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_WORK_EVENT WORK_JEITA_CC rc=%d\n", rc);
    }

    schedule_delayed_work(&asus_jeita_cc_work, 5 * HZ);
}

static void asus_panel_check_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = WORK_PANEL_CHECK;
    printk(KERN_ERR "[BAT][CHG]%s set BATTMAN_OEM_WORK_EVENT : WORK_PANEL_CHECK", __func__);
    rc = oem_prop_write(BATTMAN_OEM_WORK_EVENT, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_WORK_EVENT WORK_PANEL_CHECK rc=%d\n", rc);
    }

    schedule_delayed_work(&asus_panel_check_work, 10 * HZ);
}

static void asus_slow_charging_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = ChgPD_Info.slow_chglimit;
    CHG_DBG("%s. set BATTMAN_OEM_Slow_Chg : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_Slow_Chg, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_Slow_Chg rc=%d\n", rc);
    }
}

static void asus_charger_mode_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = g_Charger_mode;
    CHG_DBG("%s. set BATTMAN_OEM_CHG_MODE : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHG_MODE, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_CHG_MODE rc=%d\n", rc);
    }
}

static void asus_long_full_cap_monitor_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = WORK_LONG_FULL_CAP;
    CHG_DBG("[BAT][CHG]%s set BATTMAN_OEM_WORK_EVENT : WORK_LONG_FULL_CAP", __func__);
    rc = oem_prop_write(BATTMAN_OEM_WORK_EVENT, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_WORK_EVENT WORK_LONG_FULL_CAP rc=%d\n", rc);
    }

    schedule_delayed_work(&asus_long_full_cap_monitor_work, 30 * HZ);
}

static void asus_18W_workaround_worker(struct work_struct *dat){
    int rc;
    u32 tmp;

    tmp = WORK_18W_WORKAROUND;
    printk(KERN_ERR "[BAT][CHG]%s set BATTMAN_OEM_WORK_EVENT : WORK_18W_WORKAROUND", __func__);
    rc = oem_prop_write(BATTMAN_OEM_WORK_EVENT, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set BATTMAN_OEM_WORK_EVENT WORK_18W_WORKAROUND rc=%d\n", rc);
    }
}

//start plugin work +++
void asus_monitor_start(int status){
    if(!g_asuslib_init) return;
    if (asus_usb_online == status) return;

    asus_usb_online = status;
    printk(KERN_ERR "[BAT][CHG] asus_monitor_start %d\n", asus_usb_online);
    if(asus_usb_online){
        cancel_delayed_work_sync(&asus_jeita_rule_work);
        schedule_delayed_work(&asus_jeita_rule_work, 0);

        if(!g_Charger_mode){
            cancel_delayed_work_sync(&asus_panel_check_work);
            schedule_delayed_work(&asus_panel_check_work, 62 * HZ);
        }

        cancel_delayed_work_sync(&asus_slow_charging_work);
        schedule_delayed_work(&asus_slow_charging_work, 0 * HZ);

        cancel_delayed_work_sync(&asus_18W_workaround_work);
        schedule_delayed_work(&asus_18W_workaround_work, 26 * HZ);

        cancel_delayed_work_sync(&asus_thermal_policy_work);
        schedule_delayed_work(&asus_thermal_policy_work, 68 * HZ);

        qti_charge_notify_device_charge();
        __pm_wakeup_event(slowchg_ws, 60 * 1000);
    }else{
        cancel_delayed_work_sync(&asus_jeita_rule_work);
        cancel_delayed_work_sync(&asus_jeita_prechg_work);
        cancel_delayed_work_sync(&asus_jeita_cc_work);
        cancel_delayed_work_sync(&asus_panel_check_work);
        cancel_delayed_work_sync(&asus_slow_charging_work);
        cancel_delayed_work_sync(&asus_18W_workaround_work);
        cancel_delayed_work_sync(&asus_thermal_policy_work);
        qti_charge_notify_device_not_charge();
        VID_changed = false;
    }
}
//start plugin work ---

void monitor_charging_enable(void)
{
    union power_supply_propval prop = {};
    int bat_capacity;
    u32 tmp = 0;
    int rc;

    rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_CAPACITY, &prop);
    if (rc < 0)
	pr_err("Failed to get battery SOC, rc=%d\n", rc);
    bat_capacity = prop.intval;

    if (g_Charger_mode) {
        CHG_DBG("%s. Charger mode battery protetction monitor", __func__);
	if (bat_capacity == 100 && !g_cos_over_full_flag) {
		g_charger_mode_full_time ++;
		 CHG_DBG("%s. Charger mode battery g_charger_mode_full_time  : %d", __func__, g_charger_mode_full_time);
		if (g_charger_mode_full_time >= g_ultra_cos_spec_time) {
                        tmp = 1;
                        ChgPD_Info.ultra_bat_life = tmp;
               		tmp = ChgPD_Info.ultra_bat_life||ChgPD_Info.demo_app_status;
    			CHG_DBG("%s. set BATTMAN_OEM_Batt_Protection : %d", __func__, tmp);

			rc = oem_prop_write(BATTMAN_OEM_Batt_Protection, &tmp, 1);
			if (rc < 0) {
				pr_err("Failed to set BATTMAN_OEM_Batt_Protection rc=%d\n", rc);
				g_cos_over_full_flag = false;
			} else {
				g_cos_over_full_flag = true;
			}
		}
	}
    }
}

void asus_min_check_worker(struct work_struct *work)
{
    monitor_charging_enable();//Always TRUE, JEITA is decided on ADSP

    schedule_delayed_work(&asus_min_check_work, msecs_to_jiffies(60000));
}

//ASUS_BSP battery safety upgrade +++
static void init_battery_safety(struct bat_safety_condition *cond)
{
    cond->condition1_battery_time = BATTERY_USE_TIME_CONDITION1;
    cond->condition2_battery_time = BATTERY_USE_TIME_CONDITION2;
    cond->condition3_battery_time = BATTERY_USE_TIME_CONDITION3;
    cond->condition4_battery_time = BATTERY_USE_TIME_CONDITION4;
    cond->condition1_cycle_count = CYCLE_COUNT_CONDITION1;
    cond->condition2_cycle_count = CYCLE_COUNT_CONDITION2;
    cond->condition1_temp_vol_time = HIGH_TEMP_VOL_TIME_CONDITION1;
    cond->condition2_temp_vol_time = HIGH_TEMP_VOL_TIME_CONDITION2;
    cond->condition1_temp_time = HIGH_TEMP_TIME_CONDITION1;
    cond->condition2_temp_time = HIGH_TEMP_TIME_CONDITION2;
    cond->condition1_vol_time = HIGH_VOL_TIME_CONDITION1;
    cond->condition2_vol_time = HIGH_VOL_TIME_CONDITION2;
}

static void set_full_charging_voltage(void)
{
    int rc;
    u32 tmp;

#if defined ASUS_VODKA_PROJECT
    if(0 == g_cycle_count_data.reload_condition){

    }else if(1 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV;
    }else if(2 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV;
    }else if(3 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV - 50;
    }else if(4 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV - 100;
    }
#else
    if(0 == g_cycle_count_data.reload_condition){

    }else if(1 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV - 20;
    }else if(2 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV - 50;
    }else if(3 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV - 100;
    }else if(4 == g_cycle_count_data.reload_condition){
        tmp = INIT_FV - 150;
    }
#endif

    if(0 != g_cycle_count_data.reload_condition){
        CHG_DBG("%s. set BATTMAN_OEM_FV : %d", __func__, tmp);
        rc = oem_prop_write(BATTMAN_OEM_FV, &tmp, 1);
        if (rc < 0) {
            pr_err("Failed to set BATTMAN_OEM_FV rc=%d\n", rc);
        }
    }
}

static int file_op(const char *filename, loff_t offset, char *buf, int length, int operation)
{
    int filep;
    mm_segment_t old_fs;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if(FILE_OP_READ == operation)
        filep= ksys_open(filename, O_RDONLY | O_CREAT | O_SYNC, 0666);
    else if(FILE_OP_WRITE == operation)
        filep= ksys_open(filename, O_RDWR | O_CREAT | O_SYNC, 0666);
    else {
        pr_err("Unknown partition op err!\n");
        return -1;
    }
    if(filep < 0) {
        pr_err("open %s err! error code:%d\n", filename, filep);
        return -1;
    }
   else
        CHG_DBG("[BAT][CHG]%s open %s success!\n", __func__, filename);

    ksys_lseek(filep, offset, SEEK_SET);
    if(FILE_OP_READ == operation)
        ksys_read(filep, buf, length);
    else if(FILE_OP_WRITE == operation) {
        ksys_write(filep, buf, length);
        ksys_sync();
    }
    ksys_close(filep);
    set_fs(old_fs);
    return length;
}

static int backup_bat_percentage(void)
{
    char buf[5]={0};
    int bat_percent = 0;
    int rc;

    if(0 == g_cycle_count_data.reload_condition){
        bat_percent = 0;
    }else if(3 == g_cycle_count_data.reload_condition){
        bat_percent = 95;
    }else if(4 == g_cycle_count_data.reload_condition){
        bat_percent = 90;
    }
    sprintf(buf, "%d\n", bat_percent);
    CHG_DBG("[BAT][CHG]%s bat_percent=%d; reload_condition=%d\n", __func__, bat_percent, g_cycle_count_data.reload_condition);

    rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char *)&buf, sizeof(char)*5, FILE_OP_WRITE);
    if(rc<0)
        pr_err("%s:Write file:%s err!\n", __func__, BAT_PERCENT_FILE_NAME);

    return rc;
}

static int backup_bat_safety(void)
{
    char buf[70]={0};
    int rc;

    sprintf(buf, "%lu,%d,%lu,%lu,%lu\n",
        g_cycle_count_data.battery_total_time,
        g_cycle_count_data.cycle_count,
        g_cycle_count_data.high_temp_total_time,
        g_cycle_count_data.high_vol_total_time,
        g_cycle_count_data.high_temp_vol_time);

    rc = file_op(BAT_SAFETY_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char *)&buf, sizeof(char)*70, FILE_OP_WRITE);
    if(rc<0)
        pr_err("%s:Write file:%s err!\n", __func__, BAT_SAFETY_FILE_NAME);

    return rc;
}

static int init_batt_cycle_count_data(void)
{
    int rc = 0;
    struct CYCLE_COUNT_DATA buf;

    /* Read cycle count data from emmc */
    rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char*)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_READ);
    if(rc < 0) {
        pr_err("Read cycle count file failed!\n");
        return rc;
    }

    /* Check data validation */
    if(buf.magic != CYCLE_COUNT_DATA_MAGIC) {
        pr_err("data validation!\n");
        file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char*)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
        return -1;
    }else {
        /* Update current value */
        CHG_DBG("[BAT][CHG]%s Update current value!\n", __func__);
        g_cycle_count_data.cycle_count = buf.cycle_count;
        g_cycle_count_data.high_temp_total_time = buf.high_temp_total_time;
        g_cycle_count_data.high_temp_vol_time = buf.high_temp_vol_time;
        g_cycle_count_data.high_vol_total_time = buf.high_vol_total_time;
        g_cycle_count_data.reload_condition = buf.reload_condition;
        g_cycle_count_data.battery_total_time = buf.battery_total_time;

        rc = backup_bat_percentage();
        if(rc < 0){
            pr_err("backup_bat_percentage failed!\n");
            return -1;
        }

#if 0
        rc = backup_bat_cyclecount();
        if(rc < 0){
            pr_err("backup_bat_cyclecount failed!\n");
            return -1;
        }
#endif

        rc = backup_bat_safety();
        if(rc < 0){
            pr_err("backup_bat_cyclecount failed!\n");
            return -1;
        }

        CHG_DBG("[BAT][CHG]%s cycle_count=%d;reload_condition=%d;high_temp_total_time=%lu;high_temp_vol_time=%lu;high_vol_total_time=%lu;battery_total_time=%lu\n",
            __func__, buf.cycle_count,buf.reload_condition, buf.high_temp_total_time,buf.high_temp_vol_time,buf.high_vol_total_time,buf.battery_total_time);
    }
    CHG_DBG("[BAT][CHG]%s Cycle count data initialize success!\n", __func__);
    g_cyclecount_initialized = true;
    set_full_charging_voltage();
    return 0;
}

static void write_back_cycle_count_data(void)
{
    int rc;

    backup_bat_percentage();
    //backup_bat_cyclecount();
    backup_bat_safety();
    //batt_safety_csc_backup();

    rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
    if(rc<0)
        pr_err("%s:Write file:%s err!\n", __func__, CYCLE_COUNT_FILE_NAME);
}

static void asus_reload_battery_profile(int value){

    //save current status
    write_back_cycle_count_data();

    //reloade battery
    //reload_battery_profile(chip);
    set_full_charging_voltage();

    CHG_DBG("[BAT][CHG]%s !!new profile is value=%d\n", __func__, value);
}

static void asus_judge_reload_condition(struct bat_safety_condition *safety_cond)
{
    int temp_condition = 0;
    int cycle_count = 0;
    // bool full_charge;
    unsigned long local_high_vol_time = g_cycle_count_data.high_vol_total_time;
    unsigned long local_high_temp_time = g_cycle_count_data.high_temp_total_time;
    //unsigned long local_high_temp_vol_time = g_cycle_count_data.high_temp_vol_time;
    unsigned long local_battery_total_time = g_cycle_count_data.battery_total_time;

    temp_condition = g_cycle_count_data.reload_condition;
    if(temp_condition >= 4){ //if condition=2 will return
        return ;
    }

    //only full charger can load new profile
    // full_charge = fg->charge_done;
    // if(!full_charge)
    //     return ;

    //1.judge battery using total time
    if(local_battery_total_time >= safety_cond->condition4_battery_time){
        g_cycle_count_data.reload_condition = 4;
        goto DONE;
    }else if(local_battery_total_time >= safety_cond->condition3_battery_time &&
        local_battery_total_time < safety_cond->condition4_battery_time){
        g_cycle_count_data.reload_condition = 3;
    }
#if defined ASUS_SAKE_PROJECT
    else if(local_battery_total_time >= safety_cond->condition2_battery_time &&
        local_battery_total_time < safety_cond->condition3_battery_time){
        g_cycle_count_data.reload_condition = 2;
    }else if(local_battery_total_time >= safety_cond->condition1_battery_time &&
        local_battery_total_time < safety_cond->condition2_battery_time){
        g_cycle_count_data.reload_condition = 1;
    }
#endif

    //2. judge battery cycle count
    cycle_count = g_cycle_count_data.cycle_count;
#if 0 //disable reloade condition with cycle_count
    if(cycle_count >= chip->condition2_cycle_count){
        g_cycle_count_data.reload_condition = 2;
        goto DONE;
    }else if(cycle_count >= chip->condition1_cycle_count &&
        cycle_count < chip->condition2_cycle_count){
        g_cycle_count_data.reload_condition = 1;
    }
#endif
#if 0 //disable reloade condition with high_temp_vol
    //3. judge high temp and voltage condition
    if(local_high_temp_vol_time >= fg->condition2_temp_vol_time){
        g_cycle_count_data.reload_condition = 2;
        goto DONE;
    }else if(local_high_temp_vol_time >= fg->condition1_temp_vol_time &&
        local_high_temp_vol_time < fg->condition2_temp_vol_time){
        g_cycle_count_data.reload_condition = 1;
    }
#endif

    //4. judge high temp condition
    if(local_high_temp_time >= safety_cond->condition2_temp_time){
        g_cycle_count_data.reload_condition = 4;
        goto DONE;
    }else if(local_high_temp_time >= safety_cond->condition1_temp_time &&
        local_high_temp_time < safety_cond->condition2_temp_time){
        g_cycle_count_data.reload_condition = 3;
    }

    //5. judge high voltage condition
    if(local_high_vol_time >= safety_cond->condition2_vol_time){
        g_cycle_count_data.reload_condition = 4;
        goto DONE;
    }else if(local_high_vol_time >= safety_cond->condition1_vol_time &&
        local_high_vol_time < safety_cond->condition2_vol_time){
        g_cycle_count_data.reload_condition = 3;
    }

DONE:
    if(temp_condition != g_cycle_count_data.reload_condition)
        asus_reload_battery_profile(g_cycle_count_data.reload_condition);

}

unsigned long last_battery_total_time = 0;
unsigned long last_high_temp_time = 0;
unsigned long last_high_vol_time = 0;
unsigned long last_high_temp_vol_time = 0;

static void calculation_time_fun(int type)
{
    unsigned long now_time;
    unsigned long temp_time = 0;
    struct timespec64 mtNow;

    ktime_get_coarse_real_ts64(&mtNow);

    now_time = mtNow.tv_sec;

    if(now_time < 0){
        pr_err("asus read rtc time failed!\n");
        return ;
    }

    switch(type){
        case TOTOL_TIME_CAL_TYPE:
            if(0 == last_battery_total_time){
                last_battery_total_time = now_time;
                CHG_DBG("[BAT][CHG]%s now_time=%lu;last_battery_total_time=%lu\n", __func__, now_time, g_cycle_count_data.battery_total_time);
            }else{
                temp_time = now_time - last_battery_total_time;
                if(temp_time > 0)
                    g_cycle_count_data.battery_total_time += temp_time;
                last_battery_total_time = now_time;
            }
        break;

        case HIGH_VOL_CAL_TYPE:
            if(0 == last_high_vol_time){
                last_high_vol_time = now_time;
                CHG_DBG("[BAT][CHG]%s now_time=%lu;high_vol_total_time=%lu\n", __func__, now_time, g_cycle_count_data.high_vol_total_time);
            }else{
                temp_time = now_time - last_high_vol_time;
                if(temp_time > 0)
                    g_cycle_count_data.high_vol_total_time += temp_time;
                last_high_vol_time = now_time;
            }
        break;

        case HIGH_TEMP_CAL_TYPE:
            if(0 == last_high_temp_time){
                last_high_temp_time = now_time;
                CHG_DBG("[BAT][CHG]%s now_time=%lu;high_temp_total_time=%lu\n", __func__, now_time, g_cycle_count_data.high_temp_total_time);
            }else{
                temp_time = now_time - last_high_temp_time;
                if(temp_time > 0)
                    g_cycle_count_data.high_temp_total_time += temp_time;
                last_high_temp_time = now_time;
            }
        break;

        case HIGH_TEMP_VOL_CAL_TYPE:
            if(0 == last_high_temp_vol_time){
                last_high_temp_vol_time = now_time;
                CHG_DBG("[BAT][CHG]%s now_time=%lu;high_temp_vol_time=%lu\n", __func__, now_time, g_cycle_count_data.high_temp_vol_time);
            }else{
                temp_time = now_time - last_high_temp_vol_time;
                if(temp_time > 0)
                    g_cycle_count_data.high_temp_vol_time += temp_time;
                last_high_temp_vol_time = now_time;
            }
        break;
    }
}

static int write_test_value = 0;
static void update_battery_safe()
{
    int rc;
    int temp;
    int capacity;
    union power_supply_propval prop = {};

    CHG_DBG("[BAT][CHG]%s +++", __func__);

    if(g_asuslib_init != true){
        pr_err("asuslib init is not ready");
        return;
    }

    if(g_cyclecount_initialized != true){
        rc = init_batt_cycle_count_data();
        if(rc < 0){
            pr_err("cyclecount is not initialized");
            return;
        }
    }

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_TEMP, &prop);
    if (rc < 0) {
        pr_err("Error in getting battery temp, rc=%d\n", rc);
    }
    temp= prop.intval;

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CAPACITY, &prop);
    if (rc < 0) {
        pr_err("Error in getting capacity, rc=%d\n", rc);
    }
    capacity = prop.intval;

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CYCLE_COUNT, &prop);
    if (rc < 0) {
        pr_err("Error in getting cycle count, rc=%d\n", rc);
    }
    g_cycle_count_data.cycle_count = prop.intval;

    CHG_DBG("[BAT][CHG]%s temp=%d, capacity=%d, cycle_count=%d", __func__, temp, capacity, g_cycle_count_data.cycle_count);

    //check data
    calculation_time_fun(TOTOL_TIME_CAL_TYPE);

    if(capacity == FULL_CAPACITY_VALUE){
        calculation_time_fun(HIGH_VOL_CAL_TYPE);
    }else{
        last_high_vol_time = 0; //exit high vol
    }

    if(temp >= HIGHER_TEMP){
        calculation_time_fun(HIGH_TEMP_CAL_TYPE);
    }else{
        last_high_temp_time = 0; //exit high temp
    }

    if(temp >= HIGH_TEMP && capacity == FULL_CAPACITY_VALUE){
        calculation_time_fun(HIGH_TEMP_VOL_CAL_TYPE);
    }else{
        last_high_temp_vol_time = 0; //exit high temp and vol
    }

    asus_judge_reload_condition(&safety_cond);
    write_back_cycle_count_data();
    CHG_DBG("[BAT][CHG]%s ---", __func__);
}

void battery_safety_worker(struct work_struct *work)
{
    update_battery_safe();

    schedule_delayed_work(&battery_safety_work, BATTERY_SAFETY_UPGRADE_TIME * HZ);
}
//ASUS_BSP battery safety upgrade ---

//ASUS_BS battery health upgrade +++
static void battery_health_data_reset(void){
    g_bat_health_data.charge_counter_begin = 0;
    g_bat_health_data.charge_counter_end = 0;
    g_bathealth_trigger = false;
    g_last_bathealth_trigger = false;
}

int batt_health_csc_backup(void){
    int rc=0, i=0;
    struct BAT_HEALTH_DATA_BACKUP buf[BAT_HEALTH_NUMBER_MAX];
    char buf2[BAT_HEALTH_NUMBER_MAX][30];

    memset(&buf,0,sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX);
    memset(&buf2,0,sizeof(char)*BAT_HEALTH_NUMBER_MAX*30);

    rc = file_op(BAT_HEALTH_DATA_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
        (char*)&buf, sizeof(struct BAT_HEALTH_DATA)*BAT_HEALTH_NUMBER_MAX, FILE_OP_READ);
    if(rc < 0) {
        CHG_DBG_E("[BAT][CHG]%s Read bat health file failed!\n", __func__);
        return rc;
    }

    for(i=1;i<BAT_HEALTH_NUMBER_MAX;i++){
        if(buf[i].health!=0){
            sprintf(&buf2[i-1][0], "%s [%d]\n", buf[i].date, buf[i].health);
        }
    }

    rc = file_op(BAT_HEALTH_DATA_SD_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
    (char *)&buf2, sizeof(char)*BAT_HEALTH_NUMBER_MAX*30, FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s Write bat health file failed!\n", __func__);


    CHG_DBG("[BAT][CHG]%s Done! \n", __func__);
    return rc;
}

static int restore_bat_health(void)
{
    int i=0, rc = 0, count =0;

    memset(&g_bat_health_data_backup,0,sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX);

    /* Read cycle count data from emmc */
    rc = file_op(BAT_HEALTH_DATA_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
        (char*)&g_bat_health_data_backup, sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX, FILE_OP_READ);
    if(rc < 0) {
        pr_err("Read bat health file failed!\n");
        return -1;
    }

    for(i=1; i<BAT_HEALTH_NUMBER_MAX;i++){
        CHG_DBG("[BAT][CHG]%s %s %d", __func__,g_bat_health_data_backup[i].date, g_bat_health_data_backup[i].health);

        if(g_bat_health_data_backup[i].health!=0){
            count++;
        }
    }

    if(count >= BAT_HEALTH_NUMBER_MAX-1){
        g_health_upgrade_index = BAT_HEALTH_NUMBER_MAX-1;
        g_bat_health_data_backup[0].health = BAT_HEALTH_NUMBER_MAX-1;
    }

    CHG_DBG("[BAT][CHG]%s index(%d)\n", __func__, g_bat_health_data_backup[0].health);

    g_health_upgrade_index = g_bat_health_data_backup[0].health;
    g_bathealth_initialized = true;

    batt_health_csc_backup();
    //batt_safety_csc_backup();
    return 0;
}

static void resequencing_bat_health_data(void){
    int i;

    for(i=1; i < BAT_HEALTH_NUMBER_MAX-1 ; i++){
        memcpy(&g_bat_health_data_backup[i], &g_bat_health_data_backup[i+1], sizeof(struct BAT_HEALTH_DATA_BACKUP));
    }
}

static int backup_bat_health(void)
{
    int bat_health, rc;
    struct timespec ts;
    struct rtc_time tm;
    int health_t;
    int count=0, i=0;
    unsigned long long bat_health_accumulate=0;

    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec,&tm);

    bat_health = g_bat_health_data.bat_health;

    if(g_health_upgrade_index >= BAT_HEALTH_NUMBER_MAX-1){
        g_health_upgrade_index = BAT_HEALTH_NUMBER_MAX-1;
    }else{
        g_health_upgrade_index++;
    }

    if(g_health_upgrade_index >= BAT_HEALTH_NUMBER_MAX-1){
        resequencing_bat_health_data();
    }

    sprintf(g_bat_health_data_backup[g_health_upgrade_index].date, "%d-%02d-%02d %02d:%02d:%02d", tm.tm_year+1900,tm.tm_mon+1, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
    g_bat_health_data_backup[g_health_upgrade_index].health = bat_health;
    g_bat_health_data_backup[0].health = g_health_upgrade_index;

    CHG_DBG("[BAT][CHG]%s ===== Health history ====\n", __func__);
    for(i=1;i<BAT_HEALTH_NUMBER_MAX;i++){
        if(g_bat_health_data_backup[i].health!=0){
            count++;
            bat_health_accumulate += g_bat_health_data_backup[i].health;
            CHG_DBG("[BAT][CHG]%s %02d:%d\n", __func__,i,g_bat_health_data_backup[i].health);
        }
    }
    CHG_DBG("[BAT][CHG]%s  ========================\n", __func__);
    if(count==0){
        CHG_DBG("[BAT][CHG]%s battery health value is empty\n", __func__);
        return -1;
    }
    health_t = bat_health_accumulate*10/count;
    g_bat_health_avg = (int)(health_t + 5)/10;
    g_bat_health_data_backup[g_health_upgrade_index].health = g_bat_health_avg;

    rc = file_op(BAT_HEALTH_DATA_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
        (char *)&g_bat_health_data_backup, sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX, FILE_OP_WRITE);
    if(rc<0){
        pr_err("%s:Write file:%s err!\n", __func__, BAT_HEALTH_DATA_FILE_NAME);
    }

    return rc;
}

static void update_battery_health(){
    int bat_capacity, rc;
    union power_supply_propval prop = {};

    if(g_health_upgrade_enable != true){
        return;
    }

    if(g_bathealth_initialized != true){
        restore_bat_health();
        return;
    }

    if(!asus_usb_online){
        if(g_bathealth_trigger == true){
            battery_health_data_reset();
        }
        return;
    }

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CAPACITY, &prop);
    if (rc < 0) {
        pr_err("Error in getting capacity, rc=%d\n", rc);
    }
    bat_capacity = prop.intval;

    if(bat_capacity == g_health_upgrade_start_level && !g_bathealth_trigger){
        g_bathealth_trigger = true;

        rc = power_supply_get_property(qti_phy_bat,
            POWER_SUPPLY_PROP_CHARGE_COUNTER, &prop);
        if (rc < 0) {
            pr_err("Error in getting current, rc=%d\n", rc);
        }
        g_bat_health_data.charge_counter_begin = prop.intval;

        CHG_DBG("[BAT][CHG]%s battery health begin charge_counter=%d", __func__, g_bat_health_data.charge_counter_begin);
    }
    else if(bat_capacity == g_health_upgrade_end_level && g_bathealth_trigger){
        g_bathealth_trigger = false;

        rc = power_supply_get_property(qti_phy_bat,
            POWER_SUPPLY_PROP_CHARGE_COUNTER, &prop);
        if (rc < 0) {
            pr_err("Error in getting current, rc=%d\n", rc);
        }
        g_bat_health_data.charge_counter_end = prop.intval;

        //(coulomb100%-coulomb70%)/1000/30/(Realcapacity/100)*100%
        g_bat_health_data.bat_health = (g_bat_health_data.charge_counter_end - g_bat_health_data.charge_counter_begin)*10/(ZF8_DESIGNED_CAPACITY*(g_health_upgrade_end_level-g_health_upgrade_start_level));
        if(g_bat_health_data.bat_health < 0) g_bat_health_data.bat_health = 0;
        if(g_bat_health_data.bat_health >100) g_bat_health_data.bat_health = 100;

        backup_bat_health();
        batt_health_csc_backup();
        CHG_DBG("[BAT][CHG]%s battery health = (%d,%d), charge_counter begin=%d, charge_counter end=%d", __func__,
            g_bat_health_data.bat_health, g_bat_health_avg, g_bat_health_data.charge_counter_begin, g_bat_health_data.charge_counter_end);

        battery_health_data_reset();
    }
    else if(bat_capacity >= g_health_upgrade_start_level && bat_capacity <= g_health_upgrade_end_level){

    }
    else{
        g_bathealth_trigger = false;
        battery_health_data_reset();
    }
}

void battery_health_worker(struct work_struct *work)
{
    update_battery_health();

    schedule_delayed_work(&battery_health_work, g_health_upgrade_upgrade_time * HZ);
}
//ASUS_BS battery health upgrade ---

static int cycle_count_proc_show(struct seq_file *buf, void *data)
{
    seq_printf(buf, "---show cycle count value---\n");

#if 0  //for debug
    get_asus_cycle_count(&g_cycle_count_data.cycle_count);

    seq_printf(buf,"cycle[%d,%d,%d,%d,%d,%d,%d,%d]\n",
        g_fgChip->counter->count[0],g_fgChip->counter->count[1],g_fgChip->counter->count[2],
        g_fgChip->counter->count[3],g_fgChip->counter->count[4],g_fgChip->counter->count[5],
        g_fgChip->counter->count[6],g_fgChip->counter->count[7]);
#endif

    seq_printf(buf, "cycle count:%d\n", g_cycle_count_data.cycle_count);

    return 0;
}

static int cycle_count_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, cycle_count_proc_show, NULL);
}

static const struct file_operations cycle_count_fops = {
    .owner = THIS_MODULE,
    .open = cycle_count_proc_open,
    .read = seq_read,
    .release = single_release,
};

static int batt_safety_proc_show(struct seq_file *buf, void *data)
{
    int rc =0;

    rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
    (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s write cycle count file error\n", __func__);

    seq_printf(buf, "---show battery safety value---\n");
    seq_printf(buf, "cycle_count:%d\n", g_cycle_count_data.cycle_count);
    seq_printf(buf, "battery_total_time:%lu\n", g_cycle_count_data.battery_total_time);
    seq_printf(buf, "high_temp_total_time:%lu\n", g_cycle_count_data.high_temp_total_time);
    seq_printf(buf, "high_vol_total_time:%lu\n", g_cycle_count_data.high_vol_total_time);
    seq_printf(buf, "high_temp_vol_time:%lu\n", g_cycle_count_data.high_temp_vol_time);
    seq_printf(buf, "reload_condition:%d\n", g_cycle_count_data.reload_condition);

    return 0;
}
static int batt_safety_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, batt_safety_proc_show, NULL);
}

static ssize_t batt_safety_proc_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
{
    int value=0;
    unsigned long time = 0;
    char buf[30] = {0};
    size_t buf_size;
    char *start = buf;

    buf_size = min(count, (size_t)(sizeof(buf)-1));
    if (copy_from_user(buf, buffer, buf_size)) {
        CHG_DBG_E("[BAT][CHG]%s Failed to copy from user\n", __func__);
        return -EFAULT;
    }
    buf[buf_size] = 0;

    sscanf(start, "%d", &value);
    while (*start++ != ' ');
    sscanf(start, "%lu", &time);

    write_test_value = value;

    switch(value){
        case 1:
            g_cycle_count_data.battery_total_time = time;
        break;
        case 2:
            g_cycle_count_data.cycle_count = (int)time;
        break;
        case 3:
            g_cycle_count_data.high_temp_vol_time = time;
        break;
        case 4:
            g_cycle_count_data.high_temp_total_time = time;
        break;
        case 5:
            g_cycle_count_data.high_vol_total_time = time;
        break;
        default:
            CHG_DBG("[BAT][CHG]%s input error!Now return\n", __func__);
            return count;
    }
    asus_judge_reload_condition(&safety_cond);
    CHG_DBG("[BAT][CHG]%s value=%d;time=%lu\n", __func__, value, time);

    return count;
}

static const struct file_operations batt_safety_fops = {
    .owner = THIS_MODULE,
    .open = batt_safety_proc_open,
    .read = seq_read,
    .write = batt_safety_proc_write,
    .release = single_release,
};

//ASUS_BS battery health upgrade +++
static void batt_safety_csc_stop(void){

    cancel_delayed_work_sync(&battery_safety_work);
    CHG_DBG("[BAT][CHG]%s Done! \n", __func__);
}

static void batt_safety_csc_start(void){

    schedule_delayed_work(&battery_safety_work, 0);
    CHG_DBG("[BAT][CHG]%s Done! \n", __func__);
}

static void batt_health_upgrade_debug_enable(bool enable){

    g_health_debug_enable = enable;
    CHG_DBG_E("[BAT][CHG]%s %d\n", __func__,g_health_debug_enable);
}

static void batt_health_upgrade_enable(bool enable){

    g_health_upgrade_enable = enable;
    CHG_DBG_E("[BAT][CHG]%s %d\n", __func__,g_health_upgrade_enable);
}

static int batt_health_config_proc_show(struct seq_file *buf, void *data)
{
    int count=0, i=0;
    unsigned long long bat_health_accumulate=0;

    seq_printf(buf, "start level:%d\n", g_health_upgrade_start_level);
    seq_printf(buf, "end level:%d\n", g_health_upgrade_end_level);
    seq_printf(buf, "upgrade time:%d\n", g_health_upgrade_upgrade_time);

    for(i=1;i<BAT_HEALTH_NUMBER_MAX;i++){
        if(g_bat_health_data_backup[i].health!=0){
            count++;
            bat_health_accumulate += g_bat_health_data_backup[i].health;
        }
    }
    g_bat_health_avg = bat_health_accumulate/count;
    seq_printf(buf, "health_avg: %d\n", g_bat_health_avg);

    return 0;
}
static int batt_health_config_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, batt_health_config_proc_show, NULL);
}

static ssize_t batt_health_config_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
{
    int command=0;
    int value = 0;
    char buf[30] = {0};
    size_t buf_size;
    char *start = buf;

    buf_size = min(count, (size_t)(sizeof(buf)-1));
    if (copy_from_user(buf, buffer, buf_size)) {
        CHG_DBG_E("[BAT][CHG]%s Failed to copy from user\n", __func__);
        return -EFAULT;
    }
    buf[buf_size] = 0;

    sscanf(start, "%d", &command);
    while (*start++ != ' ');
    sscanf(start, "%d", &value);

    switch(command){
        case 1:
            g_health_upgrade_start_level = value;
            CHG_DBG("[BAT][CHG]%s health upgrade start_level = %d;\n", __func__, value);
        break;
        case 2:
            g_health_upgrade_end_level = value;
            CHG_DBG("[BAT][CHG]%s health upgrade end_level = %d;\n", __func__, value);
        break;
        case 3:
            g_health_upgrade_upgrade_time = value;
            CHG_DBG("[BAT][CHG]%s health upgrade time = %d;\n", __func__, value);
        break;
        default:
            CHG_DBG("[BAT][CHG]%s input error!Now return\n", __func__);
            return count;
    }

    return count;
}

static const struct file_operations batt_health_config_fops = {
    .owner = THIS_MODULE,
    .open = batt_health_config_proc_open,
    .read = seq_read,
    .write = batt_health_config_write,
    .release = single_release,
};
//ASUS_BS battery health upgrade ---

static int condition_value_proc_show(struct seq_file *buf, void *data)
{
    seq_printf(buf, "---show condition value---\n");
    seq_printf(buf, "condition1 battery time %lu\n", safety_cond.condition1_battery_time);
    seq_printf(buf, "condition2 battery time %lu\n", safety_cond.condition2_battery_time);
    seq_printf(buf, "condition3 battery time %lu\n", safety_cond.condition3_battery_time);
    seq_printf(buf, "condition4 battery time %lu\n", safety_cond.condition4_battery_time);
    seq_printf(buf, "condition1 cycle count %d\n", safety_cond.condition1_cycle_count);
    seq_printf(buf, "condition2 cycle count %d\n", safety_cond.condition2_cycle_count);
    seq_printf(buf, "condition1 temp time %lu\n", safety_cond.condition1_temp_time);
    seq_printf(buf, "condition2 temp time %lu\n", safety_cond.condition2_temp_time);
    seq_printf(buf, "condition1 temp&vol time %lu\n", safety_cond.condition1_temp_vol_time);
    seq_printf(buf, "condition2 temp&vol time %lu\n", safety_cond.condition2_temp_vol_time);
    seq_printf(buf, "condition1 vol time %lu\n", safety_cond.condition1_vol_time);
    seq_printf(buf, "condition2 vol time %lu\n", safety_cond.condition2_vol_time);

    return 0;
}

static int condition_value_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, condition_value_proc_show, NULL);
}

static ssize_t condition_value_proc_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
{
    int value = 0;
    unsigned long condition1_time = 0;
    unsigned long condition2_time = 0;
    unsigned long condition3_time = 0;
    unsigned long condition4_time = 0;
    char buf[320];
    char *start = buf;

    if (copy_from_user(buf, buffer, count-1)) {
        CHG_DBG_E("[BAT][CHG]%s Failed to copy from user\n", __func__);
        return -EFAULT;
    }
    buf[count] = 0;

    sscanf(start, "%d", &value);

    if(value != 0 && start - buf < count){
        while (*start++ != ' ');
        sscanf(start, "%lu", &condition1_time);
        while (*start++ != ' ');
        sscanf(start, "%lu", &condition2_time);
    }
    if(value == 1 && start - buf < count){
        while (*start++ != ' ');
        sscanf(start, "%lu", &condition3_time);
        while (*start++ != ' ');
        sscanf(start, "%lu", &condition4_time);
    }

    if(value && condition2_time <= condition1_time){
        CHG_DBG_E("[BAT][CHG]%s input value error,please input correct value!\n", __func__);
        return count;
    }

    switch(value){
        case 0:
            init_battery_safety(&safety_cond);
            g_cycle_count_data.reload_condition = 0;
        break;
        case 1:
            safety_cond.condition1_battery_time = condition1_time;
            safety_cond.condition2_battery_time = condition2_time;
            safety_cond.condition3_battery_time = condition3_time;
            safety_cond.condition4_battery_time = condition4_time;
        break;
        case 2:
            safety_cond.condition1_cycle_count = (int)condition1_time;
            safety_cond.condition2_cycle_count = (int)condition2_time;
        break;
        case 3:
            safety_cond.condition1_temp_vol_time = condition1_time;
            safety_cond.condition2_temp_vol_time = condition2_time;
        break;
        case 4:
            safety_cond.condition1_temp_time = condition1_time;
            safety_cond.condition2_temp_time = condition2_time;
        break;
        case 5:
            safety_cond.condition1_vol_time = condition1_time;
            safety_cond.condition2_vol_time = condition2_time;
        break;
    }

    CHG_DBG("[BAT][CHG]%s value=%d;condition1_time=%lu;condition2_time=%lu\n", __func__, value, condition1_time, condition2_time);
    return count;
}

static const struct file_operations condition_value_fops = {
    .owner = THIS_MODULE,
    .open = condition_value_proc_open,
    .read = seq_read,
    .write = condition_value_proc_write,
    .release = single_release,
};

static int batt_safety_csc_proc_show(struct seq_file *buf, void *data)
{
    int rc =0;

    rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
    (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s write cycle count file error\n", __func__);

    seq_printf(buf, "---show battery safety value---\n");
    seq_printf(buf, "cycle_count:%d\n", g_cycle_count_data.cycle_count);
    seq_printf(buf, "battery_total_time:%lu\n", g_cycle_count_data.battery_total_time);
    seq_printf(buf, "high_temp_total_time:%lu\n", g_cycle_count_data.high_temp_total_time);
    seq_printf(buf, "high_vol_total_time:%lu\n", g_cycle_count_data.high_vol_total_time);
    seq_printf(buf, "high_temp_vol_time:%lu\n", g_cycle_count_data.high_temp_vol_time);
    seq_printf(buf, "reload_condition:%d\n", g_cycle_count_data.reload_condition);

    return 0;
}
static int batt_safety_csc_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, batt_safety_csc_proc_show, NULL);
}

static int batt_safety_csc_erase(void){
    int rc =0;
    char buf[1]={0};

    g_cycle_count_data.battery_total_time = 0;
    g_cycle_count_data.cycle_count = 0;
    g_cycle_count_data.high_temp_total_time = 0;
    g_cycle_count_data.high_temp_vol_time = 0;
    g_cycle_count_data.high_vol_total_time = 0;
    g_cycle_count_data.reload_condition = 0;

    rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
    (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s Write file:%s err!\n", __func__, CYCLE_COUNT_FILE_NAME);

    rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char *)&buf, sizeof(char), FILE_OP_WRITE);
    if(rc<0)
        CHG_DBG_E("[BAT][CHG]%s Write file:%s err!\n", __func__, BAT_PERCENT_FILE_NAME);

    CHG_DBG("[BAT][CHG]%s Done! rc(%d)\n", __func__,rc);
    return rc;
}

int batt_safety_csc_backup(void){
    int rc = 0;
    struct CYCLE_COUNT_DATA buf;
//  char buf2[1]={0};

    rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char*)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_READ);
    if(rc < 0) {
        CHG_DBG_E("[BAT][CHG]%s Read cycle count file failed!\n", __func__);
        return rc;
    }

    rc = file_op(CYCLE_COUNT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
    (char *)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s Write cycle count file failed!\n", __func__);
#if 0
    rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char*)&buf2, sizeof(char), FILE_OP_READ);
    if(rc < 0) {
        CHG_DBG_E("[BAT][CHG]%s Read cycle count percent file failed!\n");
        return rc;
    }

    rc = file_op(BAT_PERCENT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
    (char *)&buf2, sizeof(char), FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s Write cycle count percent file failed!\n");
#endif
    CHG_DBG("[BAT][CHG]%s Done!\n", __func__);
    return rc;
}

static int batt_safety_csc_restore(void){
    int rc = 0;
    struct CYCLE_COUNT_DATA buf;
    char buf2[1]={0};

    rc = file_op(CYCLE_COUNT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char*)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_READ);
    if(rc < 0) {
        CHG_DBG_E("[BAT][CHG]%s Read cycle count file failed!\n", __func__);
        return rc;
    }

    rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
    (char *)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s Write cycle count file failed!\n", __func__);

    rc = file_op(BAT_PERCENT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char*)&buf2, sizeof(char), FILE_OP_READ);
    if(rc < 0) {
        CHG_DBG_E("[BAT][CHG]%s Read cycle count percent file failed!\n", __func__);
        return rc;
    }

    rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
    (char *)&buf2, sizeof(char), FILE_OP_WRITE);
    if(rc < 0 )
        CHG_DBG_E("[BAT][CHG]%s Write cycle count percent file failed!\n", __func__);

    init_batt_cycle_count_data();
    CHG_DBG("[BAT][CHG]%s Done! rc(%d)\n", __func__,rc);
    return rc;
}

#if 0
static int batt_safety_csc_getcyclecount(void){
    char buf[30]={0};
    int rc;

    sprintf(buf, "%d\n", g_cycle_count_data.cycle_count);
    CHG_DBG("[BAT][CHG]%s cycle_count=%d\n", __func__, g_cycle_count_data.cycle_count);

    rc = file_op(BAT_CYCLE_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
        (char *)&buf, sizeof(char)*30, FILE_OP_WRITE);
    if(rc<0)
        pr_err("%s:Write file:%s err!\n", __func__, BAT_CYCLE_SD_FILE_NAME);


    CHG_DBG("[BAT][CHG]%s Done! rc(%d)\n", __func__,rc);
    return rc;
}
#endif

static ssize_t batt_safety_csc_proc_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
{
    int value=0;
    char buf[2] = {0};
    size_t buf_size;
    char *start = buf;

    buf_size = min(count, (size_t)(sizeof(buf)-1));
    if (copy_from_user(buf, buffer, buf_size)) {
        CHG_DBG_E("[BAT][CHG]%s Failed to copy from user\n", __func__);
        return -EFAULT;
    }
    buf[buf_size] = 0;

    sscanf(start, "%d", &value);

    switch(value){
        case 0: //erase
            batt_safety_csc_erase();
        break;
        case 1: //backup /persist to /sdcard
            batt_safety_csc_backup();
        break;
        case 2: //resotre /sdcard from /persist
            batt_safety_csc_restore();
        break;
        case 3://stop battery safety upgrade
            batt_safety_csc_stop();
            break;
        case 4: //start safety upgrade
            batt_safety_csc_start();
            break;
        case 5: // disable battery health debug log
            batt_health_upgrade_debug_enable(false);
            break;
        case 6: // enable battery health debug log
            batt_health_upgrade_debug_enable(true);
            break;
        case 7: // disable battery health upgrade
            batt_health_upgrade_enable(false);
            break;
        case 8: // enable battery health upgrade
            batt_health_upgrade_enable(true);
            break;
        case 9: //initial battery safety upgrade
            init_batt_cycle_count_data();
            break;
        default:
            CHG_DBG_E("[BAT][CHG]%s input error!Now return\n", __func__);
            return count;
    }

    return count;
}

static const struct file_operations batt_safety_csc_fops = {
    .owner = THIS_MODULE,
    .open = batt_safety_csc_proc_open,
    .read = seq_read,
    .write = batt_safety_csc_proc_write,
    .release = single_release,
};

static void create_batt_cycle_count_proc_file(void)
{
    struct proc_dir_entry *asus_batt_cycle_count_dir = proc_mkdir("Batt_Cycle_Count", NULL);
    struct proc_dir_entry *asus_batt_cycle_count_proc_file = proc_create("cycle_count", 0666,
        asus_batt_cycle_count_dir, &cycle_count_fops);
    struct proc_dir_entry *asus_batt_batt_safety_proc_file = proc_create("batt_safety", 0666,
        asus_batt_cycle_count_dir, &batt_safety_fops);
    struct proc_dir_entry *asus_batt_batt_safety_csc_proc_file = proc_create("batt_safety_csc", 0666,
        asus_batt_cycle_count_dir, &batt_safety_csc_fops);
    struct proc_dir_entry *asus_batt_safety_condition_proc_file = proc_create("condition_value", 0666,
        asus_batt_cycle_count_dir, &condition_value_fops);
    struct proc_dir_entry *batt_health_config_proc_file = proc_create("batt_health_config", 0666,
        asus_batt_cycle_count_dir, &batt_health_config_fops);

    if (!asus_batt_cycle_count_dir)
        CHG_DBG("[BAT][CHG]%s batt_cycle_count_dir create failed!\n", __func__);
    if (!asus_batt_cycle_count_proc_file)
        CHG_DBG("[BAT][CHG]%s batt_cycle_count_proc_file create failed!\n", __func__);
    if(!asus_batt_batt_safety_proc_file)
        CHG_DBG("[BAT][CHG]%s batt_safety_proc_file create failed!\n", __func__);
    if(!asus_batt_batt_safety_csc_proc_file)
        printk("batt_safety_csc_proc_file create failed!\n");
    if (!asus_batt_safety_condition_proc_file)
        printk(" create asus_batt_safety_condition_proc_file failed!\n");
    if (!batt_health_config_proc_file)
        printk(" create batt_health_config_proc_file failed!\n");
}

static int reboot_shutdown_prep(struct notifier_block *this,
                  unsigned long event, void *ptr)
{
    switch(event) {
    case SYS_RESTART:
    case SYS_POWER_OFF:
        /* Write data back to emmc */
        CHG_DBG_E("[BAT][CHG]%s John", __func__);
        write_back_cycle_count_data();
        break;
    default:
        break;
    }
    return NOTIFY_DONE;
}
/*  Call back function for reboot notifier chain  */
static struct notifier_block reboot_blk = {
    .notifier_call  = reboot_shutdown_prep,
};

int asuslib_init(void) {
    int rc = 0;
    struct pmic_glink_client_data client_data = { };
    struct pmic_glink_client    *client;

    printk(KERN_ERR "%s +++\n", __func__);
    // Initialize the necessary power supply
    rc = asus_init_power_supply_prop();
    if (rc < 0) {
        pr_err("Failed to init power_supply chains\n");
        return rc;
    }

    // Register the class node
    rc = class_register(&asuslib_class);
    if (rc) {
        pr_err("%s: Failed to register asuslib class\n", __func__);
        return -1;
    }

    if(g_ASUS_hwID <= HW_REV_EVB2)
    {
        POGO_OTG_GPIO = of_get_named_gpio(g_bcdev->dev->of_node, "POGO_OTG_EN", 0);
        rc = gpio_request(POGO_OTG_GPIO, "POGO_OTG_EN");
        if (rc) {
            pr_err("%s: Failed to initalize the POGO_OTG_EN\n", __func__);
            return -1;
        }
    }

    if(g_ASUS_hwID >= HW_REV_ER)
    {
        OTG_LOAD_SWITCH_GPIO = of_get_named_gpio(g_bcdev->dev->of_node, "OTG_LOAD_SWITCH", 0);
        rc = gpio_request(OTG_LOAD_SWITCH_GPIO, "OTG_LOAD_SWITCH");
        if (rc) {
            pr_err("%s: Failed to initalize the OTG_LOAD_SWITCH\n", __func__);
            return -1;
        }

        if (gpio_is_valid(OTG_LOAD_SWITCH_GPIO)) {
            rc = gpio_direction_output(OTG_LOAD_SWITCH_GPIO, 0);
            if (rc)
                pr_err("%s. Failed to control OTG_Load_Switch\n", __func__);
        } else {
            CHG_DBG_E("%s. OTG_LOAD_SWITCH_GPIO is invalid\n", __func__);
        }
    }

    //[+++]Register the extcon for quick_charger
    quickchg_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(quickchg_extcon)) {
        rc = PTR_ERR(quickchg_extcon);
        printk(KERN_ERR "[BAT][CHG] failed to allocate ASUS quickchg extcon device rc=%d\n", rc);
    }
    quickchg_extcon->fnode_name = "quick_charging";

    rc = extcon_dev_register(quickchg_extcon);
    if (rc < 0)
        printk(KERN_ERR "[BAT][CHG] failed to register ASUS quickchg extcon device rc=%d\n", rc);

    asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL0_DEFAULT);

    INIT_DELAYED_WORK(&asus_set_qc_state_work, asus_set_qc_state_worker);
    //[---]Register the extcon for quick_charger

    //[+++] Init the PMIC-GLINK
    client_data.id = PMIC_GLINK_MSG_OWNER_OEM;
    client_data.name = "asus_BC";
    client_data.msg_cb = asusBC_msg_cb;
    client_data.priv = g_bcdev;
    client_data.state_cb = asusBC_state_cb;
    client = pmic_glink_register_client(g_bcdev->dev, &client_data);
    if (IS_ERR(client)) {
        rc = PTR_ERR(client);
        if (rc != -EPROBE_DEFER)
            dev_err(g_bcdev->dev, "Error in registering with pmic_glink %d\n",
                rc);
        return rc;
    }
    //[---] Init the PMIC-GLINK

    //[+++] Init the info structure of ChargerPD from ADSP
    ChgPD_Info.PlatformID = 0;
    ChgPD_Info.BATT_ID = 0;
    // ChgPD_Info.VBUS_SRC = 0;
    ChgPD_Info.chg_limit_en = 0;
    ChgPD_Info.chg_limit_cap = 0;
    ChgPD_Info.usbin_suspend_en = 0;
    ChgPD_Info.charging_suspend_en = 0;
    ChgPD_Info.firmware_version = 0;
    ChgPD_Info.pm8350b_icl = 0;
    ChgPD_Info.smb1396_icl = 0;
    ChgPD_Info.batt_fcc = 0;


    slowchg_ws = wakeup_source_register(g_bcdev->dev, "Slowchg_wakelock");
    //[---] Init the info structure of ChargerPD from ADSP
    bat_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(bat_id_extcon)) {
        rc = PTR_ERR(bat_extcon);
    }
    bat_extcon->fnode_name = "battery";
    printk("[BAT]extcon_dev_register");
    rc = extcon_dev_register(bat_extcon);
    bat_extcon->name = st_battery_name;
    bat_id_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(bat_id_extcon)) {
        rc = PTR_ERR(bat_id_extcon);
    }       
    bat_id_extcon->fnode_name = "battery_id";
    printk("[BAT]extcon_dev_register");
    rc = extcon_dev_register(bat_id_extcon);

    //[+++]Register the extcon for thermal alert
    thermal_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(thermal_extcon)) {
        rc = PTR_ERR(thermal_extcon);
        printk(KERN_ERR "[BAT][CHG] failed to allocate ASUS thermal alert extcon device rc=%d\n", rc);
    }
    thermal_extcon->fnode_name = "usb_connector";

    rc = extcon_dev_register(thermal_extcon);
    if (rc < 0)
        printk(KERN_ERR "[BAT][CHG] failed to register ASUS thermal alert extcon device rc=%d\n", rc);

    usb_conn_temp_vadc_chan = iio_channel_get(g_bcdev->dev, "pm8350b_amux_thm4");
    if (IS_ERR_OR_NULL(usb_conn_temp_vadc_chan)) {
        CHG_DBG_E("%s: usb_conn_temp iio_channel_get fail\n", __func__);
    }

    INIT_DELAYED_WORK(&g_bcdev->asus_usb_thermal_work, asus_usb_thermal_worker);
    schedule_delayed_work(&g_bcdev->asus_usb_thermal_work, msecs_to_jiffies(0));
    //[---]Register the extcon for thermal alert

    //[+++]Register the extcon for adaptervid_extcon
    adaptervid_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(adaptervid_extcon)) {
        rc = PTR_ERR(adaptervid_extcon);
        printk(KERN_ERR "[BAT][CHG] failed to allocate ASUS adaptervid extcon device rc=%d\n", rc);
    }
    adaptervid_extcon->fnode_name = "adaptervid";

    rc = extcon_dev_register(adaptervid_extcon);
    if (rc < 0)
        printk(KERN_ERR "[BAT][CHG] failed to register ASUS adaptervid extcon device rc=%d\n", rc);

    asus_extcon_set_state_sync(adaptervid_extcon, 0);
    //[---]Register the extcon for adaptervid_extcon

    //thermal policy
    INIT_DELAYED_WORK(&asus_thermal_policy_work, asus_thermal_policy_worker);

    asus_get_Batt_ID();

	create_uts_status_proc_file(); //ASUS_BSP LiJen add to printk the WIFI hotspot & QXDM UTS event
	
	INIT_DELAYED_WORK(&g_bcdev->update_gauge_status_work, update_gauge_status_worker);
	schedule_delayed_work(&g_bcdev->update_gauge_status_work, 0);
	
	INIT_DELAYED_WORK(&g_bcdev->enter_ship_work, enter_ship_mode_worker);

    //register drm notifier
    INIT_DELAYED_WORK(&asus_set_panelonoff_current_work, asus_set_panelonoff_current_worker);
    RegisterDRMCallback();

    //jeita rule work
    INIT_DELAYED_WORK(&asus_jeita_rule_work, asus_jeita_rule_worker);
    INIT_DELAYED_WORK(&asus_jeita_prechg_work, asus_jeita_prechg_worker);
    INIT_DELAYED_WORK(&asus_jeita_cc_work, asus_jeita_cc_worker);

    //panel check work
    INIT_DELAYED_WORK(&asus_panel_check_work, asus_panel_check_worker);

    //slow charging work
    INIT_DELAYED_WORK(&asus_slow_charging_work, asus_slow_charging_worker);

    INIT_DELAYED_WORK(&asus_charger_mode_work, asus_charger_mode_worker);
    schedule_delayed_work(&asus_charger_mode_work, 0);

    //implement the asus owns algorithm of detection full capacity
    INIT_DELAYED_WORK(&asus_long_full_cap_monitor_work, asus_long_full_cap_monitor_worker);
    schedule_delayed_work(&asus_long_full_cap_monitor_work, msecs_to_jiffies(0));

    //18W workaround work
    INIT_DELAYED_WORK(&asus_18W_workaround_work, asus_18W_workaround_worker);

    //battery safety upgrade
    INIT_DELAYED_WORK(&battery_safety_work, battery_safety_worker);
    init_battery_safety(&safety_cond);
    // //init_batt_cycle_count_data();
    create_batt_cycle_count_proc_file();
    register_reboot_notifier(&reboot_blk);
    schedule_delayed_work(&battery_safety_work, 30 * HZ);

    //battery_health_work
    INIT_DELAYED_WORK(&battery_health_work, battery_health_worker);
    battery_health_data_reset();
    schedule_delayed_work(&battery_health_work, 30 * HZ);

    //cos battery 48hours protect
    INIT_DELAYED_WORK(&asus_min_check_work, asus_min_check_worker);
    if(g_Charger_mode) {
	CHG_DBG_E("Charger mode , start asus timer monitor\n");
    	schedule_delayed_work(&asus_min_check_work, msecs_to_jiffies(60000));
    }

    CHG_DBG_E("Load the asuslib_init Succesfully\n");
    g_asuslib_init = true;
    return rc;
}

int asuslib_deinit(void) {
    g_asuslib_init = false;
    class_unregister(&asuslib_class);
    wakeup_source_unregister(slowchg_ws);
    return 0;
}
