/*
 * Copyright (c) 2019-2020, The ASUS Company. All rights reserved.
 */

#define pr_fmt(fmt)	"BATTERY_CHG: %s: " fmt, __func__

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
#include <dt-bindings/iio/qcom,spmi-vadc.h>
#include <dt-bindings/iio/qcom,spmi-adc7-pm8350.h>
#include <dt-bindings/iio/qcom,spmi-adc7-pm8350b.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
//ASUS BSP ---

//[+++] Add the structure for PMIC-GLINK response

#include <drm/drm_panel.h>

struct ADSP_ChargerPD_Info {
	int PlatformID;
	int BATT_ID;
	int    VBUS_SRC;//The VBUS is from side or bottom
	bool   chg_limit_en;
	u32    chg_limit_cap;
	bool   usbin_suspend_en;
    bool   charging_suspend_en;
	char   ChgPD_FW[64];
	int firmware_version;
	int batt_temp;
};

struct evtlog_context_resp_msg3 {
	struct pmic_glink_hdr		hdr;
	u8				buf[128];
	u32				reserved;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct oem_get_platformID_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_platformID_resp {
	struct pmic_glink_hdr	hdr;
	u32    PlatID_version;
};

struct oem_get_BattID_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_BattID_resp {
	struct pmic_glink_hdr	hdr;
	u32    Batt_ID;
};

struct oem_get_VBUS_SRC_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_VBUS_SRC_resp {
	struct pmic_glink_hdr	hdr;
	u32    vbus_src;
};

struct oem_set_USB2_Client_resp {
	struct pmic_glink_hdr	hdr;
	u32    on;
};

struct oem_set_BTM_OTG_req {
	struct pmic_glink_hdr	hdr;
	u32    on;
};

struct oem_set_BTM_OTG_resp {
	struct pmic_glink_hdr	hdr;
};

struct oem_chg_limit_en_req {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_chg_limit_en_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_chg_limit_cap_req {
	struct pmic_glink_hdr	hdr;
	u32    cap;
};

struct oem_chg_limit_cap_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_usbin_suspend_en_req {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_usbin_suspend_en_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_charging_suspend_en_req {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_charging_suspend_en_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_get_ChgPD_FW_Ver_resp {
	struct pmic_glink_hdr	hdr;
	char   ver[64] ;
};
struct oem_get_FW_version_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_FW_version_resp {
	struct pmic_glink_hdr	hdr;
	u16    fw_version;
};
struct oem_get_batt_temp_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_batt_temp_resp {
	struct pmic_glink_hdr	hdr;
	int    batt_temp;
};

struct oem_set_hwid_to_ADSP_req {
	struct pmic_glink_hdr	hdr;
	u32    hwid_val;
};

struct oem_set_hwid_to_ADSP_resp {
	struct pmic_glink_hdr	hdr;
};

struct oem_set_SideOTG_WA_resp {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_set_Charger_Type_resp {
	struct pmic_glink_hdr	hdr;
	u32    charger_type;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

struct oem_chg_limit_mode_req {
	struct pmic_glink_hdr	hdr;
	u32    mode;
};

struct oem_chg_limit_mode_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_chg_limit_mode2_req {
	struct pmic_glink_hdr	hdr;
	u32    mode;
	u32    value;
};

struct oem_evt_adsp_to_hlos_req {
	struct pmic_glink_hdr	hdr;
	u32    EvtID;
	u32    value;
};

struct extcon_dev	*bat_id_extcon;
struct extcon_dev	*bat_extcon;
struct extcon_dev	*quickchg_extcon;
struct extcon_dev	*thermal_extcon;
struct extcon_dev	*audio_dongle_extcon;
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state);
//Define the OWNER ID
#define PMIC_GLINK_MSG_OWNER_OEM	32782
#define MSG_OWNER_BC				32778

//Define the opcode
#define OEM_ASUS_EVTLOG_IND			0x1002
#define OEM_PD_EVTLOG_IND			0x1003
#define OME_GET_BATT_ID				0x2001
#define OEM_GET_ADSP_PLATFORM_ID	0x2002
#define OEM_GET_VBUS_SOURCE			0x2003
#define OEM_GET_CHG_LIMIT			0x2004 //OEM_SET_CHG_LIMIT = 0x2103
#define OEM_GET_CHG_LIMIT_CAP		0x2005 //OEM_SET_CHG_LIMIT_CAP = 0x2104
#define OEM_GET_USBIN_SUSPNED		0x2006 //OEM_SET_USBIN_SUSPNED = 0x2105
#define OEM_GET_ChgPD_FW_VER		0x2007
#define OEM_GET_CHARGING_SUSPNED	0x2008 //OEM_SET_CHARGING_SUSPNED = 0x2108
#define OEM_GET_CHG_LIMIT_MODE		0x2009 //OEM_SET_CHG_LIMIT_MODE = 0x2110
#define OEM_PANELONOFF_CHG_LIMIT_REQ	0x2010

#define OEM_SET_USB2_CLIENT			0x2101
#define OEM_SET_BTM_OTG				0x2102
#define OEM_SET_CHG_LIMIT			0x2103
#define OEM_SET_CHG_LIMIT_CAP		0x2104
#define OEM_SET_USBIN_SUSPNED		0x2105
#define OEM_SET_HWID_TO_ADSP		0x2106
#define OEM_SET_SideOTG_WA			0x2107
#define OEM_SET_CHARGING_SUSPNED	0x2108
#define OEM_SET_CHARGER_TYPE_CHANGE	0x2109
#define OEM_SET_CHG_LIMIT_MODE		0x2110
#define OEM_SET_DEBUG_MASK_REQ		0x2111
#define OEM_EVT_ADSP_TO_HLOS_REQ	0x2112

#define OEM_OVERWRITE_I_LIMIT_REQ	0x2113
#define OEM_OVERWRITE_V_LIMIT_REQ	0x2114
#define OEM_OVERWRITE_I_STEP_REQ	0x2115
#define OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ	0x2116
#define OEM_SET_CHG_LIMIT_MODE2		0x2117

#define OEM_GET_FW_version			0x3001
#define OEM_GET_Batt_temp			0x3002
//Define Message Type
#define MSG_TYPE_REQ_RESP	1
#define MSG_TYPE_NOTIFICATION	2

//define OEM_SET_CHG_LIMIT_MODE
#define SET_SLOW_CHG_MODE		0x1
#define SET_SIDE_THM_ALT_MODE	0x2
#define SET_BTM_THM_ALT_MODE	0x4
#define SET_SMART_CHG_MODE		0x8
#define SET_DEMO_APP_MODE		0x16

//define the event ID from ADSP to HLOS
#define ADSP_HLOS_EVT_ADUIO_INVALID 0x01
//[---] Add the structure for PMIC-GLINK response

//[+++] Add debug log
#define CHARGER_TAG "[BAT][CHG]"
#define ERROR_TAG "[ERR]"
#define CHG_DBG(...)  printk(KERN_INFO CHARGER_TAG __VA_ARGS__)
#define CHG_DBG_E(...)  printk(KERN_ERR CHARGER_TAG ERROR_TAG __VA_ARGS__)
//[---] Add debug log

//[+++] Add the global variables
#define ASUS_CHARGER_TYPE_LEVEL0 0 // For disconnection, reset to default
#define ASUS_CHARGER_TYPE_LEVEL1 1 // This is for normal 18W QC3 or PD
#define ASUS_CHARGER_TYPE_LEVEL2 2 // This is for ASUS 30W adapter
#define ASUS_CHARGER_TYPE_LEVEL3 3 // This is for ASUS 65W adapter

#define SWITCH_LEVEL4_NOT_QUICK_CHARGING    8 //Now, this is used for 65W
#define SWITCH_LEVEL4_QUICK_CHARGING        7 //Now, this is used for 65W
#define SWITCH_LEVEL3_NOT_QUICK_CHARGING    6 //EQual to SWITCH_NXP_NOT_QUICK_CHARGING(ASUS 30W)
#define SWITCH_LEVEL3_QUICK_CHARGING        5 //EQual to SWITCH_NXP_QUICK_CHARGING(ASUS 30W)
#define SWITCH_LEVEL1_NOT_QUICK_CHARGING    4 //EQual to SWITCH_QC_NOT_QUICK_CHARGING(DCP 10W)
#define SWITCH_LEVEL1_QUICK_CHARGING        3 //EQual to SWITCH_QC_QUICK_CHARGING (DCP 10W)
#define SWITCH_LEVEL2_NOT_QUICK_CHARGING    2 //EQual to SWITCH_QC_NOT_QUICK_CHARGING_PLUS (QC 18W)
#define SWITCH_LEVEL2_QUICK_CHARGING        1 //EQual to SWITCH_QC_QUICK_CHARGING_PLUS (QC 18W)
#define SWITCH_LEVEL0_DEFAULT               0 //EQual to SWITCH_QC_OTHER
int g_SWITCH_LEVEL = SWITCH_LEVEL0_DEFAULT;
struct delayed_work	asus_set_qc_state_work;

extern struct battery_chg_dev *g_bcdev;
struct power_supply *qti_phy_usb;
struct power_supply *qti_phy_bat;
int PMI_MUX_GPIO;
int POGO_OTG_GPIO;
struct ADSP_ChargerPD_Info ChgPD_Info;
char st_battery_name[64] = "C21P2001-T-03-0001-0.17";
extern int usb2_host_mode;
u32 chg_limit_mode;
//[---] Add the global variables

//[+++] Add the external function
extern int battery_chg_write(struct battery_chg_dev *bcdev, void *data, int len);
extern int rt_chg_get_during_swap(void);

typedef void(*dwc3_role_switch_fn)(bool);
dwc3_role_switch_fn dwc3_role_switch;
//[---] Add the external function

//[+++] Add thermal alert adc function
bool usb_alert_side_flag = 0;
bool usb_alert_btm_flag = 0;
bool g_once_usb_thermal_btm = 0;
bool g_once_usb_thermal_side = 0;
int g_temp_state = 0;
int g_temp_side_state = 0;
int g_temp_btm_state = 0;
static struct alarm bat_alarm;
struct timespec64  last_check_time;
struct notifier_block charge_notify;
struct iio_channel *side_usb_temp_vadc_chan;
struct iio_channel *btm_usb_temp_vadc_chan;
struct delayed_work	asus_min_check_work;
//[+++] Add thermal alert adc function

#if 0
static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
};
#endif
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

static struct notifier_block fb_notif;
static struct drm_panel *active_panel;
struct delayed_work	asus_set_panelonoff_current_work;
int g_drm_blank = 0;

int asus_set_panelonoff_charging_current_limit(u32 panelOn);
int asus_set_invalid_audio_dongle(int src, int set);

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
//drm_panel_notifier_register(active_panel, &

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

int asus_set_charger_limit_mode(u32 mode, u32 value)
{
	struct oem_chg_limit_mode2_req req_msg = { { 0 } };
	int rc;

	CHG_DBG("%s. mode : 0x%x, value : %d\n", __func__, mode, value);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHG_LIMIT_MODE2;
	req_msg.mode = mode;
	req_msg.value= value;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set charger_limit_mode rc=%d\n", rc);
		return rc;
	}
	return 0;
}

int asus_set_panelonoff_charging_current_limit(u32 panelOn)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;

	pr_err("panelOn= 0x%x\n", panelOn);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_PANELONOFF_CHG_LIMIT_REQ;
	req_msg.mode = panelOn;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set asus_set_panelonoff_charging_current_limit rc=%d\n", rc);
		return rc;
	}
	return 0;
}

//[+++]Add the PMIC-GLINK interface of external functions for USB or RT1715 drivers
int BTM_OTG_EN(bool enable)
{
	struct oem_set_BTM_OTG_req req_msg = { { 0 } };
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_BTM_OTG;
	req_msg.on = enable;
	CHG_DBG("%s. enable : %d", __func__, enable);
	if (g_bcdev == NULL) {
		pr_err("g_bcdev is null\n");
		return -1;
	}
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set BTM OTG rc=%d\n", rc);
		return rc;
	}
	return 0;
}
EXPORT_SYMBOL(BTM_OTG_EN);

int PASS_HWID_TO_ADSP() {
    struct oem_set_hwid_to_ADSP_req req_msg = { { 0 } };
	int rc;

	if (g_ASUS_hwID < 0) {
		CHG_DBG("%s. Incorrect HWID : %d", __func__, g_ASUS_hwID);
		return -1;
	}
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_HWID_TO_ADSP;
	req_msg.hwid_val = g_ASUS_hwID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set HWID to ADSP rc=%d\n", rc);
		return rc;
	}
	return 0;
}
//[---]Add the PMIC-GLINK interface of external functions for USB or RT1715 drivers

static ssize_t BTM_OTG_EN1_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int rc, tmp;
	bool btm_otg_en;
	tmp = simple_strtol(buf, NULL, 10);

	CHG_DBG("%s. Set BTM_OTG_EN : %d\n", __func__, tmp);
	btm_otg_en = tmp;
	rc = BTM_OTG_EN(btm_otg_en);
	if (rc)
		pr_err("%s. Failed to control BTM_OTG_EN\n", __func__);
	return count;
}

static ssize_t BTM_OTG_EN1_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "BTM_OTG read OK\n");
}
static CLASS_ATTR_RW(BTM_OTG_EN1);

static ssize_t pmi_mux_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int rc, tmp;
	bool pmi_mux_en;
	tmp = simple_strtol(buf, NULL, 10);

	CHG_DBG("%s. Set PMI_MUX_EN : %d\n", __func__, tmp);
	pmi_mux_en = tmp;
	rc = gpio_direction_output(PMI_MUX_GPIO, pmi_mux_en);
	if (rc)
		pr_err("%s. Failed to control PMI_MUX_EN\n", __func__);
	return count;
}

static ssize_t pmi_mux_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	bool PMI_MUX_status;
	PMI_MUX_status = gpio_get_value_cansleep(PMI_MUX_GPIO);

	return scnprintf(buf, PAGE_SIZE, "PMI_MUX_EN : %d\n", PMI_MUX_status);
}
static CLASS_ATTR_RW(pmi_mux_en);

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
	struct oem_get_platformID_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_ADSP_PLATFORM_ID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
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
	struct oem_get_BattID_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OME_GET_BATT_ID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get BattID rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.BATT_ID);
}
static CLASS_ATTR_RO(asus_get_BattID);

static ssize_t POGO_OTG_EN_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int rc, tmp;
	bool otg_en;
	tmp = simple_strtol(buf, NULL, 10);

	CHG_DBG("%s. Set POGO_OTG_EN : %d\n", __func__, tmp);
	otg_en = tmp;
	rc = gpio_direction_output(POGO_OTG_GPIO, otg_en);
	if (rc)
		pr_err("%s. Failed to control POGO_OTG_EN\n", __func__);
	return count;
}

static ssize_t POGO_OTG_EN_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	bool POGO_OTG_status;
	POGO_OTG_status = gpio_get_value_cansleep(POGO_OTG_GPIO);

	return scnprintf(buf, PAGE_SIZE, "%d\n", POGO_OTG_status);
}
static CLASS_ATTR_RW(POGO_OTG_EN);

static ssize_t get_usb_type_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	//struct power_supply *psy;
	struct psy_state *pst;
	int rc = 0 , val = 0;

	if (g_bcdev == NULL)
		return -1;
	CHG_DBG("%s\n", __func__);
	pst = &g_bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(g_bcdev, pst, 7);//7:USB_ADAP_TYPE
	if (!rc) {
		val = pst->prop[7];//7:USB_ADAP_TYPE
		CHG_DBG("%s. val : %d\n", __func__, val);
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", power_supply_usb_type_text[val]);
}
static CLASS_ATTR_RO(get_usb_type);

static ssize_t vbus_side_btm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_VBUS_SRC_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_VBUS_SOURCE;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get VBUS_SOURCE rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.VBUS_SRC);
}
static CLASS_ATTR_RO(vbus_side_btm);

static ssize_t charger_limit_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_en_req req_msg = { { 0 } };
	int rc, tmp;

	tmp = simple_strtol(buf, NULL, 10);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHG_LIMIT;
	req_msg.enable = (bool) tmp;

	CHG_DBG("%s. enable : %d", __func__, req_msg.enable);
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set CHG_LIMIT rc=%d\n", rc);
		return rc;
	}

	return count;
}

static ssize_t charger_limit_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_chg_limit_en_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHG_LIMIT;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get CHG_LIMIT rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.chg_limit_en);
}
static CLASS_ATTR_RW(charger_limit_en);

static ssize_t charger_limit_cap_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_cap_req req_msg = { { 0 } };
	int rc, tmp;

	tmp = simple_strtol(buf, NULL, 10);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHG_LIMIT_CAP;
	req_msg.cap = tmp;

	CHG_DBG("%s. cap : %d", __func__, req_msg.cap);
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set CHG_LIMIT_CAP rc=%d\n", rc);
		return rc;
	}

	return count;
}

static ssize_t charger_limit_cap_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_chg_limit_cap_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHG_LIMIT_CAP;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
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
	struct oem_usbin_suspend_en_req req_msg = { { 0 } };
	int rc, tmp;

	tmp = simple_strtol(buf, NULL, 10);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_USBIN_SUSPNED;
	req_msg.enable = tmp;

	CHG_DBG("%s. enable : %d", __func__, req_msg.enable);
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set USBIN_SUSPEND_EN rc=%d\n", rc);
		return rc;
	}

	return count;
}

static ssize_t usbin_suspend_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_usbin_suspend_en_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_USBIN_SUSPNED;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
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
	struct oem_charging_suspend_en_req req_msg = { { 0 } };
	int rc, tmp;

	tmp = simple_strtol(buf, NULL, 10);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHARGING_SUSPNED;
	req_msg.enable = tmp;

	CHG_DBG("%s. enable : %d", __func__, req_msg.enable);
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set CHARGING_SUSPEND_EN rc=%d\n", rc);
		return rc;
	}

	return count;
}

static ssize_t charging_suspend_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_charging_suspend_en_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHARGING_SUSPNED;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
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
	struct oem_get_ChgPD_FW_Ver_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_ChgPD_FW_VER;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("%s. Failed to get ChgPD_FW_Ver rc=%d\n", __func__, rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", ChgPD_Info.ChgPD_FW);
}
static CLASS_ATTR_RO(get_ChgPD_FW_Ver);
int asus_get_Batt_ID(void)
{
	struct oem_get_BattID_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OME_GET_BATT_ID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get BattID rc=%d\n", rc);
		return rc;
	}
	return 0;
}
static ssize_t asus_get_fw_version_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_FW_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_FW_version;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
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
	struct oem_get_batt_temp_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_Batt_temp;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get FW_version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.batt_temp);
}
static CLASS_ATTR_RO(asus_get_batt_temp);
//[---] Add the interface for accesing the inforamtion of ChargerPD on ADSP

//[---] Add the interface for usb thermal alert temp
static ssize_t once_usb_thermal_side_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	if (g_once_usb_thermal_side)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "PASS\n");
}
static CLASS_ATTR_RO(once_usb_thermal_side);

static ssize_t once_usb_thermal_btm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	if (g_once_usb_thermal_btm)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "PASS\n");

}
static CLASS_ATTR_RO(once_usb_thermal_btm);

static ssize_t enter_ship_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };

	int rc, tmp;
	bool ship_en;
	tmp = simple_strtol(buf, NULL, 10);
	ship_en = tmp;
	if (ship_en == 0) {
		CHG_DBG("%s. NO action for SHIP mode\n", __func__);
		return count;
	}
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = 0x36;// = BC_SHIP_MODE_REQ_SET
	msg.ship_mode_type = 0;// = SHIP_MODE_PMIC

	rc = battery_chg_write(g_bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_err("%s. Failed to write SHIP mode: %d\n", rc);
	CHG_DBG("%s. Set SHIP Mode OK\n", __func__);

	return count;
}

static ssize_t enter_ship_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(enter_ship_mode);


static ssize_t set_debugmask_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 16);

	pr_err(" mask= 0x%x\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_DEBUG_MASK_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_debugmask_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_debugmask_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_debugmask);

static ssize_t set_virtualthermal_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 16);

	pr_err(" mask= 0x%x\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_virtualthermal_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_virtualthermal_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_virtualthermal);

static ssize_t set_i_limit_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 10);

	pr_err(" set_i_limit= %d\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_OVERWRITE_I_LIMIT_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_i_limit_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_i_limit_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_i_limit);
static ssize_t set_v_limit_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 10);

	pr_err(" set_v_limit= %d\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_OVERWRITE_V_LIMIT_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_v_limit_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_v_limit_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_v_limit);

static ssize_t set_i_step_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 10);

	pr_err(" set_i_step= %d\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_OVERWRITE_I_STEP_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_i_step_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_i_step_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_i_step);
static ssize_t charger_limit_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	u32 mode = 0, value = 0;
	sscanf(buf, "%d,%d", &mode, &value);
	asus_set_charger_limit_mode(mode, value);

	return count;
}

static ssize_t charger_limit_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "No function\n");
	//Do this later.
	#if 0
	struct oem_chg_limit_mode_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHG_LIMIT_MODE;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get CHG_LIMIT_MODE rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_limit_mode);
	#endif
}
static CLASS_ATTR_RW(charger_limit_mode);
//[---] Add the interface for usb thermal alert temp

static struct attribute *asuslib_class_attrs[] = {
	&class_attr_BTM_OTG_EN1.attr,
	&class_attr_pmi_mux_en.attr,
	&class_attr_asus_get_FG_SoC.attr,
	&class_attr_asus_get_PlatformID.attr,
	&class_attr_asus_get_BattID.attr,
	&class_attr_POGO_OTG_EN.attr,
	&class_attr_get_usb_type.attr,
	&class_attr_vbus_side_btm.attr,
	&class_attr_charger_limit_en.attr,
	&class_attr_charger_limit_cap.attr,
	&class_attr_usbin_suspend_en.attr,
	&class_attr_charging_suspend_en.attr,
	&class_attr_get_ChgPD_FW_Ver.attr,
	&class_attr_asus_get_fw_version.attr,
	&class_attr_asus_get_batt_temp.attr,
	&class_attr_once_usb_thermal_side.attr,
	&class_attr_once_usb_thermal_btm.attr,
	&class_attr_enter_ship_mode.attr,
	&class_attr_charger_limit_mode.attr,
	&class_attr_set_debugmask.attr,
	&class_attr_set_i_limit.attr,
	&class_attr_set_v_limit.attr,
	&class_attr_set_i_step.attr,
	&class_attr_set_virtualthermal.attr,
	NULL,
};
ATTRIBUTE_GROUPS(asuslib_class);

struct class asuslib_class = {
	.name = "asuslib",
	.class_groups = asuslib_class_groups,
};

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
	struct evtlog_context_resp_msg3	*evtlog_msg;
	struct oem_set_USB2_Client_resp *usb2_client_msg;
	struct oem_set_SideOTG_WA_resp *SideOTG_WA_msg;
	struct oem_set_Charger_Type_resp *Update_charger_type_msg;
	struct oem_evt_adsp_to_hlos_req *evt_adsp_to_hlos_msg;
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
    case OEM_SET_USB2_CLIENT:
        if (len == sizeof(*usb2_client_msg)) {
            usb2_client_msg = data;
            CHG_DBG("%s OEM_SET_USB2_CLIENT. enable : %d, usb2_host_mode : %d\n", __func__, usb2_client_msg->on, usb2_host_mode);
            if (dwc3_role_switch != NULL) {
                if (usb2_client_msg->on == 0 && rt_chg_get_during_swap())
                    CHG_DBG("Skip to send dwc3_role_switch because swap\n");
				else if (usb2_host_mode == 1)
					CHG_DBG("Skip to send dwc3_role_switch because USB2_HOST active\n");
                else
                    dwc3_role_switch(usb2_client_msg->on);
            } else {
                CHG_DBG_E("%s OEM_SET_USB2_CLIENT. dwc3_role_switch = NULL\n", __func__);
            }
        } else {
            pr_err("Incorrect response length %zu for ome_set_USB2_client\n",
                len);
        }
        break;
    case OEM_SET_SideOTG_WA:
        if (len == sizeof(*SideOTG_WA_msg)) {
            SideOTG_WA_msg = data;
            CHG_DBG("%s OEM_SET_SideOTG_WA. enable : %d, HWID : %d\n", __func__, SideOTG_WA_msg->enable, g_ASUS_hwID);
            if (g_ASUS_hwID == HW_REV_SR || (g_ASUS_hwID >= HW_REV_ER2)) {
                CHG_DBG("%s. Skip to control SideOTG for SR or ER2 later\n", __func__);
                break;
            }
            if (gpio_is_valid(POGO_OTG_GPIO)) {
                rc = gpio_direction_output(POGO_OTG_GPIO, SideOTG_WA_msg->enable);
                if (rc)
                    pr_err("%s. Failed to control POGO_OTG_EN\n", __func__);
            } else {
                CHG_DBG_E("%s. POGO_OTG_GPIO is invalid\n", __func__);
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_SideOTG_WA\n",
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
            }
            pre_chg_type = Update_charger_type_msg->charger_type;
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_CHARGER_TYPE_CHANGE\n",
                len);
        }
        break;
        case OEM_EVT_ADSP_TO_HLOS_REQ:
        if (len == sizeof(*evt_adsp_to_hlos_msg)) {
            evt_adsp_to_hlos_msg = data;
            CHG_DBG("%s[OEM_EVT_ADSP_TO_HLOS_REQ] EVTID : 0x%x, value : %d\n", __func__
                    ,evt_adsp_to_hlos_msg->EvtID
                    ,evt_adsp_to_hlos_msg->value);
            if (evt_adsp_to_hlos_msg->EvtID == ADSP_HLOS_EVT_ADUIO_INVALID) {
                asus_set_invalid_audio_dongle(1, evt_adsp_to_hlos_msg->value);
            }
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
	struct oem_get_platformID_resp *platform_id_msg;
	struct oem_get_BattID_resp *batt_id_msg;
	struct oem_get_VBUS_SRC_resp *vbus_src_msg;
	struct oem_set_BTM_OTG_resp *btm_otg_resp_msg;
	struct oem_chg_limit_en_req *chg_limit_en_req_msg;
	struct oem_chg_limit_en_resp *chg_limit_en_resp_msg;
	struct oem_chg_limit_cap_req *chg_limit_cap_req_msg;
	struct oem_chg_limit_cap_resp *chg_limit_cap_resp_msg;
	struct oem_usbin_suspend_en_req *usbin_suspend_en_req_msg;
	struct oem_usbin_suspend_en_resp *usbin_suspend_en_resp_msg;
    struct oem_charging_suspend_en_req *charging_suspend_en_req_msg;
    struct oem_charging_suspend_en_resp *charging_suspend_en_resp_msg;
	struct oem_get_ChgPD_FW_Ver_resp *ChgPD_FW_Ver_msg;
	struct oem_set_hwid_to_ADSP_resp *set_HWID_To_ADSP_msg;
	struct oem_chg_limit_mode_req *chg_limit_mode_req_msg;
	struct oem_chg_limit_mode2_req *chg_limit_mode2_req_msg;
	struct oem_chg_limit_mode_resp *chg_limit_mode_resp_msg;
	struct pmic_glink_hdr *hdr = data;
	bool ack_set = false;
	struct oem_get_FW_version_resp *fw_version_msg;
	struct oem_get_batt_temp_resp *batt_temp_msg;

	switch (hdr->opcode) {
	case OEM_GET_ADSP_PLATFORM_ID:
		if (len == sizeof(*platform_id_msg)) {
			platform_id_msg = data;
			ChgPD_Info.PlatformID = platform_id_msg->PlatID_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for asus_get_PlatformID\n",
				len);
		}
		break;
	case OME_GET_BATT_ID:
		if (len == sizeof(*batt_id_msg)) {
			batt_id_msg = data;
			ChgPD_Info.BATT_ID = batt_id_msg->Batt_ID;
			ack_set = true;
			if(ChgPD_Info.BATT_ID < 51000*1.15 && ChgPD_Info.BATT_ID > 51000*0.85)
				asus_extcon_set_state_sync(bat_id_extcon, 1);
			else if(ChgPD_Info.BATT_ID < 100000*1.15 && ChgPD_Info.BATT_ID > 100000*0.85)
				asus_extcon_set_state_sync(bat_id_extcon, 1);
			else
				asus_extcon_set_state_sync(bat_id_extcon, 0);
		} else {
			pr_err("Incorrect response length %zu for asus_get_BattID\n",
				len);
		}
		break;
	case OEM_GET_FW_version:
		if (len == sizeof(*fw_version_msg)) {
			fw_version_msg = data;
			ChgPD_Info.firmware_version = fw_version_msg->fw_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for asus_FW_version\n",
				len);
		}
		break;
	case OEM_GET_Batt_temp:
		if (len == sizeof(*batt_temp_msg)) {
			batt_temp_msg = data;
			ChgPD_Info.batt_temp = batt_temp_msg->batt_temp;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for asus_batt_temp\n",
				len);
		}
		break;
	case OEM_GET_VBUS_SOURCE:
		if (len == sizeof(*vbus_src_msg)) {
			vbus_src_msg = data;
			ChgPD_Info.VBUS_SRC 	= vbus_src_msg->vbus_src;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for get VBUS_SRC\n",
				len);
		}
		break;
	case OEM_SET_BTM_OTG:
		if (len == sizeof(*btm_otg_resp_msg)) {
			CHG_DBG("%s OEM_SET_BTM_OTG successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_BTM_OTG\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT:
		if (len == sizeof(*chg_limit_en_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT\n",
				len);
		}
		break;
	case OEM_GET_CHG_LIMIT:
		if (len == sizeof(*chg_limit_en_resp_msg)) {
			chg_limit_en_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHG_LIMIT successfully\n", __func__);
			ChgPD_Info.chg_limit_en = chg_limit_en_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHG_LIMIT\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT_CAP:
		CHG_DBG("%s. OEM_SET_CHG_LIMIT_CAP. len :%d, sizeof : %d \n", __func__, len, sizeof(*chg_limit_cap_req_msg));
		if (len == sizeof(*chg_limit_cap_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT_CAP successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT_CAP\n",
				len);
		}
		break;
	case OEM_GET_CHG_LIMIT_CAP:
		CHG_DBG("%s. OEM_GET_CHG_LIMIT_CAP. len :%d, sizeof : %d \n", __func__, len, sizeof(*chg_limit_cap_resp_msg));
		if (len == sizeof(*chg_limit_cap_resp_msg)) {
			chg_limit_cap_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHG_LIMIT_CAP successfully\n", __func__);
			ChgPD_Info.chg_limit_cap = chg_limit_cap_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHG_LIMIT_CAP\n",
				len);
		}
		break;
	case OEM_SET_USBIN_SUSPNED:
		if (len == sizeof(*usbin_suspend_en_req_msg)) {
			CHG_DBG("%s OEM_SET_USBIN_SUSPNED successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_USBIN_SUSPNED\n",
				len);
		}
		break;
	case OEM_GET_USBIN_SUSPNED:
		if (len == sizeof(*usbin_suspend_en_resp_msg)) {
			usbin_suspend_en_resp_msg = data;
			CHG_DBG("%s OEM_GET_USBIN_SUSPNED successfully\n", __func__);
			ChgPD_Info.usbin_suspend_en = usbin_suspend_en_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_USBIN_SUSPNED\n",
				len);
		}
		break;
    case OEM_SET_CHARGING_SUSPNED:
		if (len == sizeof(*charging_suspend_en_req_msg)) {
			CHG_DBG("%s OEM_SET_CHARGING_SUSPNED successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHARGING_SUSPNED\n",
				len);
		}
		break;
	case OEM_GET_CHARGING_SUSPNED:
		if (len == sizeof(*charging_suspend_en_resp_msg)) {
			charging_suspend_en_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHARGING_SUSPNED successfully\n", __func__);
			ChgPD_Info.charging_suspend_en = charging_suspend_en_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHARGING_SUSPNED\n",
				len);
		}
		break;
	case OEM_GET_ChgPD_FW_VER:
		if (len == sizeof(*ChgPD_FW_Ver_msg)) {
			ChgPD_FW_Ver_msg = data;
			CHG_DBG("%s. ChgPD_FW : %s\n", __func__, ChgPD_FW_Ver_msg->ver);
			strcpy(ChgPD_Info.ChgPD_FW, ChgPD_FW_Ver_msg->ver);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_ChgPD_FW_VER\n",
				len);
		}
		break;
	case OEM_SET_HWID_TO_ADSP:
		if (len == sizeof(*set_HWID_To_ADSP_msg)) {
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_HWID_TO_ADSP\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT_MODE:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT_MODE successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT_MODE\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT_MODE2:
		if (len == sizeof(*chg_limit_mode2_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT_MODE2 successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT_MODE2\n",
				len);
		}
		break;
	case OEM_GET_CHG_LIMIT_MODE:
		if (len == sizeof(*chg_limit_mode_resp_msg)) {
			chg_limit_mode_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHG_LIMIT_MODE successfully\n", __func__);
			chg_limit_mode = chg_limit_mode_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHG_LIMIT_MODE\n",
				len);
		}
		break;
	case OEM_PANELONOFF_CHG_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_PANELONOFF_CHG_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_PANELONOFF_CHG_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_SET_DEBUG_MASK_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_SET_DEBUG_MASK_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_DEBUG_MASK_REQ\n",
				len);
		}
		break;

	case OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_OVERWRITE_I_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_OVERWRITE_I_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_OVERWRITE_I_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_OVERWRITE_V_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_OVERWRITE_V_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_OVERWRITE_V_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_OVERWRITE_I_STEP_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_OVERWRITE_I_STEP_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_OVERWRITE_I_STEP_REQ\n",
				len);
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

	//pr_err("owner: %u type: %u opcode: %u len: %zu\n", hdr->owner, hdr->type, hdr->opcode, len);

	if (hdr->owner == PMIC_GLINK_MSG_OWNER_OEM) {
		if (hdr->type == MSG_TYPE_NOTIFICATION)
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

void qti_battery_register_switch(void *funcPtr){

	if (funcPtr) {
		pr_info("register switch for usb2 client\n");
		dwc3_role_switch = (dwc3_role_switch_fn)funcPtr;
	}
}
EXPORT_SYMBOL(qti_battery_register_switch);

int asus_set_invalid_audio_dongle(int src, int set) {
	static int side_value = 0, btm_value = 0;
	static int final_value = 0, pre_value = 0;

	if (src == 1)
		side_value = set;
	else if (src == 2)
		btm_value = set;

	final_value = side_value | btm_value;
	CHG_DBG("invalid_audio_dongle. src = %d, set = %d, final_value = %d\n", src, set, final_value);
	if (pre_value != final_value)
		asus_extcon_set_state_sync(audio_dongle_extcon, final_value);

	pre_value = final_value;

	return 0;
}
EXPORT_SYMBOL(asus_set_invalid_audio_dongle);

static DEFINE_SPINLOCK(bat_alarm_slock);
static enum alarmtimer_restart batAlarm_handler(struct alarm *alarm, ktime_t now)
{
	CHG_DBG("%s: batAlarm triggered\n", __func__);
	return ALARMTIMER_NORESTART;
}

int g_temp_THR = 70000;
int g_recovery_temp_THR = 60000;
int asus_thermal_side(void)
{
	int rc;
	int adc_temp;

	if (IS_ERR_OR_NULL(side_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel not ready\n", __func__);
		return -ENXIO;
	}

	CHG_DBG("%s: start\n", __func__);

	rc = iio_read_channel_processed(side_usb_temp_vadc_chan, &adc_temp);
	if (rc < 0)
		CHG_DBG_E("%s: iio_read_channel_processed fail\n", __func__);
	else
		CHG_DBG("%s: side_adc_temp = %d\n", __func__, adc_temp);

	if (adc_temp > g_temp_THR && !usb_alert_side_flag) {
		usb_alert_side_flag = 1;
		g_once_usb_thermal_side = 1;
		asus_set_charger_limit_mode(SET_SIDE_THM_ALT_MODE, 1);
	} else if (adc_temp < g_recovery_temp_THR && usb_alert_side_flag) {
		usb_alert_side_flag = 0;
	}
	CHG_DBG("%s: end\n", __func__);

	return 0;
}

int asus_thermal_btm(void)
{
	int rc;
	int adc_temp;

	CHG_DBG("%s: start\n", __func__);

	if (IS_ERR_OR_NULL(btm_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel not ready\n", __func__);
		return -ENXIO;
	}

	rc = iio_read_channel_processed(btm_usb_temp_vadc_chan, &adc_temp);
	if (rc < 0)
		CHG_DBG_E("%s: iio_read_channel_processed fail\n", __func__);
	else
		CHG_DBG("%s: side_adc_temp = %d\n", __func__, adc_temp);

	if (adc_temp > g_temp_THR && !usb_alert_btm_flag) {
		usb_alert_btm_flag = 1;
		g_once_usb_thermal_btm = 1;
		asus_set_charger_limit_mode(SET_BTM_THM_ALT_MODE, 1);
	} else if (adc_temp < g_recovery_temp_THR && usb_alert_btm_flag) {
		usb_alert_btm_flag = 0;
	}

	CHG_DBG("%s: end\n", __func__);

	return 0;
}

void asus_min_check_worker(struct work_struct *work)
{
	unsigned long batflags;
	struct timespec new_batAlarm_time;
	int RTCSetInterval = 60;
	int rc;

	CHG_DBG("%s: start\n", __func__);

	if (!g_bcdev) {
		CHG_DBG("%s: driver not ready yet!\n", __func__);		
		pm_relax(g_bcdev->dev);
		return;
	}

	ktime_get_coarse_real_ts64(&last_check_time);
	rc = asus_thermal_side();
	if (rc < 0)
		CHG_DBG_E("%s: asus_thermal_side fail\n", __func__);

	rc = asus_thermal_btm();
	if (rc < 0)	
		CHG_DBG_E("%s: asus_thermal_side fail\n", __func__);

	new_batAlarm_time.tv_sec = last_check_time.tv_sec + RTCSetInterval;

	spin_lock_irqsave(&bat_alarm_slock, batflags);
	alarm_start(&bat_alarm, timespec_to_ktime(new_batAlarm_time));
	spin_unlock_irqrestore(&bat_alarm_slock, batflags);

	schedule_delayed_work(&asus_min_check_work, msecs_to_jiffies(60000));

	pm_relax(g_bcdev->dev);
}

static int charge_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
	static unsigned long pre_event = 0;

	CHG_DBG("%s: start, event:%lu\n", __func__, event);

	if (pre_event == event) {
		CHG_DBG("%s: same event %lu, return\n", __func__, event);
		return 0;
	}
	pre_event = event;

    switch (event) {
    case QTI_POWER_SUPPLY_CHARGED:
		CHG_DBG("%s: vbus plugin\n", __func__);
		schedule_delayed_work(&asus_min_check_work, 0);
        break;
    case QTI_POWER_SUPPLY_UNCHARGED:
		CHG_DBG("%s: vbus plugout\n", __func__);		
		alarm_cancel(&bat_alarm);
		cancel_delayed_work(&asus_min_check_work);
		break;
    }
	return 0;
}

#define CHECK_MINIMUM_INTERVAL (30)
int asus_chg_resume(struct device *dev)
{
	struct timespec64  mtNow;
	int nextCheckInterval;

	pm_stay_awake(g_bcdev->dev);

	ktime_get_coarse_real_ts64(&mtNow);

	/* If next check time less than 30s, do check (next check time = last check time + 60s) */
	nextCheckInterval = 60 - (mtNow.tv_sec - last_check_time.tv_sec);
	CHG_DBG("%s: nextCheckInterval = %d\n", __func__, nextCheckInterval);
	if (nextCheckInterval <= CHECK_MINIMUM_INTERVAL) {
		cancel_delayed_work(&asus_min_check_work);		
		schedule_delayed_work(&asus_min_check_work, 0);
	} else {	
		cancel_delayed_work(&asus_min_check_work);
		schedule_delayed_work(&asus_min_check_work, msecs_to_jiffies(nextCheckInterval));		
		pm_relax(g_bcdev->dev);
	}

	return 0;
}

//ASUS BSP : Show "+" on charging icon +++
void asus_set_qc_state_worker(struct work_struct *work)
{
	asus_extcon_set_state_sync(quickchg_extcon, g_SWITCH_LEVEL);
	CHG_DBG("%s: switchaa: %d\n", __func__, g_SWITCH_LEVEL);
}

void set_qc_stat(int status)
{
	if (quickchg_extcon == NULL) {
		CHG_DBG("%s: quickchg_extcon not Ready\n", __func__);
		return;
	}

	CHG_DBG("%s: status: %d\n", __func__, status);

	switch (status) {
	//"qc" stat happends in charger mode only, refer to smblib_get_prop_batt_status
	case POWER_SUPPLY_STATUS_CHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		cancel_delayed_work(&asus_set_qc_state_work);
		schedule_delayed_work(&asus_set_qc_state_work, msecs_to_jiffies(2000));
		break;
	default:
		cancel_delayed_work(&asus_set_qc_state_work);
		asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL0_DEFAULT);
		CHG_DBG("%s: switch: %d\n", __func__, SWITCH_LEVEL0_DEFAULT);
		break;
	}
}
//ASUS BSP : Show "+" on charging icon ---

int asuslib_init(void) {
	int rc = 0;
	struct pmic_glink_client_data client_data = { };
	struct pmic_glink_client	*client;

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

	//Register GPIO for PMI_MUX
	PMI_MUX_GPIO = of_get_named_gpio(g_bcdev->dev->of_node, "PMI_MUX_EN", 0);
	rc = gpio_request(PMI_MUX_GPIO, "PMI_MUX_EN");
	if (rc) {
		pr_err("%s: Failed to initalize the PMI_MUX_EN\n", __func__);
		return -1;
	}
	POGO_OTG_GPIO = of_get_named_gpio(g_bcdev->dev->of_node, "POGO_OTG_EN", 0);
	rc = gpio_request(POGO_OTG_GPIO, "POGO_OTG_EN");
	if (rc) {
		pr_err("%s: Failed to initalize the POGO_OTG_EN\n", __func__);
		return -1;
	}

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
	ChgPD_Info.VBUS_SRC = 0;
	ChgPD_Info.chg_limit_en = 0;
	ChgPD_Info.chg_limit_cap = 0;
	ChgPD_Info.usbin_suspend_en = 0;
    ChgPD_Info.charging_suspend_en = 0;
	ChgPD_Info.firmware_version = 0;
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
		
	INIT_DELAYED_WORK(&asus_set_qc_state_work, asus_set_qc_state_worker);
	//[---]Register the extcon for quick_charger

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
	//[---]Register the extcon for thermal alert

	//[+++]Register the extcon for invalid audio donlge
	audio_dongle_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
	if (IS_ERR(audio_dongle_extcon)) {
		rc = PTR_ERR(audio_dongle_extcon);
		printk(KERN_ERR "[BAT][CHG] failed to allocate audio dongle extcon device rc=%d\n", rc);
	}
	audio_dongle_extcon->fnode_name = "invalid_dongle";

	rc = extcon_dev_register(audio_dongle_extcon);
	if (rc < 0)
		printk(KERN_ERR "[BAT][CHG] failed to register audio dongle extcon device rc=%d\n", rc);
	//[---]Register the extcon for thermal alert

	asus_get_Batt_ID();

	//[+++] Init usb temp vadc channel
	alarm_init(&bat_alarm, ALARM_REALTIME, batAlarm_handler);	
	INIT_DELAYED_WORK(&asus_min_check_work, asus_min_check_worker);
	
	side_usb_temp_vadc_chan = iio_channel_get(g_bcdev->dev, "pm8350b_amux_thm6");
	if (IS_ERR_OR_NULL(side_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel_get fail\n", __func__);
	}
	btm_usb_temp_vadc_chan = iio_channel_get(g_bcdev->dev, "pm8350_amux_thm2");
	if (IS_ERR_OR_NULL(btm_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel_get fail\n", __func__);
	}

	//register USB_Online notify
	charge_notify.notifier_call = charge_notifier_callback;
	qti_charge_register_notify(&charge_notify);

	//[---] Init usb temp vadc channel
    //PASS_HWID_TO_ADSP();

	//register drm notifier
	INIT_DELAYED_WORK(&asus_set_panelonoff_current_work, asus_set_panelonoff_current_worker);
	RegisterDRMCallback();

	CHG_DBG_E("Load the asuslib_init Succesfully\n");
	return rc;
}

int asuslib_deinit(void) {
	//int rc;

	class_unregister(&asuslib_class);
	return 0;
}
