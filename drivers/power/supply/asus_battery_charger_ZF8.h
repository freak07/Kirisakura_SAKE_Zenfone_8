/*
 * Copyright (c) 2019-2020, The ASUS Company. All rights reserved.
 */

struct extcon_dev   *bat_id_extcon;
struct extcon_dev   *bat_extcon;
struct extcon_dev   *quickchg_extcon;
struct extcon_dev   *thermal_extcon;
struct extcon_dev   *adaptervid_extcon;

//qti_battery_charger+++
#define MSG_OWNER_BC                32778

struct battery_charger_req_msg {
    struct pmic_glink_hdr   hdr;
    u32         battery_id;
    u32         property_id;
    u32         value;
};

struct battery_charger_ship_mode_req_msg {
    struct pmic_glink_hdr   hdr;
    u32         ship_mode_type;
};
//qti_battery_charger---

//Define the OWNER ID
#define PMIC_GLINK_MSG_OWNER_OEM    32782

//Define Message Type
#define MSG_TYPE_REQ_RESP   1
#define MSG_TYPE_NOTIFY   2

//Define the opcode
#define OEM_OPCODE_READ_BUFFER               0x10000
#define OEM_OPCODE_WRITE_BUFFER              0x10001

#define OEM_ASUS_EVTLOG_IND                  0x1002
#define OEM_PD_EVTLOG_IND                    0x1003
#define OEM_SET_OTG_WA                       0x2107
#define OEM_USB_PRESENT                      0x2108
#define OEM_SET_CHARGER_TYPE_CHANGE          0x2109
#define OEM_ASUS_WORK_EVENT_REQ              0x2110
#define OEM_ASUS_AdapterVID_REQ              0x2111
#define OEM_JEITA_CC_STATE_REQ               0x2112


#define MAX_OEM_PROPERTY_DATA_SIZE           16

//Add the structure +++
struct ADSP_ChargerPD_Info {
    int     PlatformID;
    int     BATT_ID;
    bool    chg_limit_en;
    u32     chg_limit_cap;
    bool    usbin_suspend_en;
    bool    charging_suspend_en;
    char    ChgPD_FW[64];
    int     firmware_version;
    int     batt_temp;
    u32     pm8350b_icl;
    u32     smb1396_icl;
    u32     batt_fcc;
    unsigned long   AdapterVID;
    bool    otg_enable;
    bool    usb_present;
    u32     slow_chglimit;
    u32     ultra_bat_life;
    u32     demo_app_status;
    u32     chg_disable_jeita;
    bool    panel_status;
    int     jeita_cc_state;
    int     thermel_threshold;
    int     boot_completed;
    bool    in_call;
    u32     launchedtime;
};

struct battman_oem_read_buffer_req_msg { 
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 data_size;
};

struct battman_oem_read_buffer_resp_msg { 
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 data_buffer[MAX_OEM_PROPERTY_DATA_SIZE];
    u32 data_size; //size = 0 if failed, otherwise should be data_size.
};

struct battman_oem_write_buffer_req_msg {
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 data_buffer[MAX_OEM_PROPERTY_DATA_SIZE];
    u32 data_size;
};

struct battman_oem_write_buffer_resp_msg {
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 return_status;
};

struct evtlog_context_resp_msg3 {
    struct pmic_glink_hdr       hdr;
    u8              buf[128];
    u32             reserved;
};

struct oem_enable_change_msg {
    struct pmic_glink_hdr   hdr;
    u32    enable;
};

struct oem_set_Charger_Type_resp {
    struct pmic_glink_hdr   hdr;
    u32    charger_type;
};

struct asus_notify_work_event_msg{
  struct pmic_glink_hdr header;
  u32 work;
  u32 data_buffer[MAX_OEM_PROPERTY_DATA_SIZE];
  u32 data_size;
};

struct oem_asus_adaptervid_msg {
    struct pmic_glink_hdr header;
    u32 VID;
};

struct oem_jeita_cc_state_msg {
    struct pmic_glink_hdr header;
    u32 state;
};
//Add the structure ---

//Add oem property
enum battman_oem_property {
    BATTMAN_OEM_ADSP_PLATFORM_ID,
    BATTMAN_OEM_BATT_ID,
    BATTMAN_OEM_CHG_LIMIT_EN,
    BATTMAN_OEM_CHG_LIMIT_CAP,
    BATTMAN_OEM_USBIN_SUSPEND,
    BATTMAN_OEM_CHARGING_SUSPNED,
    BATTMAN_OEM_CHGPD_FW_VER,
    BATTMAN_OEM_FW_VERSION,
    BATTMAN_OEM_BATT_TEMP,
    BATTMAN_OEM_PM8350B_ICL,
    BATTMAN_OEM_SMB1396_ICL,
    BATTMAN_OEM_FCC,
    BATTMAN_OEM_DEBUG_MASK,
    BATTMAN_OEM_AdapterVID,
    BATTMAN_OEM_SMB_Setting,
    BATTMAN_OEM_Panel_Check,
    BATTMAN_OEM_WORK_EVENT,
    BATTMAN_OEM_THERMAL_ALERT,
    BATTMAN_OEM_Write_PM8350B_Register,
    BATTMAN_OEM_Slow_Chg,
    BATTMAN_OEM_Batt_Protection,
    BATTMAN_OEM_CHG_Disable_Jeita,
    BATTMAN_OEM_CHG_MODE,
    BATTMAN_OEM_THERMAL_THRESHOLD,
    BATTMAN_OEM_THERMAL_SENSOR,
    BATTMAN_OEM_FV,
    BATTMAN_OEM_In_Call,
    BATTMAN_OEM_PROPERTY_MAX,
};

enum Work_ID {
    WORK_JEITA_RULE,
    WORK_JEITA_PRECHG,
    WORK_JEITA_CC,
    WORK_PANEL_CHECK,
    WORK_LONG_FULL_CAP,
    WORK_18W_WORKAROUND,
    WORK_MAX
};

enum thermal_alert_state {
    THERMAL_ALERT_NONE,
    THERMAL_ALERT_NO_AC,
    THERMAL_ALERT_WITH_AC,
    THERMAL_ALERT_MAX
};

enum JETA_CURR {
    JETA_NONE,
    JETA_PRECHG,
    JETA_CC1,
    JETA_CC2,
    JETA_CV,
    JETA_MAX
};

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

//work
struct delayed_work asus_set_qc_state_work;
struct delayed_work asus_jeita_rule_work;
struct delayed_work asus_jeita_prechg_work;
struct delayed_work asus_jeita_cc_work;
struct delayed_work asus_panel_check_work;
struct delayed_work asus_slow_charging_work;
struct delayed_work asus_charger_mode_work;
struct delayed_work asus_long_full_cap_monitor_work;
struct delayed_work asus_18W_workaround_work;
struct delayed_work asus_thermal_policy_work;

extern struct battery_chg_dev *g_bcdev;
struct power_supply *qti_phy_usb;
struct power_supply *qti_phy_bat;
int POGO_OTG_GPIO;
int OTG_LOAD_SWITCH_GPIO;
struct ADSP_ChargerPD_Info ChgPD_Info;
#if defined ASUS_VODKA_PROJECT
char st_battery_name[64] = "C11P1904.O.03.0001.30.03.32.1";
#else
char st_battery_name[64] = "C11P2003.O.01.0001.30.10.46.57";
#endif

ssize_t oem_prop_read(enum battman_oem_property prop, size_t count);
ssize_t oem_prop_write(enum battman_oem_property prop, u32 *buf, size_t count);
//[---] Add the global variables

