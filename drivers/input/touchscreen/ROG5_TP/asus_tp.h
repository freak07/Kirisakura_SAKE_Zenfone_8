#include "focaltech_core.h"
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_FOD_X_H_POS_REG                 0xE1
#define FTS_FOD_X_L_POS_REG                 0xE2
#define FTS_FOD_Y_H_POS_REG                 0xE2
#define FTS_FOD_Y_L_POS_REG                 0xE2

#define FTS_REG_EDGEPALM_MODE_EN            0x8C
#define FTS_REG_EDGEPALM_LEFT               0x8D
#define FTS_REG_EDGEPALM_RIGHT              0x8E

#define FTS_REG_SLIDING_SENSITIVITY         0x8F
#define FTS_REG_SLIDING_PRECISION           0x90
#define FTS_REG_TOUCH_SENSITIVITY           0x91

#define FTS_REG_REPORT_RATE                 0x88
#define TOTAL_GAME_USED_SLOT                10

#define ANGLE_0                             0
#define ANGLE_90                            90
#define ANGLE_270                           270

#define REPORT_RATE_LEVEL9                   3300
#define REPORT_RATE_LEVEL8                   3700
#define REPORT_RATE_LEVEL7                   4000
#define REPORT_RATE_LEVEL6                   4170
#define REPORT_RATE_LEVEL5                   4550
#define REPORT_RATE_LEVEL4                   5000
#define REPORT_RATE_LEVEL3                   5556
#define REPORT_RATE_LEVEL2                   5880
/*****************************************************************************
* 1. Global variable or extern global variabls/functions
*****************************************************************************/
/*****************************************************************************
* 2. Static function prototypes
*******************************************************************************/
/*Game related*/
int asus_game_create_sysfs(struct fts_ts_data *ts_data);
int asus_game_remove_sysfs(struct fts_ts_data *ts_data);
void ATR_touch(int id,int action, int x, int y, int random);
int set_rotation_mode(void);
void asus_game_recovery(struct fts_ts_data *ts_data);
void rise_report_rate (bool rise);
void reconfig_game_reg(bool reconfig);

/*Extra mode*/
int asus_create_sysfs(struct fts_ts_data *ts_data);
int asus_remove_sysfs(struct fts_ts_data *ts_data);
void asus_tp_charge_mode (bool enable);
void fts_ex_fun_recovery(struct fts_ts_data *ts_data) ;


/*Gesture related*/
int asus_gesture_init(struct fts_ts_data *ts_data);
int is_enter_gesture_mode (struct fts_ts_data *ts_data);
int set_gesture_register (struct fts_ts_data *ts_data);
void write_fp_xy(struct fts_ts_data *ts_data);
void asus_gesture_report(struct fts_ts_data *ts_data, int gesture_id);
int asus_gesture_suspend(struct fts_ts_data *ts_data);
int asus_gesture_resume(struct fts_ts_data *ts_data);