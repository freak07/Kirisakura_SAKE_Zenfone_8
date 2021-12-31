/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
 * EXCLUDED.                                                                 *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

/*! \file
 * \brief Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and color temperature functionality within the
 * AMS TMD2755 family of devices.
 */

#ifndef __AMS_TMD2755_H__
#define __AMS_TMD2755_H__

/* Linux Include Files */
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/input/ASH.h>

/***********************************************
 *   Flags for adding/removing code            *
 ***********************************************/
/* Flag for enabling Debug input device dump   */
/* and set all registers.                      */
#define DEBUG_ABI_SET_GET_REGISTERS
#define REMOVE_INPUT_DEVICE

extern void psensor_report_abs(int abs);
extern void lsensor_report_lux(int lux);
extern struct mutex g_alsps_lock;
extern tmd2755_status_param g_tmd2755_status_param;

/* Debugging Prox feature - during development */
#define DEBUG_PROX_FEATURE
// #define AMS_MUTEX_DEBUG

#if defined(AMS_MUTEX_DEBUG)
#define AMS_MUTEX_LOCK(m) { \
        printk(KERN_INFO "%s:%s() --> Mutex Lock\n", __FILE__, __func__); \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        printk(KERN_INFO "%s:%s() --> Mutex Unlock\n", __FILE__, __func__); \
        mutex_unlock(m); \
    }
#else
#define AMS_MUTEX_LOCK(m) { \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        mutex_unlock(m); \
    }
#endif
#define MIN_KERNEL_LOG_LEN             (28)
#define MAX_KERNEL_LOG_LEN         MIN_KERNEL_LOG_LEN
#define LINE_NUM_KERNEL_LOG_LEN         (3)

#define MAX_REGS 256

#define BASE_10				   (10)


/* Mandatory register values */
#define CFG8_REG_REQUIRED_VAL       0x29
#define TST3_REG_REQUIRED_VAL       0x04 /* changed 4/13/2020 */
#define TST9_REG_REQUIRED_VAL       0x02 /* changed from 0x0B per Whitney 11/13/2019 */

enum tmd2755_regs {
	TMD2755_REG_ENABLE       = 0x80,
	TMD2755_REG_ATIME        = 0x81,
	TMD2755_REG_PRATE        = 0x82,  /* PTIME, PRATE */
	TMD2755_REG_AWTIME       = 0x83,
	TMD2755_REG_AILTL        = 0x84,
	TMD2755_REG_AILTH        = 0x85,
	TMD2755_REG_AIHTL        = 0x86,
	TMD2755_REG_AIHTH        = 0x87,
	TMD2755_REG_PILTL        = 0x88,
	TMD2755_REG_PILTH        = 0x89,
	TMD2755_REG_PIHTL        = 0x8A,
	TMD2755_REG_PIHTH        = 0x8B,
	TMD2755_REG_PERS         = 0x8C,       /* als and prox interrupt pers*/
	TMD2755_REG_CFG0         = 0x8D,
	TMD2755_REG_PCFG0        = 0x8E,
	TMD2755_REG_PCFG1        = 0x8F,
	TMD2755_REG_PCFG2        = 0x90,
	TMD2755_REG_CFG1         = 0x91,
	TMD2755_REG_REVID        = 0x92,
	TMD2755_REG_DEVICEID     = 0x93,
	TMD2755_REG_STATUS       = 0x94,
	TMD2755_REG_ALSL         = 0x95,
	TMD2755_REG_ALSH         = 0x96,
	TMD2755_REG_IRL          = 0x97,
	TMD2755_REG_IRH          = 0x98,

	TMD2755_REG_PDATAL       = 0x99,
	TMD2755_REG_PDATAH       = 0x9A,

	TMD2755_REG_REVID2       = 0xA6,

	TMD2755_REG_SOFTRST      = 0xA8,

	TMD2755_REG_PWTIME       = 0xA9,

	TMD2755_REG_CFG8         = 0xAA,
	TMD2755_REG_CFG3         = 0xAB,
	TMD2755_REG_CFG6         = 0xAE,

	TMD2755_REG_POFFSET_L    = 0xC0,
	TMD2755_REG_POFFSET_H    = 0xC1,

	TMD2755_REG_CALIB        = 0xD7,
	TMD2755_REG_CALIB_OFF    = 0xD8,
	TMD2755_REG_CALIBCFG     = 0xD9,
	TMD2755_REG_CALIBSTAT    = 0xDC,

	TMD2755_REG_INTENAB      = 0xDD,

	TMD2755_REG_FAC_L        = 0xE6,
	TMD2755_REG_FAC_H        = 0xE7,

	/* TODO remove me after new datasheet rev */
	TMD2755_REG_TEST3        = 0xF3,
	TMD2755_REG_TEST9        = 0xF9,

	TMD2755_REG_LAST         = 0xFF,
};

enum tmd2755_mask_shift_reg {
	TMD2755_MASK_DEVICEID  = 0xFC,
	TMD2755_SHIFT_DEVICEID = 2,

	TMD2755_MASK_REVID     = 0x07,
	TMD2755_SHIFT_REVID    = 0,

	TMD2755_MASK_REVID2    = 0x0F,
	TMD2755_SHIFT_REVID2   = 0,

	TMD2755_MASK_PGAIN1    = 0xC0,
	TMD2755_SHIFT_PGAIN1   = 6,

	TMD2755_MASK_PGAIN2    = 0x60,
	TMD2755_SHIFT_PGAIN2   = 5,

	TMD2755_MASK_PPULSE  = 0x3F,
	TMD2755_SHIFT_PPULSE = 0,

	TMD2755_MASK_PILTL            = 0xFF,
	TMD2755_SHIFT_PILTL           = 0,
	TMD2755_MASK_PILTH            = 0x3F,
	TMD2755_SHIFT_PILTH           = 0,

	TMD2755_MASK_PIHTL            = 0xFF,
	TMD2755_SHIFT_PIHTL           = 0,
	TMD2755_MASK_PIHTH            = 0x3F,
	TMD2755_SHIFT_PIHTH           = 0,

	TMD2755_MASK_PROX_PERS = 0xF0,
	TMD2755_SHIFT_PROX_PERS = 4,

	TMD2755_MASK_START_OFFSET_CALIB   = 0x01,
	TMD2755_SHIFT_START_OFFSET_CALIB  = 0,

	TMD2755_MASK_CALPRATE_CALIB       = 0x10,
	TMD2755_SHIFT_CALPRATE_CALIB      = 4,

	TMD2755_MASK_ELEC_OPTO_CALIB      = 0x20,
	TMD2755_SHIFT_ELEC_OPTO_CALIB     = 5,

	TMD2755_MASK_CALAVG_CALIB         = 0x80,
	TMD2755_SHIFT_CALAVG_CALIB        = 7,

	TMD2755_MASK_ENABLE_ORE      =  0x20,
	TMD2755_SHIFT_ENABLE_ORE     =  5,

	TMD2755_MASK_ORE             =  0x1F,
	TMD2755_SHIFT_ORE            =  0,


	TMD2755_MASK_BINSRCH_TARGET  = 0xE0,
	TMD2755_SHIFT_BINSRCH_TARGET = 5,

	TMD2755_MASK_PROX_AUTO_OFFSET_ADJUST  = 0x08,
	TMD2755_SHIFT_PROX_AUTO_OFFSET_ADJUST = 3,

	TMD2755_MASK_PROX_DATA_AVG   = 0x07,
	TMD2755_SHIFT_PROX_DATA_AVG  = 0,

	TMD2755_MASK_POFFSET_H       = 0x01,
	TMD2755_SHIFT_POFFSET_H      = 0,

	TMD2755_MASK_PPULSE_LEN_L    = 0x0FF,
	TMD2755_SHIFT_PPULSE_LEN_L   = 0,

	TMD2755_MASK_REG_PPULSE_LEN_H    = 0xC0,  // mask for register PCFG1
	TMD2755_MASK_DATA_PPULSE_LEN_H   = 0x300, // mask for data structure - 10 bit value
	TMD2755_SHIFT_PPULSE_LEN_H       = 2, 

	TMD2755_MASK_APC             = 0x40,
	TMD2755_SHIFT_APC            = 6,

	TMD2755_MASK_AGAIN           = 0x1F,
	TMD2755_SHIFT_AGAIN          = 0,

	TMD2755_MASK_AWLONG           = 0x04,
	TMD2755_SHIFT_AWLONG             = 2,

	TMD2755_MASK_ALS_PERS = 0x0F,
	TMD2755_SHIFT_ALS_PERS = 0,

};

enum tmd2755_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

enum tmd2755_prox_state {
	PROX_STATE_NONE = 0,
	PROX_STATE_INIT,
	PROX_STATE_CALIB,
	PROX_STATE_WAIT_AND_CALIB
};

enum tmd2755_feature_state {
	TMD2755_FEATURE_OFF    = 0,
	TMD2755_FEATURE_ON     = 1,
};

enum tmd2755_enable_state {
	TMD2755_ENABLE_OFF    = 0,
	TMD2755_ENABLE_ON     = 1,
};

enum tmd2755_ctrl_reg {
	AGAIN_16       = (5 << 0),
	AGAIN_128      = (8 << 0),
	AGAIN_1024     = (11 << 0),
	PGAIN1_1       = (0 << TMD2755_SHIFT_PGAIN1),
	PGAIN1_2       = (1 << TMD2755_SHIFT_PGAIN1),
	PGAIN1_4       = (2 << TMD2755_SHIFT_PGAIN1),
	PGAIN1_8       = (3 << TMD2755_SHIFT_PGAIN1),
	PGAIN2_1       = (0 << TMD2755_SHIFT_PGAIN2),
	PGAIN2_2_5     = (1 << TMD2755_SHIFT_PGAIN2),
	PGAIN2_5       = (2 << TMD2755_SHIFT_PGAIN2),
	PGAIN2_10      = (3 << TMD2755_SHIFT_PGAIN2),
};

enum tmd2755_enable_reg {
	TMD2755_PON   = (1 << 0),
	TMD2755_AEN   = (1 << 1),
	TMD2755_PEN   = (1 << 2),
	TMD2755_AWEN  = (1 << 3),
	TMD2755_PWEN  = (1 << 4),
	TMD2755_EN_ALL = (TMD2755_AEN |TMD2755_PEN | TMD2755_AWEN | TMD2755_PWEN),
};


enum tmd2755_int_shift {
	TMD2755_INT_PRX_SAT_SHIFT = 6,
};

enum tmd2755_int_status {
	TMD2755_INT_ST_PSAT_AMBIENT_IRQ    = (1 << 0),
	TMD2755_INT_ST_PSAT_REFLECT_IRQ    = (1 << 1),
	TMD2755_INT_ST_ZERODET_IRQ         = (1 << 2),
	TMD2755_INT_ST_CALIB_IRQ           = (1 << 3),
	TMD2755_INT_ST_ALS_IRQ             = (1 << 4),
	TMD2755_INT_ST_PRX_IRQ             = (1 << 5),
	TMD2755_INT_ST_PRX_SAT_IRQ         = (1 << 6),
	TMD2755_INT_ST_ALS_SAT_IRQ         = (1 << 7),
};

enum tmd2755_intenab_reg {
	TMD2755_ZIEN  = (1 << 2),
	TMD2755_CIEN  = (1 << 3),
	TMD2755_AIEN  = (1 << 4),
	TMD2755_PIEN  = (1 << 5),
	TMD2755_PSIEN = (1 << 6),
	TMD2755_ASIEN = (1 << 7),
};
/* Group all the prox interrupts */
#define TMD2755_PROX_INTS  (TMD2755_ZIEN | TMD2755_PIEN | TMD2755_PSIEN | TMD2755_CIEN)

// pldrive Ivcsel = (PLDRIVE + 2) mA
#define PDRIVE_MA(p) ({ \
	u8 __reg = (((u8)((p) - 2)) & 0x0F); \
	__reg = (__reg > 0x0A) ? 0x0A : __reg; \
	__reg; \
})

// pulse length (PULSE_LEN + 2) usec  - 10 bits
#define PPULSE_LEN_US(p) ({ \
	u16 __reg;          \
	if (p < 2) {        \
		__reg = 0;   \
	} else {            \
		__reg  = (((u16)((p) - 2)) & 0x3FF); \
		__reg = (__reg > 0x3FF) ? 0x03FF : __reg; \
	}      \
	__reg; \
})

#define P_TIME_US(p)   ((((p) / 88) - 1.0) + 0.5)
#define INTEGRATION_CYCLE 2780

/* Used for ALS Wait time and ALS Time */
#define AW_TIME_MS(p)  ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)

struct als_prox_pers
{
	u8   apers :    4;
	u8   ppers :    4;
};

union tmd2755_persist
{
	struct als_prox_pers pers;
	u8     persistance;
};

#define PROX_PERSIST(p) (((p) & 0x0F) << 4)

#define ALS_PERSIST(p) (((p) & 0x0F) << 0)

struct tmd2755_parameters {
	/* Common both als and prox */
	union tmd2755_persist persist;

	/* Prox */
	u16 prox_thresh_min;
	u16 prox_thresh_max;
	u8  prox_apc;
	u8  prox_pulse_cnt;
	u16 prox_pulse_len;
	u8  prox_gain1;
	u8  prox_gain2;
	s16 poffset;
	u8  prox_drive;
	u8  prox_time;
	u8  prox_wtime;
	/* prox calibration - These registers are set during   */
	/* prox offset calibration and are currently cannot    */
	/* be adjusted programatically                         */
	u8  prox_calavg;
	u8  prox_calprate;
	u8  prox_enable_ore;
	u8  prox_ore;
	u8  prox_binsrch_tgt;
	u8  prox_auto_off_adj;
	u8  prox_avg;

	/* ALS / Color */
	u8  als_gain;
	u32 als_auto_gain;
	u16 als_deltaP;
	u8  als_time;
	u8  als_wtime;
	u32 dgf;
	u32 ch0_coef0;
	u32 ch0_coef1;
	u32 ch1_coef0;
	u32 ch1_coef1;
	u32 coef_scale;
	s16 poffset_fac;
	s16 poffset_limit;
	s16 poffset_last;
};

struct tmd2755_als_info {
	u32 counts_per_lux;
	u32 saturation;
	u16 ch0_raw;
	u16 ch1_raw;
	u16 lux;
};

struct tmd2755_prox_info {
	u16 raw;
	int detected;
};

// Must match definition in ../arch file
struct tmd2755_i2c_platform_data {
	/* The following callback for power events received and handled by
	   the driver.  Currently only for SUSPEND and RESUME */
	int (*platform_power)(struct device *dev, enum tmd2755_pwr_state state);
	int (*platform_init)(void);
	void (*platform_teardown)(struct device *dev);

	char const *prox_name;
	char const *als_name;
	int   device_index;
	struct tmd2755_parameters parameters;
	// TODO: ??? bool proximity_can_wake;
	// TODO: ??? bool als_can_wake;
#if defined(CONFIG_OF)
	struct device_node  *of_node;
#endif
};

struct tmd2755_chip {
	struct mutex lock;
	struct completion calibration_done;
	struct i2c_client *client;
	struct gpio_desc *gpiod_interrupt;
	struct tmd2755_prox_info prox_info;
	struct tmd2755_als_info als_info;
	struct tmd2755_parameters params;
	struct tmd2755_i2c_platform_data *pdata;
	u8 shadow[MAX_REGS];

	struct input_dev *prox_idev;
	struct input_dev *als_idev;
#ifdef DEBUG_ABI_SET_GET_REGISTERS
	struct input_dev *dbg_idev;
#endif // #ifdef DEBUG_ABI_SET_GET_REGISTERS

	int in_suspend;
	int wake_irq;
	int irq_pending;

	int  in_psat;
	bool in_asat;
	bool in_calib;
	bool unpowered;
	bool als_enable;
	bool prox_enable;
	bool amsCalComplete;

	bool cal_en;

	u8 device_index;
};

#define TMD2755_PROXIMITY_INF_DEFAULT     (0)
#define TMD2755_PROXIMITY_OFFSET_DEFAULT     (76)
#define TMD2755_PROXIMITY_THDL_DEFAULT    (145)
#define TMD2755_PROXIMITY_THDH_DEFAULT    (437)
#define TMD2755_PROXIMITY_AUTOK_MIN       (3)
#define TMD2755_PROXIMITY_AUTOK_MAX       (300)
#define TMD2755_LIGHT_CALIBRATION_DEFAULT (1440)
#define TMD2755_LIGHT_MAX_THRESHOLD       (65534)

#define TMD2755_WAIT_I2C_DELAY 5


#endif  // __AMS_TMD2755_H__
