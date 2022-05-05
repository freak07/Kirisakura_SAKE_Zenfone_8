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
 * proximity detection (prox) functionality within the
 * AMS TMD2755 family of devices.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/gpio/consumer.h>
//#ifdef CONFIG_OF
#include <linux/of_device.h>
//#endif

#include "ams_tmd2755.h"
#include "ams_i2c.h"
#include "ams_tmd2755_prox.h"
#include "ams_tmd2755_als.h"
#include "ams_tmd2755_cfg.h"
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>


MODULE_AUTHOR("AMS AOS Software<cs.americas@ams.com>");
MODULE_DESCRIPTION("AMS tmd2755 ALS, Prox Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.10");

/******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_tmd2755"
#define SENSOR_TYPE_NAME		"ALSPS"

#undef dbg
#ifdef ASH_GPIO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s]"fmt,MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

#include <linux/of_gpio.h>

#define ALSPS_QCOM_NAME 	"qcom,alsps-gpio"
#define ALSPS_INT_NAME		"ALSPS_SENSOR_INT"
#define ALSPS_IRQ_NAME		"ALSPS_SENSOR_IRQ"
static int ALSPS_SENSOR_GPIO;
static int ALSPS_SENSOR_IRQ;
#define GPIO_LOOKUP_STATE	"alsps_gpio_high"

struct tmd2755_chip *g_tmd2755_chip;
static struct i2c_client	*g_i2c_client = NULL;
static struct wakeup_source *g_tmd2755_wake_lock;
static int tmd2755_regulator_enable(void);
extern void tmd2755_read_prox(struct tmd2755_chip *chip);

static void tmd2755_ist(struct work_struct *work);
static struct delayed_work tmd2755_ist_work;

/*********************************/

struct device_ids {
	uint8_t device;
	uint8_t rev;
	uint8_t aux;
};

/* The next two structures must match indices */
/* aux id/revid2 is not defined for tmd2755 */
static struct device_ids const dev_ids[] = {
	{.device = 0x14, .rev = 0x00, .aux = 0x00},  /* tmd2755 */
};

static char const *device_names[] = {
	"tmd2755",
};

/* Registers to restore */
static u8 const restorable_regs[] = {
	TMD2755_REG_PILTL,
	TMD2755_REG_PILTH,
	TMD2755_REG_PIHTL,
	TMD2755_REG_PIHTH,
	TMD2755_REG_PERS,
	TMD2755_REG_CFG6,
	TMD2755_REG_PCFG0,
	TMD2755_REG_PCFG1,
	TMD2755_REG_PCFG2,
	TMD2755_REG_CFG1,
	TMD2755_REG_CFG8,
	TMD2755_REG_ATIME,
	TMD2755_REG_PRATE,
	TMD2755_REG_POFFSET_L,
	TMD2755_REG_POFFSET_H,
	TMD2755_REG_TEST3,
	TMD2755_REG_TEST9,
};

#if defined(CONFIG_OF)

int tmd2755_init_dt(struct tmd2755_i2c_platform_data *pdata)
{
	struct device_node *np = pdata->of_node;
	const char *str;
	u32 val;
	s32 sval;

	if (!pdata->of_node)
		return 0;

	if (!of_property_read_string(np, "prox_name", &str))
		pdata->prox_name = str;

	if (!of_property_read_string(np, "als_name", &str))
		pdata->als_name = str;

	/*************************   PROX    ***********************************/
	if (!of_property_read_u32(np, "ppers", &val))
		pdata->parameters.persist.pers.ppers = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_PERIST_CFG]);

	if (!of_property_read_u32(np, "prox_thresh_min", &val))
		pdata->parameters.prox_thresh_min = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_THRESH_MIN_CFG]);

	if (!of_property_read_u32(np, "prox_thresh_max", &val))
		pdata->parameters.prox_thresh_max = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_THRESH_MAX_CFG]);

	if (!of_property_read_u32(np, "prox_pulse_cnt", &val))
		pdata->parameters.prox_pulse_cnt = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_PULSE_CNT_CFG]);

	if (!of_property_read_u32(np, "prox_apc", &val))
		pdata->parameters.prox_apc = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_APC_CFG]);

	if (!of_property_read_u32(np, "prox_pulse_len", &val))
		pdata->parameters.prox_pulse_len = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_PULSE_LEN_CFG]);

	if (!of_property_read_u32(np, "prox_gain1", &val))
		pdata->parameters.prox_gain1 = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_GAIN_1_CFG]);

	if (!of_property_read_u32(np, "prox_gain2", &val)) {
		pdata->parameters.prox_gain2 = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_GAIN_2_CFG]);
		/* a value of 2 for pgain_2 is reserved value */
		pdata->parameters.prox_gain2 = ((pdata->parameters.prox_gain2 != 2) ?
			pdata->parameters.prox_gain2 : tmd2755_cfg_limits[PROX_GAIN_2_CFG].def);
	}
	if (!of_property_read_s32(np, "poffset", &sval))
		pdata->parameters.poffset = CHECK_LIMITS(sval, tmd2755_cfg_limits[PROX_OFFSET_CFG]);

	if (!of_property_read_u32(np, "prox_drive", &val))
		pdata->parameters.prox_drive = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_DRIVE_CFG]);

	if (!of_property_read_u32(np, "prox_time", &val))
		pdata->parameters.prox_time = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_TIME_CFG]);

	if (!of_property_read_u32(np, "prox_wtime", &val))
		pdata->parameters.prox_wtime = CHECK_LIMITS(val, tmd2755_cfg_limits[PROX_WTIME_CFG]);

	/**********************   ALS   *******************************/
	if (!of_property_read_u32(np, "apers", &val))
		pdata->parameters.persist.pers.apers =  CHECK_LIMITS(val, tmd2755_cfg_limits[ALS_PERSIST_CFG]);

	if (!of_property_read_u32(np, "als_gain", &val)) {
		pdata->parameters.als_gain = CHECK_LIMITS(val, tmd2755_cfg_limits[ALS_GAIN_CFG]);
		/* als gain can only take on 3 values - extra checking */
		if ((val != tmd2755_cfg_limits[ALS_GAIN_CFG].min) &&
			(val != tmd2755_cfg_limits[ALS_GAIN_CFG].max) &&
			(val != tmd2755_cfg_limits[ALS_GAIN_CFG].def)){
			pdata->parameters.als_gain = tmd2755_cfg_limits[ALS_GAIN_CFG].def;
		}
	}

	if (!of_property_read_u32(np, "als_auto_gain", &val))
		pdata->parameters.als_auto_gain = CHECK_LIMITS(val, tmd2755_cfg_limits[ALS_AUTO_GAIN_CFG]);

	if (!of_property_read_u32(np, "als_deltap", &val))
		pdata->parameters.als_deltaP = CHECK_LIMITS(val, tmd2755_cfg_limits[ALS_DELTA_P_CFG]);

	if (!of_property_read_u32(np, "als_time", &val))
		pdata->parameters.als_time = CHECK_LIMITS(val, tmd2755_cfg_limits[ALS_TIME_CFG]);

	if (!of_property_read_u32(np, "als_wtime", &val))
		pdata->parameters.als_wtime = CHECK_LIMITS(val, tmd2755_cfg_limits[ALS_WTIME_CFG]);

	/* Do not limit check the ALS coefficients */
	if (!of_property_read_u32(np, "dgf", &val))
		pdata->parameters.dgf = val;

	if (!of_property_read_u32(np, "ch0_coef0", &val))
		pdata->parameters.ch0_coef0 = val;

	if (!of_property_read_u32(np, "ch0_coef1", &val))
		pdata->parameters.ch0_coef1 = val;

	if (!of_property_read_u32(np, "ch1_coef0", &val))
		pdata->parameters.ch1_coef0 = val;

	if (!of_property_read_u32(np, "ch1_coef1", &val))
		pdata->parameters.ch1_coef1 = val;

	if (!of_property_read_u32(np, "coef_scale", &val))
		pdata->parameters.coef_scale = val;

	return 0;
}

static const struct of_device_id tmd2755_of_match[] = {
	{ .compatible = "ams,tmd2755" },
	{}
};
MODULE_DEVICE_TABLE(of, tmd2755_of_match);

#endif /* CONFIG_OF */

static int tmd2755_ALSPS_hw_get_interrupt()
{
	u8 status;
	int ret;
	struct tmd2755_chip *chip = g_tmd2755_chip;
	struct device *dev = &chip->client->dev;
	bool psat_irq_enabled;

	AMS_MUTEX_LOCK(&g_alsps_lock);
	AMS_MUTEX_LOCK(&chip->lock);
	ret = ams_i2c_read(chip->client, TMD2755_REG_STATUS, &chip->shadow[TMD2755_REG_STATUS]);
	status = chip->shadow[TMD2755_REG_STATUS];

	if((status & 0x28) == 0x28){
		status = status & 0xdf;
		log("Psensor Cal & normal irq trigger at the same part, skip normal irq, 0x94=0x%x", status);
	}
	/* Interrupt was not the tmd2755 */
	if (status == 0) {
		AMS_MUTEX_UNLOCK(&chip->lock);
		AMS_MUTEX_UNLOCK(&g_alsps_lock);
		return 0;
	}

	/* Clear the Interrupt and begin processing */
	ams_i2c_write_direct(chip->client, TMD2755_REG_STATUS, status);

	/*************/
	/* ALS       */
	/*************/
	if (status & TMD2755_INT_ST_ALS_SAT_IRQ) {
		if(1 != chip->in_asat){
			dev_dbg(dev, "%*.*s():%*d --> ALS Saturation Interrupt occurred\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
			chip->in_asat = 1;
		}
	} else {
		chip->in_asat = 0;
	}

	if (status & TMD2755_INT_ST_ALS_IRQ) {
		if (status & TMD2755_INT_ST_ALS_IRQ)
			dev_dbg(dev, "%*.*s():%*d --> ALS Threshold Interrupt occurred\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		tmd2755_read_als(chip);
		tmd2755_report_als(chip);
		light_polling_work_assing();
	}

	/*************/
	/* Proximity */
	/*************/

	/*
	 * Zero Detect
	 */ 
	if (status & TMD2755_INT_ST_ZERODET_IRQ) {
		u8 val;
		dev_info(dev, "%*.*s():%*d --> ZINT Zero Detection Interrupt occurred\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		ams_i2c_read(chip->client,  TMD2755_REG_CALIB_OFF, &val);
		if (val & TMD2755_MASK_ENABLE_ORE) { 
			/* Get the new poffset */
			ams_i2c_blk_read(chip->client, TMD2755_REG_POFFSET_L, &chip->shadow[TMD2755_REG_POFFSET_L], 2);
			chip->params.poffset = chip->shadow[TMD2755_REG_POFFSET_L];
			if (chip->shadow[TMD2755_REG_POFFSET_H] & TMD2755_MASK_POFFSET_H)
				chip->params.poffset *= -1;

			dev_info(dev, "%*.*s():%*d --> \t\tAdjusting poffset down by 1: poffsetl=%d, poffseth=%d\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__,
			chip->shadow[TMD2755_REG_POFFSET_L], chip->shadow[TMD2755_REG_POFFSET_H] & TMD2755_MASK_POFFSET_H);	
		}
	}

	/* Only report saturation if PSAT IRQ is enabled.  Saturation IRQ is disabled after the first */
	/* occurence andf re-enabled on a release event.  However, the status bit is constantly updated */
	/* with the actual condition.  Ignore the status bit if the PSAT IRQ is not enabled */
	psat_irq_enabled = (chip->shadow[TMD2755_REG_INTENAB] & TMD2755_INT_ST_PRX_SAT_IRQ) >> TMD2755_INT_PRX_SAT_SHIFT;

	if (psat_irq_enabled)
		chip->in_psat = PROX_NO_SAT;  /* no saturation */

	if ((status & TMD2755_INT_ST_PRX_SAT_IRQ) && psat_irq_enabled) {
		dev_info(dev, "%*.*s():%*d --> PSAT Interrupt occurred\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		chip->in_psat = PROX_SAT;
	}

	if ((status & TMD2755_INT_ST_PSAT_AMBIENT_IRQ) && psat_irq_enabled) {
		dev_info(dev, "%*.*s():%*d --> PSAT Ambient Interrupt occurred\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		chip->in_psat = PROX_AMBIENT_SAT;
	}

	if ((status & TMD2755_INT_ST_PSAT_REFLECT_IRQ) && psat_irq_enabled) {
		dev_info(dev, "%*.*s():%*d --> PSAT Reflective Interrupt occurred\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		chip->in_psat = PROX_REFLECTIVE_SAT;
	}

	/* Report a saturation event */
	if (chip->in_psat && psat_irq_enabled)
		tmd2755_process_saturation_event(chip);

	/* Process an actual prox event detect/release */
	if (status & TMD2755_INT_ST_PRX_IRQ) {
		/* Not using chip->in_psat because after the initial saturation it is not updated,  until */
		/* the release event.                                                                     */
		dev_dbg(dev, "%*.*s():%*d --> Proximity Interrupt Occurred - Saturation = %s\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, 
			(status & TMD2755_INT_ST_PRX_SAT_IRQ) ? "TRUE" : "FALSE");

		tmd2755_process_prox_irq(chip);
	}

	/***************/
	/* Calibration */
	/***************/
	/* If you get a calibration interrupt and you are in calibration, process */
	if ((status & TMD2755_INT_ST_CALIB_IRQ) && chip->in_calib) {
		log("Calibration Interrupt Occurred, cur_offset = %d, last_offset=%d, limit = %d", 
		chip->params.poffset, chip->params.poffset_last, chip->params.poffset_limit);

		/*
		 ** Calibration has completed, no need for more
		 **  calibration interrupts. These events are one-shots.
		 **  next calibration start will re-enable.
		 */
		ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_INTENAB, TMD2755_CIEN, 0);
		ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_CALIB,   TMD2755_MASK_START_OFFSET_CALIB, 0);
		ams_i2c_read(chip->client,  TMD2755_REG_CALIB_OFF, &chip->shadow[TMD2755_REG_CALIB_OFF]);
		tmd2755_read_poffset(chip);
		if(chip->params.poffset >= chip->params.poffset_limit){
			tmd2755_read_prox(chip);
			log("Recovery poffset as last offset: %d, raw=%d", 
			chip->params.poffset_last, chip->prox_info.raw);
			if(chip->params.poffset_last < 0){
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_POFFSET_H, TMD2755_REG_POFFSET_H, TMD2755_REG_POFFSET_H);
				chip->params.poffset_last *=-1;
			}
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_POFFSET_L, chip->params.poffset_last);
			mdelay(12); //for psensor scan time, to make sure that psensor data will be updated according to right offset 
		}else{
			chip->params.poffset_last = chip->params.poffset;
		}

		complete_all(&(chip->calibration_done));
		if(chip->als_enable == true){
			log("als sensor on, enable PWEN");
			ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_ENABLE, TMD2755_PWEN, TMD2755_PWEN);
		}
	} else if (status & TMD2755_INT_ST_CALIB_IRQ)
		dev_info(dev, "%*.*s():%*d --> CINT Occurred While NOT in Calibration\n",  MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN,
			__func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);	

	AMS_MUTEX_UNLOCK(&chip->lock);
	AMS_MUTEX_UNLOCK(&g_alsps_lock);
	return 1;  /* handled the interrupt */
}

static void tmd2755_ist(struct work_struct *work){
	struct tmd2755_chip *chip = g_tmd2755_chip;
	if (chip->in_suspend) {
		schedule_delayed_work(&tmd2755_ist_work, msecs_to_jiffies(5));
	}else{
		tmd2755_ALSPS_hw_get_interrupt();
		__pm_relax(g_tmd2755_wake_lock);
		enable_irq(ALSPS_SENSOR_IRQ);
	}
}

static irqreturn_t tmd2755_irq(int irq, void *handle)
{
	struct tmd2755_chip *chip = handle;
	struct device *dev = &chip->client->dev;
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ALSPS_SENSOR_IRQ);

	if (chip->in_suspend) {
		dev_info(dev, "%*.*s():%*d --> in suspend\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		chip->irq_pending = 1;
		__pm_stay_awake(g_tmd2755_wake_lock);
		schedule_delayed_work(&tmd2755_ist_work, msecs_to_jiffies(5));
	}else{
		__pm_stay_awake(g_tmd2755_wake_lock);
		tmd2755_ALSPS_hw_get_interrupt();
		__pm_relax(g_tmd2755_wake_lock);
		enable_irq(ALSPS_SENSOR_IRQ);
	}
/*
bypass:
	return ret ? IRQ_HANDLED : IRQ_NONE;
*/
	return IRQ_HANDLED;
}

static int tmd2755_flush_regs(struct tmd2755_chip *chip)
{
	unsigned int i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%*.*s():%*d --> err on reg 0x%02X\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, reg);
			break;
		}
	}
	return rc;
}


static int tmd2755_set_defaults(struct tmd2755_chip *chip)
{
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	/* Clear the register shadow area */
	memset(chip->shadow, 0x00, sizeof(chip->shadow));

	/* If there is platform data use it */
	if (chip->pdata) {
		dev_info(dev, "%*.*s():%*d --> Loading platform data\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		chip->params.prox_thresh_max      = chip->pdata->parameters.prox_thresh_max;
		chip->params.prox_thresh_min      = chip->pdata->parameters.prox_thresh_min;
		chip->params.persist.persistance  = chip->pdata->parameters.persist.persistance; /* takes care of both als and prox */
		chip->params.prox_pulse_cnt       = chip->pdata->parameters.prox_pulse_cnt;
		chip->params.prox_apc             = chip->pdata->parameters.prox_apc;
		chip->params.prox_time            = chip->pdata->parameters.prox_time;
		chip->params.prox_wtime            = chip->pdata->parameters.prox_wtime;
		chip->params.prox_gain1           = chip->pdata->parameters.prox_gain1;
		chip->params.prox_gain2           = chip->pdata->parameters.prox_gain2;
		chip->params.prox_drive           = chip->pdata->parameters.prox_drive;
		chip->params.poffset              = chip->pdata->parameters.poffset;
		chip->params.prox_pulse_len       = chip->pdata->parameters.prox_pulse_len;

		chip->params.als_gain             = chip->pdata->parameters.als_gain;
		chip->params.als_auto_gain        = chip->pdata->parameters.als_auto_gain ? true : false;
		chip->params.als_deltaP           = chip->pdata->parameters.als_deltaP;
		chip->params.als_time             = chip->pdata->parameters.als_time;
		chip->params.als_wtime             = chip->pdata->parameters.als_wtime; /* 50ms */
		chip->params.dgf                  = chip->pdata->parameters.dgf;
		chip->params.ch0_coef0            = chip->pdata->parameters.ch0_coef0;
		chip->params.ch0_coef1            = -chip->pdata->parameters.ch0_coef1;
		chip->params.ch1_coef0            = -chip->pdata->parameters.ch1_coef0;
		chip->params.ch1_coef1            = chip->pdata->parameters.ch1_coef1;
		chip->params.coef_scale           = chip->pdata->parameters.coef_scale;
		chip->params.poffset_limit        = PROX_OFFSET_MAX;
		chip->params.poffset_fac          = PROX_OFFSET_INIT;
		chip->params.poffset_last          = PROX_OFFSET_INIT;
	} else {
		dev_info(dev, "%*.*s():%*d --> use defaults\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		chip->params.prox_thresh_min     = 64;
		chip->params.prox_thresh_max     = 128;
		chip->params.persist.persistance = PROX_PERSIST(1) | ALS_PERSIST(2);
		chip->params.prox_pulse_cnt      = 4;
		chip->params.prox_apc            = 0; /* 0 -> APC enabled */
		chip->params.prox_time           = 0x1F;
		chip->params.prox_gain1          = PGAIN1_4;
		chip->params.prox_gain2          = PGAIN2_2_5;
		chip->params.prox_drive          = PDRIVE_MA(12);  /* 12 milliamps */
		chip->params.poffset             = 0;
		chip->params.prox_pulse_len      = PPULSE_LEN_US(32);

		chip->params.als_gain            = AGAIN_128;
		chip->params.als_auto_gain       = true;
		chip->params.als_deltaP          = 10;
		chip->params.als_time            = AW_TIME_MS(200);
		chip->params.dgf                 = ALS_DGF_DEFAULT;
		chip->params.ch0_coef0           = ALS_CH0_COEFF0_DEFAULT;
		chip->params.ch0_coef1           = ALS_CH0_COEFF1_DEFAULT;
		chip->params.ch1_coef0           = ALS_CH1_COEFF0_DEFAULT;
		chip->params.ch1_coef1           = ALS_CH1_COEFF1_DEFAULT;
		chip->params.coef_scale          = ALS_COEFF_SCALE_DEFAULT;
	}
	sh[TMD2755_REG_PRATE]    = (chip->params.prox_time & 0xff);

	/* Copy the default values into the register shadow area */
	sh[TMD2755_REG_PILTL]    = (chip->params.prox_thresh_min & TMD2755_MASK_PILTL);
	sh[TMD2755_REG_PILTH]    = ((chip->params.prox_thresh_min >> 8) & TMD2755_MASK_PILTH);
	sh[TMD2755_REG_PIHTL]    = (chip->params.prox_thresh_max & TMD2755_MASK_PIHTL);
	sh[TMD2755_REG_PIHTH]    = ((chip->params.prox_thresh_max >> 8) & TMD2755_MASK_PIHTH);
	sh[TMD2755_REG_PERS]     = chip->params.persist.persistance;  /* als and prox */

	/* CFG0 - PWLONG and AWLONG */

	sh[TMD2755_REG_CFG6]    &= ~(1 << TMD2755_SHIFT_APC); /* enable apc */
	sh[TMD2755_REG_CFG6]    |= (chip->params.prox_apc << TMD2755_SHIFT_APC) | 0x3F; /* set lower 6 bits to default */

	/* Reserve 7:4 must be 0101 by default, to solve vendor's bug, default will be set 0 in tmd2755_flush_als_regs() */
	sh[TMD2755_REG_CFG0]   = 0x50;

	sh[TMD2755_REG_PCFG0]   = (chip->params.prox_gain1 << TMD2755_SHIFT_PGAIN1) |
				(chip->params.prox_pulse_cnt & TMD2755_MASK_PPULSE);

	/* bits 5:4 must be set to b11 */
	sh[TMD2755_REG_PCFG1]   = ((chip->params.prox_pulse_len & TMD2755_MASK_DATA_PPULSE_LEN_H) >> TMD2755_SHIFT_PPULSE_LEN_H) |
					(0x3 << 4) | (chip->params.prox_drive & 0x0F);

	sh[TMD2755_REG_PCFG2]   = (chip->params.prox_pulse_len & TMD2755_MASK_PPULSE_LEN_L);
	sh[TMD2755_REG_CFG1]    = (chip->params.prox_gain2 << TMD2755_SHIFT_PGAIN2) | chip->params.als_gain;
	sh[TMD2755_REG_CFG8]    = CFG8_REG_REQUIRED_VAL;

	/* Poffset */
	if (chip->params.poffset > 0) {
		sh[TMD2755_REG_POFFSET_L]     = chip->params.poffset;
		sh[TMD2755_REG_POFFSET_H]     = 0x00;
	} else {
		sh[TMD2755_REG_POFFSET_L]     = (chip->params.poffset * -1);
		sh[TMD2755_REG_POFFSET_H]     = 0x1;
	}

	sh[TMD2755_REG_ATIME]    = chip->params.als_time;
	sh[TMD2755_REG_PRATE]    = P_TIME_US(INTEGRATION_CYCLE);

	/* CFG3 - INT_READ_CLEAR and SAI */

	sh[TMD2755_REG_TEST3]    = TST3_REG_REQUIRED_VAL;
	sh[TMD2755_REG_TEST9]    = TST9_REG_REQUIRED_VAL;

	return tmd2755_flush_regs(chip);
}

static int get_id(struct i2c_client *client, struct device_ids *ids)
{
	ids->aux = 0;
	ams_i2c_read(client, TMD2755_REG_REVID,    &ids->rev);
	ams_i2c_read(client, TMD2755_REG_DEVICEID, &ids->device);
	ams_i2c_read(client, TMD2755_REG_REVID2,   &ids->aux);

	ids->device = (ids->device & TMD2755_MASK_DEVICEID) >> TMD2755_SHIFT_DEVICEID;
	ids->rev    = (ids->rev    & TMD2755_MASK_REVID)    >> TMD2755_SHIFT_REVID;
	ids->aux    = (ids->aux    & TMD2755_MASK_REVID2)   >> TMD2755_SHIFT_REVID2;

	dev_info(&client->dev, "%*.*s():%*d --> device id: %02X device revid: 0x%02X device aux: %02X\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, ids->device, ids->rev, ids->aux);

	return 0;
}

static int tmd2755_check_ID(struct tmd2755_chip *chip)
{
	int ret = EOPNOTSUPP;
	struct device_ids id = 
	{
		.device = 0,
		.rev    = 0,
		.aux    = 0,
	};
	u8 idx;

	/* Make sure id, and rev are supported by driver */
	/* do not use Aux id yet */
	if (!get_id(chip->client, &id)) {
		for (idx = 0; idx < ARRAY_SIZE(dev_ids); idx++) {
			if ((id.device == dev_ids[idx].device) &&
			    (id.rev    == dev_ids[idx].rev)/* && 
			    (id.aux    == dev_ids[idx].aux)*/) {
				    /* We have a match */
				dev_info(&chip->client->dev, "%*.*s():%*d --> device match : ID=0x%02X REV=0x%02X AUX=0x%02X\n",
					MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, id.device, id.rev,
					id.aux);
				break;
			}
		}
		if (idx < ARRAY_SIZE(device_names)) {
			dev_info(&chip->client->dev, "%*.*s():%*d --> '%s' detected\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, device_names[idx]);
			chip->device_index = idx;
			ret = 0;
		} else {
			dev_err(&chip->client->dev, "%*.*s():%*d --> not supported chip id\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
			ret = -EOPNOTSUPP;
		}
	}
	return ret;
}

static int tmd2755_pltf_power_on(struct tmd2755_chip *chip)
{
	int rc = 0;
	/* Not defined for TMD2755 */
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev, POWER_ON);
		dev_info(&chip->client->dev, "%*.*s():%*d --> platform_power() was called\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		usleep_range(10000, 11000);
	}
	chip->unpowered = rc != 0;
	dev_info(&chip->client->dev, "%*.*s()%*d: unpowered=%d\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, chip->unpowered);
	return rc;
}

static int tmd2755_pltf_power_off(struct tmd2755_chip *chip)
{
	int rc = 0;

	/* Not defined for TMD2755 */
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev, POWER_OFF);
		chip->unpowered = (rc == 0);
	} else {
		chip->unpowered = false;
	}
	dev_info(&chip->client->dev, "%*.*s():%*d --> unpowered=%d\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, 
		__func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, chip->unpowered);
	return rc;
}

static int tmd2755_power_on(struct tmd2755_chip *chip)
{
	int rc;

	dev_info(&chip->client->dev, "  %*.*s():%*d --> Power On\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
	rc = tmd2755_pltf_power_on(chip);
	if (rc)
		return rc;
	dev_info(&chip->client->dev, "%*.*s():%*d --> chip was off, restoring registers\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
	return tmd2755_flush_regs(chip);
}

static int tmd2755_proximity_hw_turn_onoff(bool bOn)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	int rc = 0;

	AMS_MUTEX_LOCK(&chip->lock);
	if(bOn){
		rc = tmd2755_regulator_enable();
		if(rc){
			return rc;
		}
		if (chip->unpowered) {
			rc = tmd2755_power_on(chip);
			if (rc)
				goto power_on_err;
		}
		rc = tmd2755_configure_prox_mode(chip, TMD2755_FEATURE_ON);
	}else{
		tmd2755_configure_prox_mode(chip, TMD2755_FEATURE_OFF);
		AMS_MUTEX_UNLOCK(&chip->lock);
	}
power_on_err:
	AMS_MUTEX_UNLOCK(&chip->lock);
	return rc;
}
static int tmd2755_proximity_hw_interrupt_onoff(bool bOn)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;
	int ret = 0;
	u8 val;

	ams_i2c_read(chip->client, TMD2755_REG_INTENAB, &val);

	if(bOn){
		ret = ams_i2c_modify(client, sh, TMD2755_REG_INTENAB, TMD2755_PIEN | TMD2755_PSIEN | TMD2755_ZIEN,
			TMD2755_PIEN | TMD2755_PSIEN | TMD2755_ZIEN);
		if(ret < 0){
			err("Proximity Sensor Enable INT ERROR (REG_INTENAB)\n");
			return ret;
		}else{
			log("Proximity Sensor Enable INT (REG_INTENAB : 0x%x -> 0x%x)\n", 
				val, val | TMD2755_PIEN | TMD2755_PSIEN | TMD2755_ZIEN);
		}
	}else{
		ret = ams_i2c_modify(client, sh, TMD2755_REG_INTENAB, TMD2755_PROX_INTS, 0);
		if(ret < 0){
			err("Proximity Sensor Disable INT ERROR (REG_INTENAB)\n");
			return ret;
		}else{
			log("Proximity Sensor Disable INT (REG_INTENAB : 0x%x -> 0x%x)\n", 
				val, val & (0xff & !TMD2755_PIEN & !TMD2755_PSIEN & !TMD2755_ZIEN));
		}
	}
	return 0;
}
	
static int tmd2755_light_hw_turn_onoff(bool bOn){

	struct tmd2755_chip *chip = g_tmd2755_chip;
	int rc = 0;

	log("%s():%d --> -------------------------------------------------", __func__, __LINE__);
	
	AMS_MUTEX_LOCK(&chip->lock);
	if(bOn){
		rc = tmd2755_regulator_enable();
		if(rc){
			return rc;
		}

		if (chip->unpowered) {
			rc = tmd2755_power_on(chip);
			if (rc)
				goto chip_on_err;
		}
		rc = tmd2755_configure_als_mode(chip, TMD2755_FEATURE_ON);
	}else{
		tmd2755_configure_als_mode(chip, TMD2755_FEATURE_OFF);
		chip->als_info.ch0_raw = 0;
		chip->als_info.ch1_raw = 0;
	}
chip_on_err:
	AMS_MUTEX_UNLOCK(&chip->lock);
	return rc;
	

}

#ifndef REMOVE_INPUT_DEVICE
static int tmd2755_prox_idev_open(struct input_dev *idev)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	int rc;
	bool als = chip->als_idev && chip->als_idev->users;

	dev_info(&idev->dev, " %*.*s():%*d --> -------------------------------------------------\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
	dev_info(&idev->dev, " %*.*s():%*d --> als = %s\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__,
		LINE_NUM_KERNEL_LOG_LEN, __LINE__,  als ? "true" : "false");

	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->unpowered) {
		rc = tmd2755_power_on(chip);
		if (rc)
			goto power_on_err;
	}
	rc = tmd2755_configure_prox_mode(chip, TMD2755_FEATURE_ON);
	/* If prox fails to configure and als is not on, power off the device */
	if (rc && !als)
		(void)tmd2755_pltf_power_off(chip);
power_on_err:
	AMS_MUTEX_UNLOCK(&chip->lock);
	return rc;
}

static void tmd2755_prox_idev_close(struct input_dev *idev)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;

	dev_info(&idev->dev, " %*.*s():%*d --> Prox Close\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);

	AMS_MUTEX_LOCK(&chip->lock);
	tmd2755_configure_prox_mode(chip, TMD2755_FEATURE_OFF);
	if (!chip->als_idev || !chip->als_idev->users)
		tmd2755_pltf_power_off(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return;
}

static int tmd2755_als_idev_open(struct input_dev *idev)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	bool prox = chip->prox_idev && chip->prox_idev->users;
	int rc = 0;

	dev_info(&idev->dev, " %*.*s():%*d --> -------------------------------------------------\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
	dev_info(&idev->dev, " %*.*s():%*d --> prox = %s\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__,
		LINE_NUM_KERNEL_LOG_LEN, __LINE__,  prox ? "true" : "false");
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->unpowered) {
		rc = tmd2755_power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = tmd2755_configure_als_mode(chip, TMD2755_FEATURE_ON);
	/* If als configure fails and prox is not on, then turn off chip */
	if (rc && !prox)
        	tmd2755_pltf_power_off(chip);
chip_on_err:
	AMS_MUTEX_UNLOCK(&chip->lock);
	return rc;
}

static void tmd2755_als_idev_close(struct input_dev *idev)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
    	dev_info(&idev->dev, " %*.*s():%*d --> ALS Close\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);

	AMS_MUTEX_LOCK(&chip->lock);

	tmd2755_configure_als_mode(chip, TMD2755_FEATURE_OFF);
	if (!chip->prox_idev || !chip->prox_idev->users)
		tmd2755_pltf_power_off(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return;
}
#endif

#ifdef DEBUG_ABI_SET_GET_REGISTERS

/* bitmap of registers that are in use */
static u8 reg_in_use[MAX_REGS / 8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    /* 0x00 - 0x3f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    /* 0x40 - 0x7f */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,    /* 0x80 - 0xbf */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,    /* 0xc0 - 0xff */
};

static ssize_t tmd2755_registers_get(struct tmd2755_chip *chip, char *buf, int bufsiz)
{
	u8 regval[16];
	int i, j, k, cnt;
	if(g_tmd2755_chip == NULL){
		log("chip is null!!!!!!!!!!!!");
	}
	/* find first non-zero bank of registers */
	for (i = 0; i < ARRAY_SIZE(reg_in_use); i++) {
		if (reg_in_use[i] != 0)
			break;
	}

	i &= ~1;  /* round down to the start of a group of 16 */
	i *= 8;   /* set to actual register id - each bit in the map represents a register byte*/

	cnt = 0;

	/* Print the index along the top of the registers */
	cnt += scnprintf(buf + cnt, bufsiz - cnt, "     ");
	for (k = 0; k < 16; k++) {
		cnt += scnprintf(buf + cnt, bufsiz - cnt, " %01x ", k);
		if (k == 7)
			cnt += scnprintf(buf + cnt, bufsiz - cnt, "  ");
	}
	cnt += scnprintf(buf + cnt, bufsiz - cnt, "\n     -------------------------------------------------\n");

	/* Dump the registers */
	for (; i < MAX_REGS; i += 16) {
		cnt += scnprintf(buf + cnt, bufsiz - cnt, "%02x: ", i);

		ams_i2c_blk_read(g_tmd2755_chip->client, i, &regval[0], 16);

		for (j = 0; j < 16; j++) {
			if (reg_in_use[(i >> 3) + (j >> 3)] & (1 << (j & 7)))
				cnt += scnprintf(buf + cnt, bufsiz - cnt, " %02x", regval[j]);
			else
				cnt += scnprintf(buf + cnt, bufsiz - cnt, " --");

			if (j == 7)
				cnt += scnprintf(buf + cnt, bufsiz - cnt, "  ");
		}
		cnt += scnprintf(buf + cnt, bufsiz - cnt, "\n");
	}

	cnt += scnprintf(buf + cnt, bufsiz - cnt, "\n");
	return cnt;
}

/**
 * Function to log register values to the /var/log/messages
 **/
void tmd2755_reg_log(struct tmd2755_chip *chip)
{
	char *buf;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf) {
		tmd2755_registers_get(chip, &buf[0], PAGE_SIZE);
		printk(KERN_ERR "%s", buf);
		kfree(buf);
	} else {
		dev_err(&chip->client->dev, "%*.*s():%*d --> Out of memory\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
	}
	return;
}

ssize_t tmd2755_registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_registers_get(g_tmd2755_chip, buf, PAGE_SIZE);
}


/*
 *  Usage:
 *		reg_addr:value:mask
 *		echo "0x80:0x44:0xFF" > sys/class/input/inputX/regs
 *	or
 *		echo 128:44:-1 > sys/class/input/inputX/regs
 *
 * The last form uses the register number in decimal.  The rest of the values would need to be
 * in decimal too.
 */
ssize_t tmd2755_registers_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int preg;
	int pval;
	int pmask = -1;
	int numparams;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	numparams = sscanf(buf, "0x%x:0x%x:0x%x", &preg, &pval, &pmask);
	if (numparams == 0) {
		/* try decimal format */
		numparams = sscanf(buf, "%d:%d:%d", &preg, &pval, &pmask);
	}

	/* Validate the input is as described by usage */
	if ((numparams < 2) || (numparams > 3))
		return -EINVAL;
	if ((numparams >= 1) && ((preg < 0) || ((reg_in_use[(preg >> 3)] & (1 << (preg & 7))) == 0)))
		return -EINVAL;
	if ((numparams >= 2) && (preg < 0 || preg > 0xff))
		return -EINVAL;
	if ((numparams >= 3) && (pmask < 0 || pmask > 0xff))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);

	if (pmask == -1)
		rc = ams_i2c_write(chip->client, chip->shadow, preg, pval);
	else
		rc = ams_i2c_modify(chip->client, chip->shadow, preg, pmask, pval);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return rc ? rc : size;
}

#ifndef REMOVE_INPUT_DEVICE
struct device_attribute tmd2755_dbg_attrs[] = {
	__ATTR(regs, 0644, tmd2755_registers_show, tmd2755_registers_store),
};

int tmd2755_dbg_attrs_size = ARRAY_SIZE(tmd2755_dbg_attrs);
#endif
#endif /* DEBUG_ABI_SET_GET_REGISTERS */

#ifndef REMOVE_INPUT_DEVICE

static int tmd2755_add_sysfs_interfaces(struct device *dev, struct device_attribute *a, int size)
{
	int i;

	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%*.*s():%*d --> failed to create sysfs interface: %s\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, dev->init_name);
	return -ENODEV;
}

static void tmd2755_remove_sysfs_interfaces(struct device *dev, struct device_attribute *a, int size)
{
	int i;

	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

#endif

//ATTR===================================================================
static struct device *g_tmd2755_als;
static struct device *g_tmd2755_prox;

int tmd2755_ATTR_register(void)
{
	int ret = 0;
	int ATTR_index;
	/* lsensor device */
	g_tmd2755_als = ASH_ATTR_device_create(lsensor_2nd);
	if (IS_ERR(g_tmd2755_als) || g_tmd2755_als == NULL) {
		ret = PTR_ERR(g_tmd2755_als);
		err("%s: psensor create ERROR.\n", __FUNCTION__);
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < tmd2755_als_attrs_size; ATTR_index++) {
		ret = device_create_file(g_tmd2755_als, &tmd2755_als_attrs[ATTR_index]);
		if (ret)
			return ret;
	}

	/* lsensor device */
	g_tmd2755_prox = ASH_ATTR_device_create(psensor_2nd);
	if (IS_ERR(g_tmd2755_prox) || g_tmd2755_prox == NULL) {
		ret = PTR_ERR(g_tmd2755_prox);
		err("%s: psensor create ERROR.\n", __FUNCTION__);
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < tmd2755_prox_attrs_size; ATTR_index++) {
		ret = device_create_file(g_tmd2755_prox, &tmd2755_prox_attrs[ATTR_index]);
		if (ret)
			return ret;
	}

	return 0;
}

//===================================================================
static void set_pinctrl(struct i2c_client *client)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(&client->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, GPIO_LOOKUP_STATE);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0)
		err("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
}

static int init_irq (void)
{
	//int ret = 0;
	int irq = 0;

	// GPIO to IRQ 
	irq = gpio_to_irq(ALSPS_SENSOR_GPIO);
	if (irq < 0) {
		err("%s: gpio_to_irq ERROR(%d). \n", __FUNCTION__, irq);
		return irq;
	}else {
		log("gpio_to_irq IRQ %d successed on GPIO:%d\n", irq, ALSPS_SENSOR_GPIO);
	}
	/*
	ret = request_threaded_irq(irq, NULL, tmd2755_irq,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, ALSPS_INT_NAME, NULL);
	*/
	//Request IRQ
	/*
	ret = request_threaded_irq(irq, NULL, tmd2755_irq,
				 IRQF_SHARED | IRQF_ONESHOT , ALSPS_INT_NAME, NULL);
	if (ret < 0) {
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	}else {		
		dbg("Disable irq !! \n");
		disable_irq(irq);
	}*/

	return irq;
}
int tmd2755_gpio_register(struct i2c_client *client)
{
	int ret = 0;
	//int irq = 0;
	unsigned long default_irq_trigger;
	// GPIO 
	log("Qcom GPIO \n");
	set_pinctrl(client);
	ALSPS_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, ALSPS_QCOM_NAME, 0);
		
	log("[GPIO] GPIO =%d(%d)\n", ALSPS_SENSOR_GPIO, gpio_get_value(ALSPS_SENSOR_GPIO));	
	// GPIO Request
	ret = gpio_request(ALSPS_SENSOR_GPIO, ALSPS_IRQ_NAME);
	if (ret) {
		err("%s: gpio_request ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	// GPIO Direction
	ret = gpio_direction_input(ALSPS_SENSOR_GPIO);

	if (ret < 0) {
		err("%s: gpio_direction_input ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	// IRQ
	ALSPS_SENSOR_IRQ = init_irq();
	log("client_irq=%d, IRQ=%d", client->irq, ALSPS_SENSOR_IRQ);
	ALSPS_SENSOR_IRQ = client->irq;
	default_irq_trigger = irqd_get_trigger_type(irq_get_irq_data(client->irq)); 	// default_irq_trigger = low level trigger
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, &tmd2755_irq, default_irq_trigger | IRQF_SHARED | IRQF_ONESHOT,
					dev_name(&client->dev), g_tmd2755_chip);

	return ALSPS_SENSOR_IRQ;

}
EXPORT_SYMBOL(tmd2755_gpio_register);

static struct regulator *reg;
static int enable_3v_count = 0;
static int tmd2755_regulator_init(struct i2c_client *client)
{
	int ret = 0;

	reg = regulator_get(&client->dev,"vcc_psensor");
	if (IS_ERR_OR_NULL(reg)) {
		ret = PTR_ERR(reg);
		err("Failed to get regulator vcc_psensor %d\n", ret);
		return ret;
	}
	
	ret = regulator_set_voltage(reg, 3300000, 3300000);
	if (ret) {
		err("Failed to set voltage for vcc_psensor reg %d\n", ret);
		return -1;
	}
	
	log("vcc_psensor regulator setting init");
	return ret;
}
static int tmd2755_regulator_enable(void)
{
	int ret = 0, idx = 0;

	if(IS_ERR_OR_NULL(reg)){
		ret = PTR_ERR(reg);
		err("Failed to get regulator vcc_psensor %d\n", ret);
		return ret;
	}
	
	ret = regulator_set_load(reg, 10000);
	if(ret < 0){
		err("Failed to set load for vcc_psensor reg %d\n", ret);
		return ret;
	}
	
	ret = regulator_enable(reg);
	if(ret){
		err("Failed to enable vcc_psensor reg %d\n", ret);
		return -1;
	}
	
	for(idx=0; idx<10; idx++){
		if(regulator_is_enabled(reg) > 0){
			dbg("vcc_psensor regulator is enabled(idx=%d)", idx);
			break;
		}
	}
	if(idx >= 10){
		err("vcc_psensor regulator is enabled fail(retry count >= %d)", idx);
		return -1;
	}
	enable_3v_count++;
	/* wait 5s for power ready */
	msleep(5);
	log("Update vcc_psensor to NPM_mode");
	return ret;
}

static int tmd2755_regulator_disable(void)
{
	int ret = 0;

	if(IS_ERR_OR_NULL(reg)){
		ret = PTR_ERR(reg);
		err("Failed to get regulator vcc_psensor %d\n", ret);
		return ret;
	}

	ret = regulator_set_load(reg, 0);
	if(ret < 0){
		err("Failed to set load for vcc_psensor reg %d\n", ret);
		return ret;
	}
	do{
		ret = regulator_disable(reg);
		if(ret){
			err("Failed to enable vincentr reg %d\n", ret);
			return -1;
		}
		enable_3v_count--;
	}while(enable_3v_count > 0);

	log("Update vcc_psensor to LPM_mode");
	return ret;
}

static int tmd2755_ALSPS_hw_check_ID(void)
{
	if(false == g_tmd2755_status_param.probe_status){
		return -1;
	}
	return tmd2755_check_ID(g_tmd2755_chip);;
}

static int tmd2755_ALSPS_hw_init(struct i2c_client *client)
{
	int ret;
	struct device *dev = &client->dev;
	struct tmd2755_i2c_platform_data *pdata = dev->platform_data;
	static struct tmd2755_chip *chip;
	bool powered = 0;
	g_i2c_client = client;

	dev_info(dev, "%*.*s():%*d --> Device <%s> with irq=%d being probed.\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, client->name, client->irq);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%*.*s():%*d --> i2c smbus byte data unsupported\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}

#if defined(CONFIG_OF)
	if (!pdata) {
		pdata = devm_kzalloc(dev, sizeof(struct tmd2755_i2c_platform_data),
					GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		if (of_match_device(tmd2755_of_match, &client->dev)) {
			pdata->of_node = client->dev.of_node;
			ret = tmd2755_init_dt(pdata);
			if (ret)
				return ret;
		}
	}
#endif /* CONFIG_OF */

	/* Normally not executed if CONFIG_OF */
	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(dev, "%*.*s():%*d --> : pltf power on failed\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	mutex_init(&chip->lock);
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	/* Init the wait for completion structure - used to sync */
	/* calibration thread and ISR */
	init_completion(&(chip->calibration_done));
	chip->cal_en = true;

	tmd2755_regulator_init(client);
	tmd2755_regulator_enable();

	/* Validate the correct AMS device is attached */
	if (tmd2755_check_ID(chip) != 0){
		ret = -EOPNOTSUPP;
		goto id_failed;
	}

	/*********************/
	/* Set Chip Defaults */
	/*********************/
	if (tmd2755_set_defaults(chip))
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}
	
#ifndef REMOVE_INPUT_DEVICE
	/***********************/
	/*   Set up Proximity  */
	/***********************/
	if (!pdata->prox_name)
		goto bypass_prox_feature;
	chip->prox_idev = devm_input_allocate_device(dev);
	if (!chip->prox_idev) {
		dev_err(dev, "%*.*s():%*d -->  no memory for input_dev '%s'\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, pdata->prox_name);
		ret = -ENODEV;
		goto input_prox_alloc_failed;
	}
	chip->prox_idev->name = pdata->prox_name;
	chip->prox_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->prox_idev->evbit); /* event types supported */
	set_bit(ABS_DISTANCE, chip->prox_idev->absbit); /* abs axes */
	/* 1 prox event 2 saturation event */
	input_set_abs_params(chip->prox_idev, ABS_DISTANCE, 0, 2, 0, 0); /* min, max, fuzz, flat */
	chip->prox_idev->open  = tmd2755_prox_idev_open;
	chip->prox_idev->close = tmd2755_prox_idev_close;
	input_set_drvdata(chip->prox_idev, chip);
	ret = input_register_device(chip->prox_idev);
	if (ret) {
		dev_err(dev, "%*.*s():%*d --> Cannot register input '%s'\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN,
			__LINE__, pdata->prox_name);
		goto input_prox_alloc_failed;
	}
	log("create prox sysfs interface +++");

	ret = tmd2755_add_sysfs_interfaces(&chip->prox_idev->dev, tmd2755_prox_attrs, tmd2755_prox_attrs_size);
	if (ret)
		goto input_prox_sysfs_failed;

	log("create prox sysfs interface ---");

bypass_prox_feature:
	/*****************************/
	/*     Set up ALS            */
	/*****************************/
	if (!pdata->als_name)
		goto bypass_als_feature;

	chip->als_idev = devm_input_allocate_device(dev);
    	if (!chip->als_idev) {
		dev_err(dev, "%*.*s():%*d -->  no memory for input_dev '%s'\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN,
			__LINE__, pdata->als_name);
		ret = -ENODEV;
		goto input_als_alloc_failed;
	}
	chip->als_idev->name = pdata->als_name;
	chip->als_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->als_idev->evbit);
	set_bit(ABS_MISC, chip->als_idev->absbit);
	input_set_abs_params(chip->als_idev, ABS_MISC, 0, USHRT_MAX, 0, 0);
	chip->als_idev->open = tmd2755_als_idev_open;
	chip->als_idev->close = tmd2755_als_idev_close;
	input_set_drvdata(chip->als_idev, chip);
	ret = input_register_device(chip->als_idev);
	if (ret) {
		dev_err(dev, "%*.*s():%*d --> Cannot register input '%s'\n", MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN,
			__LINE__, pdata->als_name);
		goto input_als_alloc_failed;
	}
bypass_als_feature:

	/*****************************/
	/*     Set up DEBUG          */
	/*****************************/
	/* Allows user to dump all registers to syfs file */
#ifdef DEBUG_ABI_SET_GET_REGISTERS
	chip->dbg_idev = devm_input_allocate_device(dev);
	if (!chip->dbg_idev) {
		dev_err(dev, "%*.*s():%*d --> no memory for debug input_dev '%s'\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__,
			device_names[chip->device_index]);
		ret = -ENODEV;
		goto input_debug_alloc_failed;
	}

	chip->dbg_idev->name = device_names[chip->device_index];
	chip->dbg_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->dbg_idev->evbit);
	set_bit(ABS_DISTANCE, chip->dbg_idev->absbit);  /* NA */
	input_set_abs_params(chip->dbg_idev, ABS_DISTANCE, 0, 1, 0, 0); /* NA */
	input_set_drvdata(chip->dbg_idev, chip);
	ret = input_register_device(chip->dbg_idev);
	if (ret) {
		dev_err(dev, "%s: cant register input '%s'\n", __func__, device_names[chip->device_index]);
		goto input_debug_alloc_failed;
	}
	ret = tmd2755_add_sysfs_interfaces(&chip->dbg_idev->dev, tmd2755_dbg_attrs, tmd2755_dbg_attrs_size);
	if (ret)
		goto input_debug_sysfs_failed;
#endif
#endif

	log("create als sysfs interface +++");
	
	ret = tmd2755_ATTR_register();
	if (ret)
		goto irq_register_fail;
	/*
	ret = tmd2755_add_sysfs_interfaces(&chip->als_idev->dev, tmd2755_als_attrs, tmd2755_als_attrs_size);
	if (ret)
		goto input_als_sysfs_failed;
	*/
	log("create als sysfs interface ---");
	

	/****************************/
	/* Initialize IRQ & Handler */
	/****************************/
	g_tmd2755_chip = chip;

	g_tmd2755_wake_lock = wakeup_source_register(NULL, "tmd2755_wake_lock");
	INIT_DELAYED_WORK(&tmd2755_ist_work, tmd2755_ist);

	/* Power up device */
	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_ENABLE, 0x01);
	log("Probe ok.\n");
	g_tmd2755_status_param.probe_status = true;

	return 0;

/******************************************************************
 * Exit points for device specific feature (ALS, prox, etc)       *
 * failures                                                       *
 ******************************************************************/
irq_register_fail:
	tmd2755_regulator_disable();
	
#ifndef REMOVE_INPUT_DEVICE
#ifdef DEBUG_ABI_SET_GET_REGISTERS
	if (chip->dbg_idev) {
		tmd2755_remove_sysfs_interfaces(&chip->dbg_idev->dev, tmd2755_dbg_attrs, tmd2755_dbg_attrs_size);
input_debug_sysfs_failed:
		input_unregister_device(chip->dbg_idev);
	}
input_debug_alloc_failed:
#endif /*  DEBUG_ABI_SET_GET_REGISTERS */
	if (chip->als_idev) {
		tmd2755_remove_sysfs_interfaces(&chip->als_idev->dev, tmd2755_als_attrs, tmd2755_als_attrs_size);
input_als_sysfs_failed:
		input_unregister_device(chip->als_idev);
	}
input_als_alloc_failed:
	if (chip->prox_idev) {
		tmd2755_remove_sysfs_interfaces(&chip->prox_idev->dev, tmd2755_prox_attrs, tmd2755_prox_attrs_size);
		
input_prox_sysfs_failed:
		input_unregister_device(chip->prox_idev);
	}
input_prox_alloc_failed:
#endif
/******************************************************************
 * Exit points for general device initialization failures         *
 ******************************************************************/
flush_regs_failed:
id_failed:
	g_tmd2755_status_param.probe_status = false;
	i2c_set_clientdata(client, NULL);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	dev_err(dev, "%*.*s():%*d --> Probe failed.\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
	return ret;
}

void tmd2755_suspend(void){
	struct tmd2755_chip *chip = g_tmd2755_chip;
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 1;
	log("in_suspend = %d", chip->in_suspend);
	if(chip->prox_enable){
		enable_irq_wake(ALSPS_SENSOR_IRQ);
		dbg("set irq wake enable");
	}
	AMS_MUTEX_UNLOCK(&chip->lock);
}
EXPORT_SYMBOL(tmd2755_suspend);

void tmd2755_resume(void){
	struct tmd2755_chip *chip = g_tmd2755_chip;
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 0;
	log("in_suspend = %d", chip->in_suspend);
	AMS_MUTEX_UNLOCK(&chip->lock);
}
EXPORT_SYMBOL(tmd2755_resume);


static int tmd2755_proximity_hw_get_adc(void)
{
	tmd2755_read_prox(g_tmd2755_chip);
	return g_tmd2755_chip->prox_info.raw;
}

static int tmd2755_proximity_hw_set_hi_threshold(int hi_threshold)
{
	unsigned long high_thresh;
	struct tmd2755_chip *chip = g_tmd2755_chip;
	u8 *sh = chip->shadow;
	high_thresh = (unsigned long)hi_threshold;

	if(false == chip->prox_info.detected){
		AMS_MUTEX_LOCK(&chip->lock);
		PROX_OFF();
		if (chip->params.prox_apc == PROX_APC_ENABLED) {
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTL, (high_thresh & 0xFF));
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTH, ((high_thresh >> 8) & TMD2755_MASK_PIHTH));
			chip->params.prox_thresh_max = high_thresh;
		} else {
			/* apc == 1 means APC is DISABLED, only 8 MSB of PIHT are used - threshold is only 14 bit number (i.e. right shift 6) */
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTL, ((high_thresh >> 6) & 0xFF));
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTH, 0);
			chip->params.prox_thresh_max = high_thresh/64;
		}
		log("write high threshold %d", chip->params.prox_thresh_max);
		PROX_ON();
		AMS_MUTEX_UNLOCK(&chip->lock);
	}else{
		chip->params.prox_thresh_max = high_thresh;
	}
	sh[TMD2755_REG_PIHTL]    = (chip->params.prox_thresh_max & TMD2755_MASK_PIHTL);
	sh[TMD2755_REG_PIHTH]    = ((chip->params.prox_thresh_max >> 8) & TMD2755_MASK_PIHTH);
	return 0;
}

static int tmd2755_proximity_hw_set_lo_threshold(int low_threshold)
{
	unsigned long low_thresh;
	struct tmd2755_chip *chip = g_tmd2755_chip;
	u8 *sh = chip->shadow;
	low_thresh = (unsigned long)low_threshold;

	if(true == chip->prox_info.detected){
		AMS_MUTEX_LOCK(&chip->lock);
		PROX_OFF();
		if (chip->params.prox_apc == PROX_APC_ENABLED) {
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PILTL, (low_thresh & 0xFF));
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PILTH, ((low_thresh >> 8) & TMD2755_MASK_PILTH));
			chip->params.prox_thresh_min = low_thresh;
		} else {
			/* apc == 1 means APC is DISABLED, only 8 MSB of PILT are used - threshold is only 14 bit number (i.e. right shift 6) */
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PILTL, ((low_thresh >> 6) & 0xFF));
			ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PILTH, 0);
			chip->params.prox_thresh_min = low_thresh/64;
		}
		log("write low threshold %d", chip->params.prox_thresh_min);
		PROX_ON();
		AMS_MUTEX_UNLOCK(&chip->lock);
	}else{
		chip->params.prox_thresh_min = low_thresh;
	}
	sh[TMD2755_REG_PILTL]    = (chip->params.prox_thresh_min & TMD2755_MASK_PILTL);
	sh[TMD2755_REG_PILTH]    = ((chip->params.prox_thresh_min >> 8) & TMD2755_MASK_PILTH);
	return 0;
}

static int tmd2755_proximity_hw_set_autoK(int autok)
{
	unsigned long high_thresh, low_thresh;
	struct tmd2755_chip *chip = g_tmd2755_chip;
	high_thresh = (unsigned long)autok + chip->params.prox_thresh_max;
	low_thresh = (unsigned long)autok + chip->params.prox_thresh_min;
	tmd2755_proximity_hw_set_hi_threshold(high_thresh);
	tmd2755_proximity_hw_set_lo_threshold(low_thresh);
	return 0;
}

static int tmd2755_proximity_hw_set_period(int period)
{
	unsigned long prate;
	struct tmd2755_chip *chip = g_tmd2755_chip;
	prate = (unsigned long)period;
	
	AMS_MUTEX_LOCK(&chip->lock);
	PROX_OFF();
	ams_i2c_write(chip->client, chip->shadow,
		TMD2755_REG_PRATE, prate);
	chip->params.prox_time = chip->shadow[TMD2755_REG_PRATE];
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int tmd2755_proximity_hw_chip_cal_en(bool flag)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	chip->cal_en = flag;
	log("calibration flag = %d", flag);
	return 0;
}

static int tmd2755_proximity_hw_get_offset(void)
{
	return g_tmd2755_chip->params.poffset;
}

static int tmd2755_proximity_hw_set_fac_offset(int offset)
{
	g_tmd2755_chip->params.poffset_fac = offset;
	return 0;
}

static int tmd2755_proximity_hw_set_offset_limit(int en, int thresh)
{
	if(en == 1){
		if(thresh < 50)
			thresh = 50;

		g_tmd2755_chip->params.poffset_limit = g_tmd2755_chip->params.poffset_fac + thresh;
	}else{
		g_tmd2755_chip->params.poffset_limit = PROX_OFFSET_MAX;
	}

	if(g_tmd2755_chip->params.poffset_limit > PROX_OFFSET_MAX){
		g_tmd2755_chip->params.poffset_limit = PROX_OFFSET_MAX;
	}

	log("en=%d, thresh=%d, poffset_limit update to %d", en, thresh, g_tmd2755_chip->params.poffset_limit);
	return 0;
}

static int tmd2755_proximity_hw_set_offset(int offset)
{
	g_tmd2755_chip->params.poffset = offset;
	log("assing offset as %d", offset);
	return 0;
}

static ssize_t tmd2755_ALSPS_hw_show_allreg(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_registers_get(g_tmd2755_chip, buf, PAGE_SIZE);
}

static int tmd2755_ALSPS_hw_get_register(uint8_t reg)
{
	int ret = 0;
	u8 buf = 0;
	int value;

	ams_i2c_read(g_i2c_client, reg,  &buf);
	if(ret < 0){
		err("ALSPS Get Register Value ERROR. (REG:0x%X)\n", reg);
		return ret;
	}
	log("Get Register Value (0x%x) = 0x%x\n", reg, buf);

	value = (int)buf;
	
	return value;
}

static int tmd2755_ALSPS_hw_set_register(uint8_t reg, int value)
{
	int ret = 0;
	u8 buf = 0;

	buf = (u8)value;

	ams_i2c_write(g_tmd2755_chip->client, g_tmd2755_chip->shadow,reg,  buf);
	if(ret < 0){
		err("ALSPS Set Register Value ERROR. (REG:0x%X)\n", reg);
		return ret;
	}
	log("Set Register Value (0x%x) = 0x%x\n", reg, buf);

	return 0;
}

static int tmd2755_ALSPS_hw_close_power(void)
{
	//tmd2755_regulator_disable_1v8();
	tmd2755_pltf_power_off(g_tmd2755_chip);
	tmd2755_regulator_disable();
	return 0;
}
static int tmd2755_light_hw_interrupt_onoff(bool bOn)
{
	return 0;
}

extern int tmd2755_get_lux(struct tmd2755_chip *chip);
static int tmd2755_light_hw_get_adc(void)
{
	tmd2755_read_als(g_tmd2755_chip);
	tmd2755_get_lux(g_tmd2755_chip);
	dbg("adc: %d, lux: %d", g_tmd2755_chip->als_info.ch0_raw, g_tmd2755_chip->als_info.lux);
	return g_tmd2755_chip->als_info.lux;
}

static int tmd2755_light_hw_set_hi_threshold(int hi_threshold)
{
	return 0;
}

static int tmd2755_light_hw_set_lo_threshold(int low_threshold)
{
	return 0;
}

static int tmd2755_light_hw_set_integration(uint8_t integration)
{
	/*Todo*/
	return 0;
}

static uint8_t tmd2755_light_hw_get_integration(void)
{
	/*Todo*/
	return 0;
}

static int tmd2755_light_hw_dynamic_check(int lux)
{
	/*Todo*/
	return 0;
}

static int tmd2755_light_hw_get_current_sensitive(void)
{
	/*Todo*/
	return 1;
}

static uint8_t tmd2755_light_hw_get_current_IT(void)
{
	/*Todo*/
	return 0;
}

static u64 tmd2755_light_hw_get_evt_skip_time_ns(void)
{
	/*Todo*/
	return 0;
}

static void tmd2755_light_hw_reset_ALS_dynamic_status(void)
{	
	/*Todo*/
	return;
}

/* Linux Driver Specific Structures */
/*
static struct i2c_device_id tmd2755_idtable[] = {
	{ "tmd2755", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tmd2755_idtable);

static const struct dev_pm_ops tmd2755_pm_ops = {
	.suspend = tmd2755_suspend,
	.resume  = tmd2755_resume,
};

static struct i2c_driver tmd2755_driver = {
	.driver = {
		.name = "tmd2755",
		.pm = &tmd2755_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(tmd2755_of_match),
#endif
	},
	.id_table = tmd2755_idtable,
	.probe = tmd2755_probe,
	.remove = tmd2755_remove,
};
module_i2c_driver(tmd2755_driver);
*/


static struct psensor_hw psensor_hw_tmd2755 = {
	.proximity_low_threshold_default = TMD2755_PROXIMITY_THDL_DEFAULT,
	.proximity_hi_threshold_default = TMD2755_PROXIMITY_THDH_DEFAULT,
	.proximity_crosstalk_default = TMD2755_PROXIMITY_INF_DEFAULT,
	.proximity_offset_default = TMD2755_PROXIMITY_OFFSET_DEFAULT,
	.proximity_autok_min = TMD2755_PROXIMITY_AUTOK_MIN,
	.proximity_autok_max = TMD2755_PROXIMITY_AUTOK_MAX,
	
	.proximity_hw_turn_onoff = tmd2755_proximity_hw_turn_onoff,
	.proximity_hw_interrupt_onoff = tmd2755_proximity_hw_interrupt_onoff,
	.proximity_hw_get_adc = tmd2755_proximity_hw_get_adc,
	.proximity_hw_set_hi_threshold = tmd2755_proximity_hw_set_hi_threshold,
	.proximity_hw_set_lo_threshold = tmd2755_proximity_hw_set_lo_threshold,
	.proximity_hw_set_autoK = tmd2755_proximity_hw_set_autoK,
	.proximity_hw_set_period = tmd2755_proximity_hw_set_period,
	.proximity_hw_chip_cal_en = tmd2755_proximity_hw_chip_cal_en,
	.proximity_hw_get_offset = tmd2755_proximity_hw_get_offset,
	.proximity_hw_set_fac_offset = tmd2755_proximity_hw_set_fac_offset,
	.proximity_hw_set_offset_limit = tmd2755_proximity_hw_set_offset_limit,
	.proximity_hw_set_offset = tmd2755_proximity_hw_set_offset,
};

static struct lsensor_hw lsensor_hw_tmd2755 = {
	.light_max_threshold = TMD2755_LIGHT_MAX_THRESHOLD,
	.light_calibration_default = TMD2755_LIGHT_CALIBRATION_DEFAULT,
		
	.light_hw_turn_onoff = tmd2755_light_hw_turn_onoff,
	.light_hw_interrupt_onoff = tmd2755_light_hw_interrupt_onoff,
	.light_hw_get_adc = tmd2755_light_hw_get_adc,
	.light_hw_set_hi_threshold = tmd2755_light_hw_set_hi_threshold,
	.light_hw_set_lo_threshold = tmd2755_light_hw_set_lo_threshold,
	.light_hw_set_integration = tmd2755_light_hw_set_integration,
	.light_hw_get_integration = tmd2755_light_hw_get_integration,
	.light_hw_dynamic_check = tmd2755_light_hw_dynamic_check,
	.light_hw_get_current_sensitive = tmd2755_light_hw_get_current_sensitive,
	.light_hw_get_current_IT = tmd2755_light_hw_get_current_IT,
	.light_hw_get_evt_skip_time_ns = tmd2755_light_hw_get_evt_skip_time_ns,
	.light_hw_reset_ALS_dynamic_status = tmd2755_light_hw_reset_ALS_dynamic_status
};


static struct ALSPS_hw ALSPS_hw_tmd2755 = {	
	.vendor = "Ams",
	.module_number = "tmd2755",

	.ALSPS_hw_check_ID = tmd2755_ALSPS_hw_check_ID,
	.ALSPS_hw_init = tmd2755_ALSPS_hw_init,
	.ALSPS_hw_get_interrupt = tmd2755_ALSPS_hw_get_interrupt,
	.ALSPS_hw_show_allreg = tmd2755_ALSPS_hw_show_allreg,
	.ALSPS_hw_set_register = tmd2755_ALSPS_hw_set_register,
	.ALSPS_hw_get_register = tmd2755_ALSPS_hw_get_register,
	.ALSPS_hw_close_power = tmd2755_ALSPS_hw_close_power,

	.mpsensor_hw = &psensor_hw_tmd2755,
	.mlsensor_hw = &lsensor_hw_tmd2755,
};

ALSPS_hw* ALSPS_hw_tmd2755_getHardware(void)
{
	ALSPS_hw* ALSPS_hw_client = NULL;
	ALSPS_hw_client = &ALSPS_hw_tmd2755;
	return ALSPS_hw_client;
}

