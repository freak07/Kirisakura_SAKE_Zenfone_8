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


#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "ams_tmd2755.h"
#include "ams_tmd2755_prox.h"
#include "ams_i2c.h"

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

extern struct tmd2755_chip *g_tmd2755_chip;

/*********************************/

/*****************************************************************
 * Local globals for prox module                                 *
 *****************************************************************/

/*****************************************************************
 *  Utility Calls                                                *
 *****************************************************************/
void tmd2755_read_poffset(struct tmd2755_chip *chip){ 
	ams_i2c_blk_read(chip->client, TMD2755_REG_POFFSET_L, &chip->shadow[TMD2755_REG_POFFSET_L], 2);
	chip->params.poffset = chip->shadow[TMD2755_REG_POFFSET_L];
	dev_dbg(&chip->client->dev, "%*.*s():%*d --> Optical Xtalk calib complete(poffsetl=%d, poffseth=%d, ore=%d, ore_en=%d)\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, chip->params.poffset,
		chip->shadow[TMD2755_REG_POFFSET_H] & TMD2755_MASK_POFFSET_H, chip->shadow[TMD2755_REG_CALIB_OFF] & TMD2755_MASK_ORE,
		(chip->shadow[TMD2755_REG_CALIB_OFF] & TMD2755_MASK_ENABLE_ORE) > TMD2755_SHIFT_ENABLE_ORE);
	
	if (chip->shadow[TMD2755_REG_POFFSET_H] & TMD2755_MASK_POFFSET_H)
		chip->params.poffset *= -1;
}
EXPORT_SYMBOL(tmd2755_read_poffset);

static void tmd2755_write_poffset(struct tmd2755_chip *chip){
	u8 *sh = chip->shadow;
	if(chip->params.poffset >= 0){
		ams_i2c_write(chip->client, sh, TMD2755_REG_POFFSET_H, 0);
		ams_i2c_write(chip->client, sh, TMD2755_REG_POFFSET_L, chip->params.poffset);
		log("wrote offset = %d", chip->params.poffset);
	}else{
		ams_i2c_write(chip->client, sh, TMD2755_REG_POFFSET_H, 1);
		ams_i2c_write(chip->client, sh, TMD2755_REG_POFFSET_L, -chip->params.poffset);
		log("wrote -offset = %d", chip->params.poffset);
	}
	log("set offset = %d", chip->params.poffset);
}

void tmd2755_read_prox(struct tmd2755_chip *chip)
{
	/* Read low and high bytes - 14 bits */
	ams_i2c_blk_read(chip->client, TMD2755_REG_PDATAL, &chip->shadow[TMD2755_REG_PDATAL], sizeof(chip->prox_info.raw));
	if (chip->params.prox_apc == 0) {
		chip->prox_info.raw = le16_to_cpu(*((const __le16 *)&chip->shadow[TMD2755_REG_PDATAL]));
	}else{
		//8bit pdata
		chip->prox_info.raw = chip->shadow[TMD2755_REG_PDATAL];
	}

	tmd2755_read_poffset(chip);
	
	return;
}

static void tmd2755_get_prox(struct tmd2755_chip *chip)
{
	u8 *sh = chip->shadow;

//#if defined(DEBUG_PROX_FEATURE)
	dev_dbg(&chip->client->dev, "%*.*s():%*d --> \t\tcurrent state: %s Raw: %d Min Thresh: %d Max Thresh:%d, offset =%d\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__,
		chip->prox_info.detected ? "<Detect>" : "<Release>", chip->prox_info.raw,
		le16_to_cpu(*((const __le16 *)&sh[TMD2755_REG_PILTL])), le16_to_cpu(*((const __le16 
		*)&sh[TMD2755_REG_PIHTL])), chip->params.poffset);
//#endif
	dbg("threshold %d, %d", chip->params.prox_thresh_min, chip->params.prox_thresh_max);
	dbg("chip->prox_info.detected=%d, chip->prox_info.raw=%d", chip->prox_info.detected, chip->prox_info.raw);
	if (chip->prox_info.raw >= chip->params.prox_thresh_max) {
		chip->prox_info.detected = true;
		log("new state: <Detect>, New Threshold limits = [%d, %d], offset=%d", 
			chip->params.prox_thresh_min, PROX_THRESH_MAX, chip->params.poffset);
		/*If proximity is detected, set the lower limit to find */
		/* proximity release and set the higher limit to avoid */
		/* repeated interrupts. */
		if (chip->params.prox_apc == 0) {
			// apc --> 0 means APC is ENABLED
			ams_i2c_write(chip->client, sh, TMD2755_REG_PILTL,
				(chip->params.prox_thresh_min & TMD2755_MASK_PILTL));
			ams_i2c_write(chip->client, sh, TMD2755_REG_PILTH,
				((chip->params.prox_thresh_min >> 8) & TMD2755_MASK_PILTH));
				ams_i2c_write(chip->client, sh, TMD2755_REG_PIHTL,
				(PROX_THRESH_MAX & TMD2755_MASK_PIHTL));
			ams_i2c_write(chip->client, sh, TMD2755_REG_PIHTH,
				((PROX_THRESH_MAX >> 8) & TMD2755_MASK_PIHTH));
		} else {
			// apc --> 1 means APC is DISABLED                        
			// The PIxTH register is ignored and                      
			// and the PIxTL is compared to the upper 8 bits of pdata 
			ams_i2c_write(chip->client, sh, TMD2755_REG_PILTL,
				((chip->params.prox_thresh_min >> 6) & TMD2755_MASK_PIHTL));
			ams_i2c_write(chip->client, sh, TMD2755_REG_PILTH, 0);
			ams_i2c_write(chip->client, sh, TMD2755_REG_PIHTL,
				((PROX_THRESH_MAX >> 6) & 0xFF));
			ams_i2c_write(chip->client, sh, TMD2755_REG_PIHTH, 0);
		}
	}	else if (chip->prox_info.raw <= chip->params.prox_thresh_min) {
		chip->prox_info.detected = false;
		log("new state: <Release>, offset=%d", chip->params.poffset);
		
		/* If proximity is released, set the higher limit to find         */
		/* proximity and set the lower limit to avoid repeated interrupts */
		/* This is same as the init state                                 */
		tmd2755_init_prox_mode(chip);
	}else{
		log("Lose condition, raw=%d, min=%d, max=%d", chip->prox_info.raw, 
			chip->params.prox_thresh_min, chip->params.prox_thresh_max);
	}
	//Since chip may send two times INT for near case, 
	//clear psensor int flag again after set new threshold
	ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_STATUS, 
					TMD2755_INT_ST_PRX_IRQ, TMD2755_INT_ST_PRX_IRQ);
	return;
}

/*
 *  Value        Meaning
 *    0.     Prox release event
 *    1.      Prox detect event
 *    2.     Prox saturation event
 */
void tmd2755_prox_report_inp_event(struct tmd2755_chip *chip, int value)
{
#ifdef REMOVE_INPUT_DEVICE
	proximity_work(value? ALSPS_INT_PS_CLOSE : ALSPS_INT_PS_AWAY);
#else
	if (chip->prox_idev) {
		input_report_abs(chip->prox_idev, ABS_DISTANCE, value);
		input_sync(chip->prox_idev);
	}
#endif
	return;
}

void tmd2755_process_prox_irq(struct tmd2755_chip *chip)
{
	tmd2755_read_prox(chip);
	tmd2755_get_prox(chip);
	tmd2755_prox_report_inp_event(chip, chip->prox_info.detected ? 1 : 0);

	return;
}

void tmd2755_process_saturation_event(struct tmd2755_chip *chip)
{
	tmd2755_prox_report_inp_event(chip, PROX_SAT_EVENT);

	/* Disable PSAT interrupt otherwise the processor will be over-burdened processing */
	/* saturation interrupts                                                           */
	ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_INTENAB,  TMD2755_PSIEN, 0);

	return;
}


void tmd2755_init_prox_mode(struct tmd2755_chip *chip)
{
	/* Init Low threshold to 0 - Only looking for Detect events after init */
	log("Setting Initial Thresholds = [%d, %d], (low=%d), offset=%d", 	MIN_PROX_THRESHOLD,
		chip->params.prox_thresh_max, chip->params.prox_thresh_min, chip->params.poffset);

	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PILTL, MIN_PROX_THRESHOLD);
	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PILTH, MIN_PROX_THRESHOLD);
	chip->prox_info.raw = 0;

	if (chip->params.prox_apc == PROX_APC_ENABLED) {
		// apc == 0 means APC is ENABLED
		ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTL, (chip->params.prox_thresh_max & 0xFF));
		ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTH, ((chip->params.prox_thresh_max >> 8) & TMD2755_MASK_PIHTH));
	} else {
		// apc == 1 means APC is DISABLED, only 8 MSB of PITH are used - threshold is only 14 bit number (i.e. right shift 6)
		ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTL, ((chip->params.prox_thresh_max >> 6) & 0xFF));
		ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PIHTH, 0);
	}

	
	/* Enable PSAT interrupt during init or after release event */
	ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_INTENAB,  TMD2755_PSIEN, TMD2755_PSIEN);
	return;
}

int tmd2755_offset_calibration(struct tmd2755_chip *chip)
{
	u8 *sh = chip->shadow;
	u8 saveenab, saveint;
	int ret;
	unsigned long wt;
	static int time=1;
	/* clean up the sync mechanism between here and ISR */
	reinit_completion(&(chip->calibration_done));

	/* save PEN state */
	ams_i2c_read(chip->client, TMD2755_REG_ENABLE, &saveenab);

	/* save prox intr state */
	ams_i2c_read(chip->client, TMD2755_REG_INTENAB, &saveint);

	/* turn on power, disable prox */
	ams_i2c_modify(chip->client, sh, TMD2755_REG_ENABLE, TMD2755_PEN | TMD2755_PON, TMD2755_PON);

	/* enable calib intr, disable prox intr */
	ams_i2c_modify(chip->client, sh, TMD2755_REG_INTENAB, TMD2755_PIEN | TMD2755_CIEN, TMD2755_CIEN);

	/*
	 ** Prox Offset calibration
	 **   binsrch_target (15 counts)
	 **   prox averaging (2 reading window)
	 **   prox_auto_offset_adjust
	 */
	ams_i2c_modify(chip->client, sh, TMD2755_REG_CALIBCFG,
			TMD2755_MASK_BINSRCH_TARGET | TMD2755_MASK_PROX_AUTO_OFFSET_ADJUST | TMD2755_MASK_PROX_DATA_AVG,
			(PROX_BINSRCH_TARGET_VALUE << TMD2755_SHIFT_BINSRCH_TARGET) | (PROX_AUTO_OFFSET_ADJ_VAL << TMD2755_SHIFT_PROX_AUTO_OFFSET_ADJUST) |
			(PROX_AVG_VAL << TMD2755_SHIFT_PROX_DATA_AVG));
	/*
	 ** Calibration Offset
	 **   enable offset range extension
	 **   set ORE to PROX_ORE_VAL
	 */
	 /*
	ams_i2c_modify(chip->client, sh, TMD2755_REG_CALIB_OFF,
			TMD2755_MASK_ENABLE_ORE | TMD2755_MASK_ORE,
			(PROX_ORE_EN << TMD2755_SHIFT_ENABLE_ORE) | (PROX_ORE_VAL << TMD2755_SHIFT_ORE));
	*/
	
	/*
	 * CALAVG: Enable to allow HW averaging
	 * ELEC_CALIB: perform both electrical and optical - default
	 * CALPRATE: Enables PRATE during calibration
	 */

	//ams_i2c_modify(chip->client, sh, TMD2755_REG_CALIB, TMD2755_MASK_CALAVG_CALIB | TMD2755_MASK_CALPRATE_CALIB,
	//		(PROX_HWAVG_CAL << TMD2755_SHIFT_CALAVG_CALIB) |  (PROX_PRATE_CAL << TMD2755_SHIFT_CALPRATE_CALIB));

	ams_i2c_modify(chip->client, sh, TMD2755_REG_CALIB, TMD2755_MASK_CALPRATE_CALIB,
			(PROX_PRATE_CAL << TMD2755_SHIFT_CALPRATE_CALIB));

	/* trigger calibration sequence */
	dev_info(&chip->client->dev, "%*.*s():%*d --> offset calibration started (waiting for completion).\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
	chip->amsCalComplete = false;

	chip->in_calib = true;
	ams_i2c_modify(chip->client, sh, TMD2755_REG_CALIB, TMD2755_MASK_START_OFFSET_CALIB, (0x01 << TMD2755_SHIFT_START_OFFSET_CALIB));
	
	/* wait for ISR to signal calibration int complete */
	AMS_MUTEX_UNLOCK(&chip->lock);

	wt = wait_for_completion_interruptible_timeout(&(chip->calibration_done), msecs_to_jiffies(PROX_CALIB_TIMEOUT_MS*(time)));
	
	AMS_MUTEX_LOCK(&chip->lock);
	if (wt <= 0) {
		dev_info(&chip->client->dev, "%*.*s():%*d --> Proximity Calibration timeout occurred: %lu\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__,  wt);
		ret = -1;
		tmd2755_read_poffset(chip);
	} else {
		ret = 0;
		chip->in_calib = false;
		chip->amsCalComplete = true;
		/* get updated prox offset */
		tmd2755_read_poffset(chip);
	}
	/* Restore register enable and interrupt enable */
	ams_i2c_modify(chip->client, sh, TMD2755_REG_ENABLE,  TMD2755_PEN, saveenab);
	ams_i2c_modify(chip->client, sh, TMD2755_REG_INTENAB, TMD2755_PIEN, saveint);
	return ret;
}

int tmd2755_configure_prox_mode(struct tmd2755_chip *chip, u8 state)
{
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	dev_dbg(&chip->client->dev, "%*.*s():%*d --> Configuring Prox Mode %s\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, state ? "ON" : "OFF");

	/* Turning on prox */
	if (state) {
		/* Configure default proximity settings - thresholds */
		tmd2755_init_prox_mode(chip);
		if(chip->cal_en){
		  tmd2755_offset_calibration(chip);
		}else{
			log("use previous offset=%d", chip->params.poffset);
			tmd2755_write_poffset(chip);
		}

		ams_i2c_modify(client, sh, TMD2755_REG_PERS, TMD2755_MASK_PROX_PERS, chip->params.persist.persistance & TMD2755_MASK_PROX_PERS);
		ams_i2c_write(client,  sh, TMD2755_REG_PCFG2, (chip->params.prox_pulse_len  & TMD2755_MASK_PPULSE_LEN_L));
		ams_i2c_modify(client, sh, TMD2755_REG_PCFG1, TMD2755_MASK_REG_PPULSE_LEN_H,
			(chip->params.prox_pulse_len & TMD2755_MASK_DATA_PPULSE_LEN_H) >> TMD2755_SHIFT_PPULSE_LEN_H);

		/* PGAIN2 */
		ams_i2c_modify(client, sh, TMD2755_REG_CFG1,  TMD2755_MASK_PGAIN2, (chip->params.prox_gain2 << TMD2755_SHIFT_PGAIN2));
		/* PGAIN1 */
		ams_i2c_modify(client, sh, TMD2755_REG_PCFG0, TMD2755_MASK_PGAIN1, (chip->params.prox_gain1 << TMD2755_SHIFT_PGAIN1));

		/* PRATE defaults to 0x1F */
		ams_i2c_write(client, sh, TMD2755_REG_PRATE, chip->params.prox_time/* P_TIME_US(INTEGRATION_CYCLE) */);

		/* PWTIME */
		ams_i2c_write(client, sh, TMD2755_REG_PWTIME, chip->params.prox_wtime);

		/* Enable Proximity and Proximity Interrupt */
		/* PWEN is required to be enabled if ALS feature is enabled */
		/* By default, enable PWTIME with Bit 4 in REG_ENABLE */
		//ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_PWEN | TMD2755_PEN | TMD2755_PON, TMD2755_PWEN | TMD2755_PEN | TMD2755_PON);
		
		ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_PWEN | TMD2755_PEN | TMD2755_PON, 0| TMD2755_PEN | TMD2755_PON);
		
		/* Enable Prox interrupt, saturation interrupt and zero detect interrupt */
		ams_i2c_modify(client, sh, TMD2755_REG_INTENAB, TMD2755_PIEN | TMD2755_PSIEN | TMD2755_ZIEN,
			TMD2755_PIEN | TMD2755_PSIEN | TMD2755_ZIEN);

		chip->prox_enable = true;
		chip->prox_info.detected = true;
		chip->prox_info.raw = 0;
	} else {
		chip->params.poffset_last = chip->params.poffset;
		/* Turning off prox */
		/* Disable Proximity feature, and all interrupts associated with prox */
		ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_PEN, 0);
		ams_i2c_modify(client, sh, TMD2755_REG_INTENAB, TMD2755_PROX_INTS, 0);

		/* If nothing else is enabled set PON = 0 */
		if (!(sh[TMD2755_REG_ENABLE] & TMD2755_EN_ALL))
			ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_PON, 0x00);

		chip->prox_enable = false;
	}

	return 0;
}
void tmd2755_offset_recalibration(void){
	struct tmd2755_chip *chip = g_tmd2755_chip;
	AMS_MUTEX_LOCK(&chip->lock); 
	tmd2755_offset_calibration(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);
}


/*****************************************************************
 *   sysfs utility functions                                     *
 *****************************************************************/
static ssize_t tmd2755_prox_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	u8 idx;
	ssize_t ret = 0;

	for (idx = 0; idx < tmd2755_prox_attrs_size; idx++) {
		if (!strncmp(tmd2755_prox_attrs[idx].attr.name, attr->attr.name, strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			switch (idx) {
			case PROX_RAW_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n",  chip->prox_info.raw);
				break;
			case PROX_GAIN1_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_gain1);
				break;
			case PROX_GAIN2_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_gain2);
				break;
			case PROX_OFFSET_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.poffset);
				break;
			case PROX_PERSIST_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.persist.pers.ppers);
				break;
			case PROX_PULSE_LEN_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_pulse_len);
				break;
			case PROX_ENABLE_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->prox_enable);
				break;
			case PROX_TIME_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_time);
				break;
			case PROX_WTIME_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_wtime);
				break;
			case PROX_LOW_THRESH_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_thresh_min);
				break;
			case PROX_HIGH_THRESH_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_thresh_max);
				break;
			case PROX_SATURATION_ATTR:
				/* Determine if prox is in saturation */
				ret = scnprintf(buf, PAGE_SIZE, "%s\n", 
					(chip->in_psat == PROX_NO_SAT ? "Normal_Operation" : 
					(chip->in_psat == PROX_AMBIENT_SAT ? "Ambient_Saturation" : "Reflective_Saturation")));
				break;
			default:
				ret = scnprintf(buf, PAGE_SIZE, "not found\n");
				break;
			}
			AMS_MUTEX_UNLOCK(&chip->lock);
			break;
		}
	}
	return ret;
}

/*****************************************************************
 *   ABI FUNCTIONS - sysfs callbacks                             *
 *****************************************************************/
static ssize_t tmd2755_prox_raw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;

	/* populates the prox_info structure */
	tmd2755_read_prox(chip);

	/* Use scnprintf() */
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_detected_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	u8 val;

	/* if prox INT is enabled, just grab the data flag that is defined in prox_info */
	/* Otherwise read the prox */
	ams_i2c_read(chip->client, TMD2755_REG_INTENAB, &val);
	if (!(val & TMD2755_PIEN)) {
		tmd2755_read_prox(chip);
		tmd2755_get_prox(chip);
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", chip->prox_info.detected ? "detected" : "released");
}


 /* gains */
 /* stage 1 - */
static ssize_t tmd2755_prox_gain1_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_gain1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long gain;
	struct tmd2755_chip *chip = g_tmd2755_chip;
	int rc;

	rc = kstrtoul(buf, BASE_10, &gain);

	if (rc || (gain > PROX_GAIN_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	PROX_OFF();
	rc = ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_PCFG0, TMD2755_MASK_PGAIN1, gain << TMD2755_SHIFT_PGAIN1);
	chip->params.prox_gain1 = gain;
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

/* stage 2 - proximity IR sensor */
static ssize_t tmd2755_prox_gain2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_gain2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long gain;
	struct tmd2755_chip *chip = g_tmd2755_chip;
	int rc;

	rc = kstrtoul(buf, BASE_10, &gain);

	if (rc || (gain > PROX_GAIN_MAX) || (gain == 2))   /* 2 is a reserved value for pgain2 */
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	PROX_OFF();
	rc = ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_CFG1, TMD2755_MASK_PGAIN2, gain << TMD2755_SHIFT_PGAIN2);
	chip->params.prox_gain2 = gain;
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long offset;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &offset);
	log("apply offset = %d, input=%s", chip->params.poffset, buf);

	chip->params.poffset = offset;
	return size;
}

static ssize_t tmd2755_prox_persist_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_persist_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long persist;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &persist);
	if (rc || (persist > PROX_PERSIST_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	PROX_OFF();
	ams_i2c_modify(chip->client, chip->shadow,
		TMD2755_REG_PERS, TMD2755_MASK_PROX_PERS, persist << TMD2755_SHIFT_PROX_PERS);
	chip->params.persist.persistance = chip->shadow[TMD2755_REG_PERS];
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_pulse_len_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_pulse_len_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long pulse_len;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &pulse_len);
	if (rc || (pulse_len < PROX_PULSE_LEN_MIN) || (pulse_len > PROX_PULSE_LEN_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	PROX_OFF();

	/* pulse length is 10 bits: 8 bits in PCFG2 and 2 MSBs in PCFG1 */
	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_PCFG2, (pulse_len  & TMD2755_MASK_PPULSE_LEN_L));
	ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_PCFG1, TMD2755_MASK_REG_PPULSE_LEN_H,
		(pulse_len & TMD2755_MASK_DATA_PPULSE_LEN_H) >> TMD2755_SHIFT_PPULSE_LEN_H);
	chip->params.prox_pulse_len = pulse_len;
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long prate;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &prate);
	if (rc || (prate > PROX_TIME_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	PROX_OFF();
	ams_i2c_write(chip->client, chip->shadow,
		TMD2755_REG_PRATE, prate);
	chip->params.prox_time = chip->shadow[TMD2755_REG_PRATE];
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_wtime_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_wtime_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long pwtime;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &pwtime);

	AMS_MUTEX_LOCK(&chip->lock);
	PROX_OFF();
	ams_i2c_write(chip->client, chip->shadow,
		TMD2755_REG_PWTIME, pwtime);
	chip->params.prox_wtime = chip->shadow[TMD2755_REG_PWTIME];
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long enable;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, 10, &enable);
	/* if error on conversion, value greater than 1(true) */
	if (rc || (enable > 1))
		return -EINVAL;
	/* if its the same, no need to do any work. */
	if (enable == chip->prox_enable)
		return size;

	AMS_MUTEX_LOCK(&chip->lock);

	if (enable)
		tmd2755_configure_prox_mode(chip, TMD2755_FEATURE_ON);
	else /* disable */
		tmd2755_configure_prox_mode(chip, TMD2755_FEATURE_OFF);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_low_thresh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_low_thresh_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long low_thresh;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &low_thresh);
	if (rc || (low_thresh > PROX_THRESH_MAX))
		return -EINVAL;

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
	
	log("APC disable, thresh_min=%d", chip->params.prox_thresh_min);
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_high_thresh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

static ssize_t tmd2755_prox_high_thresh_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long high_thresh;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &high_thresh);
	if (rc || (high_thresh > PROX_THRESH_MAX))
		return -EINVAL;

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
	log("APC disable, thresh_max=%d", chip->params.prox_thresh_max);
	PROX_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_prox_regs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;

	u8 rEnable  = 0, rPrate  = 0, rPiltl  = 0, rPilth   = 0;
	u8 rPihtl   = 0, rPihth  = 0, rPers   = 0, rCfg0    = 0;
	u8 rPcfg0   = 0, rCfg1   = 0, rStatus = 0, rPcfg1   = 0;
	u8 rCfg8    = 0, rCfg3   = 0, rCfg6   = 0, rIntenab = 0;
	u8 rPwtime  = 0, rCalib  = 0, rCalOff = 0, rCalcfg  = 0;
	u8 rPcfg2   = 0, rCalstat = 0;

	ams_i2c_read(chip->client, TMD2755_REG_ENABLE,  &rEnable);
	ams_i2c_read(chip->client, TMD2755_REG_PRATE,   &rPrate);
	ams_i2c_read(chip->client, TMD2755_REG_PILTL,   &rPiltl);
	ams_i2c_read(chip->client, TMD2755_REG_PILTH,   &rPilth);
	ams_i2c_read(chip->client, TMD2755_REG_PIHTL,   &rPihtl);
	ams_i2c_read(chip->client, TMD2755_REG_PIHTH,   &rPihth);
	ams_i2c_read(chip->client, TMD2755_REG_PERS,    &rPers);

	ams_i2c_read(chip->client, TMD2755_REG_CFG0,    &rCfg0);
	ams_i2c_read(chip->client, TMD2755_REG_CFG1,    &rCfg1);
	ams_i2c_read(chip->client, TMD2755_REG_CFG8,    &rCfg8);
	ams_i2c_read(chip->client, TMD2755_REG_CFG3,    &rCfg3);
	ams_i2c_read(chip->client, TMD2755_REG_CFG6,    &rCfg6);

	ams_i2c_read(chip->client, TMD2755_REG_PCFG0,   &rPcfg0);
	ams_i2c_read(chip->client, TMD2755_REG_PCFG1,   &rPcfg1);
	ams_i2c_read(chip->client, TMD2755_REG_PCFG2,   &rPcfg2);

	ams_i2c_read(chip->client, TMD2755_REG_STATUS,  &rStatus);
	ams_i2c_read(chip->client, TMD2755_REG_PWTIME,  &rPwtime);

	ams_i2c_read(chip->client, TMD2755_REG_INTENAB, &rIntenab);

	/* Calibration Registers */
	ams_i2c_read(chip->client, TMD2755_REG_CALIB,     &rCalib);
	ams_i2c_read(chip->client, TMD2755_REG_CALIB_OFF, &rCalOff);
	ams_i2c_read(chip->client, TMD2755_REG_CALIBCFG,  &rCalcfg);
	ams_i2c_read(chip->client, TMD2755_REG_CALIBSTAT, &rCalstat);


	return scnprintf(buf, PAGE_SIZE,
		"ENABLE   =   0x%02X\nPRATE    =   0x%02X\nPILT     =   0x%04X\nPIHT     =   0x%04X\n"
		"PERS     =   0x%02X\nPCFG0    =   0x%02X\nPCFG1    =   0x%02X\nPCFG2    =   0x%02X\n"
		"CFG0     =   0x%02X\nCFG1     =   0x%02X\nCFG8     =   0x%02X\nCFG3     =   0x%02X\n"
		"CFG6     =   0x%02X\nSTATUS   =   0x%02X\nPWTIME   =   0x%02X\nINTENAB  =   0x%02X\n"
		"CALIB    =   0x%02X\nCAL_OFF  =   0x%02X\nCAL_CFG  =   0x%02X\nCAL_STAT =   0x%02X\n"
		"%s\n",
		rEnable, rPrate, (rPilth << 8) | rPiltl, (rPihth << 8) | rPihtl, rPers,
		rPcfg0, rPcfg1, rPcfg2,
		rCfg0, rCfg1, rCfg8, rCfg3, rCfg6,
		rStatus, rPwtime, rIntenab,
		rCalib, rCalOff, rCalcfg, rCalstat,
		chip->prox_info.detected ? "Prox Detect" : "Prox Release");
}

static ssize_t tmd2755_prox_sat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_prox_show(dev, attr, buf);
}

/**********************************************************************************************************************
 *  END OF SYSFS FUNCTIONS                                                                                            *
 **********************************************************************************************************************/

/**************************************************************
 *                   Global Sys Fs definitions                *
 **************************************************************/
extern ssize_t tmd2755_registers_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t tmd2755_registers_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
struct device_attribute tmd2755_prox_attrs[] = {
	__ATTR(prox_raw,         0444, tmd2755_prox_raw_show,         NULL),
	__ATTR(prox_detect,      0444, tmd2755_prox_detected_show,    NULL),
	__ATTR(prox_gain1,       0644, tmd2755_prox_gain1_show,       tmd2755_prox_gain1_store),
	__ATTR(prox_gain2,       0644, tmd2755_prox_gain2_show,       tmd2755_prox_gain2_store),
	__ATTR(prox_offset,      0644, tmd2755_prox_offset_show,      tmd2755_prox_offset_store),
	__ATTR(prox_persist,     0644, tmd2755_prox_persist_show,     tmd2755_prox_persist_store),
	__ATTR(prox_pulse_len,   0644, tmd2755_prox_pulse_len_show,   tmd2755_prox_pulse_len_store),
	__ATTR(prox_enable,      0644, tmd2755_prox_enable_show,      tmd2755_prox_enable_store),
	__ATTR(prox_time,        0644, tmd2755_prox_time_show,        tmd2755_prox_time_store),
	__ATTR(prox_wtime,        0644, tmd2755_prox_wtime_show,        	tmd2755_prox_wtime_store),
	__ATTR(prox_low_thresh,  0644, tmd2755_prox_low_thresh_show,  tmd2755_prox_low_thresh_store),
	__ATTR(prox_high_thresh, 0644, tmd2755_prox_high_thresh_show, tmd2755_prox_high_thresh_store),
	__ATTR(prox_regs,        0444, tmd2755_prox_regs_show,        NULL),
	__ATTR(prox_sat,         0444, tmd2755_prox_sat_show,         NULL),
	__ATTR(regs, 0644, tmd2755_registers_show, tmd2755_registers_store),
};

int tmd2755_prox_attrs_size = ARRAY_SIZE(tmd2755_prox_attrs);

