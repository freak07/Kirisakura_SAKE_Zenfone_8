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
#include "ams_tmd2755_als.h"
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
 * Local globals for als module                                  *
 *****************************************************************/
static u8 const restorable_als_regs[] = {
	TMD2755_REG_ATIME,
	TMD2755_REG_AWTIME,
	TMD2755_REG_PERS,
	TMD2755_REG_CFG0,
	TMD2755_REG_CFG1,
};


/*****************************************************************
 *  Local Functions                                              *
 *****************************************************************/
static int tmd2755_convert_again(int reg_value)
{
	int ret_val = ALS_GAIN_128;
	
	switch (reg_value)
	{
		case ALS_GAIN_REG_VAL_16:
		{
			ret_val = ALS_GAIN_16; 
			break;
		}
		case ALS_GAIN_REG_VAL_128:
		{
			ret_val = ALS_GAIN_128;
			break;
		}
		case ALS_GAIN_REG_VAL_1024:
		{
			ret_val = ALS_GAIN_1024;
			break;
		}
		default:
		{
			break;
		}
	}

	return(ret_val);
}

static int tmd2755_flush_als_regs(struct tmd2755_chip *chip)
{
	unsigned int i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_als_regs); i++) {
		reg = restorable_als_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%*.*s():%*d --> err on reg 0x%02X\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, reg);
			break;
		}
	}
	return rc;
}

/*
 * Refer to Design Notebook 40 for a better understanding of CPL.
 */
static void tmd2755_calc_counts_per_lux(struct tmd2755_chip *chip)
{
	u64 cpl;
	u32 sat;
	u8  atime;
	u16 again;

	atime = chip->shadow[TMD2755_REG_ATIME];

	cpl = (u64) atime + 1; /* add one because atime reg is base 0: 0 = 1 integration time */
	cpl *= INTEGRATION_CYCLE; /* using microseconds cancels out the scale factor of the coeffs */
	
	again = tmd2755_convert_again(chip->shadow[TMD2755_REG_CFG1] & 	TMD2755_MASK_AGAIN);

	cpl *= again;

	/* Optimization if coefficients are scaled by 1000, adjust accordingly */
	if (chip->params.coef_scale != ALS_COEFF_SCALE_FACTOR) {
		/* Possible overflow if coefficients are scaled > 1000 */
		cpl *= chip->params.coef_scale; /* rgb coeff scaling factor */
		/* do_div returns the quotient in the divident */
		do_div(cpl, (chip->params.dgf * ALS_COEFF_SCALE_FACTOR));
	} else {
		do_div(cpl, chip->params.dgf);
	}


	/* min of max_als for max amount based on atime - see datasheet for atime */
	/* each 2.78 ms, increase max count by 1024  - left shift of 10 multiplies by */
	/* 1024 */
	sat = min_t(u32, TMD2755_MAX_ALS_VALUE, (u32) (((atime + 1) << TMD2755_SHIFT_MULT_BY_1024) - 1));

	chip->als_info.counts_per_lux = (u32) cpl;
	/* Set saturation at 90% of max count*/
	chip->als_info.saturation = TENTH_FRACTION_OF_VAL(sat, TMD2755_SATURATION_THRESHOLD);

	dev_dbg(&chip->client->dev, "%*.*s():%*d --> Calculating cpl = %llu, count_per_lux=%u, saturation = %d (%d0%% of %d)\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, cpl, 
		chip->als_info.counts_per_lux, chip->als_info.saturation,
		TMD2755_SATURATION_THRESHOLD, sat);

	return;
}

static int tmd2755_set_als_gain(struct tmd2755_chip *chip, u8 gain)
{
	int rc = 0;
	 u8 saved_enable;

	/* Turn off ALS, so that new ALS gain value will take effect at start of
	 * new integration cycle.
	 * New ALS gain value will then be used in next lux calculation.
	 */
	ams_i2c_read(chip->client, TMD2755_REG_ENABLE, &saved_enable);
	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_ENABLE, 0); /* turn off everything */
	rc = ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_CFG1, TMD2755_MASK_AGAIN, gain);
	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_ENABLE, saved_enable);

	if (rc >= 0) {
		chip->params.als_gain = chip->shadow[TMD2755_REG_CFG1] & TMD2755_MASK_AGAIN;
		dev_info(&chip->client->dev, "%*.*s():%*d --> New ALS Gain set %d (%dx)\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, gain,
			tmd2755_convert_again(gain));
	}
	return rc;
}

static void tmd2755_get_rawdata(struct tmd2755_chip *chip)
{
	u8 *sh = chip->shadow;

	/* extract raw channel data */
	chip->als_info.ch0_raw = le16_to_cpup((const __le16 *)&sh[TMD2755_REG_ALSL]);
	chip->als_info.ch1_raw = le16_to_cpup((const __le16 *)&sh[TMD2755_REG_IRL]);

	return;
}

static void tmd2755_inc_gain(struct tmd2755_chip *chip)
{
	int rc;
	u8 gain = (chip->shadow[TMD2755_REG_CFG1] & TMD2755_MASK_AGAIN);

	if (gain == ALS_GAIN_REG_VAL_1024) {
		dev_dbg(&chip->client->dev, "%*.*s():%*d -->\tCannot Increment ALS Gain: already at maximum (%d x).\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, ALS_GAIN_1024);
		return; /* already at highest gain setting */
	}

	/* if gain is set to 16x, move it to 128x.	If it is 128x, move it to 1024 */
	gain =	((gain == ALS_GAIN_REG_VAL_16) ? ALS_GAIN_REG_VAL_128 : ALS_GAIN_REG_VAL_1024);

	rc = tmd2755_set_als_gain(chip, gain);
	if (rc == 0)
		tmd2755_calc_counts_per_lux(chip);

	return;
}

static void tmd2755_dec_gain(struct tmd2755_chip *chip)
{
	int rc;
	u8 gain = (chip->shadow[TMD2755_REG_CFG1] & TMD2755_MASK_AGAIN);

	if (gain == ALS_GAIN_REG_VAL_16) {
		dev_dbg(&chip->client->dev, "%*.*s():%*d --> \tCannot Decrement ALS Gain: already at minimum (%d x).\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, ALS_GAIN_16);
		return; /* already at lowest gain */
	}

	gain = ((gain == ALS_GAIN_REG_VAL_1024) ? ALS_GAIN_REG_VAL_128 : ALS_GAIN_REG_VAL_16);

	rc = tmd2755_set_als_gain(chip, gain);
	if (rc == 0)
		tmd2755_calc_counts_per_lux(chip);

	return;
}

static int tmd2755_max_als_value(struct tmd2755_chip *chip)
{
	int val;

	val = chip->shadow[TMD2755_REG_ATIME];
	if (val > ALS_ATIME_FOR_MAX_CNT) /* after a certain atime, the max als count stays the same */
		val = ALS_FULL_SCALE_CNT;
	else
		val = ((val * ALS_ATIME_INCREMENT_CNT) + ALS_ATIME_INCREMENT_CNT-1); /* Initial atime only causes (1024 -1) full scale */

	return val;
}

static int tmd2755_update_als_threshold(struct tmd2755_chip *chip, enum tmd2755_enable_state on_enable)
{
	s32 ret = 0;
	u16 deltaP = chip->params.als_deltaP; /* integer percentage to generate thresholds */
	u16 from, to, cur;
	u16 saturation = chip->als_info.saturation;
	static u16 last_from = 0, last_to = 0;
	cur = chip->als_info.ch0_raw;

	if (on_enable == TMD2755_ENABLE_ON) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > (saturation / 2) ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		to = cur + deltaP;
		/* prevent high threshold overflow */
		if(to > TMD2755_CH0_MAXIMUM){
			to = TMD2755_CH0_MAXIMUM;
		}
	}
	
	*((__le16 *) &chip->shadow[TMD2755_REG_AILTL]) = cpu_to_le16(from);
	*((__le16 *) &chip->shadow[TMD2755_REG_AIHTL]) = cpu_to_le16(to);

	if(from != last_from || to != last_to || true == g_tmd2755_status_param.log_first_evt){
		last_from = from;
		last_to = to;
		dev_dbg(&chip->client->dev, "%*.*s():%*d --> Low Thresh: %d  High Thresh: : %d  Current Ch0: %d  deltaP: %d  Sat Level: %d\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__,
				from, to, cur, deltaP, saturation);
		g_tmd2755_status_param.log_first_evt = false;
	}

	ret = ams_i2c_reg_blk_write(chip->client, TMD2755_REG_AILTL, &chip->shadow[TMD2755_REG_AILTL],
		(TMD2755_REG_AIHTH - TMD2755_REG_AILTL) + 1);

	return (ret < 0) ? ret : 0;
}

int tmd2755_get_lux(struct tmd2755_chip *chip)
{
	s64 _lux1, _lux2, _lux3;
	s32 __lux1, __lux2, __lux3;
	//long lux1, lux2, lux3;
	s32 lux = 0;
	//s64 sf;
	//s32 coefa = chip->params.ch0_coef0;
	//s32 coefc = chip->params.ch0_coef1;
	//s32 coefb = chip->params.ch1_coef0;
	//s32 coefd = chip->params.ch1_coef1;
	long ch0 = (long)chip->als_info.ch0_raw;
	long ch1 = (long)chip->als_info.ch1_raw;
	long coefa = 678;  //should / 1000
	long coefb = 1050; //should /1000
	long coefc = 4;

	long DFA = 3850;
	long DFB = 18000;
	long DFC = 16500;
	long div_val = 1000;
	long diff1 = 1500; //should /10000
	long diff2 = 9900; //should /10000
	long ch_temp = 0; // ch0/ch1
	int low_thres;
	s64 sf = 0;
	long ch_data_limit = 0;
	static long ch0_last = 0;

	// use time in ms get scaling factor 
	tmd2755_calc_counts_per_lux(chip);
	if (!chip->als_info.counts_per_lux) {
		dev_info(&chip->client->dev, "%*.*s():%*d --> CPL = 0... Setting to 1\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		chip->als_info.counts_per_lux = 1;
	}
	dbg("count_per_lux=%u", chip->als_info.counts_per_lux);
	sf = (s64)chip->als_info.counts_per_lux;  // atime_ms*again/DGF 
	sf = div64_s64(sf, 1000);  // atime_ms*again/DGF 
	dbg("sf=%lld, count_per_lux=%u", sf, chip->als_info.counts_per_lux);
	dbg("DFA=%ld, DFB=%ld, DFC=%ld, coefa=%ld, coefb=%ld, coefc=%ld", DFA, DFB, DFC, coefa, coefb, coefc);
	dbg("ch0=%ld, ch1=%ld", ch0, ch1);
	dbg("DFA*ch0=%ld, DFA*coefa*ch1=%ld", DFA*ch0, DFA*coefa*ch1);
	dbg("DFB*ch0=%ld (/1000, DFB*coefb*ch1=%ld (/1000000", DFB*ch0, DFB*coefb*ch1);
	dbg("DFC*ch0=%ld, DFC*coefc*ch1=%ld", DFC*ch0, DFC*coefc*ch1);
	_lux1 = (s64)DFA*ch0-(DFA*coefa*ch1)/div_val;
	_lux2 = (s64)(DFB*ch0)-(DFB*coefb*ch1)/div_val;
	_lux3 = (s64)DFC*ch0-DFC*coefc*ch1;

	__lux1 = (s32)div64_s64(_lux1, sf);
	__lux2 = (s32)div64_s64(_lux2, sf);
	__lux3 = (s32)div64_s64(_lux3, sf);

	ch_temp = (ch1*10000)/ch0;
	if(diff2 < ch_temp){
		lux = __lux1;
	}else if((diff2 >= ch_temp) && (diff1 < ch_temp)){
		lux = __lux2;
	}else{
		lux = __lux3;
	}
	
	if(ch0 > 1000){
		ch_data_limit = ch0_last / 10;
	}else{
		ch_data_limit = ch0_last * 98 / 100;
	}

	if(ch0 != ch0_last){
		if((ch0_last >= (ch0+ch_data_limit)) || (ch0_last <= (ch0-ch_data_limit)) || g_tmd2755_status_param.log_first_evt == true){
			log("lux=%ld, ch0=%u, ch1=%u, ch0_last=%ld, (ch1*10000/ch0)=%ld, sf=%lld", 
			lux, ch0, ch1, ch0_last, ch_temp, sf);
		}
        }
	ch0_last = ch0;
	
	if (lux < 0) {
		dev_info(&chip->client->dev, "%*.*s():%*d --> lux < 0,  Use previous value.\n",
		MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
		return chip->als_info.lux; /* use previous value */
	} else {
		lux = min(ALS_MAX_LUX_VAL, max(0, lux));
	}

	/* Determine threshold for switching gains */
	low_thres = tmd2755_max_als_value(chip) / TMD2755_THRESHOLD_PERCENT;

	/* clamp counts to prevent flip-flopping between gains */
	low_thres = clamp(low_thres, 0, TMD2755_AUTO_GAIN_CLAMP_CNT);

	/* There is a case where one channel can saturate and the other channel can be below the clamping value.  */
	/* This algorithm does not gracefully handle it. */
	if (!chip->params.als_auto_gain) { /* auto gain is off */
		if ((chip->als_info.ch0_raw <= TMD2755_MIN_ALS_VALUE) || (chip->als_info.ch1_raw <= TMD2755_MIN_ALS_VALUE)) {
			dev_info(&chip->client->dev, "%*.*s():%*d --> Darkness entered: Channel count less than minimum ([%d || %d] <= %d).\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__,
				chip->als_info.ch0_raw, chip->als_info.ch1_raw, TMD2755_MIN_ALS_VALUE);
		} else if ((chip->als_info.ch0_raw >= chip->als_info.saturation) || (chip->als_info.ch1_raw >= chip->als_info.saturation)) {
			dev_info(&chip->client->dev, "%*.*s():%*d -->  Saturation occurred: Channel count exceeds maximum ([%d || %d] >= %d).\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__,
				chip->als_info.ch0_raw, chip->als_info.ch1_raw, chip->als_info.saturation);
		}
	} else {
		if (chip->als_info.ch0_raw < low_thres || chip->als_info.ch1_raw < low_thres) {
			if((chip->shadow[TMD2755_REG_CFG1] & TMD2755_MASK_AGAIN) != ALS_GAIN_REG_VAL_1024){
				dev_info(&chip->client->dev, "%*.*s():%*d --> Autogain Incrementing, ch0:%d, ch1:%d, low_thres=%d\n",
					MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, 
					__LINE__, chip->als_info.ch0_raw, chip->als_info.ch1_raw, low_thres);
				tmd2755_inc_gain(chip);
			}
			tmd2755_flush_als_regs(chip);
		} else if ((chip->als_info.ch0_raw >= chip->als_info.saturation) || (chip->als_info.ch1_raw >= chip->als_info.saturation) || chip->in_asat) {
			if ((chip->shadow[TMD2755_REG_CFG1] & TMD2755_MASK_AGAIN) != ALS_GAIN_REG_VAL_16) {
				dev_info(&chip->client->dev, "%*.*s():%*d --> Autogain Decrementing, sat=%d, in_asat=%d \n",
					MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, 
					__LINE__, chip->als_info.saturation, chip->in_asat);
				tmd2755_dec_gain(chip);
			}
			tmd2755_flush_als_regs(chip);
		}
	}

	chip->als_info.lux = lux;
	dev_dbg(&chip->client->dev, "%*.*s():%*d --> Lux Calculation: %d, lux=%d\n",
				MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__, 
				lux, chip->als_info.lux);

	return 0;
}

/*****************************************************************
 *  Utility Calls                                                *
 *****************************************************************/
int tmd2755_configure_als_mode(struct tmd2755_chip *chip, u8 state)
{
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	if (state) { /* Enable ALS */
		g_tmd2755_status_param.log_first_evt = true;
		dev_dbg(&chip->client->dev, "%*.*s():%*d --> Enabling and Configuring ALS\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);

		chip->shadow[TMD2755_REG_ATIME] = chip->params.als_time;
		tmd2755_calc_counts_per_lux(chip);

		/* only adjust als peristance */
		chip->shadow[TMD2755_REG_PERS] &= (~TMD2755_MASK_ALS_PERS);
		chip->shadow[TMD2755_REG_PERS] |= chip->params.persist.pers.apers;

		tmd2755_flush_als_regs(chip);

		/* AWTIME */
		ams_i2c_write(client, sh, TMD2755_REG_AWTIME, chip->params.als_wtime);

		/* Enable ALS interrupt */
		ams_i2c_modify(client, sh, TMD2755_REG_INTENAB, TMD2755_AIEN, TMD2755_AIEN);
		/* When enabling ALS - AEN in Register 0x80, PWEN must be set to 1 if prox is enabled - See datasheet */
		/* As of release 1.9, PWEN is active when proximity is turned oa - see ams_tmd2755_prox.c filen */
		/* Also Power On */
		ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_AEN | TMD2755_PON,
			TMD2755_AEN | TMD2755_PON);
		chip->als_enable = true;
		
		/* Enable PWTIME */
		if(chip->prox_enable == true){
			log("pon, enable PWEN");
			ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_PWEN, TMD2755_PWEN);
		}
	} else  { /* Disable ALS */
		dev_info(&chip->client->dev, "%*.*s():%*d --> Disable ALS\n",
			MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);

		ams_i2c_modify(client, sh, TMD2755_REG_INTENAB, TMD2755_AIEN, 0);
		/* Cannot turn off PWEN, in case prox feature is still using it */
		ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_AEN, 0);
		chip->als_enable = false;
		
		*((__le16 *) &chip->shadow[TMD2755_REG_AILTL]) = cpu_to_le16(0);
		*((__le16 *) &chip->shadow[TMD2755_REG_AIHTL]) = cpu_to_le16(0);
		ams_i2c_reg_blk_write(chip->client, TMD2755_REG_AILTL, &chip->shadow[TMD2755_REG_AILTL],
			(TMD2755_REG_AIHTH - TMD2755_REG_AILTL) + 1);
		log("set hi/low threshold to 0");

		/* Close PWTIME */
		ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_PWEN, 0);

		/* If Prox is not on, turn off chip */
		if (!(sh[TMD2755_REG_ENABLE] & TMD2755_EN_ALL)){
			ams_i2c_modify(client, sh, TMD2755_REG_ENABLE, TMD2755_PON, 0);
		}

	}

	return 0;
}

int tmd2755_read_als(struct tmd2755_chip *chip)
{
	int ret;

	ret = ams_i2c_blk_read(chip->client, TMD2755_REG_ALSL, &chip->shadow[TMD2755_REG_ALSL], ALS_NUM_CH * ALS_CH_SIZE);

	if (ret >= 0) {
		tmd2755_get_rawdata(chip);
		ret = 0;
	}

	return ret;
}

extern int light_get_lux(int adc);
void tmd2755_report_als(struct tmd2755_chip *chip)
{
	int lux;
	int rc;
	static int last_lux = 0;
#ifdef REMOVE_INPUT_DEVICE
	rc = tmd2755_get_lux(chip); /* always returns 0 */
	if (!rc) {
		lux = chip->als_info.lux;
		lux = light_get_lux(chip->als_info.lux);
		lsensor_report_lux(lux);
		if(g_tmd2755_status_param.log_first_evt == true){
			log("ALS lux First= %d (orig lux=%d)", lux, chip->als_info.lux);
		}else if(lux != last_lux){
			log("ALS lux = %d (orig lux=%d), last_lux=%d", lux, chip->als_info.lux, last_lux);
		}
		last_lux = lux;
		tmd2755_update_als_threshold(chip, TMD2755_ENABLE_OFF);
	} else {
		tmd2755_update_als_threshold(chip, TMD2755_ENABLE_ON);
	}
	
#else
	if (chip->als_idev) {
		rc = tmd2755_get_lux(chip); /* always returns 0 */
		if (!rc) {
			lux = chip->als_info.lux;
			input_report_abs(chip->als_idev, ABS_MISC, lux);
			input_sync(chip->als_idev);
			dbg("ALS lux = %d", lux);
			tmd2755_update_als_threshold(chip, TMD2755_ENABLE_OFF);
		} else
			tmd2755_update_als_threshold(chip, TMD2755_ENABLE_ON);
	}
#endif
}


/*****************************************************************
 *   sysfs utility functions                                     *
 *****************************************************************/
static ssize_t tmd2755_als_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	u8 idx, temp;
	int val;
	ssize_t ret = 0;
	if(chip == NULL){
		log("chip null");
		return ret;
	}else{
		log("chip on");
	}
	
	for (idx = 0; idx < tmd2755_als_attrs_size; idx++) {
		if (!strncmp(tmd2755_als_attrs[idx].attr.name, attr->attr.name, strlen(attr->attr.name))) {
			if(NULL == chip){
				dev_info(&chip->client->dev, "%*.*s():%*d --> lock = NULL\n",
					MIN_KERNEL_LOG_LEN, MAX_KERNEL_LOG_LEN, __func__, LINE_NUM_KERNEL_LOG_LEN, __LINE__);
				return ret;
			}else{
				AMS_MUTEX_LOCK(&chip->lock);
			}
			switch (idx) {
			case ALS_ATIME_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d (~%d us)\n",  chip->shadow[TMD2755_REG_ATIME],
					(chip->shadow[TMD2755_REG_ATIME]+1)*INTEGRATION_CYCLE);
				break;
			case ALS_AWTIME_ATTR:
				temp = (chip->shadow[TMD2755_REG_CFG0] & TMD2755_MASK_AWLONG) >> TMD2755_SHIFT_AWLONG;
				/* if long bit is set, multiply by 12 */
				val = (temp ? AWTIME_LONG_FACTOR : 1);
				ret = scnprintf(buf, PAGE_SIZE, "%d (~%d us %s)\n", chip->shadow[TMD2755_REG_AWTIME],
					(chip->shadow[TMD2755_REG_AWTIME]+1)*val*INTEGRATION_CYCLE,
					val == AWTIME_LONG_FACTOR ? "<awlong bit set:12x>" : "<>");
				break;
			case ALS_AWLONG_ATTR:
				temp = (chip->shadow[TMD2755_REG_CFG0] & TMD2755_MASK_AWLONG) >> TMD2755_SHIFT_AWLONG;
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", temp);
				break;
			case ALS_LUX_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->als_info.lux);
				break;
			case ALS_GAIN_ATTR:
				if(NULL == chip){
					log("chip null1");
					return ret;
				}
				
				log("get als_gain params, 0x%x", chip->params.als_gain);
				ret = scnprintf(buf, PAGE_SIZE, "%d (%dx)\n", 	chip->params.als_gain, tmd2755_convert_again(chip->params.als_gain));
				break;
			case ALS_CPL_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d (atime_us*again/DGF)\n", chip->als_info.counts_per_lux);
				break;
			case ALS_CH0_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->als_info.ch0_raw);
				break;
			case ALS_CH1_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->als_info.ch1_raw);
				break;
			case ALS_THRESH_DELTAP_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->params.als_deltaP);
				break;
			case ALS_AUTO_GAIN_ATTR:
				ret = snprintf(buf, PAGE_SIZE, "%d\n", chip->params.als_auto_gain);
				break;
			case ALS_LUX_COEFF_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "DGF = %d\nSCALE FACTOR = %d\nCOEFF_A = %d/1000\nCOEFF_B = %d\nCOEFF_C = %d/1000\nCOEFF_D = %d\n",
					chip->params.dgf, chip->params.coef_scale, chip->params.ch0_coef0, chip->params.ch1_coef0,
					chip->params.ch0_coef1, chip->params.ch1_coef1);
				break;
			case ALS_ENABLE_ATTR:
				ret = scnprintf(buf, PAGE_SIZE, "%d\n", chip->als_enable);
				break;
			case ALS_PERSIST_ATTR:  /* see datasheet for pattern */
				if (chip->params.persist.pers.apers >= 4)
					val = 10 + ((chip->params.persist.pers.apers - 5) * 5);
				else
					val = chip->params.persist.pers.apers;

				ret = scnprintf(buf, PAGE_SIZE, "%d (%d consecutive OOR)\n", chip->params.persist.pers.apers, val);
				break;
			case ALS_SATURATION_ATTR:
				/* Determine if ALS is in saturation */
				ret = scnprintf(buf, PAGE_SIZE, "%s\n", (chip->in_asat ? "ALS_Saturation" : "Normal_Operation"));
				break;
			case ALS_ADC_ATTR:
				tmd2755_get_lux(chip);
				ret = scnprintf(buf, PAGE_SIZE, "lux: %d, ch0: %d, ch1:%d\n", chip->als_info.lux,
					chip->als_info.ch0_raw, chip->als_info.ch1_raw);
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
 *   sysfs utility functions                                     *
 *****************************************************************/
static ssize_t tmd2755_als_atime_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_atime_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long atime;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &atime);
	if (rc || (atime > ALS_TIME_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();
	log("atime value = %lu", atime);
	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_ATIME, atime);
	chip->params.als_time = chip->shadow[TMD2755_REG_ATIME];
	tmd2755_calc_counts_per_lux(chip);

	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_als_wtime_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_wtime_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long awtime;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &awtime);
	if (rc || (awtime > ALS_WTIME_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();

	ams_i2c_write(chip->client, chip->shadow, TMD2755_REG_AWTIME, awtime);
	chip->params.als_wtime = chip->shadow[TMD2755_REG_AWTIME];

	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_als_wlong_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_wlong_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long awlong;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &awlong);
	if (rc || (awlong > ALS_WLONG_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();

	ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_CFG0, TMD2755_MASK_AWLONG, (u8)(awlong << TMD2755_SHIFT_AWLONG));

	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;

	AMS_MUTEX_LOCK(&chip->lock);

	tmd2755_read_als(chip);
	tmd2755_get_lux(chip);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_gain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long again;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &again);
	if (rc || ((again != ALS_GAIN_MIN) && (again != ALS_GAIN_MAX)  && (again != ALS_GAIN_DEFAULT)))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();

	ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_CFG1, TMD2755_MASK_AGAIN, (u8)(again << TMD2755_SHIFT_AGAIN));
	chip->params.als_gain = (u8)again;

	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_als_cpl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_ch0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_ch1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_deltaP_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_deltaP_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long deltaP;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &deltaP);
	if (rc || (deltaP > ALS_DELTA_P_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();
	chip->params.als_deltaP = deltaP;
	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);
	return size;
}

static ssize_t tmd2755_auto_gain_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}
static ssize_t tmd2755_auto_gain_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long auto_gain;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, 10, &auto_gain);
	/* if error on conversion, value greater than 1(true) */
	if (rc || (auto_gain > 1))
		return -EINVAL;
	/* if its the same, no need to do any work. */
	if (auto_gain == chip->params.als_auto_gain)
		return size;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();
	chip->params.als_auto_gain = auto_gain;
	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_lux_coef_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}
static ssize_t tmd2755_lux_coef_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	u32 dgf, ch0_coeff_0, ch1_coeff_0, ch0_coeff_1, ch1_coeff_1, coeff_scale;

	if (sscanf(buf, "%10d,%10d,%10d,%10d,%10d,%10d", &dgf, &coeff_scale, &ch0_coeff_0, &ch1_coeff_0, &ch0_coeff_1, &ch1_coeff_1) != 6)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();
	chip->params.dgf         = dgf;
	chip->params.coef_scale  = coeff_scale;
	chip->params.ch0_coef0   = ch0_coeff_0;
	chip->params.ch0_coef1   = ch0_coeff_1;
	chip->params.ch1_coef0   = ch1_coeff_0;
	chip->params.ch1_coef1   = ch1_coeff_1;
	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long enable;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, 10, &enable);
	/* if error on conversion, or value greater than 1(true) */
	if (rc || (enable > 1))
		return -EINVAL;
	/* if its the same, no need to do any work. */
	if (enable == chip->als_enable)
		return size;

	AMS_MUTEX_LOCK(&chip->lock);

	if (enable)
		tmd2755_configure_als_mode(chip, TMD2755_FEATURE_ON);
	else /* disable */
		tmd2755_configure_als_mode(chip, TMD2755_FEATURE_OFF);

	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_als_persist_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_persist_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long persist;
	int rc;
	struct tmd2755_chip *chip = g_tmd2755_chip;

	rc = kstrtoul(buf, BASE_10, &persist);
	if (rc || (persist > ALS_PERSIST_MAX))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();
	ams_i2c_modify(chip->client, chip->shadow,
		TMD2755_REG_PERS, TMD2755_MASK_ALS_PERS, persist << TMD2755_SHIFT_ALS_PERS);
	chip->params.persist.persistance = chip->shadow[TMD2755_REG_PERS];
	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}

static ssize_t tmd2755_als_sat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}
static ssize_t tmd2755_als_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2755_als_show(dev, attr, buf);
}

static ssize_t tmd2755_als_adc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd2755_chip *chip = g_tmd2755_chip;
	u32 ch0, ch1;

	if (sscanf(buf, "%10d,%10d", &ch0, &ch1) != 2)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);
	ALS_OFF();
	chip->als_info.ch0_raw = ch0;
	chip->als_info.ch1_raw = ch1;
	ALS_ON();
	AMS_MUTEX_UNLOCK(&chip->lock);

	return size;
}


struct device_attribute tmd2755_als_attrs[] = {
	__ATTR(als_atime,         0664, tmd2755_als_atime_show,        tmd2755_als_atime_store),
	__ATTR(als_wtime,         0664, tmd2755_als_wtime_show,        tmd2755_als_wtime_store),
	__ATTR(als_wlong,         0664, tmd2755_als_wlong_show,        tmd2755_als_wlong_store),
	__ATTR(als_lux,           0444, tmd2755_als_lux_show,          NULL),
	__ATTR(als_gain,          0664, tmd2755_als_gain_show,         tmd2755_als_gain_store),
	__ATTR(als_cpl,           0444, tmd2755_als_cpl_show,          NULL),
	__ATTR(als_ch0,           0444, tmd2755_als_ch0_show,          NULL),
	__ATTR(als_ch1,           0444, tmd2755_als_ch1_show,          NULL),
	__ATTR(als_thresh_deltaP, 0664, tmd2755_als_deltaP_show,       tmd2755_als_deltaP_store),
	__ATTR(als_auto_gain,     0664, tmd2755_auto_gain_enable_show, tmd2755_auto_gain_enable_store),
	__ATTR(als_lux_coef,      0664, tmd2755_lux_coef_show,         tmd2755_lux_coef_store),
	__ATTR(als_enable,        0664, tmd2755_als_enable_show,       tmd2755_als_enable_store),
	__ATTR(als_persist,       0664, tmd2755_als_persist_show,      tmd2755_als_persist_store),
	__ATTR(als_sat,           0444, tmd2755_als_sat_show,          NULL),
	__ATTR(regs, 0644, tmd2755_registers_show, tmd2755_registers_store),
	__ATTR(als_adc,           0664, tmd2755_als_adc_show,          tmd2755_als_adc_store),
};

int tmd2755_als_attrs_size = ARRAY_SIZE(tmd2755_als_attrs);
