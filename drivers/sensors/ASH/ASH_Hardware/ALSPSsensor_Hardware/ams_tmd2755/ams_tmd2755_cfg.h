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

/*
 * This file is meant to be used as defensive programming to protect invalid
 * values passed in via the DTS overlay.  Only include this file once in the entire
 * project.  Include it in the source file that reads the DTS database.
 */

#pragma once

#define CHECK_LIMITS(val, y)  ({  \
	s32 ret_val = val;                        \
	if ((val < y.min) || (val > y.max)) {     \
		ret_val = y.def;                  \
		printk(KERN_WARNING "WARNING: Default parameter value being used: %s (attempted val=%d min=%d max=%d def=%d)\n", y.name, val, y.min, y.max, y.def);  \
	}                                         \
	ret_val;                                  \
})

enum dts_parameters {
	PROX_PERIST_CFG,
	PROX_THRESH_MIN_CFG,
	PROX_THRESH_MAX_CFG,
	PROX_PULSE_CNT_CFG,
	PROX_APC_CFG,
	PROX_PULSE_LEN_CFG,
	PROX_GAIN_1_CFG,
	PROX_GAIN_2_CFG,
	PROX_OFFSET_CFG,
	PROX_DRIVE_CFG,
	PROX_TIME_CFG,
	PROX_WTIME_CFG,
	/*********************/
	ALS_PERSIST_CFG,
	ALS_GAIN_CFG,
	ALS_AUTO_GAIN_CFG,
	ALS_DELTA_P_CFG,
	ALS_TIME_CFG,
	ALS_WTIME_CFG,
	ALS_DGF_CFG,
	ALS_CH0_COEFF0_CFG,
	ALS_CH1_COEFF0_CFG,
	ALS_CH0_COEFF1_CFG,
	ALS_CH1_COEFF1_CFG,
	ALS_COEFF_SCALE_CFG,
};

static struct  {
	s32   min;
	s32   max;
	s32   def;
	char *name;
} tmd2755_cfg_limits[] = {
	/**********************************   PROX    *******************************************************/
	[PROX_PERIST_CFG]     = {PROX_PERSIST_MIN,   PROX_PERSIST_MAX,   PROX_PERSIST_DEFAULT,   "ppersist"},
	[PROX_THRESH_MIN_CFG] = {PROX_THRESH_MIN,    PROX_THRESH_MAX,    PROX_THRESH_DEFAULT,    "pmin_thresh"},
	[PROX_THRESH_MAX_CFG] = {PROX_THRESH_MIN,    PROX_THRESH_MAX,    PROX_THRESH_DEFAULT,    "pmax_thresh"},
	[PROX_PULSE_CNT_CFG]  = {PROX_PULSE_CNT_MIN, PROX_PULSE_CNT_MAX, PROX_PULSE_CNT_DEFAULT, "ppulse_cnt"},
	[PROX_APC_CFG]        = {PROX_APC_MIN,       PROX_APC_MAX,       PROX_APC_DEFAULT,       "papc"},
	[PROX_PULSE_LEN_CFG]  = {PROX_PULSE_LEN_MIN, PROX_PULSE_LEN_MAX, PROX_PULSE_LEN_DEFAULT, "ppulse_len"},
	[PROX_GAIN_1_CFG]     = {PROX_GAIN_MIN,      PROX_GAIN_MAX,      PROX_GAIN_DEFAULT,      "pgain_1"},
	[PROX_GAIN_2_CFG]     = {PROX_GAIN_MIN,      PROX_GAIN_MAX,      PROX_GAIN_DEFAULT,      "pgain_2"}, /* cannot be 2 */
	[PROX_OFFSET_CFG]     = {PROX_OFFSET_MIN,    PROX_OFFSET_MAX,    PROX_OFFSET_DEFAULT,    "poffset"},
	[PROX_DRIVE_CFG]      = {PROX_DRIVE_MIN,     PROX_DRIVE_MAX,     PROX_DRIVE_DEFAULT,     "pdrive"},
	[PROX_TIME_CFG]       = {PROX_TIME_MIN,      PROX_TIME_MAX,      PROX_TIME_DEFAULT,      "ptime"},
	[PROX_WTIME_CFG]      = {PROX_WTIME_MIN,     PROX_WTIME_MAX,     PROX_WTIME_DEFAULT,     "pwtime"},
	/****************************************************************************************************/
	/**********************************   ALS     *******************************************************/
	[ALS_PERSIST_CFG]      = {ALS_PERSIST_MIN,     ALS_PERSIST_MAX,      ALS_PERSIST_DEFAULT,      "apers"},
	[ALS_GAIN_CFG]         = {ALS_GAIN_MIN,        ALS_GAIN_MAX,         ALS_GAIN_DEFAULT,         "als_gain"}, /* 5, 8 or 11 */
	[ALS_AUTO_GAIN_CFG]    = {ALS_AUTO_GAIN_MIN,   ALS_AUTO_GAIN_MAX,    ALS_AUTO_GAIN_DEFAULT,    "als_auto_gain"},
	[ALS_DELTA_P_CFG]      = {ALS_DELTA_P_MIN,     ALS_DELTA_P_MAX,      ALS_DELTA_P_DEFAULT,      "als_deltap"},
	[ALS_TIME_CFG]         = {ALS_TIME_MIN,        ALS_TIME_MAX,         ALS_TIME_DEFAULT,         "als_time"},
	[ALS_WTIME_CFG]         = {ALS_WTIME_MIN,        ALS_WTIME_MAX,         	ALS_WTIME_DEFAULT,         "als_wtime"},
	[ALS_DGF_CFG]          = {ALS_DGF_MIN,         ALS_DGF_MAX,          ALS_DGF_DEFAULT,          "dgf"},
	[ALS_CH0_COEFF0_CFG]   = {ALS_CH0_COEFF0_MIN,  ALS_CH0_COEFF0_MAX,   ALS_CH0_COEFF0_DEFAULT,   "ch0_coef0"},
	[ALS_CH1_COEFF0_CFG]   = {ALS_CH1_COEFF0_MIN,  ALS_CH1_COEFF0_MAX,   ALS_CH1_COEFF0_DEFAULT,   "ch1_coef0"},
	[ALS_CH0_COEFF1_CFG]   = {ALS_CH0_COEFF1_MIN,  ALS_CH0_COEFF1_MAX,   ALS_CH0_COEFF1_DEFAULT,   "ch0_coef1"},
	[ALS_CH1_COEFF1_CFG]   = {ALS_CH1_COEFF1_MIN,  ALS_CH1_COEFF1_MAX,   ALS_CH1_COEFF1_DEFAULT,   "ch1_coef1"},
	[ALS_COEFF_SCALE_CFG]  = {ALS_COEFF_SCALE_MIN, ALS_COEFF_SCALE_MAX,  ALS_COEFF_SCALE_DEFAULT,  "coef_scale"},
	/****************************************************************************************************/
};
