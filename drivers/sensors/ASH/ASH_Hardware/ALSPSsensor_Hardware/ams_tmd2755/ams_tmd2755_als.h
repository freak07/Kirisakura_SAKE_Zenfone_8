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

#ifndef __AMS_TMD2755_ALS_H__
#define __AMS_TMD2755_ALS_H__

/* Debug flags for ALS specific */
// #define DEBUG_LUX

#define ALS_MAX_LUX_VAL          (65535)    /* (USHRT_MAX)  */
#define ALS_FULL_SCALE_CNT       (ALS_MAX_LUX_VAL)
#define ALS_ATIME_FOR_MAX_CNT     (0x3F) /* this is the first atime value that maxes out the counts - full scale */
#define ALS_ATIME_INCREMENT_CNT   (1024) /* each increase in atime, causes, this increase in full scale count */

#define ALS_GAIN_REG_VAL_16       (5)  /* Bit value in register */
#define ALS_GAIN_16               (16) /* magnitude of gain */
#define ALS_GAIN_REG_VAL_128      (8) /* Bit value in register */
#define ALS_GAIN_128             (128) /* magnitude of gain */
#define ALS_GAIN_REG_VAL_1024    (11)
#define ALS_GAIN_1024            (1024)

#define ALS_COEFF_SCALE_FACTOR    (1000) /* retrieved from DTS */

#define AWTIME_LONG_FACTOR         (12)

/* This only works if the channel data is stored in consecutive registers */
#define ALS_NUM_CH                  (2)         /* number of channels performing ALS */
#define ALS_CH_SIZE            (sizeof(u8) * 2) /* 16 bit registers */

#define TMD2755_MAX_ALS_VALUE     (0xFFFF) /* 16 bit */
#define TMD2755_MIN_ALS_VALUE      (10)

#define TMD2755_THRESHOLD_PERCENT   (200) /* equates to 0.5% */

#define TMD2755_SATURATION_THRESHOLD (9) /* 90% of max ALS count */

#define TMD2755_CH0_MAXIMUM (65535)

#define TMD2755_AUTO_GAIN_CLAMP_CNT	(8)

#define TMD2755_SHIFT_MULT_BY_1024    (10)

/* fraction out of 10 */
#define TENTH_FRACTION_OF_VAL(v, x) ({ \
	int __frac = v; \
	if (((x) > 0) && ((x) < 10)) \
		__frac = (__frac*(x)) / 10 ; \
	__frac; \
})

#define ALS_ON()        do {                  \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_ENABLE,  TMD2755_AEN,  TMD2755_AEN); \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_INTENAB, TMD2755_AIEN, TMD2755_AIEN); \
				chip->als_enable = true;   \
			} while (0)

#define ALS_OFF()       do {                  \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_INTENAB, TMD2755_AIEN,      0); \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_ENABLE,  TMD2755_AEN,       0); \
				chip->als_enable = false;   \
			} while (0)


/* Min/Max/Default Values for parameters */
/**********************************************************************************/
#define ALS_PERSIST_MIN			(0)
#define ALS_PERSIST_MAX			(0x0F)
#define ALS_PERSIST_DEFAULT		(2)

#define ALS_GAIN_MIN			(5)	/* 16x */
#define ALS_GAIN_MAX			(11)	/* 1024x - only 5, 8 and 11 are acceptable values */
#define ALS_GAIN_DEFAULT		(8)	/* 128x */

#define ALS_AUTO_GAIN_MIN		(0)
#define ALS_AUTO_GAIN_MAX		(1)
#define ALS_AUTO_GAIN_DEFAULT		(1)

/* Variation allowed in ALS to cause interrupt */
#define ALS_DELTA_P_MIN			(0)
#define ALS_DELTA_P_MAX			(20)
#define ALS_DELTA_P_DEFAULT		(10)

#define ALS_TIME_MIN			(0)
#define ALS_TIME_MAX			(0xFF)
#define ALS_TIME_DEFAULT		(0x3F)

#define ALS_WTIME_MIN			(0)
#define ALS_WTIME_MAX			(0xFF)
#define ALS_WTIME_DEFAULT		(0)

#define ALS_WLONG_MIN			(0)
#define ALS_WLONG_MAX			(1)
#define ALS_WLONG_DEFAULT		(0)



/* coefficient limits are assumed to be scaled                   */
/* Coefficients vary so placing a limit on them may be difficult */
/* to validate  - While these values are here, the actual values */
/* read in from the DT overlay are not boundary checked          */

/* Device & Glass Factor */
#define ALS_DGF_MIN			(0)
#define ALS_DGF_MAX			(200)
#define ALS_DGF_DEFAULT			(123)

/* Coeff A */
#define ALS_CH0_COEFF0_MIN		(0)
#define ALS_CH0_COEFF0_MAX		(1000)
#define ALS_CH0_COEFF0_DEFAULT		(1000)

/* Coeff B */
#define ALS_CH1_COEFF0_MIN		(0)
#define ALS_CH1_COEFF0_MAX		(1000)
#define ALS_CH1_COEFF0_DEFAULT		(1427)

/* Coeff C */
#define ALS_CH0_COEFF1_MIN		(0)
#define ALS_CH0_COEFF1_MAX		(1000)
#define ALS_CH0_COEFF1_DEFAULT		(849)

/* Coeff D */
#define ALS_CH1_COEFF1_MIN		(0)
#define ALS_CH1_COEFF1_MAX		(1000)
#define ALS_CH1_COEFF1_DEFAULT		(834)

#define ALS_COEFF_SCALE_MIN		(1)
#define ALS_COEFF_SCALE_MAX		(1000)
#define ALS_COEFF_SCALE_DEFAULT		(1000)

/*
 * This must match the order of ATTR defined in
 * struct device_attribute tmd2755_als_attrs[]
 * in ams_tmd2755_als.c
 */
enum tmd2755_als_attrs {
	ALS_ATIME_ATTR,
	ALS_AWTIME_ATTR,
	ALS_AWLONG_ATTR,
	ALS_LUX_ATTR,
	ALS_GAIN_ATTR,
	ALS_CPL_ATTR,
	ALS_CH0_ATTR,
	ALS_CH1_ATTR,
	ALS_THRESH_DELTAP_ATTR,
	ALS_AUTO_GAIN_ATTR,
	ALS_LUX_COEFF_ATTR,
	ALS_ENABLE_ATTR,
	ALS_PERSIST_ATTR,
	ALS_SATURATION_ATTR,
	ALS_ADC_ATTR,
};

extern struct device_attribute tmd2755_als_attrs[];
extern int tmd2755_als_attrs_size;


int tmd2755_configure_als_mode(struct tmd2755_chip *chip, u8 state);
int tmd2755_read_als(struct tmd2755_chip *chip);
void tmd2755_report_als(struct tmd2755_chip *chip);
extern ssize_t tmd2755_registers_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t tmd2755_registers_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern void light_polling_work_assing(void);
#endif // __AMS_TMD2755_ALS_H__
