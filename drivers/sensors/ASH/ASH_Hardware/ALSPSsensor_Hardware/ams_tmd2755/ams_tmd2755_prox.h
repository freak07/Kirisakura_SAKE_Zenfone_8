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

#ifndef __AMS_TMD2755_PROX_H__
#define __AMS_TMD2755_PROX_H__

#define MIN_PROX_THRESHOLD                   (0)

/* Values used during calibration */
#define PROX_BINSRCH_TARGET_VALUE            (4) /* pdata target = 31 */
#define PROX_AUTO_OFFSET_ADJ_VAL             (1) /* enabled */
//#define PROX_AVG_VAL                         (2) /* enabled */
#define PROX_AVG_VAL                         (0) /* disabled will save 30ms int */

#define PROX_ORE_EN                          (1)    /* enable */
#define PROX_ORE_VAL                         (0x01) /* nominal + 1 step */

#define PROX_HWAVG_CAL                       (1) /* Allows hardware averaging during calibration */
#define PROX_PRATE_CAL                       (0) /* enables PRATE during calibration */

#define PROX_CALIB_TIMEOUT_MS		  (80)

#define PROX_APC_ENABLED                (0)
#define PROX_APC_DISABLED               (1)

#define PROX_SAT_EVENT                  (2)

/* Min/Max/Default Values for parameters */
/**********************************************************************************/
#define PROX_PERSIST_MIN                   (0)
#define PROX_PERSIST_MAX                   (15)
#define PROX_PERSIST_DEFAULT               (1)

#define PROX_OFFSET_INIT                   (100)


/* Thresholds limits - up to the user to determine if min < max */
/* Routines adjust value based on APC */
#define PROX_THRESH_MIN			(0)
#define PROX_THRESH_MAX             (0x3FFF)
#define PROX_THRESH_DEFAULT             (55)

#define PROX_PULSE_CNT_MIN		(0)
#define PROX_PULSE_CNT_MAX		(63)
#define PROX_PULSE_CNT_DEFAULT		(15)

#define PROX_APC_MIN			PROX_APC_ENABLED
#define PROX_APC_MAX			PROX_APC_DISABLED
#define PROX_APC_DEFAULT		PROX_APC_ENABLED

#define PROX_PULSE_LEN_MIN		(30)
#define PROX_PULSE_LEN_MAX		(0x3FF)
#define PROX_PULSE_LEN_DEFAULT		(30)

#define PROX_GAIN_MIN                      (0)
#define PROX_GAIN_MAX                      (3)    /* 2 bits gain_1 and gain_2 */
#define PROX_GAIN_DEFAULT                  (2)

#define PROX_OFFSET_MIN			(-255)
#define PROX_OFFSET_MAX			(255)
#define PROX_OFFSET_DEFAULT		(6)

#define PROX_DRIVE_MIN			(5)
#define PROX_DRIVE_MAX			(8)
#define PROX_DRIVE_DEFAULT		(5)

#define PROX_TIME_MIN                      (0)
#define PROX_TIME_MAX                     (255)
#define PROX_TIME_DEFAULT                 (31)

#define PROX_WTIME_MIN                      (0)
#define PROX_WTIME_MAX                     (255)
#define PROX_WTIME_DEFAULT                 (0)



/*****************************************************************************/




#define PROX_ON()       do {                  \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_ENABLE,  TMD2755_PEN,  TMD2755_PEN); \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_INTENAB, TMD2755_PIEN, TMD2755_PIEN); \
			} while (0)

#define PROX_OFF()       do {                  \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_INTENAB, TMD2755_PIEN,      0); \
				ams_i2c_modify(chip->client, chip->shadow, TMD2755_REG_ENABLE,  TMD2755_PEN,       0); \
			} while (0)

/*
 * This must match the order of ATTR defined in
 * struct device_attribute tmd2755_prox_attrs[]
 * in ams_tmd2755_prox.c
 */
enum tmd2755_prox_attrs {
	PROX_RAW_ATTR,
	PROX_DETECT_ATTR,
	PROX_GAIN1_ATTR,
	PROX_GAIN2_ATTR,
	PROX_OFFSET_ATTR,
	PROX_PERSIST_ATTR,
	PROX_PULSE_LEN_ATTR,
	PROX_ENABLE_ATTR,
	PROX_TIME_ATTR,
	PROX_WTIME_ATTR,
	PROX_LOW_THRESH_ATTR,
	PROX_HIGH_THRESH_ATTR,
	PROX_REGS_ATTR,
    PROX_SATURATION_ATTR,
};

enum tmd2755_prox_sat_states {
	PROX_NO_SAT,
	PROX_SAT,            /* general saturation */
	PROX_AMBIENT_SAT,
	PROX_REFLECTIVE_SAT,
};


extern struct device_attribute tmd2755_prox_attrs[];
extern int tmd2755_prox_attrs_size;


int tmd2755_configure_prox_mode(struct tmd2755_chip *chip, u8 state);
void tmd2755_init_prox_mode(struct tmd2755_chip *chip);
void tmd2755_process_prox_irq(struct tmd2755_chip *chip);
void tmd2755_prox_report_inp_event(struct tmd2755_chip *chip, int value);
void tmd2755_process_saturation_event(struct tmd2755_chip *chip);
extern void proximity_work(int state);
void tmd2755_read_poffset(struct tmd2755_chip *chip);


#endif // __AMS_TMD2755_PROX_H__
