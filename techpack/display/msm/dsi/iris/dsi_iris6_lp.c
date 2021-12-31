#include <video/mipi_display.h>
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include "dsi_drm.h"
#include <sde_encoder.h>
#include <sde_encoder_phys.h>
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_lightup_ocp.h"
#include "dsi_iris6_lp.h"
#include "dsi_iris6_pq.h"
#include "dsi_iris6_gpio.h"
#include "dsi_iris6_log.h"
#include "dsi_iris6_api.h"
#include "sde_trace.h"
#include <linux/semaphore.h>

static int debug_lp_opt;
/* abyp light up option (need panel off/on to take effect)
bit[0]: 0 -- light up with PT, 1 -- light up with ABYP
bit[2]: 0 -- use GPIO to swith, 1 -- use mipi command to switch
bit[3]: 0 -- non force, 1 -- force abyp during panel switch
bit[4]: 0 -- stay on ABYP, 1-- stay on PT.
*/
static int debug_on_opt;
static int lp_hdr_power_type;
static uint8_t iris_abyp_mode;
static struct semaphore abyp_sem;
static ktime_t lp_ktime0;

static void _iris_init_abyp_check_work(void);
static void _iris_abp_ctrl_init(bool chain);
static void _iris_dbp_init(bool enable, bool chain);

void iris_lp_init(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	iris_abyp_mode = IRIS_PT_MODE;
	if (pcfg->valid < PARAM_PARSED)
		iris_abyp_mode = IRIS_ABYP_MODE;
#ifdef IRIS_ABYP_LIGHTUP
	iris_abyp_mode = IRIS_ABYP_MODE;
#endif

	pcfg->read_path = PATH_DSI;

	pcfg->abyp_ctrl.pending_mode = MAX_MODE;
	mutex_init(&pcfg->abyp_ctrl.abyp_mutex);
	sema_init(&abyp_sem, 1);
	_iris_init_abyp_check_work();
}

void iris_lp_enable_pre(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (debug_on_opt & 0x1)
		iris_abyp_mode = IRIS_ABYP_MODE;

#ifdef IRIS_ABYP_LIGHTUP
	iris_abyp_mode = IRIS_ABYP_MODE;
#endif

	if (pcfg->valid < PARAM_PARSED)
		return;
	iris_init_one_wired();
}

/* init iris low power */
void iris_lp_enable_post(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (pcfg->valid < PARAM_PARSED)
		return;

	IRIS_LOGI("lp dpg:%d, ulps:%d, abyp:%d qsync:%d dpp_only:%d esd_ctrl:%d, read_path:%d",
			  pcfg->lp_ctrl.dynamic_power, pcfg->lp_ctrl.ulps_lp,
			  pcfg->lp_ctrl.abyp_lp, pcfg->lp_ctrl.qsync_mode,
			  pcfg->dpp_only_enable, pcfg->lp_ctrl.esd_ctrl, pcfg->read_path);

	_iris_abp_ctrl_init(1);

	lp_hdr_power_type = 2;	//to make hdr power type is different
	iris_pmu_hdr_set(0, 1); //set hdr power off
	iris_ulps_set(pcfg->lp_ctrl.ulps_lp, 1);
	iris_dynamic_power_set(pcfg->lp_ctrl.dynamic_power, 1);
	_iris_dbp_init(pcfg->dpp_only_enable, 0);
}

/*== PMU related APIs ==*/

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	IRIS_LOGI("%s: %d, chain %d", __func__, enable, chain);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ALT_CTRL0;
	regval.mask = 0x200000; //MIPI_CMD_SEL -- HDR auto dma load
	regval.value = (enable ? 0x200000 : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = 0xe5;
	regval.mask = 0x1;
	regval.value = (enable ? 0x1 : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	if (pcfg->lp_ctrl.qsync_mode) {
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf6;
		regval.mask = 0xc00;
		regval.value = (enable ? 0x0 : 0x800);
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
	}

	if (pcfg->lp_ctrl.dbp_mode && enable) {
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_OFF, ID_SYS_MPG_OFF, 0x1);
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_CTRL, ID_SYS_MPG_CTRL, 0x1);
		if (lp_hdr_power_type > 0) {
			// set pwil pp_en
			regval.ip = IRIS_IP_PWIL;
			regval.opt_id = 0xfc;
			regval.mask = 0x00000004;
			regval.value = 0x00000004;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 0x1);
		}
	}

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_DPG_CTRL;
	regval.mask = 0x00000031; //DYG_EN
	regval.value = (enable ? 0x31 : 0x30);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt();

	pcfg->lp_ctrl.dynamic_power = enable;
}

/* dynamic power gating get */
bool iris_dynamic_power_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.dynamic_power;
}

void iris_dpg_event(bool start, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;
	uint8_t last;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s: %s chain %d", __func__, start ? "start" : "end", chain);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = start ? 0x10 : 0x11;
	if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power)
		last = 0x1;
	else
		last = chain;
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, last);

	if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power) {
		// set dma trig gen mode
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf6;
		regval.mask = 0xc00;
		regval.value = start ? 0x800 : 0x0;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain ? 1 : 0);
	}
}

/* power on & off HDR domain
 *   type: 0 -- power off HDR & HDR_COLOR
 *         1 -- power on HDR, power off HDR_COLOER
 *         2 -- power on HDR & HDR_COLOR
 */
int iris_pmu_hdr_set(int type, bool chain)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s: type %d, cur type %d, chain %d", __func__, type, lp_hdr_power_type, chain);

	if (type < 0 || type > 2) {
		IRIS_LOGW("%s: type %d is wrong.", __func__, type);
		return 1;
	}

	if (lp_hdr_power_type == type) {
		IRIS_LOGI("%s: same type %d", __func__, type);
		return 2;
	}

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xe6; //power protection
	regval.mask = 0x7;
	if (pcfg->lp_ctrl.dbp_mode)
		regval.value = 0x4;
	else if (type > 0)
		regval.value = 0x6;
	else
		regval.value = 0x5;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power) {
		// set pwil pb_req
		regval.ip = IRIS_IP_PWIL;
		regval.opt_id = 0xfd;
		regval.mask = 0x00020000;
		if (type > 0)
			regval.value = 0x00020000;
		else
			regval.value = 0x00000000;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf5; //pmu ctrl
		regval.mask = 0x200;  //HDR_IDLE_MASK_EN
		if (type > lp_hdr_power_type)
			regval.value = 0x200;
		else
			regval.value = 0x0;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
	}

	lp_hdr_power_type = type;

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_MPG_CTRL;
	regval.mask = 0x18;
	if (lp_hdr_power_type == 2)
		regval.value = 0x18;
	else if (lp_hdr_power_type == 1)
		regval.value = 0x8;
	else
		regval.value = 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);

	if (!chain)
		iris_update_pq_opt();

	return 0;
}

void iris_ulps_set(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (debug_lp_opt & 0x200) {
		IRIS_LOGI("not set ulps: %d", enable);
		return;
	}
	IRIS_LOGI("ulps set: %d, chain %d", enable, chain);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ULPS_CTRL;
	regval.mask = 0x40;
	regval.value = enable ? 0x40 : 0x0;
	pcfg->lp_ctrl.ulps_lp = enable;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt();
}

bool iris_ulps_enable_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGI("ulps ap:%d, iris:%d",
			  pcfg->display->panel->ulps_feature_enabled, pcfg->lp_ctrl.ulps_lp);

	if (pcfg->display->panel->ulps_feature_enabled && pcfg->lp_ctrl.ulps_lp)
		return true;
	else
		return false;
}

/*== Analog bypass related APIs ==*/
static void _iris_abp_ctrl_init(bool chain)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ABP_CTRL;
	regval.mask = 0x0CC00000;
	if (pcfg->lp_ctrl.abyp_lp == 1)
		regval.value = 0x00800000;
	else if (pcfg->lp_ctrl.abyp_lp == 2)
		regval.value = 0x00400000;
	else
		regval.value = 0x00000000;

	if (pcfg->dpp_only_enable)
		regval.value = 0x00C00000;

	/* w/o dbp, digital_bypass_i2a_en = 1 */
	if (!pcfg->dpp_only_enable)
		regval.value |= 0x04000000;
	/* Set digital_bypass_a2i_en/digital_bypass_i2a_en = 1 for video mode */
	if (pcfg->panel->panel_mode == DSI_OP_VIDEO_MODE)
		regval.value |= 0x0C000000;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt();
}

// to get correct iris chip abyp status, should call this API.
int iris_get_abyp_mode_blocking(void)
{
	uint8_t mode;
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	down(&abyp_sem);
	mode = iris_abyp_mode;
	up(&abyp_sem);
	return mode;
}

// only for get current status quickly
int iris_get_abyp_mode_nonblocking(void)
{
	return iris_abyp_mode;
}

void iris_force_abyp_mode(uint8_t mode)
{
	IRIS_LOGD("%s mode:%d", __func__, mode);
	down(&abyp_sem);
	if (mode == IRIS_PT_MODE || mode == IRIS_ABYP_MODE)
		iris_abyp_mode = mode;
	else
		IRIS_LOGE("%s not support mode:%d", __func__, mode);
	up(&abyp_sem);
}

/* Switch ABYP by GRCP commands
 * enter_abyp: true -- Enter ABYP, false -- Exit ABYP
*/
static void _iris_send_grcp_abyp(bool enter_abyp)
{
	if (enter_abyp) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		IRIS_LOGI("%s, Enter ABYP.", __func__);
	} else {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
		IRIS_LOGI("%s, Exit ABYP.", __func__);
	}
}

static int _iris_set_max_return_size(void)
{
	int rc;
	struct iris_cfg *pcfg;
	static char max_pktsize[2] = {0x01, 0x00}; /* LSB tx first, 2 bytes */
	static struct dsi_cmd_desc pkt_size_cmd = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK, 0, 0,
		 sizeof(max_pktsize), max_pktsize, 0, NULL},
		1,
		0};
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &pkt_size_cmd,
	};

	pcfg = iris_get_cfg();

	IRIS_LOGD("%s", __func__);

	rc = iris_dsi_send_cmds(pcfg->panel, cmdset.cmds, cmdset.count,
							cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);
	if (rc)
		IRIS_LOGE("failed to send max return size packet, rc=%d", rc);

	return rc;
}

bool iris_lp_abyp_enter(bool pending)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;
	int i;
	int abyp_status_gpio;

	pcfg = iris_get_cfg();
	if (debug_lp_opt & 0x400)
		lp_ktime0 = ktime_get();

	down(&abyp_sem);
	IRIS_LOGI("Enter abyp mode start, pending: %d", pending);

	if (pcfg->rx_mode == 1) {
		if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power) {
			/* change dma ch1 trigger src sel to SW manual mode */
			regval.ip = IRIS_IP_SYS;
			regval.opt_id = 0xf6;
			regval.mask = 0xc00;
			regval.value = 0x800;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

			/* set rx pb req */
			regval.ip = IRIS_IP_RX;
			regval.opt_id = 0xe4;
			regval.mask = 0x3;
			regval.value = 0x2;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
		}

		/* mask dpg wakeup src */
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf0;
		regval.mask = 0x30;
		regval.value = 0x0;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x0);
		iris_update_pq_opt();
	}

	/* send enter analog bypass */
	SDE_ATRACE_BEGIN("iris_abyp_enter_cmd");
	iris_send_one_wired_cmd(IRIS_ENTER_ANALOG_BYPASS);
	SDE_ATRACE_END("iris_abyp_enter_cmd");

	if (pending) {
		iris_abyp_mode = IRIS_PT_TO_ABYP_MODE;
		cancel_work_sync(&pcfg->abyp_check_work);
		schedule_work(&pcfg->abyp_check_work);
		return false;
	}

	/* check abyp gpio status */
	for (i = 0; i < 50; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s, %d, ABYP status: %d.", __func__, i, abyp_status_gpio);
		if (abyp_status_gpio == 1) {
			iris_abyp_mode = IRIS_ABYP_MODE;
			break;
		}
		udelay(3 * 1000);
	}
	up(&abyp_sem);

	if (debug_lp_opt & 0x400) {
		IRIS_LOGI("Enter abyp spend time %d us",
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
	}
	if (abyp_status_gpio == 0) {
		IRIS_LOGE("Enter abyp mode failed!");
		return true;
	}
	IRIS_LOGI("Enter abyp done");
	return false;
}

static void _iris_proc_timing_switch(struct iris_cfg *pcfg)
{
	ktime_t ktime = 0;

	if (pcfg->cur_timing == pcfg->prev_timing)
		return;

	if (IRIS_IF_LOGI())
		ktime = ktime_get();

	SDE_ATRACE_BEGIN("_iris_proc_timing_switch");
	iris_send_mode_switch_pkt();
	SDE_ATRACE_END("_iris_proc_timing_switch");

	if (IRIS_IF_LOGI())
		IRIS_LOGI("%s() takes %d us.", __func__,
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime));
}

bool iris_lp_abyp_exit(bool pending)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;
	int i;
	int abyp_status_gpio;

	pcfg = iris_get_cfg();
	if (debug_lp_opt & 0x400)
		lp_ktime0 = ktime_get();

	if (pcfg->cur_timing != pcfg->prev_timing)
		pending = false;

	down(&abyp_sem);
	IRIS_LOGI("Exit abyp mode start, pending: %d", pending);
	/* exit analog bypass */
	iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
	SDE_ATRACE_BEGIN("iris_abyp_exit_cmd");
	if (pcfg->rx_mode == 1) {
		udelay(1000);
		_iris_set_max_return_size();
	}
	SDE_ATRACE_END("iris_abyp_exit_cmd");
	//mutex_lock(&pcfg->abyp_ctrl.abyp_mutex);

	if (pending) {
		iris_abyp_mode = IRIS_ABYP_TO_PT_MODE;
		cancel_work_sync(&pcfg->abyp_check_work);
		schedule_work(&pcfg->abyp_check_work);
		return false;
	}

	/* check abyp gpio status */
	for (i = 0; i < 50; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s, %d, ABYP status: %d.", __func__, i, abyp_status_gpio);
		if (abyp_status_gpio == 0) {
			iris_abyp_mode = IRIS_PT_MODE;
			break;
		}
		udelay(3 * 1000);
	}
	if (debug_lp_opt & 0x400) {
		IRIS_LOGI("Exit abyp spend time %d us",
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
	}

	/* restore dpg wakeup src value */
	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0xf0;
	regval.mask = 0x30;
	regval.value = 0x30;
	iris_update_bitmask_regval_nonread(&regval, false);
	up(&abyp_sem);

	if (abyp_status_gpio == 1) {
		IRIS_LOGE("Exit abyp mode failed!");
		return true;
	}

	if (pcfg->lp_ctrl.abyp_lp == 2) {
		IRIS_LOGI("abyp light up iris");
		iris_lightup(pcfg->panel, NULL);
		if (debug_lp_opt & 0x400) {
			IRIS_LOGI("Light up time %d us",
					(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
		}
	} else {
		_iris_proc_timing_switch(pcfg);
	}

	IRIS_LOGI("Exit abyp done");
	return false;
}

static void _iris_abyp_check_work(struct work_struct *work)
{
	//struct iris_cfg *pcfg = container_of(work, struct iris_cfg, abyp_check_work);
	struct iris_update_regval regval;
	int i;
	int abyp_status_gpio;
	bool checkdone = false;
	bool pt2abyp = false;

	/* check abyp gpio status */
	for (i = 0; i < 50; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s, %d, status: %d.", __func__, i, abyp_status_gpio);
		if (iris_abyp_mode == IRIS_PT_TO_ABYP_MODE) {
			pt2abyp = true;
			if (abyp_status_gpio == 1) {
				iris_abyp_mode = IRIS_ABYP_MODE;
				checkdone = true;
				break;
			}
		} else if(iris_abyp_mode == IRIS_ABYP_TO_PT_MODE) {
			pt2abyp = false;
			if (abyp_status_gpio == 0) {
				iris_abyp_mode = IRIS_PT_MODE;
				checkdone = true;
				/* restore dpg wakeup src value */
				regval.ip = IRIS_IP_SYS;
				regval.opt_id = 0xf0;
				regval.mask = 0x30;
				regval.value = 0x30;
				iris_update_bitmask_regval_nonread(&regval, false);
				break;
			}
		} else {
			IRIS_LOGI("Not switch mode: %d", iris_abyp_mode);
			break;
		}
		udelay(3 * 1000);
	}
	up(&abyp_sem);
	if (debug_lp_opt & 0x400) {
		IRIS_LOGI("ABYP switch time %d us",
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
	}
	IRIS_LOGI("%s ABYP %s, status %d", pt2abyp ? "Enter" : "Exit",
			checkdone ? "done" : "failed", iris_abyp_mode);
}

static void _iris_init_abyp_check_work(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();
	INIT_WORK(&pcfg->abyp_check_work, _iris_abyp_check_work);
}

void iris_abyp_switch_proc(struct dsi_display *display, int mode, bool pending)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->rx_mode != pcfg->tx_mode) {
		IRIS_LOGE("abyp can't be supported! rx_mode != tx_mode!");
		return;
	}

	if (pcfg->abyp_ctrl.abyp_disable) {
		IRIS_LOGE("gpio is not setting for abypass");
		return;
	}
#if 0
	if (pending) {
		mutex_lock(&pcfg->abyp_ctrl.abyp_mutex);
		pcfg->abyp_ctrl.pending_mode = mode;
		mutex_unlock(&pcfg->abyp_ctrl.abyp_mutex);
		return;
	}
#endif
	if (mode == iris_abyp_mode) {
		IRIS_LOGW("%s same mode:%d!", __func__, mode);
		return;
	}
	if (debug_lp_opt & 0x80)
		pending = false;
	if (mode == IRIS_ABYP_MODE) {
		SDE_ATRACE_BEGIN("iris_abyp_enter");
		iris_lp_abyp_enter(pending);
		SDE_ATRACE_END("iris_abyp_enter");
	} else if (mode == IRIS_PT_MODE) {
		SDE_ATRACE_BEGIN("iris_abyp_exit");
		iris_lp_abyp_exit(pending);
		SDE_ATRACE_END("iris_abyp_exit");
	} else
		IRIS_LOGE("%s: switch mode: %d not supported!", __func__, mode);
}

int iris_lightup_exit_abyp(bool one_wired, bool reset)
{
	int i = 0;
	int iris_abyp_ready_gpio = 0;

	/* check abyp gpio status */
	iris_abyp_ready_gpio = iris_check_abyp_ready();

	if (iris_abyp_ready_gpio != 1) {
		IRIS_LOGW("%s, Iris isn't in ABYP.", __func__);
	}

	if (reset) {
		iris_send_one_wired_cmd(IRIS_POWER_DOWN_MIPI);
		udelay(1000);
		iris_send_one_wired_cmd(IRIS_POWER_UP_MIPI);
		udelay(1000);
	}

	/* try to exit analog bypass */
	if (one_wired)
		iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
	else
		_iris_send_grcp_abyp(false); /* switch by GRCP command */

	/* check abyp gpio status */
	for (i = 0; i < 50; i++) {
		udelay(3 * 1000);
		iris_abyp_ready_gpio = iris_check_abyp_ready();
		IRIS_LOGI("%s(%d), ABYP status: %d.", __func__, __LINE__, iris_abyp_ready_gpio);
		if (iris_abyp_ready_gpio == 0) {
			iris_abyp_mode = IRIS_PT_MODE;
			break;
		}
	}
	if (iris_abyp_ready_gpio == 1) {
		IRIS_LOGE("Exit abyp mode failed!");
		return 1;
	}
	IRIS_LOGI("Exit abyp done");
	return 0;
}

void iris_dbp_switch(bool enter)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;

	if (pcfg->lp_ctrl.dbp_mode == enter) {
		IRIS_LOGW("%s same mode:%d!", __func__, enter);
		return;
	}

	if (!pcfg->dpp_only_enable) {
		IRIS_LOGW("%s dpp_only is disable!", __func__);
		return;
	}
	IRIS_LOGI("%s, enter: %d.", __func__, enter);

	if (enter)
		iris_dpp_digitalBypass_metaEn(false);

	if (!pcfg->lp_ctrl.dynamic_power && !enter) {
		//manual power on PQ/HDR domains in DBP if need,
		//should set 0 then set 1
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_OFF, ID_SYS_MPG_OFF, 0x1);
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_CTRL, ID_SYS_MPG_CTRL, 0x1);
		if (lp_hdr_power_type > 0) {
			// set pwil pp_en
			regval.ip = IRIS_IP_PWIL;
			regval.opt_id = 0xfc;
			regval.mask = 0x00000004;
			regval.value = 0x00000004;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 0x1);
		}
	}

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xe6;
	regval.mask = 0x7;
	if (enter)
		regval.value = 0x4;
	else if (lp_hdr_power_type > 0)
		regval.value = 0x6;
	else
		regval.value = 0x5;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_DMA;
	regval.mask = 0x2;
	regval.value = enter ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_REG;
	regval.mask = 0x2;
	regval.value = enter ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	if (!pcfg->lp_ctrl.dynamic_power && enter) {
		//manual power off PQ/HDR domains in DBP
		if (lp_hdr_power_type > 0) {
			// set pwil pp_en
			regval.ip = IRIS_IP_PWIL;
			regval.opt_id = 0xfc;
			regval.mask = 0x00000004;
			regval.value = 0x00000000;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 0x1);
		}
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_OFF, ID_SYS_MPG_OFF, 0x1);
	}

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ABP_CTRL;
	regval.mask = 0x00C00000;
	if (enter) {
		if (pcfg->lp_ctrl.abyp_lp != 2)
			regval.value = 0x00C00000;
	} else {
		if (pcfg->lp_ctrl.abyp_lp == 1)
			regval.value = 0x00800000;
		else if (pcfg->lp_ctrl.abyp_lp == 2)
			regval.value = 0x00400000;
		else
			regval.value = 0x00000000;
	}
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x00);

	iris_update_pq_opt();

	if (!enter)
		iris_dpp_digitalBypass_metaEn(true);

	pcfg->lp_ctrl.dbp_mode = enter;
}

void _iris_dbp_init(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg = iris_get_cfg();
	IRIS_LOGI("%s enable: %d, chain: %d", __func__, enable, chain);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x10;	  //DPP_2
	regval.mask = 0x00000100; //meta_en
	regval.value = enable ? 0x0 : 0x100;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x10, 0x10, 0x1);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0xa0; //gc_ctrl
	regval.mask = 0x800;
	regval.value = 0x800;
	if (pcfg->panel->panel_mode == DSI_OP_VIDEO_MODE)
		regval.value = enable ? 0x0 : 0x800;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xa0, 0xa0, 0x1);

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xe6;
	regval.mask = 0x7;
	if (enable)
		regval.value = 0x4;
	else if (lp_hdr_power_type > 0)
		regval.value = 0x6;
	else
		regval.value = 0x5;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_DMA;
	regval.mask = 0xa;
	regval.value = enable ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0xf7;
	regval.mask = 0x4; //PB_BPS_MODE_DPP_BPS_EN
	regval.value = enable ? 0x0 : 0x4;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt();

	pcfg->lp_ctrl.dbp_mode = enable;
}

int iris_lightup_opt_get(void)
{
	return debug_on_opt;
}

void iris_lp_setting_off(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_DMA;
	regval.mask = 0x2;
	regval.value = 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	pcfg->lp_ctrl.dbp_mode = false;

	pcfg->abyp_ctrl.pending_mode = MAX_MODE;
}

int iris_prepare_for_kickoff(void *phys_enc)
{
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct iris_cfg *pcfg;
	//int mode;

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	SDE_ATRACE_BEGIN("iris_prepare_for_kickoff");
	iris_update_dc_brightness();
	pcfg = iris_get_cfg();
#if 0
	if (pcfg->abyp_ctrl.pending_mode != MAX_MODE) {
		mutex_lock(&pcfg->abyp_ctrl.abyp_mutex);
		mode = pcfg->abyp_ctrl.pending_mode;
		pcfg->abyp_ctrl.pending_mode = MAX_MODE;
		mutex_unlock(&pcfg->abyp_ctrl.abyp_mutex);

		mutex_lock(&pcfg->panel->panel_lock);
		iris_abyp_switch_proc(pcfg->display, mode, false);
		mutex_unlock(&pcfg->panel->panel_lock);
	}
#endif
	SDE_ATRACE_END("iris_prepare_for_kickoff");
	return 0;
}

static u32 iris_register_list[] = {
	0xf1100004,
	0xf110000c,
	0xf1100034,
	0xf1100204,
	0xf0000038,
	0xf000003c,
	0xf0000040,
	0xf1140000,
	0xf1140004,
	0xf1140008,
	0xf114000c,
	0xf1140130,
	0xf1140138,
	0xf115ffe4,
	0xf10dffe4,
	0xf119ffe4,
	0xf11dffe4,
	0xf125ffe4,
	0xf129ffe4,
	0xf131ffe4,
	0xf139ffe4,
	0xf13dffe4,
	0xf141ffe4,
	0xf149ffe4,
	0xf161ffe4,
};

static void _iris_esd_register_dump(void)
{
	u32 value = 0;
	u32 index = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGE("iris esd register dump: ");
	for (index = 0; index < sizeof(iris_register_list) / 4; index++) {
		if (pcfg->read_path == PATH_I2C) //use i2c to read
			iris_ioctl_i2c_read(iris_register_list[index], &value);
		else {
			value = iris_ocp_read(iris_register_list[index], DSI_CMD_SET_STATE_HS);
		}
		IRIS_LOGE("%08x : %08x", iris_register_list[index], value);
	}
}

int iris_vc_id_get(int type)
{
	struct iris_cfg *pcfg;
	int vc_id = 0;

	pcfg = iris_get_cfg();
	if (type == 0)
		vc_id = pcfg->vc_ctrl.to_iris_vc_id;
	else if (type == 1)
		vc_id = pcfg->vc_ctrl.to_panel_hs_vc_id;
	else if (type == 2)
		vc_id = pcfg->vc_ctrl.to_panel_lp_vc_id;
	return vc_id;
}

static int _iris_esd_read(void)
{
	int rc = 1;
	unsigned int run_status = 0x00;
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	if (pcfg->read_path == PATH_I2C) { //use i2c to read
		uint32_t val;

		rc = iris_ioctl_i2c_read(IRIS_RUN_STATUS, &val);
		run_status = val & 0x3;

		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("i2c read iris esd value: 0x%0x. run_status:0x%x. rc:%d",
					  val, run_status, rc);
		}

		if (rc) {
			IRIS_LOGE("%s i2c read iris esd err: %d", __func__, rc);
			rc = -1;
			goto exit;
		}
	} else {
		char get_diag_result[1] = {0x0f};
		char rbuf[16] = {0};
		struct dsi_cmd_desc cmds = {
			{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			 sizeof(get_diag_result), get_diag_result, 2, rbuf},
			1,
			0};
		struct dsi_panel_cmd_set cmdset = {
			.state = DSI_CMD_SET_STATE_HS,
			.count = 1,
			.cmds = &cmds,
		};

		rc = iris_dsi_send_cmds(pcfg->panel, cmdset.cmds, cmdset.count,
								cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);

		run_status = rbuf[1] & 0x3;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("dsi read iris esd value: 0x%02x 0x%02x. run_status:0x%x. rc:%d.",
					  rbuf[0], rbuf[1], run_status, rc);
		}
		if (rc) {
			IRIS_LOGE("%s dsi read iris esd err: %d", __func__, rc);
			rc = -1;
			goto exit;
		}
	}

	if (run_status != 0) {
		pcfg->lp_ctrl.esd_cnt_iris++;
		IRIS_LOGE("iris esd err 0x%x detected. ctrl: %d; cnt: %d", run_status,
				  pcfg->lp_ctrl.esd_ctrl, pcfg->lp_ctrl.esd_cnt_iris);
		_iris_esd_register_dump();
		rc = -1;
	} else {
		rc = 1;
	}

exit:
	IRIS_LOGD("%s rc:%d", __func__, rc);

	return rc;
}

static bool _iris_dsi_display_validate_reg_read(struct dsi_panel *panel)
{
	int i, j = 0;
	int len = 0, *lenp;
	int group = 0, count = 0;
	struct drm_panel_esd_config *config;

	if (!panel)
		return false;

	config = &(panel->esd_config);

	lenp = config->status_valid_params ?: config->status_cmds_rlen;
	count = config->status_cmd.count;

	for (i = 0; i < count; i++)
		len += lenp[i];

	for (i = 0; i < len; i++)
		j += len;

	for (j = 0; j < config->groups; ++j) {
		for (i = 0; i < len; ++i) {
			if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
				IRIS_LOGI("panel esd:[%d] 0x%x", i, config->return_buf[i]);
			}
			if (config->return_buf[i] != config->status_value[group + i]) {
				IRIS_LOGE("panel esd err: [%d] 0x%x != 0x%x!", i,
						  config->return_buf[i], config->status_value[group + i]);
			}

			if (config->return_buf[i] !=
				config->status_value[group + i])
				break;
		}

		if (i == len)
			return true;
		group += len;
	}

	return false;
}

static int _iris_display_read(struct dsi_display_ctrl *ctrl,
							  struct dsi_panel *panel, int mode)
{
	int i, rc = 0, count = 0, start = 0, *lenp;
	struct drm_panel_esd_config *config;
	struct dsi_cmd_desc *cmds;
	u32 flags = 0;
	struct dsi_panel_cmd_set cmdset;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (!panel || !ctrl || !ctrl->ctrl)
		return -EINVAL;

	/*
	 * When DSI controller is not in initialized state, we do not want to
	 * report a false ESD failure and hence we defer until next read
	 * happen.
	 */
	if (dsi_ctrl_validate_host_state(ctrl->ctrl))
		return 1;

	config = &(panel->esd_config);
	lenp = config->status_valid_params ?: config->status_cmds_rlen;
	count = config->status_cmd.count;
	cmds = config->status_cmd.cmds;
	flags |= (DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ);

	memset(&cmdset, 0x00, sizeof(cmdset));
	cmdset.state = config->status_cmd.state;
	cmdset.count = 1;

	for (i = 0; i < count; ++i) {
		memset(config->status_buf, 0x0, SZ_4K);
		if (cmds[i].last_command) {
			cmds[i].msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
			flags |= DSI_CTRL_CMD_LAST_COMMAND;
		}
		cmds[i].msg.rx_buf = config->status_buf;
		cmds[i].msg.rx_len = config->status_cmds_rlen[i];

		if (mode == IRIS_ABYP_MODE) {
			cmds[i].msg.channel = 0;
			rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &cmds[i].msg, &flags);
			if (rc <= 0) {
				pr_err("rx cmd transfer failed rc=%d\n", rc);
				goto error;
			}
		} else {
			if (pcfg->vc_ctrl.vc_enable) {
				//use aux channel
				if (config->status_cmd.state == DSI_CMD_SET_STATE_HS)
					cmds[i].msg.channel = pcfg->vc_ctrl.to_panel_hs_vc_id;
				else
					cmds[i].msg.channel = pcfg->vc_ctrl.to_panel_lp_vc_id;
				rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &cmds[i].msg, &flags);
				if (rc <= 0) {
					IRIS_LOGE("rx cmd transfer failed rc=%d", rc);
					goto error;
				}
			} else {
				cmdset.cmds = &cmds[i];

				rc = iris_pt_send_panel_cmd(panel, &cmdset);

				if (rc < 0) {
					IRIS_LOGE("iris_pt_send_panel_cmd transfer failed rc=%d", rc);
					goto error;
				}
				rc = 1;
			}
		}
		memcpy(config->return_buf + start,
			   config->status_buf, lenp[i]);
		start += lenp[i];
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("%s rc=%d, len[%d]:%d buf[0]:0x%x. mode: %d", __func__, rc, i,
					  lenp[i], config->status_buf[0], mode);
		}
	}

error:

	return rc;
}

int iris_esd_ctrl_get(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.esd_ctrl;
}

int iris_status_get(struct dsi_display_ctrl *ctrl, struct dsi_panel *panel)
{
	int rc = 0;
	int mode;
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("esd %s start", __func__);

	mutex_lock(&pcfg->panel->panel_lock);
	// check abyp mode
	mode = iris_get_abyp_mode_nonblocking();

	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("esd %s, mode: %d", __func__, mode);

	if ((mode != IRIS_PT_MODE) && (mode != IRIS_ABYP_MODE)) {
		rc = 1;
		goto exit;
	}

	if ((mode == IRIS_PT_MODE) && (iris_esd_ctrl_get() & 0x1)) {

		if (pcfg->read_path == PATH_I2C) {
			rc = iris_dsirecover_check(IRIS_ESDCHECK_OP);
			if (rc != 0) {
				IRIS_LOGE("%s iris_dsirecover_check err", __func__);
				rc = -1;
				goto exit;
			}
		}

		// iris esd read in pt mode
		rc = _iris_esd_read();
		if (rc <= 0)
			goto exit;
		rc = 1;
	}
	if (iris_esd_ctrl_get() & 0x2) {
		// panel esd read in abyp & pt mode
		rc = _iris_display_read(ctrl, panel, mode);
		if (rc <= 0) {
			goto exit;
		}
		rc = _iris_dsi_display_validate_reg_read(panel);
		if (rc <= 0) {
			pcfg->lp_ctrl.esd_cnt_panel++;
			IRIS_LOGI("iris panel esd err detected. ctrl: %d; cnt: %d",
					  pcfg->lp_ctrl.esd_ctrl, pcfg->lp_ctrl.esd_cnt_panel);
			if (mode == IRIS_PT_MODE)
				_iris_esd_register_dump();
			goto exit;
		}
	} else {
		rc = 1;
		goto exit;
	}

exit:
	mutex_unlock(&pcfg->panel->panel_lock);
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("esd %s done rc: %d", __func__, rc);
	return rc;
}

/*== Low Power debug related ==*/

static ssize_t _iris_abyp_dbg_write(struct file *file,
				const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;
	static int cnt;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (!mutex_trylock(&pcfg->panel->panel_lock))
		return -EFAULT;

	if (val == 0) {
		iris_lp_abyp_exit(true);
		IRIS_LOGI("analog bypass->pt, %d", cnt);
	} else if (val == 1) {
		iris_lp_abyp_enter(true);
		IRIS_LOGI("pt->analog bypass, %d", cnt);
	} else if (val >= 11 && val <= 19) {
		IRIS_LOGI("%s one wired %d", __func__, (int)(val - 11));
		iris_send_one_wired_cmd((int)(val - 11));
	} else if (val == 20) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
		IRIS_LOGI("miniPMU analog bypass->pt");
	} else if (val == 21) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		IRIS_LOGI("miniPMU pt->analog bypass");
	} else if (val == 30) {
		iris_dbp_switch(false);
	} else if (val == 31) {
		iris_dbp_switch(true);
	} else if (val == 32) {
		iris_lightup_exit_abyp(true, false);
	} else if (val == 33) {
		iris_lightup_exit_abyp(true, true);
	} else if (val == 100) {
		iris_check_abyp_ready();
	}
	mutex_unlock(&pcfg->panel->panel_lock);

	return count;
}

static ssize_t _iris_abyp_dbg_read(struct file *file, char __user *buff,
								  size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	int buf_len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	buf_len = sizeof(bp);

	if (*ppos)
		return 0;

	tot = scnprintf(bp, buf_len,
					"abyp status gpio: %d\n", iris_check_abyp_ready());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp mode: %d\n", iris_abyp_mode);
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t _iris_lp_dbg_write(struct file *file,
								 const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (!mutex_trylock(&pcfg->panel->panel_lock))
		return -EFAULT;

	if (val == 0) {
		iris_dynamic_power_set(false, 0);
		iris_ulps_set(false, 0);
		IRIS_LOGI("disable dpg & ulps lp");
	} else if (val == 1) {
		iris_dynamic_power_set(true, 0);
		iris_ulps_set(true, 0);
		IRIS_LOGI("enable dpg & ulps lp");
	} else if (val == 2) {
		iris_dynamic_power_set(false, 0);
		IRIS_LOGI("disable dpg");
	} else if (val == 3) {
		iris_dynamic_power_set(true, 0);
		IRIS_LOGI("enable dpg");
	} else if (val == 10) {
		pcfg->lp_ctrl.abyp_lp = 0;
		IRIS_LOGI("disable abyp lp");
	} else if (val == 11) {
		pcfg->lp_ctrl.abyp_lp = 1;
		IRIS_LOGI("set abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	} else if (val == 12) {
		pcfg->lp_ctrl.abyp_lp = 2;
		IRIS_LOGI("set abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	} else if (val == 20) {
		iris_ulps_set(false, 0);
		IRIS_LOGI("disable iris ulps lp.");
	} else if (val == 21) {
		iris_ulps_set(true, 0);
		IRIS_LOGI("enable iris ulps lp.");
	} else if (val == 255) {
		IRIS_LOGI("lp debug usages:");
		IRIS_LOGI("0  -- disable dpg & ulps lp");
		IRIS_LOGI("1  -- enable dpg & ulps lp");
		IRIS_LOGI("2  -- disable dpg");
		IRIS_LOGI("3  -- enable dpg");
		IRIS_LOGI("10  -- disable abyp lp");
		IRIS_LOGI("11  -- set abyp lp 1");
		IRIS_LOGI("12  -- set abyp lp 2");
		IRIS_LOGI("20  -- disable ulps lp");
		IRIS_LOGI("21  -- enable ulps lp");
		IRIS_LOGI("255 -- show debug usages.");
	}
	mutex_unlock(&pcfg->panel->panel_lock);
	return count;
}

static ssize_t _iris_lp_dbg_read(struct file *file, char __user *buff,
								 size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	int buf_len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	buf_len = sizeof(bp);

	if (*ppos)
		return 0;

	IRIS_LOGI("dynamic power: %d", iris_dynamic_power_get());
	IRIS_LOGI("abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	IRIS_LOGI("ulps enable: %d", iris_ulps_enable_get());

	tot = scnprintf(bp, buf_len,
					"dpg: %d\n", iris_dynamic_power_get());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "ulps enable: %d\n", iris_ulps_enable_get());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

int iris_dbg_init_lp(struct dsi_display *display)
{
	struct iris_cfg *pcfg;
	static const struct file_operations iris_abyp_dbg_fops = {
		.open = simple_open,
		.write = _iris_abyp_dbg_write,
		.read = _iris_abyp_dbg_read,
	};

	static const struct file_operations iris_lp_dbg_fops = {
		.open = simple_open,
		.write = _iris_lp_dbg_write,
		.read = _iris_lp_dbg_read,
	};

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					  PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("lp_opt", 0644, pcfg->dbg_root,
					   (u32 *)&debug_lp_opt);

	debugfs_create_u32("abyp_opt", 0644, pcfg->dbg_root,
					   (u32 *)&debug_on_opt);

	debugfs_create_u32("esd_ctrl", 0644, pcfg->dbg_root,
					   &(pcfg->lp_ctrl.esd_ctrl));

	debugfs_create_u32("esd_cnt_iris", 0644, pcfg->dbg_root,
					   (u32 *)&(pcfg->lp_ctrl.esd_cnt_iris));

	debugfs_create_u32("esd_cnt_panel", 0644, pcfg->dbg_root,
					   (u32 *)&(pcfg->lp_ctrl.esd_cnt_panel));

	debugfs_create_u32("dpp_only", 0644, pcfg->dbg_root,
					   (u32 *)&(pcfg->dpp_only_enable));

	debugfs_create_u8("read_path", 0644, pcfg->dbg_root,
					  (u8 *)&(pcfg->read_path));

	debugfs_create_u8("vc_enable", 0644, pcfg->dbg_root,
					  (u8 *)&(pcfg->vc_ctrl.vc_enable));

	if (debugfs_create_file("abyp", 0644, pcfg->dbg_root, display,
							&iris_abyp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				  __FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("lp", 0644, pcfg->dbg_root, display,
							&iris_lp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				  __FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

void iris_dsi_recover(void)
{
	SDE_ATRACE_BEGIN("iris_sys_powerdown_cmd");
	iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);
	iris_send_one_wired_cmd(IRIS_POWER_UP_SYS);
	SDE_ATRACE_BEGIN("iris_sys_powerup_cmd");
}

void iris_dump_regs(void)
{
	int val;
	int i;
	int regs[] = {0xf1100808, 0xf1100818, 0xf1100a00, 0xf1100034, 0xf1100204};

	for (i = 0; i < 5; i++) {
		val = 0;
		iris_ioctl_i2c_read(regs[i], &val);
		IRIS_LOGE("%s Read 0x%x = 0x%x", __func__, regs[i], val);
	}
}
