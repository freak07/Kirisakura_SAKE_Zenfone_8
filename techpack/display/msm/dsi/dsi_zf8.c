/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */

#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <drm/drm_zf8.h>

#include "dsi_zf8.h"

#if defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
struct dsi_display *g_display;
char g_reg_buffer[REG_BUF_SIZE];
int display_commit_cnt = COMMIT_FRAMES_COUNT;
extern char g_lcd_unique_id[10];
#if defined ASUS_SAKE_PROJECT
extern int g_lcd_stage_id;
#endif
// FOD feature
bool has_fod_masker;
bool old_has_fod_masker;
bool has_fod_spot;
bool old_has_fod_spot;
extern bool has_fod_spot;   //flag indicate the fod spot layer exist
extern int fod_spot_ui_ready; //flag indicate the fod spot has shown on screen

// check display or panal valid
static inline bool display_panel_valid(void)
{
	if (!g_display || !g_display->panel) {
		DSI_LOG("display or panel is not valid.\n");
		return false;
	}

	return true;
}

// write panel command
static void set_tcon_cmd(char *cmd, short len)
{
	int i = 0;
	ssize_t rc;
	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd = {0, 0x15, 0, 0, 0, len, cmd, 0, NULL};
	const struct mipi_dsi_host_ops *ops = g_display->panel->host->ops;

	if (!g_display || !g_display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	for(i=0; i<1; i++)
		DSI_LOG("cmd[%d] = 0x%02x\n", i, cmd[i]);

	if(len > 2)
		tcon_cmd.type = 0x39;

	mutex_lock(&g_display->display_lock);
	dsi_panel_acquire_panel_lock(g_display->panel);

	if (g_display->ctrl[0].ctrl->current_state.controller_state == DSI_CTRL_ENGINE_ON) {
		// cmds assigned
		cmds.msg = tcon_cmd;
		cmds.last_command = 1;
		cmds.post_wait_ms = 1;
		if (cmds.last_command) {
			cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		}

		rc = ops->transfer(g_display->panel->host, &cmds.msg);
		if (rc < 0) {
			DSI_LOG("tx cmd transfer failed rc=%d\n", rc);
		}
	} else {
		DSI_LOG("DSI_CTRL_ENGINE is off\n");
	}

	dsi_panel_release_panel_lock(g_display->panel);
	mutex_unlock(&g_display->display_lock);
}

// read panel command
static void get_tcon_cmd(char cmd, int rlen)
{
	char tmp[256];
	int i = 0, rc = 0, start = 0;
	u8 *tx_buf, *return_buf, *status_buf;

	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd = {0, 0x06, 0, 0, 0, sizeof(cmd), NULL, rlen, NULL};
	const struct mipi_dsi_host_ops *ops = g_display->panel->host->ops;

	if (!g_display || !g_display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	mutex_lock(&g_display->display_lock);
	dsi_panel_acquire_panel_lock(g_display->panel);

	if (g_display->ctrl[0].ctrl->current_state.controller_state == DSI_CTRL_ENGINE_ON) {
		// buffer assigned
		tx_buf = &cmd;
		return_buf = kcalloc(rlen, sizeof(unsigned char), GFP_KERNEL);
		status_buf = kzalloc(SZ_4K, GFP_KERNEL);
		memset(status_buf, 0x0, SZ_4K);

		// cmds assigned
		cmds.msg = tcon_cmd;
		cmds.last_command = 1;
		cmds.post_wait_ms = 0;
		if (cmds.last_command) {
			cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		}
		cmds.msg.flags |= MIPI_DSI_MSG_CMD_READ;

		cmds.msg.tx_buf = tx_buf;
		cmds.msg.rx_buf = status_buf;
		cmds.msg.rx_len = rlen;
		memset(g_reg_buffer, 0, REG_BUF_SIZE*sizeof(char));

		rc = ops->transfer(g_display->panel->host, &cmds.msg);
		if (rc <= 0) {
			DSI_LOG("rx cmd transfer failed rc=%d\n", rc);
		} else {
			memcpy(return_buf + start, status_buf, rlen);
			start += rlen;

			for(i=0; i<rlen; i++) {
				memset(tmp, 0, 256*sizeof(char));
				snprintf(tmp, sizeof(tmp), "0x%02x = 0x%02x\n", cmd, return_buf[i]);
				strcat(g_reg_buffer,tmp);
			}
		}
	} else {
		DSI_LOG("DSI_CTRL_ENGINE is off\n");
	}

	dsi_panel_release_panel_lock(g_display->panel);
	mutex_unlock(&g_display->display_lock);
}

static int dsi_zf8_tx_cmd_set(struct dsi_panel *panel,
				enum dsi_cmd_set_type type)
{
	int rc = 0, i = 0;
	ssize_t len;
	struct dsi_cmd_desc *cmds;
	u32 count;
	enum dsi_cmd_set_state state;
	struct dsi_display_mode *mode;
	const struct mipi_dsi_host_ops *ops = panel->host->ops;

	if (!panel || !panel->cur_mode)
		return -EINVAL;

	mode = panel->cur_mode;

	cmds = mode->priv_info->cmd_sets[type].cmds;
	count = mode->priv_info->cmd_sets[type].count;
	state = mode->priv_info->cmd_sets[type].state;

	if (count == 0) {
		DSI_LOG("[%s] No commands to be sent for state(%d)\n",
			 panel->name, type);
		goto error;
	}

	for (i = 0; i < count; i++) {
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		if (type == DSI_CMD_SET_VID_TO_CMD_SWITCH)
			cmds->msg.flags |= MIPI_DSI_MSG_ASYNC_OVERRIDE;

		len = ops->transfer(panel->host, &cmds->msg);
		if (len < 0) {
			rc = len;
			DSI_LOG("failed to set cmds(%d), rc=%d\n", type, rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms*1000,
					((cmds->post_wait_ms*1000)+10));
		cmds++;
	}
error:
	return rc;
}

// send HBM command to panel
static int dsi_zf8_set_hbm(struct dsi_panel *panel, int enable)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (1 == enable) {
#if defined ASUS_SAKE_PROJECT
		if(1 == g_lcd_stage_id) {
			DSI_LOG("[%s] send DSI_CMD_SET_HBM_ON cmd \n",panel->name);
			rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_HBM_ON);
		} else {
			DSI_LOG("[%s] send DSI_CMD_SET_HBM_ER2_ON cmd \n",panel->name);
			rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_HBM_ER2_ON);
		}
#else
		rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_HBM_ON);
#endif
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_ON cmd, rc=%d\n",
				   panel->name, rc);
	}
#if defined ASUS_SAKE_PROJECT
	else if(2 == enable) {
		if(1 == g_lcd_stage_id) {
			DSI_LOG("[%s] send DSI_CMD_SET_ER1_CAM_HBM_ON cmd \n",panel->name);
			rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_ER1_CAM_HBM_ON);
		} else {
			DSI_LOG("[%s] send DSI_CMD_SET_ER2_CAM_HBM_ON cmd \n",panel->name);
			rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_ER2_CAM_HBM_ON);
		}
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_CAM_HBM_ON cmd, rc=%d\n",
				   panel->name, rc);
	}
#endif
	else {
		rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_HBM_OFF);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_ON cmd, rc=%d\n",
				   panel->name, rc);
	}
exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}
// called for /proc/hbm_mode
static void display_set_hbm_mode(int mode)
{
	if (mode == g_display->panel->panel_hbm_mode) {
		DSI_LOG("hbm mode same.\n");
		return;
	}

	g_display->panel->panel_hbm_mode = mode;
	
	DSI_LOG("hbm mode is (%d)\n", g_display->panel->panel_hbm_mode);
	dsi_zf8_set_hbm(g_display->panel, mode);
}

// panel fps read
static ssize_t panel_fps_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	DSI_LOG("refreshrate  %d\n",g_display->panel->cur_mode->timing.refresh_rate);
	buff = kzalloc(256, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", g_display->panel->cur_mode->timing.refresh_rate);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations panel_fps_ops = {
	.read = panel_fps_read,
};

// panel_vendor_id_ops() - show panel vendor id
static ssize_t panel_vendor_id_show(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(SZ_4K, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;
	len += sprintf(buff, "%s\n", g_display->panel->panel_vendor_id);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations panel_vendor_id_ops = {
	.read = panel_vendor_id_show,
};

// panel uid read
static ssize_t lcd_unique_id_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(256, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", g_lcd_unique_id);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations lcd_unique_id_ops = {
	.read = lcd_unique_id_read,
};

// exit idle mode
static void display_exit_idle_mode()
{
	int rc = 0;
		
	//#define SDE_MODE_DPMS_LP1	1      sde_drm.h
	//#define  SDE_MODE_DPMS_LP2    2
	DSI_LOG("power_mode/aod_state are (%d, %d)\n",
			g_display->panel->power_mode, g_display->panel->aod_state);
	
	if ((g_display->panel->power_mode != 1 && g_display->panel->power_mode != 2) && !g_display->panel->aod_state) {
		DSI_LOG("return display_exit_idle_mode!\n");
		return;
	}

	if (!g_display->panel->has_enter_aod_before && !g_display->panel->allow_panel_fod_hbm) {
		DSI_LOG("Not has_enter_aod_before return display_exit_idle_mode!\n");
		return;
	}
	
	DSI_LOG("Will enter NOLP mode !\n");
	
#if defined ASUS_SAKE_PROJECT
	if(1 == g_lcd_stage_id) {
		rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_NOLP);
		g_display->panel->has_enter_aod_before = false;
	} else {
		if(g_display->panel->panel_fod_hbm_mode == 0) {
			DSI_LOG("Send DSI_CMD_SET_ER2_NOLP\n");
			rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_ER2_NOLP);
			g_display->panel->has_enter_aod_before = false;
		}else {
			DSI_LOG("Send DSI_CMD_SET_ER2_FOD_NOLP\n");
			rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_ER2_FOD_NOLP);
			g_display->panel->has_enter_aod_before = false;
		}
	}
#endif

#if defined ASUS_VODKA_PROJECT
	if(g_display->panel->panel_fod_hbm_mode == 0) {
		DSI_LOG("Send DSI_CMD_SET_NOLP\n");
		rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_NOLP);
		g_display->panel->has_enter_aod_before = false;
	}else {
		DSI_LOG("Send DSI_CMD_SET_ER2_FOD_NOLP\n");
		rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_ER2_FOD_NOLP);
		g_display->panel->has_enter_aod_before = false;
	}
#endif

	if (rc) {
		DSI_LOG("[%s] failed to send DSI_CMD_SET_NOLP cmd, rc=%d\n",
			   g_display->panel->name, rc);
		g_display->panel->has_enter_aod_before = true;
	}  else {
		DSI_LOG("enter NOLP mode sucess , set fod_in_doze true \n");
		g_display->panel->fod_in_doze = true;
		g_display->panel->aod_state = false;
	}
}

// send Fod HBM command to panel
static int dsi_zf8_set_fod_hbm(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel) {
		pr_err("[Display] : invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;
	
	if (enable) {
		// exit idle mode if enter doze before
		display_exit_idle_mode();

	DSI_LOG("Will Set FOD HBM ON\n");

#if defined ASUS_SAKE_PROJECT	
		// to aviod ghbm without mask
		if (panel->fod_in_doze) {
			DSI_LOG("set display off first\n");
			rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_AOD_OFF);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_LP1 cmd, rc=%d\n",
						panel->name, rc);
		}else {
			DSI_LOG("set display on directly\n");

			if(1 == g_lcd_stage_id) {
				rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_FOD_HBM_ON);
			}else {
				rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_FOD_ER2_HBM_ON);
			}
		}
#endif

#if defined ASUS_VODKA_PROJECT
		rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_FOD_HBM_ON);
#endif

		if (rc)
			pr_err("[Display][%s] failed to send DSI_CMD_SET_FOD_HBM_ON cmd, rc=%d\n",
				panel->name, rc);
	} else {
		dsi_zf8_restore_backlight();

		DSI_LOG("Will Set FOD HBM OFF\n");
		rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_FOD_HBM_OFF);
		if (rc)
			pr_err("[Display][%s] failed to send DSI_CMD_SET_FOD_HBM_OFF cmd, rc=%d\n",
				   panel->name, rc);

		//panel->allow_dimming_smooth = true;
	}

exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

// called for /proc/globalHbm
static void display_set_fod_hbm_mode(int mode)
{
	if (mode == g_display->panel->panel_fod_hbm_mode) {
		DSI_LOG("fod hbm mode same.\n");
		return;
	}

	g_display->panel->panel_fod_hbm_mode = mode;
	DSI_LOG("fod hbm mode is (%d)\n", g_display->panel->panel_fod_hbm_mode);
	dsi_zf8_set_fod_hbm(g_display->panel, mode);
}

// called for FOD flow, as /proc/globalHbm
static void display_set_fod_hbm(void)
{
	if (!display_panel_valid())
		return;

	if (!g_display->panel->panel_is_on) {
		DSI_LOG("display is off.\n");
		return;
	}

	//pr_err(" [Display] : FOD: global hbm <fod> (%d) +++ \n", g_display->panel->allow_panel_fod_hbm);
	dsi_zf8_set_fod_hbm(g_display->panel, g_display->panel->allow_panel_fod_hbm);
	zf8_drm_notify(ASUS_NOTIFY_GHBM_ON_REQ, g_display->panel->allow_panel_fod_hbm);
	//pr_err("[Display] : FOD: global hbm <fod> (%d) --- \n", g_display->panel->allow_panel_fod_hbm);

	// reset fod hbm pending immediately, need be guard by lock
	g_display->panel->allow_fod_hbm_process = false;
}

//send Dimming Smooth command to panel
void dsi_zf8_set_dimming_smooth(struct dsi_panel *panel, u32 backlight)
{
	int rc = 0;

	// don't allow in fod hbm
	if (panel->allow_panel_fod_hbm == 1)
		return;

	if (panel->aod_state ||panel->allow_fod_hbm_process || backlight == 0) {
		panel->panel_bl_count = 0;
		return;
	}

	if (panel->panel_bl_count == 1) {
		
		if(!panel->dc_fps_change) {
			DSI_LOG("restore dimming smooth\n");
			rc = dsi_zf8_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SMOOTH);
			if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_DIMMING_SMOOTH cmd, rc=%d\n",
				   panel->name, rc);
		}else {
			panel->dc_fps_change = false;
		}
		
		return;
	}
	
	panel->panel_bl_count++;

}

// panel_reg_rw_ops() - read/write/show panel register
static ssize_t panel_reg_rw(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *messages, *tmp, *cur;
	char *token, *token_par;
	char *put_cmd;
	bool flag = 0; /* w/r type : w=1, r=0 */
	int *store;
	int i = 0, cnt = 0, cmd_cnt = 0;
	int ret = 0;
	uint8_t str_len = 0;

	pr_err("[Display] : panel_reg_rw \n");
	messages = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
	if(!messages)
		return -EFAULT;

	tmp = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
	memset(tmp, 0, len*sizeof(char));
	store =  (int*) kmalloc((len/MIN_LEN)*sizeof(int), GFP_KERNEL);
	put_cmd = (char*) kmalloc((len/MIN_LEN)*sizeof(char), GFP_KERNEL);
	memset(g_reg_buffer, 0, REG_BUF_SIZE*sizeof(char));

	/* add '\0' to end of string */
	if (copy_from_user(messages, buff, len)) {
		ret = -1;
		goto error;
	}

	cur = messages;
	*(cur+len-1) = '\0';
	pr_err("[Display] : %s +++\n", cur);

	if (strncmp(cur, "w", 1) == 0)
		flag = true;
	else if(strncmp(cur, "r", 1) == 0)
		flag = false;
	else {
		ret = -1;
		goto error;
	}

	while ((token = strsep(&cur, "wr")) != NULL) {
		str_len = strlen(token);

		if(str_len > 0) { /* filter zero length */
			if(!(strncmp(token, ",", 1) == 0) || (str_len < MAX_LEN)) {
				ret = -1;
				goto error;
			}

			memset(store, 0, (len/MIN_LEN)*sizeof(int));
			memset(put_cmd, 0, (len/MIN_LEN)*sizeof(char));
			cmd_cnt++;

			/* register parameter */
			while ((token_par = strsep(&token, ",")) != NULL) {
				if(strlen(token_par) > MIN_LEN) {
					ret = -1;
					goto error;
				}
				if(strlen(token_par)) {
					sscanf(token_par, "%x", &(store[cnt]));
					cnt++;
				}
			}

			for(i=0; i<cnt; i++)
				put_cmd[i] = store[i]&0xff;

			if(flag) {
				pr_err("[Display] : write panel command\n");
				set_tcon_cmd(put_cmd, cnt);
			}
			else {
				pr_err("[Display] : read panel command\n");
				get_tcon_cmd(put_cmd[0], store[1]);
			}

			if(cur != NULL) {
				if (*(tmp+str_len) == 'w')
					flag = true;
				else if (*(tmp+str_len) == 'r')
					flag = false;
			}
			cnt = 0;
		}

		memset(tmp, 0, len*sizeof(char));

		if(cur != NULL)
			strcpy(tmp, cur);
	}

	if(cmd_cnt == 0) {
		ret = -1;
		goto error;
	}

	ret = len;

error:
	pr_err("[Display] : len = %d ---\n", ret);
	kfree(messages);
	kfree(tmp);
	kfree(store);
	kfree(put_cmd);
	return ret;
}

static ssize_t panel_reg_show(struct file *file, char __user *buf,
                 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(SZ_4K, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", g_reg_buffer);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations panel_reg_rw_ops = {
	.write = panel_reg_rw,
	.read = panel_reg_show,
};

// hbm_mode_ops() - set HBM on/off & read HBM status
static ssize_t hbm_mode_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EINVAL;

	if (g_display->panel->panel_is_on &&
			(g_display->panel->allow_panel_fod_hbm == 0)) {
		if (strncmp(messages, "0", 1) == 0) {
			display_set_hbm_mode(0);
		} else if (strncmp(messages, "1", 1) == 0) {
			display_set_hbm_mode(1);
		} 
#if defined ASUS_SAKE_PROJECT
		else if (strncmp(messages, "2", 1) == 0) {
			display_set_hbm_mode(2);
		}
#endif
		else {
			DSI_LOG("don't match any hbm mode.\n");
		}
	} else {
		DSI_LOG("unable to set in display off\n");
		g_display->panel->panel_hbm_mode = 0;
	}

	return len;
}

static ssize_t hbm_mode_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	if (*ppos)
		return 0;

	DSI_LOG("hbm mode is %d\n", g_display->panel->panel_hbm_mode);

	len += sprintf(buff, "%d\n", g_display->panel->panel_hbm_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations hbm_mode_ops = {
	.write = hbm_mode_write,
	.read  = hbm_mode_read,
};

static ssize_t dimming_speed_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int rc = 0;
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EFAULT;

	DSI_LOG("dimming speed write +++ val:%s \n",messages);

	if (g_display->panel->panel_is_on) {
		if (strncmp(messages, "1", 1) == 0) {
			rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_DIMMING_SPEED_1);
		} else if (strncmp(messages, "20", 20) == 0) {
			rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_DIMMING_SPEED_20);
		} else {
			DSI_LOG("doesn't match any dimming speed .\n");
		}
	} else {
		DSI_LOG("unable to set in display off\n");
	}

	if(rc < 0) {
		DSI_LOG("panel set dimming speed failed\n");
	}

	DSI_LOG("dimming speed write --- \n");

	return len;
}

static struct file_operations dimming_speed_ops = {
	.write = dimming_speed_write,
};

static ssize_t lcd_brightness_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	u32 backlight_lvl = 0;
	int rc = 0;

	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EFAULT;

	DSI_LOG("dc lcd brightess write +++ val:%s \n",messages);

	sscanf(messages, "%u", &backlight_lvl);

	DSI_LOG("dc lcd brightess need change backlightnes : %d\n",backlight_lvl);
	// restrict backlight range 1 - 1023
	if (g_display->panel->panel_is_on && backlight_lvl > 0 && backlight_lvl < 1024) {
		if(backlight_lvl < 248) {
			DSI_LOG("FPS Change restore brightness < 248!\n");
			g_display->panel->dc_fps_change = true ;
		}else {
			DSI_LOG("FPS Change restore brightness >= 248!\n");
			g_display->panel->dc_fps_change = false;
		}
		
		rc =  dsi_panel_set_backlight(g_display->panel,backlight_lvl);
		if(rc < 0)
			DSI_LOG("dsi_panel_update_backlight error! \n");
	}
	else {
		DSI_LOG("unable to set in display off\n");
	}
	DSI_LOG("dc lcd brightess --- \n");

	return len;
}

static struct file_operations lcd_brightness_ops = {
	.write = lcd_brightness_write,
};
// global_hbm_mode_ops() - set Fod HBM on/off & read Fod HBM status
static ssize_t global_hbm_mode_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EINVAL;

	if (g_display->panel->panel_is_on) {
		if (strncmp(messages, "0", 1) == 0) {
			display_set_fod_hbm_mode(0);
		} else if (strncmp(messages, "1", 1) == 0) {
			display_set_fod_hbm_mode(1);
		} else {
			DSI_LOG("don't match any hbm mode.\n");
		}
	} else {
		DSI_LOG("unable to set in display off\n");
		g_display->panel->panel_fod_hbm_mode = 0;
	}

	return len;
}

static ssize_t global_hbm_mode_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	if (*ppos)
		return 0;

	DSI_LOG("fod hbm mode is %d\n", g_display->panel->panel_fod_hbm_mode);

	len += sprintf(buff, "%d\n", g_display->panel->panel_fod_hbm_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations global_hbm_mode_ops = {
	.write = global_hbm_mode_write,
	.read  = global_hbm_mode_read,
};

#if defined ASUS_VODKA_PROJECT
int hbm_mode_delay_num = 10000;
#else
int hbm_mode_delay_num = 0;
#endif

static ssize_t global_hbm_mode_delay_read(struct file *file, char __user *user_buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	if (*ppos)
		return 0;

	DSI_LOG("hbm_mode_delay_num is %d\n", hbm_mode_delay_num);

	len += sprintf(buff, "%d\n", hbm_mode_delay_num);
	ret = simple_read_from_buffer(user_buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

// global_hbm_mode_ops() - set Fod HBM on/off & read Fod HBM status
static ssize_t global_hbm_mode_delay_write(struct file *filp, const char *user_buf, size_t user_len, loff_t *ppos)
{
	char *buf;
 	int rc = 0;
  	size_t len;
 	unsigned int new_delay;
  
  	if (*ppos)
 		return 0;
  
  	buf = kzalloc(255, GFP_KERNEL);
  	if (!buf)
  		return -ENOMEM;
  
 	/* leave room for termination char */
  	len = min_t(size_t, user_len, 255);
 	if (copy_from_user(buf, user_buf, len)) {
  		rc = -EINVAL;
  		goto error;
  	}
  
  	buf[len] = '\0'; /* terminate the string */
  
  	if (sscanf(buf, "%d", &new_delay) != 1) {
  		rc = -EINVAL;
  		goto error;
  	}
        hbm_mode_delay_num = (int)new_delay;
 	rc = user_len;
  
error:
  	kfree(buf);
  	return rc;
}

static struct file_operations global_hbm_mode_delay_ops = {
	.write = global_hbm_mode_delay_write,
	.read  = global_hbm_mode_delay_read,
};


int hbm_mode_count = 0;

static ssize_t hbm_mode_count_read(struct file *file, char __user *user_buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	if (*ppos)
		return 0;

	DSI_LOG("hbm_mode_count is %d\n", hbm_mode_count);

	len += sprintf(buff, "%d\n", hbm_mode_count);
	ret = simple_read_from_buffer(user_buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

// global_hbm_mode_ops() - set Fod HBM on/off & read Fod HBM status
static ssize_t hbm_mode_count_write(struct file *filp, const char *user_buf, size_t user_len, loff_t *ppos)
{
	char *buf;
 	int rc = 0;
  	size_t len;
 	unsigned int new_delay;
  
  	if (*ppos)
 		return 0;
  
  	buf = kzalloc(255, GFP_KERNEL);
  	if (!buf)
  		return -ENOMEM;
  
 	/* leave room for termination char */
  	len = min_t(size_t, user_len,255);
 	if (copy_from_user(buf, user_buf, len)) {
  		rc = -EINVAL;
  		goto error;
  	}
  
  	buf[len] = '\0'; /* terminate the string */
  
  	if (sscanf(buf, "%d", &new_delay) != 1) {
  		rc = -EINVAL;
  		goto error;
  	}
        hbm_mode_count = (int)new_delay;
 	rc = user_len;
  
error:
  	kfree(buf);
  	return rc;
}

static struct file_operations hbm_mode_count_ops = {
	.write = hbm_mode_count_write,
	.read  = hbm_mode_count_read,
};

// to support DSI_CTRL_CMD_READ if MIPI_DSI_MSG_CMD_READ is enabled
u32 dsi_zf8_support_cmd_read_flags(u32 flags)
{
	u32 ret_flags = 0;

	if (flags & MIPI_DSI_MSG_CMD_READ) {
		ret_flags |= (DSI_CTRL_CMD_LAST_COMMAND | DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ);
		DSI_LOG("DSI_CTRL_CMD is 0x%x\n", ret_flags);
	}

	return ret_flags;
}

// to show & clear frame commit count
void dsi_zf8_clear_commit_cnt(void)
{
	display_commit_cnt = COMMIT_FRAMES_COUNT;
}

void dsi_zf8_frame_commit_cnt(struct drm_crtc *crtc)
{
	//static int fod_off_frame_cnt = 0;

	if (display_commit_cnt > 0 && !strcmp(crtc->name, "crtc-0")) {
		DSI_LOG("fbc%d\n", display_commit_cnt);
		display_commit_cnt--;
	}


	/*if (g_display->panel->allow_dimming_smooth && fod_off_frame_cnt <= 2) {
		fod_off_frame_cnt++;

		if (fod_off_frame_cnt == 2) {
			DSI_LOG("restore dimming smooth\n");
			dsi_zf8_set_dimming_smooth(g_display->panel);
			fod_off_frame_cnt = 0;
		}
	}*/

	if(g_display->panel->aod_state && display_commit_cnt < 1 && g_display->panel->aod_first_time) {
		DSI_LOG("AOD notification case restore bl = %d\n", g_display->panel->panel_last_backlight);
		dsi_zf8_restore_backlight();
		g_display->panel->aod_first_time = false;
	}
}

// to initial asus display parameters
void dsi_zf8_display_init(struct dsi_display *display)
{
	DSI_LOG("dsi_zf8_display_init  !\n ");
	g_display = display;
	dsi_zf8_parse_panel_vendor_id(g_display->panel);

	g_display->panel->panel_hbm_mode = 0;
	g_display->panel->panel_fod_hbm_mode = 0;
	g_display->panel->allow_panel_fod_hbm = 0;
	g_display->panel->allow_fod_hbm_process = false;
	//g_display->panel->allow_dimming_smooth = false;
	g_display->panel->panel_is_on = false;
	g_display->panel->panel_last_backlight = 0;
	g_display->panel->aod_state = false;
	g_display->panel->aod_first_time = false;
	g_display->panel->has_enter_aod_before = false;
	g_display->panel->fod_in_doze = false;
	g_display->panel->panel_bl_count = 0;
	g_display->panel->aod_mode = 0;
	g_display->panel->dc_fps_change = false;
	
	proc_create(PANEL_REGISTER_RW, 0640, NULL, &panel_reg_rw_ops);
	proc_create(PANEL_VENDOR_ID, 0640, NULL, &panel_vendor_id_ops);
	proc_create(PANEL_FPS, 0660, NULL, &panel_fps_ops);
	proc_create(LCD_UNIQUE_ID, 0444, NULL, &lcd_unique_id_ops);
	proc_create(HBM_MODE, 0666, NULL, &hbm_mode_ops);
	proc_create(GLOBAL_HBM_MODE, 0666, NULL, &global_hbm_mode_ops);
	proc_create(DIMMING_SPEED, 0666, NULL, &dimming_speed_ops);
	proc_create(LCD_BACKLIGNTNESS, 0666, NULL, &lcd_brightness_ops);
	proc_create(GLOBAL_HBM_MODE_DELAY, 0666, NULL, &global_hbm_mode_delay_ops);
	proc_create(HBM_MODE_COUNT, 0666, NULL, &hbm_mode_count_ops);
}

// to parse panel_vendor_id from panel dtsi
void dsi_zf8_parse_panel_vendor_id(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = &panel->utils;

	panel->panel_vendor_id = utils->get_property(utils->data,
			"qcom,mdss-dsi-panel-vendor-id", NULL);
	DSI_LOG("panel vendor id = %s", panel->panel_vendor_id);
}

// to store panel status & reset display parameter
void dsi_zf8_set_panel_is_on(bool on)
{

     // DSI_LOG("dsi_zf8_set_panel_is_on  !\n ");
	g_display->panel->panel_is_on = on;

	if (on == false) {
		DSI_LOG("dsi_zf8_set_panel_is_on  false !\n ");
		g_display->panel->panel_hbm_mode = 0;
		g_display->panel->panel_fod_hbm_mode = 0;
		g_display->panel->allow_panel_fod_hbm = 0;
		g_display->panel->allow_fod_hbm_process = false;
		//g_display->panel->allow_dimming_smooth = false;
		g_display->panel->fod_in_doze = false;
		g_display->panel->aod_state = false;
		g_display->panel->panel_bl_count = 0;
		has_fod_masker = false;
		old_has_fod_masker = false;
		has_fod_spot = false;
		old_has_fod_spot = false;
		g_display->panel->dc_fps_change = false;
		zf8_drm_notify(ASUS_NOTIFY_SPOT_READY, 0);
		zf8_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 0);
		zf8_drm_notify(ASUS_NOTIFY_GHBM_ON_REQ, 0);
	}
}

// to record & restore user's last backlight
void dsi_zf8_record_backlight(u32 bl_lvl)
{
	int rc = 0;
	if (bl_lvl == 0) {
#if defined ASUS_VODKA_PROJECT || defined ASUS_SAKE_PROJECT
		if ((g_display->panel->power_mode == 1  || g_display->panel->power_mode == 2)&& !g_display->panel->allow_panel_fod_hbm) {
			DSI_LOG("Display off first!\n");
			dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_AOD_OFF);
		}
#endif
		return;
	}

	g_display->panel->panel_last_backlight = bl_lvl;
	//#define SDE_MODE_DPMS_LP1	1      sde_drm.h
	// skip if fod hbm is processing
	if ((g_display->panel->power_mode == 1  || g_display->panel->power_mode == 2)&& !g_display->panel->allow_panel_fod_hbm) {
		
		if(g_display->panel->panel_last_backlight == 248) {
			if(g_display->panel->aod_mode == 1)
				g_display->panel->panel_last_backlight = 4;
			else if(g_display->panel->aod_mode == 2)
				g_display->panel->panel_last_backlight = 64;
			else 
				DSI_LOG("Can not run here , This is a Bug!\n");
		}
		
		//DSI_LOG("AOD last_backlight = %d \n", g_display->panel->panel_last_backlight);
		DSI_LOG("Will enter AOD Mode !\n");
		if(g_display->panel->panel_last_backlight > 4) {
			
#if defined ASUS_SAKE_PROJECT
			if(1 == g_lcd_stage_id) {
				rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_HIGH);
				g_display->panel->has_enter_aod_before = true;
			}else {
				rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_ER2_HIGH);
				g_display->panel->has_enter_aod_before = true;
			}
#else
			rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_HIGH);
			g_display->panel->has_enter_aod_before = true;
#endif
			DSI_LOG("set aod_mode 2 \n");
			g_display->panel->aod_mode = 2;
	     } else if (g_display->panel->panel_last_backlight == 4) {
#if defined ASUS_SAKE_PROJECT
			if(1 == g_lcd_stage_id) {
					rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_LOW);
					g_display->panel->has_enter_aod_before = true;
			}else {
					rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_ER2_LOW);
					g_display->panel->has_enter_aod_before = true;
			}
#else
			rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_LOW);
			g_display->panel->has_enter_aod_before = true;
#endif
			DSI_LOG("set aod_mode 1 \n");
			g_display->panel->aod_mode = 1;
	    }
		// for non 4 / 64 bl && aod on state, prevent display keep off
		else if(g_display->panel->aod_state){
			DSI_LOG("Send DSI_CMD_SET_AOD_OTHER !\n");
			rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_OTHER);
			g_display->panel->has_enter_aod_before = false;
			g_display->panel->aod_mode = 0;
		}
		
		if(rc) {
			DSI_LOG("unable to set AOD command\n");
			g_display->panel->has_enter_aod_before = false;
			g_display->panel->aod_mode = 0;
		} else {    // if AOD cmd set sucess, set up first_time as true
			g_display->panel->aod_first_time = true;
		}
	}
}

void dsi_zf8_restore_backlight(void)
{
	int rc = 0;

	pr_err("[Display] restore bl=%d\n", g_display->panel->panel_last_backlight);
	rc = dsi_panel_set_backlight(g_display->panel, g_display->panel->panel_last_backlight);
	if (rc)
		pr_err("[Display] unable to set backlight\n");
}

// called from sde_crtc_atomic_set_property, ready for FOD ON/OFF
void zf8_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val)
{
	if (!strcmp(crtc->name, "crtc-0")) {
		switch (idx) {
		case CRTC_PROP_FOD_MASKER:
			has_fod_masker = val;
			if (old_has_fod_masker == false && has_fod_masker == true) {
				g_display->panel->allow_panel_fod_hbm = 1;
				g_display->panel->allow_fod_hbm_process = true;
				DSI_LOG("FOD MASKER:OFF->ON");
			} else if (old_has_fod_masker == true && has_fod_masker == false) {
				g_display->panel->allow_panel_fod_hbm = 0;
				g_display->panel->allow_fod_hbm_process = true;
				DSI_LOG("FOD MASKER:ON->OFF");
			}
			old_has_fod_masker = has_fod_masker;
			break;
		case CRTC_PROP_FOD_SPOT:
			has_fod_spot = val;
			if (old_has_fod_spot == false && has_fod_spot == true) {
				DSI_LOG("FOD SPOT:OFF->ON");
			} else if (old_has_fod_spot == true && has_fod_spot == false) {
				DSI_LOG("FOD SPOT:ON->OFF");
			}
			old_has_fod_spot = has_fod_spot;
			break;
		default:
			break;
		}
	}
}

// called from sde_crtc_commit_kickoff, to enable panel global HBM
void zf8_crtc_display_commit(struct drm_crtc *crtc)
{
	if (g_display->panel->allow_fod_hbm_process && !strcmp(crtc->name, "crtc-0")) {
		DSI_LOG("FOD HBM setting +++\n");

		if (g_display->panel->allow_panel_fod_hbm == 1) {
			display_set_fod_hbm();
			// need delay time, waiting for fine tune
			zf8_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 1);
			g_display->panel->panel_fod_hbm_mode = 1;
			DSI_LOG("panel_fod_hbm_mode set to 1");
		} else if (g_display->panel->allow_panel_fod_hbm == 0) {
#if defined ASUS_SAKE_PROJECT
			if (g_display->panel->cur_mode->timing.refresh_rate == 60) {
				// need delay time, waiting for fine tune
				DSI_LOG("Delay 10ms for 60 fps");
				udelay(10000);
			}
#endif
			display_set_fod_hbm();
			// need delay time, waiting for fine tune
			udelay(hbm_mode_delay_num);
			zf8_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 0);
			g_display->panel->panel_fod_hbm_mode = 0;
			DSI_LOG("panel_fod_hbm_mode set to 0");
		}
		DSI_LOG("FOD HBM setting ---\n");
	}
}

// called from _msm_drm_commit_work_cb, to notify spot ready
// type 0: commit_for_fod_spot
//      1: report_fod_spot_disappear
bool zf8_atomic_get_spot_status(int type)
{
	if (type == 0) {
		if (has_fod_spot && !fod_spot_ui_ready) {
			DSI_LOG("commit FOD spot to panel +++ \n");
			return true;
		}
	} else if (type == 1) {
		if (!has_fod_spot && fod_spot_ui_ready)
			return true;
	}

	return false;
}

void zf8_atomic_set_spot_status(int type)
{
#if defined ASUS_SAKE_PROJECT
	int rc = 0;
#endif

	if (type == 0) {
		DSI_LOG("commit FOD spot to panel --- \n");
#if defined ASUS_SAKE_PROJECT
		if (g_display->panel->fod_in_doze) {
				rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_AOD_OTHER);		

				if(1 == g_lcd_stage_id) {
					rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_FOD_HBM_ON);
				}else {
					rc = dsi_zf8_tx_cmd_set(g_display->panel, DSI_CMD_SET_FOD_ER2_HBM_ON);
				}
				
				if (rc) {
					DSI_LOG("[%s] failed to send DSI_CMD_SET_POST_FOD_HBM_ON cmd, rc=%d\n",
					  	 g_display->panel->name, rc);
				} else {
					g_display->panel->fod_in_doze = false;
					zf8_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
				}
		} else {
			zf8_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
		}
#else
		zf8_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
#endif
	} else if (type == 1) {
		zf8_drm_notify(ASUS_NOTIFY_SPOT_READY, 0);
		DSI_LOG("removed fod spot \n");
	}
}

// add for set min backlight to 2
u32 dsi_zf8_backlightupdate(u32 bl_lvl){
	if (bl_lvl == 1) {
		return 2;
	}
	else {
		return bl_lvl;
	}
}

void dsi_zf8_sake_power_on_delay(void)
{

#if defined ASUS_SAKE_PROJECT
	DSI_LOG("Sake display use this 10 ms delay\n");
	msleep(10);
#endif

#if defined ASUS_VODKA_PROJECT
	DSI_LOG("Vodka display ignore this delay\n");
#endif

}

#endif

