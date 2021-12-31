/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */

#ifndef _DSI_ANAKIN_H_
#define _DSI_ANAKIN_H_

#include "dsi_display.h"


#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
/* for panel_reg_rw parameters */
#define MIN_LEN 2
#define MAX_LEN 4
#define REG_BUF_SIZE 4096
#define PANEL_REGISTER_RW        "driver/panel_reg_rw"
#define PANEL_VENDOR_ID          "driver/panel_vendor_id"
#define PANEL_FPS                "driver/panel_fps"
#define LCD_UNIQUE_ID            "lcd_unique_id"
#define HBM_MODE                 "hbm_mode"
#define GLOBAL_HBM_MODE          "globalHbm"
#define DIMMING_SPEED            "lcd_dimming_speed"
#define LCD_BACKLIGNTNESS        "lcd_brightness"
#define MIPI_DSI_MSG_CMD_READ BIT(8)

/* for frame commit count */
#define COMMIT_FRAMES_COUNT 5

u32 dsi_anakin_support_cmd_read_flags(u32 flags);
void dsi_anakin_clear_commit_cnt(void);
void dsi_anakin_frame_commit_cnt(struct drm_crtc *crtc);
void dsi_anakin_display_init(struct dsi_display *display);
void dsi_anakin_parse_panel_vendor_id(struct dsi_panel *panel);
void dsi_anakin_set_panel_is_on(bool on);
void dsi_anakin_record_backlight(u32 bl_lvl);
u32 dsi_anakin_backlightupdate(u32 bl_lvl);
void dsi_anakin_restore_backlight(void);
void anakin_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val);
void anakin_crtc_display_commit(struct drm_crtc *crtc);
bool anakin_atomic_get_spot_status(int type);
void anakin_atomic_set_spot_status(int type);
bool anakin_get_charger_mode(void);

#else
static inline u32 dsi_anakin_support_cmd_read_flags(u32 flags){ return 0; }
static inline void dsi_anakin_clear_commit_cnt(void) {}
static inline void dsi_anakin_frame_commit_cnt(struct drm_crtc *crtc) {}
static inline void dsi_anakin_display_init(struct dsi_display *display) {}
static inline void dsi_anakin_parse_panel_vendor_id(struct dsi_panel *panel) {}
static inline void dsi_anakin_set_panel_is_on(bool on) {}
static inline void dsi_anakin_record_backlight(u32 bl_lvl) {}
static inline u32 dsi_anakin_backlightupdate(u32 bl_lvl) {return bl_lvl;}
static inline void dsi_anakin_restore_backlight(void) {}
static inline void anakin_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val) {}
static inline void anakin_crtc_display_commit(struct drm_crtc *crtc) {}
static inline bool anakin_atomic_get_spot_status(int type) { return false; }
static inline void anakin_atomic_set_spot_status(int type) {}
static inline bool anakin_get_charger_mode(void) { return false; }
#endif
#endif /* _DSI_ANAKIN_H_ */
