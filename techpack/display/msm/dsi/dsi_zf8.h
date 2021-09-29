/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */

#ifndef _DSI_ZF8_H_
#define _DSI_ZF8_H_

#include "dsi_display.h"

#if defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
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
#define GLOBAL_HBM_MODE_DELAY          "Hbm_delay"
#define HBM_MODE_COUNT              "Hbm_count"

#define MIPI_DSI_MSG_CMD_READ BIT(8)

/* for frame commit count */
#define COMMIT_FRAMES_COUNT 5

u32 dsi_zf8_support_cmd_read_flags(u32 flags);
void dsi_zf8_clear_commit_cnt(void);
void dsi_zf8_frame_commit_cnt(struct drm_crtc *crtc);
void dsi_zf8_display_init(struct dsi_display *display);
void dsi_zf8_parse_panel_vendor_id(struct dsi_panel *panel);
void dsi_zf8_set_panel_is_on(bool on);
void dsi_zf8_record_backlight(u32 bl_lvl);
u32 dsi_zf8_backlightupdate(u32 bl_lvl);
void dsi_zf8_restore_backlight(void);
void zf8_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val);
void zf8_crtc_display_commit(struct drm_crtc *crtc);
bool zf8_atomic_get_spot_status(int type);
void zf8_atomic_set_spot_status(int type);
void dsi_zf8_sake_power_on_delay(void);
void dsi_zf8_set_dimming_smooth(struct dsi_panel *panel, u32 backlight);
#else

static inline u32 dsi_zf8_support_cmd_read_flags(u32 flags) { return 0; }
static inline void dsi_zf8_clear_commit_cnt(void) {}
static inline void dsi_zf8_frame_commit_cnt(struct drm_crtc *crtc) {}
static inline void dsi_zf8_display_init(struct dsi_display *display) {}
static inline void dsi_zf8_parse_panel_vendor_id(struct dsi_panel *panel) {}
static inline void dsi_zf8_set_panel_is_on(bool on) {}
static inline void dsi_zf8_record_backlight(u32 bl_lvl) {}
static inline u32 dsi_zf8_backlightupdate(u32 bl_lvl) {return bl_lvl;}
static inline void dsi_zf8_restore_backlight(void) {}
static inline void zf8_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val) {}
static inline void zf8_crtc_display_commit(struct drm_crtc *crtc) {}
static inline bool zf8_atomic_get_spot_status(int type) { return false; }
static inline void zf8_atomic_set_spot_status(int type) {}
static inline void dsi_zf8_sake_power_on_delay(void) {}
static inline void dsi_zf8_set_dimming_smooth(struct dsi_panel *panel, u32 backlight) {}
#endif

#endif /* _DSI_ZF8_H_ */
