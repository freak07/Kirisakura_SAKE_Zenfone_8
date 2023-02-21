/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */

#ifndef _DRM_ZF8_H_
#define _DRM_ZF8_H_

#if defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT

#define ASUS_NOTIFY_GHBM_ON_REQ        0
#define ASUS_NOTIFY_GHBM_ON_READY      1
#define ASUS_NOTIFY_SPOT_READY         2
#define ASUS_NOTIFY_FOD_TOUCHED        3

void zf8_drm_notify(int var, int value);
void drm_zf8_sysfs_destroy(void);
int drm_zf8_sysfs_init(void);
bool is_ZF8_DSI_mode(int vdisplay, int vtotal);
bool zf8_refreshrate_match(int refresh1, int refresh2);

#else

static inline void zf8_drm_notify(int var, int value) {}
static inline void drm_zf8_sysfs_destroy(void) {}
static inline int drm_zf8_sysfs_init(void) { return 0; }
static inline bool is_ZF8_DSI_mode(int vdisplay, int vtotal) { return false; }
static inline bool zf8_refreshrate_match(int refresh1, int refresh2) { return true; }

#endif
#endif /* _DRM_ANAKIN_H_ */
