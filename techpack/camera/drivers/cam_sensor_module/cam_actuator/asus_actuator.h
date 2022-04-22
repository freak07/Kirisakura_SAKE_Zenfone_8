#ifndef ASUS_ACTUATOR_H
#define ASUS_ACTUATOR_H
#include "cam_actuator_dev.h"
#include "asus_ois.h"

extern uint8_t g_actuator_power_state;
extern uint8_t g_actuator_camera_open;
//extern int g_actuator_init_setting;

extern void asus_actuator_init(struct cam_actuator_ctrl_t * ctrl);
extern int onsemi_actuator_init_setting(struct cam_actuator_ctrl_t *a_ctrl);

extern void actuator_lock(void);
extern void actuator_unlock(void);

extern int actuator_busy_job_trylock(void);
extern void actuator_busy_job_unlock(void);

extern int32_t trans_dac_value(uint32_t input_dac);

extern int actuator_power_up(struct cam_actuator_ctrl_t *actuator_ctrl);
extern int actuator_power_down(struct cam_actuator_ctrl_t *actuator_ctrl);
extern void actuator_probe_check(void);
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
extern uint8_t asus_allow_vcm_move(void);
extern void asus_vcm_move_write(int enable); //ASUS_BSP Jason fix multi actuator write
#endif
#if defined ASUS_SAKE_PROJECT
extern int get_current_lens_position(uint32_t *dac_value,uint32_t index);
#endif
#endif
