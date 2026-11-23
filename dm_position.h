/**
  ******************************************************************************
  * @file    dm_position.h
  * @brief   DM4340 position control over classic CAN.
  *
  *  Two frame formats are supported, selected by dm_pos_config_t.mode:
  *
  *    DM_MODE_MIT (0)  MIT mode.  The target angle is sent as the MIT
  *                     position setpoint; kp/kd produce the P/D pull toward
  *                     that angle.  This is the mode the motor is in after it
  *                     was read by the scan firmware (CTRL_MODE = 1), so no
  *                     motor-side mode change is needed.
  *
  *    DM_MODE_POS (2)  Position-velocity mode, same as the official
  *                     dm_motor_drv.c pos_ctrl(): float position (rad) +
  *                     float velocity (rad/s), command ID = CAN_ID + 0x100.
  *                     The motor must have CTRL_MODE = 2; switch it with
  *                     DMTool or by writing register 0x0A and saving.
  *
  *  Feedback is the DM feedback frame on MST_ID (0x21 for this motor):
  *    D[0] = ID | ERR<<4, D[1..2] position, D[3..4] velocity,
  *    D[4..5] torque, D[6] T_MOS, D[7] T_coil.
  ******************************************************************************
  */
#ifndef DM_POSITION_H
#define DM_POSITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

#define DM_MODE_MIT 0u
#define DM_MODE_POS 2u

#define DM_ID_OFFSET_MIT 0x000u
#define DM_ID_OFFSET_POS 0x100u

#define DM_KP_MAX 500.0f
#define DM_KD_MAX 5.0f

typedef struct {
    uint8_t  can_id;     /* motor ESC_ID, e.g. 0x01                  */
    uint8_t  master_id;  /* feedback MST_ID, e.g. 0x21               */
    uint32_t mode;       /* DM_MODE_MIT or DM_MODE_POS               */
    float    pmax;       /* position mapping range, rad (e.g. 12.5)  */
    float    vmax;       /* velocity mapping range, rad/s (e.g. 30)  */
    float    tmax;       /* torque mapping range, Nm (e.g. 10)       */
    float    kp;         /* MIT position gain (ignored in POS mode)  */
    float    kd;         /* MIT damping gain                          */
} dm_pos_config_t;

typedef struct {
    uint8_t  id;        /* controller ID (D[0] low nibble)           */
    uint8_t  state;     /* 0 disabled, 1 enabled, 8.. fault          */
    float    pos;       /* rad                                       */
    float    vel;       /* rad/s                                     */
    float    torque;    /* Nm                                        */
    uint8_t  t_mos;     /* deg C                                     */
    uint8_t  t_coil;    /* deg C                                     */
    uint32_t tick_ms;
} dm_pos_feedback_t;

/** @brief Store configuration.  Motor stays disabled. */
void dm_pos_init(const dm_pos_config_t *cfg);

/** @brief Send enable and request one feedback frame.  Non-blocking: the
  *        motor is kept at kp = 0 until the first feedback arrives, so the
  *        first setpoint cannot jerk the shaft.  In POS mode no position
  *        frame is sent before feedback is known. */
bool dm_pos_enable(void);

/** @brief Send disable (motor free). */
void dm_pos_disable(void);

/** @brief Set target angle in degrees (clamped to +/- pmax).
  * @return false if not enabled or no feedback has been seen yet. */
bool dm_pos_set_angle_deg(float deg);

/** @brief Same as above, radians. */
bool dm_pos_set_angle_rad(float rad);

/** @brief Re-send the current setpoint every 20 ms while enabled.
  *        Call from the main loop; the DM4340 has CAN timeout = 0, but a
  *        periodic stream also keeps the feedback coming. */
void dm_pos_service(void);

/** @brief Drain received frames and decode the latest MST_ID feedback.
  * @param[out] out optional copy of the latest feedback.
  * @return true if a new feedback frame was decoded. */
bool dm_pos_poll(dm_pos_feedback_t *out);

/** @brief Decode one frame that was already popped from the RX queue.
  *        Use this when another module (e.g. can_comm) owns the RX drain:
  *        register it as a receive hook and call dm_pos_service() from the
  *        main loop instead of calling dm_pos_poll().
  * @return true if the frame was the DM feedback frame. */
bool dm_pos_on_frame(const fdcan_frame_t *frame, dm_pos_feedback_t *out);

bool  dm_pos_is_enabled(void);
bool  dm_pos_has_feedback(void);
float dm_pos_target_deg(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_POSITION_H */
