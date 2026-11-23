/**
  ******************************************************************************
  * @file    dm_position.c
  * @brief   DM4340 position control (MIT / position-velocity) over classic CAN.
  *
  *  MIT frame layout (same as the official dm_motor_drv.c mit_ctrl()):
  *    D0..D1 pos[15:0]  D2..D3 vel[11:0]  D3..D4 kp[11:0]
  *    D5..D6 kd[11:0]   D6..D7 torque[11:0]
  *
  *  Position-velocity frame (official pos_ctrl()):
  *    D0..D3 float position (rad), D4..D7 float velocity (rad/s),
  *    command ID = CAN_ID + 0x100.
  ******************************************************************************
  */
#include "dm_position.h"

#include <string.h>

#include "fdcan.h"
#include "main.h"

#define DM_TX_PERIOD_MS 20u
#define DM_FB_REQUEST_MS 100u

static dm_pos_config_t s_cfg;
static bool s_enabled;
static bool s_have_fb;
static float s_target_rad;
static uint32_t s_last_tx;
static uint32_t s_last_req;
static dm_pos_feedback_t s_fb;

/* -------------------------------------------------------------------------- */
/* Range mapping (official float_to_uint / uint_to_float)                     */
/* -------------------------------------------------------------------------- */

static uint16_t float_to_uint(float x, float min, float max, int bits)
{
    const float span = max - min;
    const uint32_t max_int = (1u << (uint32_t)bits) - 1u;

    if (x <= min) {
        return 0u;
    }
    if (x >= max) {
        return (uint16_t)max_int;
    }
    return (uint16_t)((x - min) * (float)max_int / span);
}

static float uint_to_float(uint32_t x, float min, float max, int bits)
{
    const float span = max - min;
    const uint32_t max_int = (1u << (uint32_t)bits) - 1u;

    return (float)x * span / (float)max_int + min;
}

/* -------------------------------------------------------------------------- */
/* Frame builders                                                             */
/* -------------------------------------------------------------------------- */

static uint32_t mode_offset(void)
{
    return (s_cfg.mode == DM_MODE_POS) ? DM_ID_OFFSET_POS : DM_ID_OFFSET_MIT;
}

static bool send_mode_cmd(bool enable)
{
    uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu,
                       0xFFu, 0xFFu, 0xFFu, 0xFCu};

    if (!enable) {
        data[7] = 0xFDu;
    }
    return fdcan_send(s_cfg.can_id + mode_offset(), false, data, 8u);
}

static bool send_mit(float pos, float vel, float kp, float kd, float torque)
{
    uint8_t data[8];
    uint16_t p = float_to_uint(pos, -s_cfg.pmax, s_cfg.pmax, 16);
    uint16_t v = float_to_uint(vel, -s_cfg.vmax, s_cfg.vmax, 12);
    uint16_t kpu = float_to_uint(kp, 0.0f, DM_KP_MAX, 12);
    uint16_t kdu = float_to_uint(kd, 0.0f, DM_KD_MAX, 12);
    uint16_t t = float_to_uint(torque, -s_cfg.tmax, s_cfg.tmax, 12);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0Fu) << 4) | (kpu >> 8));
    data[4] = (uint8_t)kpu;
    data[5] = (uint8_t)(kdu >> 4);
    data[6] = (uint8_t)(((kdu & 0x0Fu) << 4) | (t >> 8));
    data[7] = (uint8_t)t;

    return fdcan_send(s_cfg.can_id + DM_ID_OFFSET_MIT, false, data, 8u);
}

static bool send_pos(float pos, float vel)
{
    uint8_t data[8];

    memcpy(&data[0], &pos, sizeof(pos));
    memcpy(&data[4], &vel, sizeof(vel));
    return fdcan_send(s_cfg.can_id + DM_ID_OFFSET_POS, false, data, 8u);
}

/* Official read_motor_ctrl_fbdata(): ask the driver for one feedback frame.
 * This makes the first feedback independent of whether the enable frame
 * itself is answered. */
static bool request_feedback(void)
{
    uint8_t data[4] = {s_cfg.can_id, 0u, 0xCCu, 0x00u};

    return fdcan_send(0x7FFu, false, data, 4u);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void dm_pos_init(const dm_pos_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    s_cfg = *cfg;
    if (s_cfg.kp < 0.0f) {
        s_cfg.kp = 0.0f;
    }
    if (s_cfg.kp > DM_KP_MAX) {
        s_cfg.kp = DM_KP_MAX;
    }
    if (s_cfg.kd < 0.0f) {
        s_cfg.kd = 0.0f;
    }
    if (s_cfg.kd > DM_KD_MAX) {
        s_cfg.kd = DM_KD_MAX;
    }
    s_enabled = false;
    s_have_fb = false;
    s_target_rad = 0.0f;
    s_last_tx = 0u;
    s_last_req = 0u;
    memset(&s_fb, 0, sizeof(s_fb));
}

bool dm_pos_enable(void)
{
    if (s_enabled) {
        return true;
    }

    s_enabled = true;
    s_have_fb = false;
    s_target_rad = 0.0f;
    s_last_tx = 0u;
    s_last_req = 0u;

    (void)send_mode_cmd(true);
    (void)request_feedback();
    dm_pos_service();

    /* In MIT mode the motor is enabled with kp = 0 until the first feedback
     * arrives, so the first setpoint cannot jerk the output shaft.
     * In POS mode no position frame is sent until feedback is known. */
    return true;
}

void dm_pos_disable(void)
{
    if (s_enabled) {
        (void)send_mode_cmd(false);
    }
    s_enabled = false;
}

bool dm_pos_set_angle_rad(float rad)
{
    if (!s_enabled || !s_have_fb) {
        return false;
    }
    if (rad > s_cfg.pmax) {
        rad = s_cfg.pmax;
    }
    if (rad < -s_cfg.pmax) {
        rad = -s_cfg.pmax;
    }
    s_target_rad = rad;
    s_last_tx = 0u;
    dm_pos_service();
    return true;
}

bool dm_pos_set_angle_deg(float deg)
{
    return dm_pos_set_angle_rad(deg * 0.017453292519943295f);
}

void dm_pos_service(void)
{
    uint32_t now;

    if (!s_enabled) {
        return;
    }
    now = HAL_GetTick();

    if (!s_have_fb && (now - s_last_req) >= DM_FB_REQUEST_MS) {
        s_last_req = now;
        (void)request_feedback();
    }

    if ((now - s_last_tx) < DM_TX_PERIOD_MS) {
        return;
    }
    s_last_tx = now;

    if (s_cfg.mode == DM_MODE_POS) {
        if (s_have_fb) {
            (void)send_pos(s_target_rad, 0.0f);
        }
    } else {
        const float kp = s_have_fb ? s_cfg.kp : 0.0f;
        const float kd = s_have_fb ? s_cfg.kd : 0.0f;
        (void)send_mit(s_target_rad, 0.0f, kp, kd, 0.0f);
    }
}

bool dm_pos_on_frame(const fdcan_frame_t *frame, dm_pos_feedback_t *out)
{
    if (frame == NULL || frame->extended || frame->dlc < 6u) {
        return false;
    }
    if (frame->id != s_cfg.master_id) {
        return false;
    }

    s_fb.id = (uint8_t)(frame->data[0] & 0x0Fu);
    s_fb.state = (uint8_t)(frame->data[0] >> 4);
    s_fb.pos = uint_to_float(((uint32_t)frame->data[1] << 8) | frame->data[2],
                             -s_cfg.pmax, s_cfg.pmax, 16);
    s_fb.vel = uint_to_float(((uint32_t)frame->data[3] << 4) |
                                 (frame->data[4] >> 4),
                             -s_cfg.vmax, s_cfg.vmax, 12);
    s_fb.torque = uint_to_float((((uint32_t)frame->data[4] & 0x0Fu) << 8) |
                                    frame->data[5],
                                -s_cfg.tmax, s_cfg.tmax, 12);
    s_fb.t_mos = frame->data[6];
    s_fb.t_coil = frame->data[7];
    s_fb.tick_ms = HAL_GetTick();

    if (!s_have_fb) {
        /* First feedback after enable: hold the position we are at. */
        s_target_rad = s_fb.pos;
        s_have_fb = true;
    }

    if (out != NULL) {
        *out = s_fb;
    }
    return true;
}

bool dm_pos_poll(dm_pos_feedback_t *out)
{
    fdcan_frame_t frame;
    bool got = false;

    while (fdcan_receive(&frame)) {
        if (dm_pos_on_frame(&frame, out)) {
            got = true;
        }
    }
    return got;
}

bool dm_pos_is_enabled(void)
{
    return s_enabled;
}

bool dm_pos_has_feedback(void)
{
    return s_have_fb;
}

float dm_pos_target_deg(void)
{
    return s_target_rad * 57.29577951308232f;
}
