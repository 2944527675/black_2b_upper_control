/**
  ******************************************************************************
  * @file    can_comm.c
  * @brief   Console-driven RS01 CAN test: get-ID, enable/disable and PP/CSP
  *          position moves.
  *
  * Frame layout (same as rs01_motor_control):
  *   ID = (comm << 24) | (extra << 8) | motor_id
  *
  *   get-ID  TX 0x0000FF09 -> RX 0x000009FE (64-bit MCU UID in payload)
  *   enable  TX 0x0300FF09 (payload 8 x 00)
  *   disable TX 0x0400FF09 (payload[0] = clear-error)
  *   write   TX 0x1200FF09 (payload[0..1] = parameter index, [4..7] = value)
  *
  * PP sequence  (run_mode = 1): disable -> run_mode=1 -> pp_vel_max ->
  *                               pp_accel -> limit_current -> enable -> loc_ref
  * CSP sequence (run_mode = 5): disable -> run_mode=5 -> limit_speed ->
  *                               limit_current -> enable -> stream loc_ref 100 Hz
  ******************************************************************************
  */
#include "can_comm.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fdcan.h"
#include "main.h"
#include "rs01.h"

static uint32_t s_tx_sent;
static uint32_t s_rx_seen;
static uint32_t s_getid_replies;
static uint32_t s_last_reply_tick;
static uint8_t s_last_reply_motor_id;
static uint32_t s_feedback_frames;
static uint32_t s_feedback_tick;
static uint8_t s_feedback_mode;
static uint8_t s_feedback_fault;
static float s_fb_position;
static float s_fb_velocity;
static float s_fb_torque;
static float s_fb_temperature;
static bool s_fb_valid;
static uint32_t s_position_tick;
static bool s_param_valid;
static uint16_t s_param_index;
static float s_param_float;
static bool s_rx_print = true;
static uint32_t s_enable_sent;
static uint32_t s_disable_sent;

static const char *mode_name(uint8_t mode)
{
    switch (mode) {
    case 0u: return "reset";
    case 1u: return "calibration";
    case 2u: return "motor";
    default: return "unknown";
    }
}

static const char *run_mode_name(uint8_t mode)
{
    switch (mode) {
    case 0u: return "motion";
    case 1u: return "position-pp";
    case 2u: return "velocity";
    case 3u: return "current";
    case 5u: return "position-csp";
    default: return "unknown";
    }
}

static float clamp_float(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static float u16_to_float(uint16_t raw, float lo, float hi)
{
    return lo + (hi - lo) * (float)raw / 65535.0f;
}

static uint16_t get_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static float get_f32_le(const uint8_t *p)
{
    uint32_t raw = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void print_payload(const uint8_t *data, uint8_t dlc)
{
    printf("data=");
    for (uint8_t i = 0u; i < 8u; i++) {
        if (i < dlc) {
            printf("%02X", data[i]);
        } else {
            printf("--");
        }
        if (i != 7u) {
            printf(" ");
        }
    }
}

static void handle_frame(const fdcan_frame_t *frame, bool print)
{
    if (!frame->extended) {
        if (print) {
            printf("[RX] id=0x%03lX std dlc=%u ",
                   (unsigned long)frame->id, (unsigned)frame->dlc);
            print_payload(frame->data, frame->dlc);
            printf("\r\n");
        }
        return;
    }

    const rs01_can_id_t id = rs01_decode_can_id(frame->id);
    const bool is_getid_reply =
        (id.comm == RS01_COMM_GET_ID) && (id.motor_id == 0xFEu);

    if (print) {
        printf("[RX] id=0x%08lX ext dlc=%u comm=0x%02lX(%s) extra=0x%04X id_lo=0x%02X ",
               (unsigned long)frame->id, (unsigned)frame->dlc,
               (unsigned long)id.comm, rs01_comm_name(id.comm),
               (unsigned)id.extra, (unsigned)id.motor_id);
        print_payload(frame->data, frame->dlc);
        printf("\r\n");
    }

    if (is_getid_reply) {
        /* For this reply the motor ID sits in bits 8..15 and the low byte
         * is the 0xFE broadcast marker (same as rs01_motor_control). */
        const uint8_t reply_id = (uint8_t)((frame->id >> 8) & 0xFFu);

        s_getid_replies++;
        s_last_reply_tick = frame->tick_ms;
        s_last_reply_motor_id = reply_id;
        if (print) {
            printf("       -> get-id reply, motor answers with ID 0x%02X\r\n",
                   (unsigned)reply_id);
        }
    }

    if (id.comm == RS01_COMM_READ_PARAM && frame->dlc >= 8u) {
        const uint8_t status = (uint8_t)((id.extra >> 8) & 0xFFu);

        s_param_index = (uint16_t)((uint16_t)frame->data[0] |
                                   ((uint16_t)frame->data[1] << 8));
        if (status == 0u) {
            s_param_float = get_f32_le(&frame->data[4]);
            s_param_valid = true;

            if (s_param_index == RS01_PARAM_MECH_POS) {
                s_fb_position = s_param_float;
                s_fb_valid = true;
                s_position_tick = frame->tick_ms;
            } else if (s_param_index == RS01_PARAM_MECH_VEL) {
                s_fb_velocity = s_param_float;
            }

            if (print) {
                printf("       -> param 0x%04X = %.4f\r\n",
                       (unsigned)s_param_index, (double)s_param_float);
            }
        } else if (print) {
            printf("       -> param 0x%04X read FAILED (status=%u)\r\n",
                   (unsigned)s_param_index, (unsigned)status);
        }
    }

    /* Type-2 feedback / type-24 active report ID layout (RS01 manual):
     *   bits 8..15  = motor CAN ID
     *   bits 16..21 = 6-bit fault summary
     *   bits 22..23 = mode
     *   bits 7..0   = host CAN ID
     * In rs01_decode_can_id() terms: extra bits 7..0 = motor ID,
     * extra bits 13..8 = fault, extra bits 15..14 = mode. */
    const bool from_this_motor =
        (((frame->id >> 8) & 0xFFu) == APP_MOTOR_ID) ||
        ((frame->id & 0xFFu) == APP_MOTOR_ID);

    if ((id.comm == RS01_COMM_FEEDBACK ||
         id.comm == RS01_COMM_ACTIVE_REPORT) &&
        from_this_motor) {
        s_feedback_frames++;
        s_feedback_tick = frame->tick_ms;
        s_feedback_mode = (uint8_t)((id.extra >> 14) & 0x03u);
        s_feedback_fault = (uint8_t)((id.extra >> 8) & 0x3Fu);

        if (frame->dlc >= 8u) {
            s_fb_position = u16_to_float(get_u16_be(&frame->data[0]),
                                         -RS01_P_MAX, RS01_P_MAX);
            s_fb_velocity = u16_to_float(get_u16_be(&frame->data[2]),
                                         -RS01_V_MAX, RS01_V_MAX);
            s_fb_torque = u16_to_float(get_u16_be(&frame->data[4]),
                                       -RS01_T_MAX, RS01_T_MAX);
            s_fb_temperature = (float)get_u16_be(&frame->data[6]) * 0.1f;
            s_fb_valid = true;
            s_position_tick = frame->tick_ms;
        }

        if (print) {
            if (frame->dlc >= 8u) {
                printf("       -> feedback: motor=0x%02X host=0x%02X"
                       "  pos %7.3f rad  vel %7.3f rad/s  trq %6.2f Nm"
                       "  temp %5.1f C  mode=%u(%s) fault=0x%02X\r\n",
                       (unsigned)((frame->id >> 8) & 0xFFu),
                       (unsigned)(frame->id & 0xFFu),
                       (double)s_fb_position, (double)s_fb_velocity,
                       (double)s_fb_torque, (double)s_fb_temperature,
                       (unsigned)s_feedback_mode, mode_name(s_feedback_mode),
                       (unsigned)s_feedback_fault);
            } else {
                printf("       -> feedback: motor=0x%02X host=0x%02X"
                       " short frame, mode=%u(%s) fault=0x%02X\r\n",
                       (unsigned)((frame->id >> 8) & 0xFFu),
                       (unsigned)(frame->id & 0xFFu),
                       (unsigned)s_feedback_mode, mode_name(s_feedback_mode),
                       (unsigned)s_feedback_fault);
            }
        }
    }
}

bool can_comm_init(void)
{
    s_tx_sent = 0u;
    s_rx_seen = 0u;
    s_getid_replies = 0u;
    s_last_reply_tick = 0u;
    s_last_reply_motor_id = 0u;
    s_feedback_frames = 0u;
    s_feedback_tick = 0u;
    s_feedback_mode = 0u;
    s_feedback_fault = 0u;
    s_fb_position = 0.0f;
    s_fb_velocity = 0.0f;
    s_fb_torque = 0.0f;
    s_fb_temperature = 0.0f;
    s_fb_valid = false;
    s_position_tick = 0u;
    s_param_valid = false;
    s_param_index = 0u;
    s_param_float = 0.0f;
    s_rx_print = true;
    s_enable_sent = 0u;
    s_disable_sent = 0u;

    if (!fdcan_init()) {
        return false;
    }

    printf("[CAN] FDCAN1 up: classic CAN, 1 Mbit/s, PA11=RX PA12=TX\r\n"
           "      filter: accept all standard + extended IDs into RX FIFO0\r\n");
    return true;
}

void can_comm_poll(void)
{
    fdcan_frame_t frame;

    while (fdcan_receive(&frame)) {
        s_rx_seen++;
        handle_frame(&frame, s_rx_print);
    }
}

void can_comm_poll_quiet(void)
{
    const bool old = s_rx_print;

    s_rx_print = false;
    can_comm_poll();
    s_rx_print = old;
}

void can_comm_print_test_frame_description(void)
{
    uint32_t can_id;
    uint8_t payload[8];
    rs01_can_id_t id_fields;

    rs01_make_get_device_id_frame(APP_MOTOR_ID, APP_HOST_ID, &can_id, payload);
    id_fields = rs01_decode_can_id(can_id);

    printf("  'a' sends : id=0x%08lX (29-bit extended), DLC 8, payload 8 x 00\r\n"
           "              comm=0x%02lX(%s) extra=0x%04X motor=0x%02X\r\n",
           (unsigned long)can_id, (unsigned long)id_fields.comm,
           rs01_comm_name(id_fields.comm), (unsigned)id_fields.extra,
           (unsigned)id_fields.motor_id);
}

static bool send_frame_raw(uint32_t can_id, const uint8_t payload[8])
{
    if (!fdcan_send(can_id, true, payload, 8u)) {
        return false;
    }
    s_tx_sent++;
    return true;
}

static bool send_simple_command(uint32_t can_id, const uint8_t payload[8],
                                const char *name)
{
    const rs01_can_id_t id_fields = rs01_decode_can_id(can_id);

    printf("\r\n[TX] %s id=0x%08lX ext dlc=8 comm=0x%02lX(%s) extra=0x%04X motor=0x%02X ",
           name, (unsigned long)can_id, (unsigned long)id_fields.comm,
           rs01_comm_name(id_fields.comm), (unsigned)id_fields.extra,
           (unsigned)id_fields.motor_id);
    print_payload(payload, 8u);
    printf("\r\n");

    if (!send_frame_raw(can_id, payload)) {
        printf("[TX] FAILED: TX FIFO rejected the frame (FDCAN not started?)\r\n");
        can_comm_print_status();
        return false;
    }
    return true;
}

static uint32_t wait_feedback(uint32_t timeout_ms)
{
    const uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        can_comm_poll();
        HAL_Delay(1u);
    }
    can_comm_poll();
    return HAL_GetTick() - t0;
}

static bool wait_new_feedback(uint32_t before, uint32_t timeout_ms)
{
    const uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        can_comm_poll();
        if (s_feedback_frames != before) {
            return true;
        }
        HAL_Delay(1u);
    }
    can_comm_poll();
    return s_feedback_frames != before;
}

/** @brief Send a parameter-read request and wait for its reply.
  *        The reply parser in handle_frame() stores the float value. */
static bool read_param_float(uint16_t index, float *out, uint32_t timeout_ms)
{
    uint32_t can_id;
    uint8_t payload[8];
    const uint32_t t0 = HAL_GetTick();

    s_param_valid = false;
    s_param_index = 0u;

    rs01_make_read_param_frame(APP_MOTOR_ID, APP_HOST_ID, index, &can_id, payload);
    if (!send_frame_raw(can_id, payload)) {
        return false;
    }

    while ((HAL_GetTick() - t0) < timeout_ms) {
        can_comm_poll();
        if (s_param_valid && s_param_index == index) {
            if (out != NULL) {
                *out = s_param_float;
            }
            return true;
        }
        HAL_Delay(1u);
    }
    can_comm_poll();
    return false;
}

/** @brief Read mech_pos (0x7019) and update the cached position. */
static bool refresh_position(uint32_t timeout_ms)
{
    float pos;

    if (!read_param_float(RS01_PARAM_MECH_POS, &pos, timeout_ms)) {
        return false;
    }
    s_fb_position = pos;
    s_fb_valid = true;
    s_position_tick = HAL_GetTick();
    return true;
}

static bool key_pressed(void)
{
    uint8_t ch;

    /* Ignore the CR/LF left in the UART buffer after a line command; only a
     * real key should abort a position move. */
    while (HAL_UART_Receive(&hconsole, &ch, 1u, 0u) == HAL_OK) {
        if (ch != '\r' && ch != '\n') {
            return true;
        }
    }
    return false;
}

bool can_comm_send_test_frame(void)
{
    uint32_t can_id;
    uint8_t payload[8];
    const uint32_t replies_before = s_getid_replies;
    const uint32_t rx_before = s_rx_seen;
    const uint32_t t0 = HAL_GetTick();
    rs01_can_id_t id_fields;

    rs01_make_get_device_id_frame(APP_MOTOR_ID, APP_HOST_ID, &can_id, payload);
    id_fields = rs01_decode_can_id(can_id);

    printf("\r\n[TX] id=0x%08lX ext dlc=8 comm=0x%02lX(%s) extra=0x%04X motor=0x%02X ",
           (unsigned long)can_id, (unsigned long)id_fields.comm,
           rs01_comm_name(id_fields.comm), (unsigned)id_fields.extra,
           (unsigned)id_fields.motor_id);
    print_payload(payload, 8u);
    printf("\r\n");

    if (!send_frame_raw(can_id, payload)) {
        printf("[TX] FAILED: TX FIFO rejected the frame (FDCAN not started?)\r\n");
        can_comm_print_status();
        return false;
    }

    /* Keep polling while waiting so an incoming reply is printed immediately. */
    while ((HAL_GetTick() - t0) < APP_REPLY_WAIT_MS) {
        can_comm_poll();
        HAL_Delay(1u);
    }
    can_comm_poll();

    printf("[TX] done, waited %lu ms: %lu frame(s) received",
           (unsigned long)(HAL_GetTick() - t0),
           (unsigned long)(s_rx_seen - rx_before));

    if (s_getid_replies != replies_before) {
        printf(", get-id reply OK (motor 0x%02X)\r\n",
               (unsigned)s_last_reply_motor_id);
    } else if (s_rx_seen != rx_before) {
        printf(", but no get-id reply\r\n"
               "      (frames arrive; check motor ID / another node is talking)\r\n");
    } else {
        printf(", no reply\r\n"
               "      -> check CAN_H/CAN_L, 120 ohm, transceiver, motor power,\r\n"
               "         bitrate (1 Mbit/s) and motor ID (0x%02X)\r\n",
               (unsigned)APP_MOTOR_ID);
    }

    return (s_getid_replies != replies_before);
}

bool can_comm_send_enable(void)
{
    uint32_t can_id;
    uint8_t payload[8];
    const uint32_t before = s_feedback_frames;

    rs01_make_enable_frame(APP_MOTOR_ID, APP_HOST_ID, &can_id, payload);

    if (!send_simple_command(can_id, payload, "enable")) {
        return false;
    }
    s_enable_sent++;

    printf("[TX] enable sent; waiting %lu ms for type-2 feedback...\r\n",
           (unsigned long)APP_FEEDBACK_WAIT_MS);
    (void)wait_new_feedback(before, APP_FEEDBACK_WAIT_MS);

    if (s_feedback_frames == before) {
        printf("[TX] enable: no feedback frame in %lu ms\r\n"
               "      -> check motor 24 V, CAN wiring, motor ID 0x%02X\r\n",
               (unsigned long)APP_FEEDBACK_WAIT_MS,
               (unsigned)APP_MOTOR_ID);
        return false;
    }

    printf("[TX] enable: feedback received, mode=%u(%s) fault=0x%02X\r\n",
           (unsigned)s_feedback_mode, mode_name(s_feedback_mode),
           (unsigned)s_feedback_fault);
    if (s_feedback_mode == 2u) {
        printf("      -> motor reports run mode; enable accepted\r\n");
    } else {
        printf("      -> motor replied, but mode is not run; check run_mode/fault\r\n");
    }
    return true;
}

bool can_comm_send_disable(bool clear_error)
{
    uint32_t can_id;
    uint8_t payload[8];
    const uint32_t before = s_feedback_frames;

    rs01_make_disable_frame(APP_MOTOR_ID, APP_HOST_ID, clear_error,
                            &can_id, payload);

    if (!send_simple_command(can_id, payload,
                             clear_error ? "disable+clear-error" : "disable")) {
        return false;
    }
    s_disable_sent++;

    printf("[TX] disable sent; waiting %lu ms for type-2 feedback...\r\n",
           (unsigned long)APP_FEEDBACK_WAIT_MS);
    (void)wait_new_feedback(before, APP_FEEDBACK_WAIT_MS);

    if (s_feedback_frames == before) {
        printf("[TX] disable: no feedback frame (command already sent)\r\n");
        return false;
    }

    printf("[TX] disable: feedback received, mode=%u(%s) fault=0x%02X\r\n",
           (unsigned)s_feedback_mode, mode_name(s_feedback_mode),
           (unsigned)s_feedback_fault);
    return true;
}

static bool write_param_u8(uint16_t index, uint8_t value)
{
    uint32_t can_id;
    uint8_t payload[8];
    char label[32];
    const uint32_t before = s_feedback_frames;

    rs01_make_write_param_u8_frame(APP_MOTOR_ID, APP_HOST_ID, index, value,
                                   &can_id, payload);
    (void)snprintf(label, sizeof(label), "write 0x%04X", (unsigned)index);

    if (!send_simple_command(can_id, payload, label)) {
        return false;
    }
    return wait_new_feedback(before, 100u);
}

static bool write_param_float(uint16_t index, float value)
{
    uint32_t can_id;
    uint8_t payload[8];
    char label[32];
    const uint32_t before = s_feedback_frames;

    rs01_make_write_param_float_frame(APP_MOTOR_ID, APP_HOST_ID, index, value,
                                      &can_id, payload);
    (void)snprintf(label, sizeof(label), "write 0x%04X", (unsigned)index);

    if (!send_simple_command(can_id, payload, label)) {
        return false;
    }
    return wait_new_feedback(before, 100u);
}

/** @brief Disable, then write run_mode (same order as Rs01Motor::set_mode). */
static bool set_run_mode(uint8_t mode)
{
    uint32_t can_id;
    uint8_t payload[8];

    const uint32_t before = s_feedback_frames;

    rs01_make_disable_frame(APP_MOTOR_ID, APP_HOST_ID, false, &can_id, payload);
    if (!send_simple_command(can_id, payload, "disable before mode")) {
        return false;
    }
    (void)wait_new_feedback(before, 100u);

    if (!write_param_u8(RS01_PARAM_RUN_MODE, mode)) {
        return false;
    }

    printf("[POS] run_mode -> %u(%s)\r\n", (unsigned)mode,
           run_mode_name(mode));
    return true;
}

bool can_comm_position_pp(float target_rad)
{
    uint32_t can_id;
    uint8_t payload[8];
    uint32_t t0;
    uint32_t last_print = 0u;
    bool reached = false;
    bool aborted = false;

    if (!isfinite(target_rad)) {
        printf("\r\n[POS-PP] invalid target (NaN/Inf)\r\n");
        return false;
    }
    target_rad = clamp_float(target_rad, -APP_POS_MAX_RAD, APP_POS_MAX_RAD);

    printf("\r\n[POS-PP] target=%.3f rad, vmax=%.2f rad/s, accel=%.2f rad/s^2,"
           " Ilim=%.1f A\r\n",
           (double)target_rad, (double)APP_PP_VELOCITY,
           (double)APP_PP_ACCELERATION, (double)APP_POS_CURRENT);

    if (!set_run_mode(RS01_MODE_PP)) {
        return false;
    }
    if (!write_param_float(RS01_PARAM_PP_VEL_MAX, APP_PP_VELOCITY) ||
        !write_param_float(RS01_PARAM_PP_ACCEL, APP_PP_ACCELERATION) ||
        !write_param_float(RS01_PARAM_LIMIT_CURRENT, APP_POS_CURRENT)) {
        return false;
    }
    if (!can_comm_send_enable()) {
        return false;
    }

    rs01_make_write_param_float_frame(APP_MOTOR_ID, APP_HOST_ID,
                                      RS01_PARAM_LOC_REF, target_rad,
                                      &can_id, payload);
    if (!send_simple_command(can_id, payload, "write loc_ref")) {
        (void)can_comm_send_disable(false);
        return false;
    }

    printf("[POS-PP] moving... (any key aborts)\r\n");
    s_rx_print = false;
    t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < APP_POS_TIMEOUT_MS) {
        can_comm_poll();

        /* If the motor only answers commands (no periodic feedback), poll
         * mech_pos so the "reached" check still works. */
        if ((HAL_GetTick() - s_position_tick) > 100u) {
            (void)refresh_position(20u);
        }

        if (key_pressed()) {
            aborted = true;
            break;
        }
        if (s_fb_valid &&
            fabsf(s_fb_position - target_rad) <= APP_POS_TOLERANCE) {
            reached = true;
            break;
        }
        if ((HAL_GetTick() - last_print) >= 250u) {
            last_print = HAL_GetTick();
            printf("[POS-PP] pos=%7.3f rad  target=%7.3f rad  vel=%7.3f rad/s\r\n",
                   (double)s_fb_position, (double)target_rad,
                   (double)s_fb_velocity);
        }
        HAL_Delay(10u);
    }
    s_rx_print = true;
    can_comm_poll();

    if (aborted) {
        printf("[POS-PP] abort key: sending DISABLE...\r\n");
        (void)can_comm_send_disable(false);
    }

    (void)refresh_position(50u);
    if (!reached && s_fb_valid &&
        fabsf(s_fb_position - target_rad) <= APP_POS_TOLERANCE) {
        reached = true;
    }

    if (!s_fb_valid) {
        printf("[POS-PP] no feedback position; cannot confirm the move\r\n");
    } else {
        printf("[POS-PP] final pos=%7.3f rad (target %7.3f), %s\r\n",
               (double)s_fb_position, (double)target_rad,
               reached ? "target reached" : "timeout / not reached");
    }
    if (aborted) {
        printf("[POS-PP] aborted by key; DISABLE was sent\r\n");
    } else {
        printf("[POS-PP] motor remains enabled at the target; press 'd' to disable\r\n");
    }
    return reached;
}

bool can_comm_position_csp(float target_rad)
{
    uint32_t can_id;
    uint8_t payload[8];
    uint32_t k;
    uint32_t last_print = 0u;
    uint32_t tx_fail = 0u;
    bool aborted = false;
    float start_rad;
    uint32_t steps = APP_CSP_RAMP_MS / 10u;

    if (!isfinite(target_rad)) {
        printf("\r\n[POS-CSP] invalid target (NaN/Inf)\r\n");
        return false;
    }
    target_rad = clamp_float(target_rad, -APP_POS_MAX_RAD, APP_POS_MAX_RAD);
    if (steps == 0u) {
        steps = 1u;
    }

    if (!s_fb_valid || (HAL_GetTick() - s_position_tick) > 500u) {
        if (!refresh_position(100u)) {
            printf("\r\n[POS-CSP] cannot read mech_pos (0x7019); check feedback/"
                   "wiring before a CSP move\r\n");
            return false;
        }
    }
    start_rad = s_fb_position;

    printf("\r\n[POS-CSP] %.3f -> %.3f rad over %lu ms, speed limit=%.2f rad/s,"
           " Ilim=%.1f A\r\n",
           (double)start_rad, (double)target_rad,
           (unsigned long)APP_CSP_RAMP_MS, (double)APP_CSP_VELOCITY,
           (double)APP_POS_CURRENT);

    if (!set_run_mode(RS01_MODE_CSP)) {
        return false;
    }
    if (!write_param_float(RS01_PARAM_LIMIT_SPEED, APP_CSP_VELOCITY) ||
        !write_param_float(RS01_PARAM_LIMIT_CURRENT, APP_POS_CURRENT)) {
        return false;
    }
    if (!can_comm_send_enable()) {
        return false;
    }

    printf("[POS-CSP] streaming loc_ref at 100 Hz... (any key aborts)\r\n");
    s_rx_print = false;

    for (k = 0u; k <= steps; k++) {
        const float a = (float)k / (float)steps;
        const float cmd = start_rad + (target_rad - start_rad) * a;

        rs01_make_write_param_float_frame(APP_MOTOR_ID, APP_HOST_ID,
                                          RS01_PARAM_LOC_REF, cmd,
                                          &can_id, payload);
        if (!send_frame_raw(can_id, payload)) {
            tx_fail++;
        }
        can_comm_poll();

        if (key_pressed()) {
            aborted = true;
            break;
        }
        if ((HAL_GetTick() - last_print) >= 250u) {
            last_print = HAL_GetTick();
            printf("[POS-CSP] cmd=%7.3f rad  pos=%7.3f rad  vel=%7.3f rad/s\r\n",
                   (double)cmd, (double)s_fb_position, (double)s_fb_velocity);
        }
        HAL_Delay(10u);
    }

    if (!aborted) {
        rs01_make_write_param_float_frame(APP_MOTOR_ID, APP_HOST_ID,
                                          RS01_PARAM_LOC_REF, target_rad,
                                          &can_id, payload);
        if (!send_frame_raw(can_id, payload)) {
            tx_fail++;
        }
        (void)wait_feedback(200u);
    }
    s_rx_print = true;
    can_comm_poll();

    if (aborted || (tx_fail > 0u)) {
        printf("[POS-CSP] %s: sending DISABLE...\r\n",
               aborted ? "abort key" : "TX failure");
        (void)can_comm_send_disable(false);
    }

    (void)refresh_position(50u);

    printf("[POS-CSP] final pos=%7.3f rad (target %7.3f), tx_fail=%lu%s\r\n",
           (double)s_fb_position, (double)target_rad, (unsigned long)tx_fail,
           aborted ? ", aborted by key" : "");
    if (aborted || (tx_fail > 0u)) {
        printf("[POS-CSP] DISABLE was sent\r\n");
    } else {
        printf("[POS-CSP] motor remains enabled; press 'd' to disable\r\n");
    }
    return !aborted && (tx_fail == 0u);
}

static void print_tx_error_hint(uint32_t lec)
{
    if (lec == 5u) {
        printf("     LEC=bit0: the controller did not see its own dominant bits.\r\n"
               "     Check, in this order:\r\n"
               "       1. Is there a CAN transceiver between PA11/PA12 and the bus?\r\n"
               "          PA11/PA12 are FDCAN1 RX/TX, NOT CAN_H/CAN_L.\r\n"
               "       2. Transceiver VCC/GND and STB/RS/EN pin (must be enabled).\r\n"
               "       3. PA12 -> transceiver TXD, PA11 <- transceiver RXD (not crossed).\r\n"
               "       4. CAN_H/CAN_L not swapped at the transceiver pins.\r\n");
    } else if (lec == 3u) {
        printf("     LEC=ack: the frame reached the ACK slot but no other node ACKed.\r\n"
               "     Check: motor/analyzer powered and connected, CAN_H/CAN_L continuity,\r\n"
               "     120 ohm termination, and the peer bitrate (1 Mbit/s).\r\n");
    } else {
        printf("     Check: transceiver power/enable, PA11/PA12 wiring, CAN_H/CAN_L,\r\n"
               "     120 ohm termination, and the peer bitrate (1 Mbit/s).\r\n");
    }
}

void can_comm_print_status(void)
{
    fdcan_status_t status;

    can_comm_poll();

    printf("\r\n--- CAN status ---\r\n");
    printf("  FDCAN1 classic CAN, 1 Mbit/s, 75 MHz kernel clock\r\n");
    printf("  pins: PA11=FDCAN1_RX, PA12=FDCAN1_TX (AF9)\r\n");
    printf("  board terminals: 11A=CAN_H, 12A=CAN_L\r\n");
    printf("  tx_queued=%lu tx_queue_fail=%lu rx_frames=%lu rx_dropped=%lu\r\n",
           (unsigned long)g_fdcan_stats.tx_ok,
           (unsigned long)g_fdcan_stats.tx_fail,
           (unsigned long)g_fdcan_stats.rx_ok,
           (unsigned long)g_fdcan_stats.rx_dropped);
    printf("  app: sent=%lu received=%lu get-id-replies=%lu\r\n",
           (unsigned long)s_tx_sent, (unsigned long)s_rx_seen,
           (unsigned long)s_getid_replies);
    printf("  app: enable=%lu disable=%lu feedback-frames=%lu\r\n",
           (unsigned long)s_enable_sent, (unsigned long)s_disable_sent,
           (unsigned long)s_feedback_frames);
    if (s_getid_replies > 0u) {
        printf("  last get-id reply: motor 0x%02X at tick %lu ms\r\n",
               (unsigned)s_last_reply_motor_id,
               (unsigned long)s_last_reply_tick);
    }
    if (s_feedback_frames > 0u) {
        printf("  last feedback: mode=%u(%s) fault=0x%02X at tick %lu ms\r\n",
               (unsigned)s_feedback_mode, mode_name(s_feedback_mode),
               (unsigned)s_feedback_fault, (unsigned long)s_feedback_tick);
    }
    if (!s_fb_valid || (HAL_GetTick() - s_position_tick) > 500u) {
        (void)refresh_position(50u);
    }
    if (s_fb_valid) {
        printf("  last position: pos=%7.3f rad  vel=%7.3f rad/s  trq=%6.2f Nm"
               "  temp=%5.1f C\r\n",
               (double)s_fb_position, (double)s_fb_velocity,
               (double)s_fb_torque, (double)s_fb_temperature);
    }

    if (!fdcan_get_status(&status)) {
        printf("  protocol status: unavailable\r\n");
        return;
    }

    printf("  lec=%lu(%s) tx_err=%lu rx_err=%lu bus_off=%lu warning=%lu passive=%lu\r\n",
           (unsigned long)status.last_error_code,
           fdcan_lec_name(status.last_error_code),
           (unsigned long)status.tx_error_cnt,
           (unsigned long)status.rx_error_cnt,
           (unsigned long)status.bus_off,
           (unsigned long)status.warning,
           (unsigned long)status.error_passive);

    if (status.bus_off != 0u) {
        printf("  -> BUS-OFF: the controller left the bus.\r\n");
        print_tx_error_hint(status.last_error_code);
        printf("     Fix the cause, then press 'n' to restart FDCAN1.\r\n");
    } else if (status.tx_error_cnt > 0u) {
        printf("  -> TX errors: frames are not being acknowledged.\r\n");
        print_tx_error_hint(status.last_error_code);
    } else if (status.rx_error_cnt > 0u) {
        printf("  -> RX errors: bit/format errors. Bitrate mismatch or bad\r\n"
               "     wiring/termination.\r\n");
    } else {
        printf("  -> no bus errors. If 'a' still gets no reply, the controller\r\n"
               "     side is fine; check the other node / analyzer.\r\n");
    }
}

void can_comm_print_help(void)
{
    printf("\r\ncommands:\r\n"
           "  a        get-ID frame, wait for the motor's reply\r\n"
           "  e        RS01 ENABLE frame\r\n"
           "  d        RS01 DISABLE frame\r\n"
           "  D        DISABLE + clear-error bit\r\n"
           "  p <rad>  PP position move (run_mode=1, point-to-point)\r\n"
           "  P <rad>  CSP position move (run_mode=5, 100 Hz loc_ref ramp)\r\n"
           "  1        PP move to 0 deg   (0 rad)\r\n"
           "  2        PP move to +90 deg (+1.5708 rad)\r\n"
           "  3        PP move to -90 deg (-1.5708 rad)\r\n"
           "  x        CAN status / feedback position / error counters\r\n"
           "  l        FDCAN internal loopback self test (MCU side only)\r\n"
           "  n        re-init FDCAN1 (clears bus-off)\r\n"
           "  r        software reset of the MCU (no reset button needed)\r\n"
           "  h        this help\r\n"
           "\r\n"
           "p/P are line commands: type the target and Enter; if the value is "
           "omitted the firmware prompts for it.\r\n"
           "Units are radians, range +-12.566 rad (+-4*pi). "
           "No velocity/current/torque command exists.\r\n");
}
