/**
  ******************************************************************************
  * @file    rs01.h
  * @brief   Minimal RS01 frame builders: extended-ID layout, get-ID,
  *          enable/disable, parameter write, and PP/CSP position setup.
  *
  * Scope is deliberately small: the frame builders needed for CAN bring-up,
  * enable/disable and position mode (PP/CSP).  No velocity/current/torque
  * control frame builder is provided.
  *
  * Extended CAN ID layout (29 bit, same as rs01_motor_control):
  *
  *   bit 28..24 : communication type
  *   bit 23..8  : extra data (meaning depends on the communication type)
  *   bit 7..0   : target motor ID
  *
  *   ID = (comm << 24) | (extra << 8) | motor_id
  *
  * Example: get-device-ID request for motor 0x09 from host 0xFF:
  *   ID = (0x00 << 24) | (0xFF << 8) | 0x09 = 0x0000FF09
  ******************************************************************************
  */
#ifndef RS01_H
#define RS01_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- communication types (reference / frame decoding only) -------------- */
#define RS01_COMM_GET_ID        0x00u
#define RS01_COMM_MOTION        0x01u
#define RS01_COMM_FEEDBACK      0x02u
#define RS01_COMM_ENABLE        0x03u
#define RS01_COMM_DISABLE       0x04u
#define RS01_COMM_SET_ZERO      0x06u
#define RS01_COMM_SET_CAN_ID    0x07u
#define RS01_COMM_READ_PARAM    0x11u
#define RS01_COMM_WRITE_PARAM   0x12u
#define RS01_COMM_FAULT_REPORT  0x15u
#define RS01_COMM_SAVE_PARAMS   0x16u
#define RS01_COMM_ACTIVE_REPORT 0x18u

/* Only 29 bits are valid in a CAN extended identifier. */
#define RS01_CAN_ID_MASK 0x1FFFFFFFu

/* ---- physical ranges (RS01 manual) ------------------------------------- */
#define RS01_P_MAX (4.0f * 3.14159265358979323846f) /* +-4*pi rad */
#define RS01_V_MAX 44.0f                            /* rad/s      */
#define RS01_T_MAX 17.0f                            /* Nm         */

/* ---- run_mode values ---------------------------------------------------- */
#define RS01_MODE_PP  1u
#define RS01_MODE_CSP 5u

/* ---- parameter indexes used by the position sequences ------------------- */
#define RS01_PARAM_RUN_MODE      0x7005u
#define RS01_PARAM_LOC_REF       0x7016u
#define RS01_PARAM_LIMIT_SPEED   0x7017u
#define RS01_PARAM_LIMIT_CURRENT 0x7018u
#define RS01_PARAM_MECH_POS      0x7019u
#define RS01_PARAM_MECH_VEL      0x701Bu
#define RS01_PARAM_PP_VEL_MAX    0x7024u
#define RS01_PARAM_PP_ACCEL      0x7025u

/* Decoded fields of an extended CAN ID. */
typedef struct {
    uint32_t comm;     /* bits 28..24 */
    uint16_t extra;    /* bits 23..8  */
    uint8_t  motor_id; /* bits 7..0   */
} rs01_can_id_t;

/** @brief Build a 29-bit extended CAN ID:
  *        (comm << 24) | (extra << 8) | motor_id. */
uint32_t rs01_make_can_id(uint32_t comm, uint16_t extra, uint8_t motor_id);

/** @brief Split an extended CAN ID into comm / extra / motor_id. */
rs01_can_id_t rs01_decode_can_id(uint32_t can_id);

/** @brief Human-readable communication-type name ("get-id", "feedback", ...). */
const char *rs01_comm_name(uint32_t comm);

/** @brief Build the harmless get-device-ID request used by the 'a' key.
  *
  * @param target_id  motor ID in bits 7..0 (0x09 on this setup)
  * @param host_id    host ID in bits 23..8 (0xFF, same as rs01_motor_control)
  * @param can_id     output: 29-bit extended ID (0x0000FF09 for 0x09/0xFF)
  * @param payload    output: 8 data bytes, all zero
  */
void rs01_make_get_device_id_frame(uint8_t target_id, uint16_t host_id,
                                   uint32_t *can_id, uint8_t payload[8]);

/** @brief Build the "enable" frame (communication type 3).
  *        Payload is 8 zero bytes; the motor answers with a type-2 feedback.
  *        This only enables the drive - it does not send a motion target. */
void rs01_make_enable_frame(uint8_t target_id, uint16_t host_id,
                            uint32_t *can_id, uint8_t payload[8]);

/** @brief Build the "disable" frame (communication type 4).
  * @param clear_error  true -> payload[0] = 1 (clear fault), false -> all zero */
void rs01_make_disable_frame(uint8_t target_id, uint16_t host_id,
                             bool clear_error, uint32_t *can_id,
                             uint8_t payload[8]);

/** @brief Build a uint8 parameter-write frame (communication type 0x12).
  *        payload[0..1] = index (little endian), payload[4] = value. */
void rs01_make_write_param_u8_frame(uint8_t target_id, uint16_t host_id,
                                    uint16_t index, uint8_t value,
                                    uint32_t *can_id, uint8_t payload[8]);

/** @brief Build a float parameter-write frame (communication type 0x12).
  *        payload[0..1] = index (little endian), payload[4..7] = IEEE-754 float
  *        (little endian).  Used for loc_ref, speed/accel/current limits. */
void rs01_make_write_param_float_frame(uint8_t target_id, uint16_t host_id,
                                       uint16_t index, float value,
                                       uint32_t *can_id, uint8_t payload[8]);

/** @brief Build a parameter-read frame (communication type 0x11).
  *        payload[0..1] = index (little endian); the reply carries the value
  *        in payload[4..7] (float) or payload[4] (u8). */
void rs01_make_read_param_frame(uint8_t target_id, uint16_t host_id,
                                uint16_t index, uint32_t *can_id,
                                uint8_t payload[8]);

#ifdef __cplusplus
}
#endif

#endif /* RS01_H */
