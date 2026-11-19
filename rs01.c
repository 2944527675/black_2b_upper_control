/**
  ******************************************************************************
  * @file    rs01.c
  * @brief   Minimal RS01 frame builders: extended ID, get-ID, enable/disable,
  *          parameter write, PP/CSP position setup.
  ******************************************************************************
  */
#include "rs01.h"

#include <string.h>

static void put_u16_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void put_f32_le(uint8_t *p, float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    p[0] = (uint8_t)(raw & 0xFFu);
    p[1] = (uint8_t)((raw >> 8) & 0xFFu);
    p[2] = (uint8_t)((raw >> 16) & 0xFFu);
    p[3] = (uint8_t)((raw >> 24) & 0xFFu);
}

uint32_t rs01_make_can_id(uint32_t comm, uint16_t extra, uint8_t motor_id)
{
    return ((comm & 0x1Fu) << 24) | ((uint32_t)extra << 8) | motor_id;
}

rs01_can_id_t rs01_decode_can_id(uint32_t can_id)
{
    rs01_can_id_t out;

    can_id &= RS01_CAN_ID_MASK;
    out.comm = (can_id >> 24) & 0x1Fu;
    out.extra = (uint16_t)((can_id >> 8) & 0xFFFFu);
    out.motor_id = (uint8_t)(can_id & 0xFFu);
    return out;
}

const char *rs01_comm_name(uint32_t comm)
{
    switch (comm & 0x1Fu) {
    case RS01_COMM_GET_ID:        return "get-id";
    case RS01_COMM_MOTION:        return "motion";
    case RS01_COMM_FEEDBACK:      return "feedback";
    case RS01_COMM_ENABLE:        return "enable";
    case RS01_COMM_DISABLE:       return "disable";
    case RS01_COMM_SET_ZERO:      return "set-zero";
    case RS01_COMM_SET_CAN_ID:    return "set-can-id";
    case RS01_COMM_READ_PARAM:    return "read-param";
    case RS01_COMM_WRITE_PARAM:   return "write-param";
    case RS01_COMM_FAULT_REPORT:  return "fault-report";
    case RS01_COMM_SAVE_PARAMS:   return "save-params";
    case RS01_COMM_ACTIVE_REPORT: return "active-report";
    default:                      return "unknown";
    }
}

void rs01_make_get_device_id_frame(uint8_t target_id, uint16_t host_id,
                                   uint32_t *can_id, uint8_t payload[8])
{
    memset(payload, 0, 8u);
    *can_id = rs01_make_can_id(RS01_COMM_GET_ID, host_id, target_id);
}

void rs01_make_enable_frame(uint8_t target_id, uint16_t host_id,
                            uint32_t *can_id, uint8_t payload[8])
{
    memset(payload, 0, 8u);
    *can_id = rs01_make_can_id(RS01_COMM_ENABLE, host_id, target_id);
}

void rs01_make_disable_frame(uint8_t target_id, uint16_t host_id,
                             bool clear_error, uint32_t *can_id,
                             uint8_t payload[8])
{
    memset(payload, 0, 8u);
    payload[0] = clear_error ? 1u : 0u;
    *can_id = rs01_make_can_id(RS01_COMM_DISABLE, host_id, target_id);
}

void rs01_make_write_param_u8_frame(uint8_t target_id, uint16_t host_id,
                                    uint16_t index, uint8_t value,
                                    uint32_t *can_id, uint8_t payload[8])
{
    memset(payload, 0, 8u);
    put_u16_le(&payload[0], index);
    payload[4] = value;
    *can_id = rs01_make_can_id(RS01_COMM_WRITE_PARAM, host_id, target_id);
}

void rs01_make_write_param_float_frame(uint8_t target_id, uint16_t host_id,
                                       uint16_t index, float value,
                                       uint32_t *can_id, uint8_t payload[8])
{
    memset(payload, 0, 8u);
    put_u16_le(&payload[0], index);
    put_f32_le(&payload[4], value);
    *can_id = rs01_make_can_id(RS01_COMM_WRITE_PARAM, host_id, target_id);
}

void rs01_make_read_param_frame(uint8_t target_id, uint16_t host_id,
                                uint16_t index, uint32_t *can_id,
                                uint8_t payload[8])
{
    memset(payload, 0, 8u);
    put_u16_le(&payload[0], index);
    *can_id = rs01_make_can_id(RS01_COMM_READ_PARAM, host_id, target_id);
}
