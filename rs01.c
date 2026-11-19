/**
  ******************************************************************************
  * @file    rs01.c
  * @brief   Minimal RS01 frame builders: extended ID, get-ID, enable/disable.
  *
  * No motion/position/velocity/current function is implemented here on purpose.
  * See rs01.h.
  ******************************************************************************
  */
#include "rs01.h"

#include <string.h>

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
