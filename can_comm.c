/**
  ******************************************************************************
  * @file    can_comm.c
  * @brief   Console-driven CAN send/receive test.
  *
  * 'a' builds the harmless "get device ID" frame in the rs01_motor_control
  * extended-ID layout and queues it on FDCAN1.  'e' / 'd' / 'D' build the
  * RS01 enable / disable frames.  Every received frame is printed; type-2
  * feedback frames show their mode and fault fields.
  *
  *   get-ID  TX ID = (0x00 << 24) | (0xFF << 8) | 0x09 = 0x0000FF09
  *           RX ID = (0x00 << 24) | (0x09 << 8) | 0xFE = 0x000009FE
  *   enable  TX ID = (0x03 << 24) | (0xFF << 8) | 0x09 = 0x0300FF09
  *   disable TX ID = (0x04 << 24) | (0xFF << 8) | 0x09 = 0x0400FF09
  *
  * No motion, position, velocity, current or parameter-write frame is ever
  * transmitted by this file.
  ******************************************************************************
  */
#include "can_comm.h"

#include <stdio.h>

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

static void print_frame(const fdcan_frame_t *frame)
{
    if (frame->extended) {
        const rs01_can_id_t id = rs01_decode_can_id(frame->id);
        const bool is_getid_reply =
            (id.comm == RS01_COMM_GET_ID) && (id.motor_id == 0xFEu);

        printf("[RX] id=0x%08lX ext dlc=%u comm=0x%02lX(%s) extra=0x%04X motor=0x%02X ",
               (unsigned long)frame->id, (unsigned)frame->dlc,
               (unsigned long)id.comm, rs01_comm_name(id.comm),
               (unsigned)id.extra, (unsigned)id.motor_id);
        print_payload(frame->data, frame->dlc);
        printf("\r\n");

        if (is_getid_reply) {
            /* For this reply the motor ID sits in bits 8..15 and the low byte
             * is the 0xFE broadcast marker (same as rs01_motor_control). */
            const uint8_t reply_id = (uint8_t)((frame->id >> 8) & 0xFFu);

            s_getid_replies++;
            s_last_reply_tick = frame->tick_ms;
            s_last_reply_motor_id = reply_id;
            printf("       -> get-id reply, motor answers with ID 0x%02X\r\n",
                   (unsigned)reply_id);
        }

        if ((id.comm == RS01_COMM_FEEDBACK ||
             id.comm == RS01_COMM_ACTIVE_REPORT) &&
            id.motor_id == APP_MOTOR_ID) {
            /* Type-2 feedback / type-24 active report: bits 15..14 = mode,
             * bits 13..8 = 6-bit fault summary. */
            s_feedback_frames++;
            s_feedback_tick = frame->tick_ms;
            s_feedback_mode = (uint8_t)((id.extra >> 14) & 0x03u);
            s_feedback_fault = (uint8_t)((id.extra >> 8) & 0x3Fu);
            printf("       -> feedback: mode=%u(%s) fault=0x%02X\r\n",
                   (unsigned)s_feedback_mode, mode_name(s_feedback_mode),
                   (unsigned)s_feedback_fault);
        }
    } else {
        printf("[RX] id=0x%03lX std dlc=%u ",
               (unsigned long)frame->id, (unsigned)frame->dlc);
        print_payload(frame->data, frame->dlc);
        printf("\r\n");
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
        print_frame(&frame);
    }
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

    if (!fdcan_send(can_id, true, payload, 8u)) {
        printf("[TX] FAILED: TX FIFO rejected the frame (FDCAN not started?)\r\n");
        can_comm_print_status();
        return false;
    }
    s_tx_sent++;

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

    if (!fdcan_send(can_id, true, payload, 8u)) {
        printf("[TX] FAILED: TX FIFO rejected the frame (FDCAN not started?)\r\n");
        can_comm_print_status();
        return false;
    }
    s_tx_sent++;
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
    (void)wait_feedback(APP_FEEDBACK_WAIT_MS);

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
    (void)wait_feedback(APP_FEEDBACK_WAIT_MS);

    if (s_feedback_frames == before) {
        printf("[TX] disable: no feedback frame (command already sent)\r\n");
        return false;
    }

    printf("[TX] disable: feedback received, mode=%u(%s) fault=0x%02X\r\n",
           (unsigned)s_feedback_mode, mode_name(s_feedback_mode),
           (unsigned)s_feedback_fault);
    return true;
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
    printf("\r\ncommands (single key, no Enter needed):\r\n"
           "  a   send ONE get-device-ID frame and wait for a reply\r\n"
           "  e   send the RS01 ENABLE frame (motor may hold/move!)\r\n"
           "  d   send the RS01 DISABLE frame\r\n"
           "  D   send DISABLE with clear-error bit\r\n"
           "  h   this help\r\n"
           "  x   CAN status / error counters / statistics\r\n"
           "  l   FDCAN internal loopback self test (MCU side only)\r\n"
           "  n   stop and re-init FDCAN1 (clears bus-off)\r\n"
           "  r   software reset of the MCU (no reset button needed)\r\n"
           "\r\n"
           "Only get-ID / enable / disable frames are implemented.\r\n"
           "No position, velocity, motion or current command is sent.\r\n");
}
