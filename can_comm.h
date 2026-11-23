/**
  ******************************************************************************
  * @file    can_comm.h
  * @brief   Application-level CAN test: send get-ID / enable / disable,
  *          print every received frame.
  *
  * This layer builds the extended-ID frames, calls the low-level FDCAN driver
  * and prints every received frame.  Besides get-ID/enable/disable it
  * implements the PP and CSP position sequences (mode -> limits -> enable ->
  * loc_ref); no velocity/current/torque control sequence is provided.
  ******************************************************************************
  */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

/** Optional receive hook, called for every frame before the RS01 decoder.
  * The DM4340 module uses it to decode standard-ID feedback while can_comm
  * still owns the RX queue drain. */
typedef void (*can_comm_rx_hook_t)(const fdcan_frame_t *frame);

/** @brief Install (or clear with NULL) the per-frame receive hook. */
void can_comm_set_rx_hook(can_comm_rx_hook_t hook);

/** @brief Start FDCAN1 and clear the application counters. */
bool can_comm_init(void);

/** @brief Drain the software RX queue and print every received frame.
  *        Call this regularly from the main loop. */
void can_comm_poll(void);

/** @brief Drain the RX queue and update feedback state without printing.
  *        Used while waiting for console input or streaming position targets. */
void can_comm_poll_quiet(void);

/** @brief Send the single get-device-ID frame ('a' command), then wait
  *        APP_REPLY_WAIT_MS for a get-device-ID reply and report the result. */
bool can_comm_send_test_frame(void);

/** @brief Send the RS01 "enable" frame ('e' command) and wait briefly for a
  *        type-2 feedback frame.  This only enables the drive; it does not
  *        send any motion target. */
bool can_comm_send_enable(void);

/** @brief Send the RS01 "disable" frame ('d' / 'D' command) and wait briefly
  *        for a type-2 feedback frame.
  * @param clear_error  true for 'D' (payload[0] = 1), false for 'd'. */
bool can_comm_send_disable(bool clear_error);

/** @brief Disable, then send "set mechanical zero" (communication type 6).
  *        The present output-shaft position becomes 0 rad.  The motor replies
  *        with a type-2 feedback frame. */
bool can_comm_set_zero(void);

/** @brief Print FDCAN protocol status, error counters and statistics. */
void can_comm_print_status(void);

/** @brief Execute a PP (point-to-point, run_mode=1) position move.
  * @param target_rad absolute target position in radians (clamped to +-4*pi).
  * @return true if the feedback position reached the target before timeout. */
bool can_comm_position_pp(float target_rad);

/** @brief Execute a CSP (run_mode=5) position move by streaming loc_ref at
  *        100 Hz from the current feedback position to @p target_rad.
  *        Requires a valid feedback position first.
  * @return true if the move completed and the final target was sent. */
bool can_comm_position_csp(float target_rad);

/** @brief Print the console command list. */
void can_comm_print_help(void);

/** @brief Print the frame that 'a' will send (used in the boot banner). */
void can_comm_print_test_frame_description(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_COMM_H */
