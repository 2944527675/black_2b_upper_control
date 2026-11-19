/**
  ******************************************************************************
  * @file    can_comm.h
  * @brief   Application-level CAN test: send get-ID / enable / disable,
  *          print every received frame.
  *
  * This layer builds the extended-ID frames, calls the low-level FDCAN driver
  * and prints every received frame.  It still does not send motion, position,
  * velocity or current commands.
  ******************************************************************************
  */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** @brief Start FDCAN1 and clear the application counters. */
bool can_comm_init(void);

/** @brief Drain the software RX queue and print every received frame.
  *        Call this regularly from the main loop. */
void can_comm_poll(void);

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

/** @brief Print FDCAN protocol status, error counters and statistics. */
void can_comm_print_status(void);

/** @brief Print the console command list. */
void can_comm_print_help(void);

/** @brief Print the frame that 'a' will send (used in the boot banner). */
void can_comm_print_test_frame_description(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_COMM_H */
