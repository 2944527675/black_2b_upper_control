/**
  ******************************************************************************
  * @file    main.h
  * @brief   upper_control - STM32G474RET6 CAN bus send/receive test.
  *
  * Scope: verify that the STM32G474 FDCAN1 peripheral can transmit and receive
  * classic CAN frames at 1 Mbit/s, and provide get-ID / enable / disable and
  * PP/CSP position moves.  No velocity, current or torque control frame is sent.
  *
  * Board / wiring:
  *   CAN1  : PA11 = FDCAN1_RX (AF9), PA12 = FDCAN1_TX (AF9)
  *           board terminals: 11A = CAN_H, 12A = CAN_L
  *   UART  : USART1, PB6 = TX (AF7), PB7 = RX (AF7), 115200 8N1
  *   Probe : CMSIS-DAP (DAP-Link), see the Makefile `flash` target.
  ******************************************************************************
  */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* Console UART handle, defined in main.c and used by syscalls.c / can_comm.c. */
extern UART_HandleTypeDef hconsole;

/* -------------------------------------------------------------------------- */
/* Board / clock                                                              */
/* -------------------------------------------------------------------------- */

#define APP_BOARD_NAME   "STM32G474RET6"
#define APP_SYSCLK_HZ    150000000u
/* FDCAN kernel clock = PLLQ = 75 MHz (see SystemClock_Config in main.c). */
#define APP_FDCAN_CLK_HZ 75000000u
#define APP_CAN_BITRATE  1000000u

/* -------------------------------------------------------------------------- */
/* Console UART: USART1 on PB6 (TX) / PB7 (RX), AF7                           */
/* -------------------------------------------------------------------------- */

#define APP_CONSOLE_BAUD 115200u
#define APP_CONSOLE_GPIO GPIOB
#define APP_CONSOLE_TX_PIN GPIO_PIN_6
#define APP_CONSOLE_RX_PIN GPIO_PIN_7
#define APP_CONSOLE_AF GPIO_AF7_USART1

/* -------------------------------------------------------------------------- */
/* CAN pins: FDCAN1 on PA11 (RX) / PA12 (TX), AF9                             */
/* -------------------------------------------------------------------------- */

#define APP_CAN_GPIO GPIOA
#define APP_CAN_RX_PIN GPIO_PIN_11
#define APP_CAN_TX_PIN GPIO_PIN_12
#define APP_CAN_AF GPIO_AF9_FDCAN1
#define APP_CAN_IRQ_PRIORITY 5u

/* -------------------------------------------------------------------------- */
/* Test frame configuration                                                   */
/* -------------------------------------------------------------------------- */

/* Motor CAN ID.  The RS01 in this workspace has been re-IDed to 0x09
 * (factory default is 0x7F).  Only used to fill the target-ID field of the
 * test frame; it does not change the motor. */
#define APP_MOTOR_ID 0x09u

/* Host ID placed in the extra-data field, same as rs01_motor_control. */
#define APP_HOST_ID 0xFFu

/* How long 'a' waits for a get-ID answer before reporting "no reply". */
#define APP_REPLY_WAIT_MS 200u

/* How long 'e' / 'd' wait for a type-2 feedback frame after the command. */
#define APP_FEEDBACK_WAIT_MS 500u

/* ---- position control (PP / CSP), units are radians ---------------------- */
/* RS01 position range is +-4*pi rad (about +-12.57 rad). */
#define APP_POS_MAX_RAD      12.566370614359172f
#define APP_PI               3.14159265358979323846f
#define APP_DEG_TO_RAD(d)    ((d) * (APP_PI / 180.0f))

/* Preset buttons 1/2/3: absolute motor positions in degrees. */
#define APP_PRESET_ZERO_DEG   0.0f
#define APP_PRESET_POS90_DEG  90.0f
#define APP_PRESET_NEG90_DEG  (-90.0f)
#define APP_PP_VELOCITY      2.0f   /* rad/s, PP max speed        */
#define APP_PP_ACCELERATION  5.0f   /* rad/s^2, PP acceleration   */
#define APP_CSP_VELOCITY     2.0f   /* rad/s, CSP speed limit     */
#define APP_POS_CURRENT      4.0f   /* A, safety current limit    */
#define APP_POS_TOLERANCE    0.05f  /* rad, "target reached" band */
#define APP_POS_TIMEOUT_MS   5000u  /* PP move timeout            */
#define APP_CSP_RAMP_MS      2000u  /* CSP ramp duration          */
#define APP_LINE_TIMEOUT_MS  15000u /* console line input timeout */

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
