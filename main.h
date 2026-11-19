/**
  ******************************************************************************
  * @file    main.h
  * @brief   upper_control - STM32G474RET6 CAN bus send/receive test.
  *
  * Scope: verify that the STM32G474 FDCAN1 peripheral can transmit and receive
  * classic CAN frames at 1 Mbit/s, and provide get-ID / enable / disable
  * commands.  No motion, position, velocity, current or parameter-write frame
  * is sent.
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

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
