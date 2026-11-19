/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   Low-level FDCAN1 driver for the STM32G474RET6 CAN bus test.
  *
  *   CAN1: PA11 = FDCAN1_RX, PA12 = FDCAN1_TX (AF9)
  *   Classic CAN (not CAN-FD), 1 Mbit/s from the 75 MHz FDCAN kernel clock.
  *   RX is interrupt driven and queued into a small software ring buffer.
  ******************************************************************************
  */
#ifndef FDCAN_H
#define FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* Software RX queue length (frames).  Deep enough for a short burst. */
#define FDCAN_RX_RING_SIZE 32u

typedef struct {
    uint32_t id;        /* 11-bit or 29-bit identifier, without flag bits */
    bool     extended;  /* true = 29-bit extended ID                     */
    uint8_t  dlc;       /* data length in bytes, 0..8                    */
    uint8_t  data[8];
    uint32_t tick_ms;   /* HAL_GetTick() when the frame was received     */
} fdcan_frame_t;

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t rx_ok;      /* frames taken from the hardware RX FIFO       */
    uint32_t rx_dropped; /* frames lost because the software queue was full */
} fdcan_stats_t;

typedef struct {
    uint32_t last_error_code;  /* 0 none, 1 stuff, 2 form, 3 ack, ... */
    uint32_t error_passive;
    uint32_t warning;
    uint32_t bus_off;
    uint32_t protocol_exception;
    uint32_t tx_error_cnt;     /* grows when nobody ACKs our frames   */
    uint32_t rx_error_cnt;     /* grows on bit / format errors        */
} fdcan_status_t;

extern FDCAN_HandleTypeDef hfdcan1;
extern volatile fdcan_stats_t g_fdcan_stats;

/** @brief Initialise FDCAN1 (classic CAN, 1 Mbit/s), filters, RX interrupt
  *        and start the peripheral.  Returns false on any failure. */
bool fdcan_init(void);

/** @brief Queue one classic CAN frame.  @p dlc must be 0..8. */
bool fdcan_send(uint32_t id, bool extended, const uint8_t *data, uint8_t dlc);

/** @brief Pop one frame from the software RX queue. */
bool fdcan_receive(fdcan_frame_t *out);

/** @brief Read FDCAN protocol status and error counters. */
bool fdcan_get_status(fdcan_status_t *out);

/** @brief Name for the FDCAN last-error-code value. */
const char *fdcan_lec_name(uint32_t lec);

/** @brief Internal loopback self test (MCU/FDCAN/timing only, no bus).
  * @return 0 = pass, 1 = init failed, 2 = TX request failed,
  *         3 = no frame received, 4 = filter config failed.
  * The normal bus configuration is restored before returning. */
uint32_t fdcan_loopback_test(void);

/** @brief Stop and re-initialise FDCAN1 (clears bus-off / error state). */
bool fdcan_restart(void);

#ifdef __cplusplus
}
#endif

#endif /* FDCAN_H */
