/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   FDCAN1 bring-up for STM32G474RET6: classic CAN at 1 Mbit/s.
  *
  * Bit timing
  *   FDCAN kernel clock = PLLQ = 75 MHz
  *   prescaler 3        -> 25 MHz tq clock (40 ns)
  *   1 + 21 + 3 tq      -> 25 tq = 1 us = 1 Mbit/s, sample point 88 %
  *
  * Same bitrate and frame format as rs01_motor_control (1 Mbit/s, classic CAN,
  * 29-bit extended IDs).  This file only moves raw frames; it knows nothing
  * about motor commands.
  ******************************************************************************
  */
#include "fdcan.h"

#include <string.h>

FDCAN_HandleTypeDef hfdcan1;
volatile fdcan_stats_t g_fdcan_stats;

static bool s_initialized;
static fdcan_frame_t s_rx_ring[FDCAN_RX_RING_SIZE];
static volatile uint32_t s_rx_head;
static volatile uint32_t s_rx_tail;

/* -------------------------------------------------------------------------- */
/* Pins / clocks (HAL MSP callbacks)                                          */
/* -------------------------------------------------------------------------- */

static void gpio_af_config(GPIO_TypeDef *port, uint32_t pin, uint32_t alternate)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = alternate;
    HAL_GPIO_Init(port, &gpio);
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA11 = FDCAN1_RX, PA12 = FDCAN1_TX (AF9).  The transceiver turns these
     * single-ended signals into the differential CAN_H / CAN_L pair. */
    gpio_af_config(APP_CAN_GPIO, APP_CAN_RX_PIN, APP_CAN_AF);
    gpio_af_config(APP_CAN_GPIO, APP_CAN_TX_PIN, APP_CAN_AF);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, APP_CAN_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    __HAL_RCC_FDCAN_CLK_DISABLE();
    HAL_GPIO_DeInit(APP_CAN_GPIO, APP_CAN_RX_PIN | APP_CAN_TX_PIN);
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
}

/* -------------------------------------------------------------------------- */
/* Initialisation                                                             */
/* -------------------------------------------------------------------------- */

static void fdcan_common_init(FDCAN_HandleTypeDef *hfdcan)
{
    memset(&hfdcan->Init, 0, sizeof(hfdcan->Init));

    hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC; /* classic CAN, not FD */
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = ENABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = DISABLE;

    /* 1 Mbit/s from the 75 MHz FDCAN kernel clock. */
    hfdcan->Init.NominalPrescaler = 3;
    hfdcan->Init.NominalSyncJumpWidth = 2;
    hfdcan->Init.NominalTimeSeg1 = 21;
    hfdcan->Init.NominalTimeSeg2 = 3;

    /* Data phase is unused in classic mode but must hold valid values. */
    hfdcan->Init.DataPrescaler = 3;
    hfdcan->Init.DataSyncJumpWidth = 2;
    hfdcan->Init.DataTimeSeg1 = 21;
    hfdcan->Init.DataTimeSeg2 = 3;

    hfdcan->Init.StdFiltersNbr = 1;
    hfdcan->Init.ExtFiltersNbr = 1;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
}

static bool fdcan_configure_filters(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    /* Accept every standard ID into Rx FIFO0. */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000u;
    filter.FilterID2 = 0x000u; /* mask 0 = accept all */
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return false;
    }

    /* Accept every extended ID into Rx FIFO0. */
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x00000000u;
    filter.FilterID2 = 0x00000000u;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return false;
    }

    /* Reject remote frames and anything that does not match a filter. */
    return HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_FILTER_REMOTE,
                                        FDCAN_FILTER_REMOTE) == HAL_OK;
}

bool fdcan_init(void)
{
    s_initialized = false;
    s_rx_head = 0u;
    s_rx_tail = 0u;

    hfdcan1.Instance = FDCAN1;
    fdcan_common_init(&hfdcan1);

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        return false;
    }
    if (!fdcan_configure_filters(&hfdcan1)) {
        (void)HAL_FDCAN_DeInit(&hfdcan1);
        return false;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        (void)HAL_FDCAN_DeInit(&hfdcan1);
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0u) != HAL_OK) {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        (void)HAL_FDCAN_DeInit(&hfdcan1);
        return false;
    }

    s_initialized = true;
    return true;
}

bool fdcan_restart(void)
{
    if (hfdcan1.State != HAL_FDCAN_STATE_RESET) {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        (void)HAL_FDCAN_DeInit(&hfdcan1);
    }
    return fdcan_init();
}

/* -------------------------------------------------------------------------- */
/* Transmit                                                                   */
/* -------------------------------------------------------------------------- */

static uint32_t dlc_to_hal(uint8_t dlc)
{
    switch (dlc) {
    case 0u: return FDCAN_DLC_BYTES_0;
    case 1u: return FDCAN_DLC_BYTES_1;
    case 2u: return FDCAN_DLC_BYTES_2;
    case 3u: return FDCAN_DLC_BYTES_3;
    case 4u: return FDCAN_DLC_BYTES_4;
    case 5u: return FDCAN_DLC_BYTES_5;
    case 6u: return FDCAN_DLC_BYTES_6;
    case 7u: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
    }
}

bool fdcan_send(uint32_t id, bool extended, const uint8_t *data, uint8_t dlc)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t payload[8] = {0};

    if (!s_initialized || data == NULL) {
        g_fdcan_stats.tx_fail++;
        return false;
    }
    if (dlc > 8u) {
        dlc = 8u;
    }
    memcpy(payload, data, dlc);

    tx.Identifier = extended ? (id & 0x1FFFFFFFu) : (id & 0x7FFu);
    tx.IdType = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = dlc_to_hal(dlc);
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0u;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, payload) != HAL_OK) {
        g_fdcan_stats.tx_fail++;
        return false;
    }

    g_fdcan_stats.tx_ok++;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Receive                                                                    */
/* -------------------------------------------------------------------------- */

bool fdcan_receive(fdcan_frame_t *out)
{
    bool available = false;
    const uint32_t primask = __get_PRIMASK();

    if (out == NULL) {
        return false;
    }

    __disable_irq();
    if (s_rx_tail != s_rx_head) {
        *out = s_rx_ring[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1u) % FDCAN_RX_RING_SIZE;
        available = true;
    }
    if (primask == 0u) {
        __enable_irq();
    }

    return available;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t data[8] = {0};
        uint32_t next;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            g_fdcan_stats.rx_dropped++;
            break;
        }

        g_fdcan_stats.rx_ok++;

        next = (s_rx_head + 1u) % FDCAN_RX_RING_SIZE;
        if (next == s_rx_tail) {
            /* Software queue full: drop the newest frame but keep counting. */
            g_fdcan_stats.rx_dropped++;
            continue;
        }

        s_rx_ring[s_rx_head].id = rx.Identifier;
        s_rx_ring[s_rx_head].extended = (rx.IdType == FDCAN_EXTENDED_ID);
        s_rx_ring[s_rx_head].dlc =
            (uint8_t)((rx.DataLength > 8u) ? 8u : rx.DataLength);
        memcpy(s_rx_ring[s_rx_head].data, data, 8u);
        s_rx_ring[s_rx_head].tick_ms = HAL_GetTick();
        s_rx_head = next;
    }
}

/* -------------------------------------------------------------------------- */
/* Diagnostics                                                                */
/* -------------------------------------------------------------------------- */

const char *fdcan_lec_name(uint32_t lec)
{
    switch (lec) {
    case 0u: return "none";
    case 1u: return "stuff";
    case 2u: return "form";
    case 3u: return "ack";
    case 4u: return "bit1";
    case 5u: return "bit0";
    case 6u: return "crc";
    case 7u: return "no-change";
    default: return "other";
    }
}

bool fdcan_get_status(fdcan_status_t *out)
{
    FDCAN_ProtocolStatusTypeDef ps = {0};
    FDCAN_ErrorCountersTypeDef ec = {0};

    if (!s_initialized || out == NULL) {
        return false;
    }
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) != HAL_OK) {
        return false;
    }

    out->last_error_code = ps.LastErrorCode;
    out->error_passive = ps.ErrorPassive;
    out->warning = ps.Warning;
    out->bus_off = ps.BusOff;
    out->protocol_exception = ps.ProtocolException;
    out->tx_error_cnt = ec.TxErrorCnt;
    out->rx_error_cnt = ec.RxErrorCnt;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Internal loopback self test                                                */
/* -------------------------------------------------------------------------- */

uint32_t fdcan_loopback_test(void)
{
    FDCAN_FilterTypeDef filter = {0};
    FDCAN_TxHeaderTypeDef tx = {0};
    const uint8_t payload[8] = {0x11, 0x22, 0x33, 0x44,
                                0x55, 0x66, 0x77, 0x88};
    uint32_t result = 3u;

    s_initialized = false;

    (void)HAL_FDCAN_Stop(&hfdcan1);
    (void)HAL_FDCAN_DeInit(&hfdcan1);

    hfdcan1.Instance = FDCAN1;
    fdcan_common_init(&hfdcan1);
    hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    hfdcan1.Init.ExtFiltersNbr = 0u; /* only the standard filter is used */

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        (void)fdcan_init();
        return 1u;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0u;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000u;
    filter.FilterID2 = 0x000u;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        (void)HAL_FDCAN_DeInit(&hfdcan1);
        (void)fdcan_init();
        return 4u;
    }
    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                       FDCAN_FILTER_REMOTE);
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        (void)HAL_FDCAN_DeInit(&hfdcan1);
        (void)fdcan_init();
        return 1u;
    }

    tx.Identifier = 0x123u;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0u;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)payload) != HAL_OK) {
        result = 2u;
    } else {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t received[8] = {0};
        const uint32_t t0 = HAL_GetTick();

        while ((HAL_GetTick() - t0) < 100u) {
            if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u) {
                if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx,
                                           received) == HAL_OK &&
                    rx.Identifier == 0x123u &&
                    rx.IdType == FDCAN_STANDARD_ID &&
                    received[0] == 0x11u) {
                    result = 0u;
                }
                break;
            }
        }
    }

    (void)HAL_FDCAN_Stop(&hfdcan1);
    (void)HAL_FDCAN_DeInit(&hfdcan1);

    if (!fdcan_init()) {
        return 5u; /* loopback result was good/bad, but the bus is not back */
    }
    return result;
}
