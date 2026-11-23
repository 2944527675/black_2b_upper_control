/**
  ******************************************************************************
  * @file    main.c
  * @brief   upper_control - STM32G474RET6 CAN bus send/receive test.
  *
  * What this firmware does
  *   - brings up FDCAN1 as classic CAN at 1 Mbit/s (same frame format and
  *     bitrate as rs01_motor_control)
  *   - waits for console input on USART1 (PB6 TX / PB7 RX, 115200 8N1)
  *   - pressing 'a' sends a harmless get-device-ID frame with motor ID 0x09
  *   - 'e' sends the RS01 enable frame, 'd'/'D' send disable/clear-error
  *   - 'p <rad>' runs a PP position move, 'P <rad>' runs a CSP position move
  *   - every received frame is printed with its decoded extended-ID fields;
  *     feedback frames show position, velocity, torque, temperature, mode/fault
  *
  * What it deliberately does NOT do
  *   - no velocity, current or torque control frame is sent
  *
  * Wiring
  *   CAN1 : PA11 = FDCAN1_RX, PA12 = FDCAN1_TX (AF9)
  *          board terminals 11A = CAN_H, 12A = CAN_L
  *   UART : USART1 PB6 = TX, PB7 = RX (AF7), 115200 8N1
  *   DAP  : CMSIS-DAP, `make flash`
  ******************************************************************************
  */
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_comm.h"
#include "dm_position.h"
#include "fdcan.h"
#include "rs01.h"

UART_HandleTypeDef hconsole;

/* ---- DM4340 (standard 11-bit IDs, same bus as the RS01) ------------------ */
#define DM_CAN_ID 0x01u
#define DM_MST_ID 0x21u
#define DM_PMAX   12.5f
#define DM_VMAX   30.0f
#define DM_TMAX   10.0f
#define DM_KP     2.0f
#define DM_KD     0.2f

static dm_pos_feedback_t s_dm_fb;
static bool s_dm_fb_valid;

/* -------------------------------------------------------------------------- */
/* Clock: HSI16 -> PLL -> 150 MHz SYSCLK, PLLQ 75 MHz for FDCAN               */
/* -------------------------------------------------------------------------- */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = RCC_PLLM_DIV4;
    osc.PLL.PLLN = 75;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV4;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    periph.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
        Error_Handler();
    }
}

/* -------------------------------------------------------------------------- */
/* Console UART: USART1 PB6 (TX) / PB7 (RX)                                   */
/* -------------------------------------------------------------------------- */

static void console_init(void)
{
    hconsole.Instance = USART1;
    hconsole.Init.BaudRate = APP_CONSOLE_BAUD;
    hconsole.Init.WordLength = UART_WORDLENGTH_8B;
    hconsole.Init.StopBits = UART_STOPBITS_1;
    hconsole.Init.Parity = UART_PARITY_NONE;
    hconsole.Init.Mode = UART_MODE_TX_RX;
    hconsole.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hconsole.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hconsole.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hconsole.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hconsole) != HAL_OK) {
        Error_Handler();
    }
}

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

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio_af_config(APP_CONSOLE_GPIO, APP_CONSOLE_TX_PIN, APP_CONSOLE_AF);
    gpio_af_config(APP_CONSOLE_GPIO, APP_CONSOLE_RX_PIN, APP_CONSOLE_AF);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(APP_CONSOLE_GPIO, APP_CONSOLE_TX_PIN | APP_CONSOLE_RX_PIN);
}

/* -------------------------------------------------------------------------- */
/* Banner / commands                                                          */
/* -------------------------------------------------------------------------- */

static void print_banner(void)
{
    printf("\r\n==================================================\r\n");
    printf(" upper_control - CAN bus send/receive test\r\n");
    printf(" board    : %s\r\n", APP_BOARD_NAME);
    printf(" CAN      : FDCAN1, classic CAN, %lu bit/s\r\n",
           (unsigned long)APP_CAN_BITRATE);
    printf(" pins     : PA11=FDCAN1_RX, PA12=FDCAN1_TX (AF9)\r\n");
    printf(" terminal : 11A=CAN_H, 12A=CAN_L\r\n");
    printf(" console  : USART1 PB6=TX, PB7=RX, %lu 8N1\r\n",
           (unsigned long)APP_CONSOLE_BAUD);
    printf(" motor id : 0x%02X\r\n", (unsigned)APP_MOTOR_ID);
    printf("--------------------------------------------------\r\n");
    can_comm_print_test_frame_description();
    printf("--------------------------------------------------\r\n");
    printf(" get-ID / enable / disable / PP / CSP are implemented.\r\n");
    printf(" No velocity/current/torque control frame is sent.\r\n");
    printf(" Press 'a' get-ID, 'e' enable, 'd' disable, 'h' help.\r\n");
    printf(" Press 'p <rad>' PP move, 'P <rad>' CSP move.\r\n");
    printf(" Presets: '1'=0 deg, '2'=+90 deg, '3'=-90 deg (PP).\r\n");
    printf(" Press 'z' set mechanical zero (disabled, asks 'yes').\r\n");
    printf(" Press 'm' for DM4340 position control (MIT + kp).\r\n");
    printf("==================================================\r\n");
}

/* -------------------------------------------------------------------------- */
/* Line input for the position commands                                       */
/* -------------------------------------------------------------------------- */

/** Blocking line input with idle timeout; echoes and handles backspace.
  * RX frames are drained quietly while waiting so the feedback position stays
  * fresh even if the motor is streaming active reports. */
static int read_line(char *buf, int max_len, uint32_t idle_ms)
{
    int n = 0;
    uint32_t last = HAL_GetTick();

    while ((HAL_GetTick() - last) < idle_ms) {
        uint8_t ch;

        can_comm_poll_quiet();

        if (HAL_UART_Receive(&hconsole, &ch, 1u, 10u) != HAL_OK) {
            continue;
        }
        last = HAL_GetTick();

        if (ch == '\r' || ch == '\n') {
            if (n == 0) {
                printf("\r\n");
                return -1; /* bare Enter cancels immediately */
            }
            break;
        }
        if (ch == 0x7Fu || ch == '\b') {
            if (n > 0) {
                n--;
                printf("\b \b");
            }
            continue;
        }
        if (ch >= 0x20u && ch < 0x7Fu && n < (max_len - 1)) {
            buf[n++] = (char)ch;
            (void)HAL_UART_Transmit(&hconsole, &ch, 1u, 10u);
        }
    }

    buf[n] = '\0';
    printf("\r\n");
    return (n > 0) ? n : -1;
}

/** Prompt for a target position in radians.  Returns false on empty/invalid.
  * Accepts "0.5", "p 0.5" and "P 0.5": the p/P prefix is stripped so a second
  * full command typed at the prompt does not fail. */
static bool prompt_target_rad(float *target)
{
    char line[32];
    char *arg;
    float value = 0.0f;

    printf("target position [rad] (-%.3f .. %.3f), value only e.g. 0.5: ",
           (double)APP_POS_MAX_RAD, (double)APP_POS_MAX_RAD);

    if (read_line(line, (int)sizeof(line), APP_LINE_TIMEOUT_MS) < 0) {
        printf("cancelled\r\n");
        return false;
    }

    arg = line;
    while (*arg == ' ' || *arg == '\t') {
        arg++;
    }
    if (*arg == 'p' || *arg == 'P') {
        arg++;
        while (*arg == ' ' || *arg == '\t') {
            arg++;
        }
    }

    if (sscanf(arg, "%f", &value) != 1) {
        printf("invalid number: '%s' (enter only the value, e.g. 0.5)\r\n",
               line);
        return false;
    }
    *target = value;
    return true;
}

/** Move to an absolute preset angle via PP.  The motor's zero is 0 rad. */
static void move_preset_deg(float degrees)
{
    const float rad = APP_DEG_TO_RAD(degrees);

    printf("\r\n[PRESET] %.1f deg -> %.4f rad (PP)\r\n",
           (double)degrees, (double)rad);
    (void)can_comm_position_pp(rad);
}

/* -------------------------------------------------------------------------- */
/* DM4340 position menu                                                       */
/* -------------------------------------------------------------------------- */

static const char *dm_state_name(uint8_t state)
{
    switch (state) {
    case 0u: return "DISABLED";
    case 1u: return "ENABLED";
    case 8u: return "OVERVOLTAGE";
    case 9u: return "UNDERVOLTAGE";
    case 0xAu: return "OVERCURRENT";
    case 0xBu: return "MOS_OVERTEMP";
    case 0xCu: return "COIL_OVERTEMP";
    case 0xDu: return "COMM_LOSS";
    case 0xEu: return "OVERLOAD";
    default: return "UNKNOWN";
    }
}

static void dm_print_fb(const dm_pos_feedback_t *fb)
{
    printf("[DM-FB] state=0x%X %-12s pos=%.3f rad (%.2f deg) "
           "vel=%.2f rad/s tor=%.2f Nm Tmos=%u Tcoil=%u\r\n",
           (unsigned)fb->state, dm_state_name(fb->state),
           (double)fb->pos, (double)(fb->pos * 57.29577951308232f),
           (double)fb->vel, (double)fb->torque,
           (unsigned)fb->t_mos, (unsigned)fb->t_coil);
}

static void dm_print_help(void)
{
    printf("  e            enable + hold present position\r\n"
           "  a <deg>      target angle in degrees, e.g. a 30\r\n"
           "  s            status + latest feedback\r\n"
           "  d            disable (motor free)\r\n"
           "  q            disable and return to RS01 menu\r\n"
           "  ?            this help\r\n");
}

static void dm_rx_hook(const fdcan_frame_t *frame)
{
    if (dm_pos_on_frame(frame, &s_dm_fb)) {
        s_dm_fb_valid = true;
    }
}

/** DM4340 submenu.  Entered with 'm' from the RS01 menu; the DM motor is
  *  always disabled on entry and on exit. */
static void dm_menu(void)
{
    uint32_t last_print = 0u;
    char line[32];

    s_dm_fb_valid = false;
    dm_pos_disable(); /* in case a previous visit left it enabled */

    printf("\r\n=== DM4340 position control (MIT, kp + angle) ===\r\n");
    printf("  motor CAN_ID=0x%02X  MST_ID=0x%02X  1 Mbit/s\r\n",
           DM_CAN_ID, DM_MST_ID);
    printf("  motor is DISABLED; 'e' enables and holds present position.\r\n");
    dm_print_help();

    for (;;) {
        uint8_t ch;

        can_comm_poll_quiet();
        dm_pos_service();

        if (HAL_UART_Receive(&hconsole, &ch, 1u, 5u) == HAL_OK) {
            if (ch == '\r' || ch == '\n') {
                continue;
            }
            printf("\r\n");
            switch (ch) {
            case 'e':
            case 'E': {
                const uint32_t t0 = HAL_GetTick();

                (void)dm_pos_enable();
                while (!dm_pos_has_feedback() &&
                       (HAL_GetTick() - t0) < 300u) {
                    can_comm_poll_quiet();
                    HAL_Delay(5u);
                }
                if (dm_pos_has_feedback()) {
                    printf("[DM] enabled, holding present position\r\n");
                } else {
                    printf("[DM] enabled, but no feedback on 0x%02X yet;\r\n"
                           "     check +24 V, CAN wiring and MasterID.\r\n",
                           DM_MST_ID);
                }
                break;
            }

            case 'a':
            case 'A': {
                char *end = NULL;
                float deg;

                printf("target angle [deg] (-%.1f .. %.1f): ",
                       (double)(DM_PMAX * 57.29577951308232f),
                       (double)(DM_PMAX * 57.29577951308232f));
                if (read_line(line, (int)sizeof(line),
                              APP_LINE_TIMEOUT_MS) < 0 ||
                    line[0] == '\0') {
                    printf("[DM] cancelled\r\n");
                    break;
                }
                deg = strtof(line, &end);
                if (end == line) {
                    printf("[DM] invalid angle\r\n");
                } else if (!dm_pos_set_angle_deg(deg)) {
                    printf("[DM] not enabled, or no feedback yet\r\n");
                } else {
                    printf("[DM] target = %.2f deg (%.3f rad)\r\n",
                           (double)dm_pos_target_deg(),
                           (double)(dm_pos_target_deg() *
                                    0.017453292519943295f));
                }
                break;
            }

            case 's':
            case 'S':
                printf("[DM] enabled=%u feedback=%u target=%.2f deg\r\n",
                       (unsigned)dm_pos_is_enabled(),
                       (unsigned)dm_pos_has_feedback(),
                       (double)dm_pos_target_deg());
                if (s_dm_fb_valid) {
                    dm_print_fb(&s_dm_fb);
                } else {
                    printf("[DM] no feedback yet\r\n");
                }
                break;

            case 'd':
            case 'D':
                dm_pos_disable();
                printf("[DM] disabled; motor free\r\n");
                break;

            case 'q':
            case 'Q':
            case 'b':
            case 'B':
                dm_pos_disable();
                printf("[DM] back to RS01 menu\r\n");
                return;

            case 'h':
            case 'H':
            case '?':
                dm_print_help();
                break;

            default:
                if (ch >= 0x20u && ch < 0x7Fu) {
                    printf("unknown DM command '%c'\r\n", (int)ch);
                }
                break;
            }
        }

        if (dm_pos_is_enabled() &&
            (HAL_GetTick() - last_print) >= 500u) {
            last_print = HAL_GetTick();
            if (s_dm_fb_valid) {
                dm_print_fb(&s_dm_fb);
            } else {
                printf("[DM] waiting for feedback on MST_ID 0x%02X...\r\n",
                       DM_MST_ID);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    console_init();

    print_banner();

    if (!can_comm_init()) {
        printf("[CAN] FDCAN1 init FAILED. Check clock config; press 'n' to retry.\r\n");
    }

    {
        static const dm_pos_config_t dm_cfg = {
            .can_id = DM_CAN_ID,
            .master_id = DM_MST_ID,
            .mode = DM_MODE_MIT,
            .pmax = DM_PMAX,
            .vmax = DM_VMAX,
            .tmax = DM_TMAX,
            .kp = DM_KP,
            .kd = DM_KD,
        };

        dm_pos_init(&dm_cfg);
        can_comm_set_rx_hook(dm_rx_hook);
    }

    can_comm_print_help();
    printf(" 'm' = DM4340 position control (MIT + kp, angle in deg)\r\n");

    for (;;) {
        uint8_t ch;

        can_comm_poll();

        if (HAL_UART_Receive(&hconsole, &ch, 1u, 5u) != HAL_OK) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            continue;
        }

        printf("\r\n");

        switch (ch) {
        case 'a':
        case 'A':
            (void)can_comm_send_test_frame();
            break;

        case 'e':
        case 'E':
            (void)can_comm_send_enable();
            break;

        case 'd':
            (void)can_comm_send_disable(false);
            break;

        case 'D':
            (void)can_comm_send_disable(true);
            break;

        case 'z':
        case 'Z': {
            char line[16];

            printf("[ZERO] Disable the RS01, then set the present output-shaft\r\n"
                   "       position as 0 rad.  This changes the coordinate\r\n"
                   "       reference: all later position targets shift.\r\n"
                   "       Type 'yes' to confirm: ");
            if (read_line(line, (int)sizeof(line),
                          APP_LINE_TIMEOUT_MS) < 0 ||
                strcmp(line, "yes") != 0) {
                printf("[ZERO] cancelled\r\n");
                break;
            }
            if (can_comm_set_zero()) {
                printf("[ZERO] done; motor is disabled and position is 0 rad.\r\n"
                       "       Press '1' or 'p 0' to verify, then 'e' to enable.\r\n");
            }
            break;
        }

        case '1':
            move_preset_deg(APP_PRESET_ZERO_DEG);
            break;

        case '2':
            move_preset_deg(APP_PRESET_POS90_DEG);
            break;

        case '3':
            move_preset_deg(APP_PRESET_NEG90_DEG);
            break;

        case 'p': {
            float target;

            if (prompt_target_rad(&target)) {
                (void)can_comm_position_pp(target);
            }
            break;
        }

        case 'P': {
            float target;

            if (prompt_target_rad(&target)) {
                (void)can_comm_position_csp(target);
            }
            break;
        }

        case 'r':
        case 'R':
            printf("[SYS] MCU reset requested...\r\n");
            HAL_Delay(50u);
            NVIC_SystemReset();
            break;

        case 'h':
        case 'H':
            can_comm_print_help();
            printf(" 'm' = DM4340 position control (MIT + kp, angle in deg)\r\n");
            break;

        case 'x':
        case 'X':
            can_comm_print_status();
            break;

        case 'm':
        case 'M':
            dm_menu();
            printf("\r\n[RS01] back to main menu ('h' for help)\r\n");
            break;

        case 'l':
        case 'L': {
            const uint32_t rc = fdcan_loopback_test();

            if (rc == 0u) {
                printf("[LOOPBACK] PASS - MCU, FDCAN1 and bit timing are OK.\r\n"
                       "           A remaining problem is on the bus side:\r\n"
                       "           transceiver, CAN_H/CAN_L, 120 ohm or the peer.\r\n");
            } else {
                printf("[LOOPBACK] FAIL (code %lu): 1=init 2=tx 3=no-rx 4=filter 5=restore\r\n",
                       (unsigned long)rc);
            }
            break;
        }

        case 'n':
        case 'N':
            if (fdcan_restart()) {
                printf("[CAN] FDCAN1 restarted.\r\n");
            } else {
                printf("[CAN] FDCAN1 restart FAILED.\r\n");
            }
            break;

        default:
            if (ch >= 0x20u && ch < 0x7Fu) {
                printf("unknown command '%c' (press 'h' for help)\r\n", ch);
            }
            break;
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
