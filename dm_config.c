/**
  ******************************************************************************
  * @file    dm_config.c
  * @brief   DM4340 SlaveID scan and MasterID setup over the STM32 CAN1 bus.
  *
  * Protocol facts (Damiao DM4340):
  *   - classic CAN, standard 11-bit IDs, 1 Mbit/s
  *   - parameter read/write/save use standard ID 0x7FF
  *   - read  request: {slave_lo, slave_hi, 0x33, RID, 0,0,0,0}
  *   - write request: {slave_lo, slave_hi, 0x55, RID, value[0..3]}
  *   - save  request: {slave_lo, slave_hi, 0xAA, 0x01, 0,0,0,0}
  *   - response data[0..1] = slave ID, data[2] = 0x33/0x55,
  *     data[3] = RID, data[4..7] = value (little endian)
  *
  * Registers:
  *   7  = MST_ID (MasterID / feedback ID)
  *   8  = ESC_ID (SlaveID / motor command ID)
  *   10 = CTRL_MODE
  *   21 = PMAX, 22 = VMAX, 23 = TMAX
  *
  * The scan only reads.  Writing MST_ID is done after an explicit "yes".
  ******************************************************************************
  */
#include "dm_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_comm.h"
#include "fdcan.h"
#include "main.h"

#define DM_STD_PARAM_ID   0x7FFu
#define DM_CMD_READ       0x33u
#define DM_CMD_WRITE      0x55u
#define DM_CMD_SAVE       0xAAu

#define DM_RID_MST_ID     7u
#define DM_RID_ESC_ID     8u
#define DM_RID_CTRL_MODE  10u
#define DM_RID_PMAX       21u
#define DM_RID_VMAX       22u
#define DM_RID_TMAX       23u

typedef struct {
    uint8_t slave_id;
    uint8_t master_id;
    uint8_t control_mode;
    float pmax;
    float vmax;
    float tmax;
} dm_info_t;

static uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float get_f32_le(const uint8_t *p)
{
    uint32_t raw = get_u32_le(p);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static int read_line(char *buf, int max_len, uint32_t idle_ms)
{
    int n = 0;
    bool got_line = false;
    uint32_t last = HAL_GetTick();

    while ((HAL_GetTick() - last) < idle_ms) {
        uint8_t ch;

        can_comm_poll_quiet();

        if (HAL_UART_Receive(&hconsole, &ch, 1u, 10u) != HAL_OK) {
            continue;
        }
        last = HAL_GetTick();

        if (ch == '\r' || ch == '\n') {
            got_line = true;
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
    if (!got_line) {
        return -1; /* idle timeout */
    }
    return n;      /* 0 = bare Enter (accept default) */
}

/* Drain frames and wait for a parameter response for (slave, rid, cmd).
 * Returns true and copies data[4..7] to value. */
static bool wait_param_reply(uint8_t slave, uint8_t rid, uint8_t cmd,
                             uint8_t value[4], uint32_t timeout_ms)
{
    const uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        fdcan_frame_t frame;

        while (fdcan_receive(&frame)) {
            if (frame.extended || frame.dlc < 8u) {
                continue;
            }
            const bool cmd_ok =
                (frame.data[2] == cmd) ||
                (cmd == DM_CMD_READ && frame.data[2] == DM_CMD_WRITE);

            if (frame.data[0] == slave && frame.data[1] == 0u &&
                cmd_ok && frame.data[3] == rid) {
                if (value != NULL) {
                    memcpy(value, &frame.data[4], 4u);
                }
                return true;
            }
        }
        HAL_Delay(1u);
    }
    return false;
}

static bool dm_read_param_raw(uint8_t slave, uint8_t rid, uint8_t value[4],
                              uint32_t timeout_ms)
{
    uint8_t data[8] = {slave, 0u, DM_CMD_READ, rid, 0u, 0u, 0u, 0u};

    if (!fdcan_send(DM_STD_PARAM_ID, false, data, 8u)) {
        return false;
    }
    return wait_param_reply(slave, rid, DM_CMD_READ, value, timeout_ms);
}

static bool dm_write_param_raw(uint8_t slave, uint8_t rid,
                               const uint8_t value[4], uint32_t timeout_ms)
{
    uint8_t data[8] = {slave, 0u, DM_CMD_WRITE, rid,
                       value[0], value[1], value[2], value[3]};

    if (!fdcan_send(DM_STD_PARAM_ID, false, data, 8u)) {
        return false;
    }
    return wait_param_reply(slave, rid, DM_CMD_WRITE, NULL, timeout_ms);
}

static bool dm_read_u32(uint8_t slave, uint8_t rid, uint32_t *out,
                        uint32_t timeout_ms)
{
    uint8_t value[4];

    if (!dm_read_param_raw(slave, rid, value, timeout_ms)) {
        return false;
    }
    *out = get_u32_le(value);
    return true;
}

static bool dm_read_float(uint8_t slave, uint8_t rid, float *out,
                          uint32_t timeout_ms)
{
    uint8_t value[4];

    if (!dm_read_param_raw(slave, rid, value, timeout_ms)) {
        return false;
    }
    *out = get_f32_le(value);
    return true;
}

static void dm_send_disable(uint8_t slave)
{
    const uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu,
                             0xFFu, 0xFFu, 0xFFu, 0xFDu};

    (void)fdcan_send(slave, false, data, 8u);
    HAL_Delay(100u);
}

static bool dm_save_params(uint8_t slave)
{
    uint8_t data[8] = {slave, 0u, DM_CMD_SAVE, 0x01u, 0u, 0u, 0u, 0u};

    if (!fdcan_send(DM_STD_PARAM_ID, false, data, 8u)) {
        return false;
    }
    HAL_Delay(300u);
    return true;
}

static bool dm_read_info(uint8_t slave, dm_info_t *info)
{
    uint32_t u32;

    memset(info, 0, sizeof(*info));
    info->slave_id = slave;

    if (!dm_read_u32(slave, DM_RID_ESC_ID, &u32, 50u)) {
        return false;
    }
    printf("  ESC_ID(8)  = 0x%02lX\r\n", (unsigned long)(u32 & 0xFFu));

    if (!dm_read_u32(slave, DM_RID_MST_ID, &u32, 50u)) {
        printf("  MST_ID(7)  read timeout\r\n");
        return false;
    }
    info->master_id = (uint8_t)(u32 & 0xFFu);
    printf("  MST_ID(7)  = 0x%02X\r\n", (unsigned)info->master_id);

    if (dm_read_u32(slave, DM_RID_CTRL_MODE, &u32, 50u)) {
        info->control_mode = (uint8_t)(u32 & 0xFFu);
    }
    printf("  CTRL_MODE(10) = %u\r\n", (unsigned)info->control_mode);

    if (!dm_read_float(slave, DM_RID_PMAX, &info->pmax, 50u)) {
        info->pmax = 0.0f;
    }
    if (!dm_read_float(slave, DM_RID_VMAX, &info->vmax, 50u)) {
        info->vmax = 0.0f;
    }
    if (!dm_read_float(slave, DM_RID_TMAX, &info->tmax, 50u)) {
        info->tmax = 0.0f;
    }
    printf("  PMAX=%.2f rad  VMAX=%.2f rad/s  TMAX=%.2f Nm\r\n",
           (double)info->pmax, (double)info->vmax, (double)info->tmax);
    return true;
}

/* Scan SlaveID 1..127 with a read-only ESC_ID request. */
static bool dm_scan(dm_info_t *info)
{
    uint16_t id;

    printf("\r\n[DM] scanning SlaveID 1..127 with read-only parameter reads...\r\n"
           "     (no enable, no motion)\r\n");

    for (id = 1u; id <= 0x7Fu; id++) {
        uint8_t value[4];

        if (dm_read_param_raw((uint8_t)id, DM_RID_ESC_ID, value, 20u)) {
            const uint32_t esc = get_u32_le(value);

            printf("[DM] response at candidate 0x%02X, ESC_ID=0x%02lX\r\n",
                   (unsigned)id, (unsigned long)(esc & 0xFFu));
            if (dm_read_info((uint8_t)id, info)) {
                return true;
            }
            printf("[DM] candidate 0x%02X responded but info read failed\r\n",
                   (unsigned)id);
        }
        if ((id % 16u) == 0u) {
            printf("  ...%u\r\n", (unsigned)id);
        }
    }

    printf("[DM] no DM4340 response.\r\n"
           "     Check: DM motor 24V, CAN_H/CAN_L, common ground, 1 Mbit/s,\r\n"
           "     and that the motor is in the Damiao private CAN protocol.\r\n");
    return false;
}

static bool dm_set_master(uint8_t slave, uint8_t master)
{
    uint8_t value[4] = {master, 0u, 0u, 0u};
    uint32_t readback = 0u;

    printf("\r\n[DM] setting MasterID: SlaveID=0x%02X -> MasterID=0x%02X\r\n",
           (unsigned)slave, (unsigned)master);

    /* Damiao save routine disables the motor first (no motion). */
    dm_send_disable(slave);

    if (!dm_write_param_raw(slave, DM_RID_MST_ID, value, 100u)) {
        printf("[DM] write MST_ID timeout/response missing\r\n"
               "     The write may still have reached the motor; continuing to save.\r\n");
    }

    if (!dm_save_params(slave)) {
        printf("[DM] save frame send failed\r\n");
        return false;
    }

    if (dm_read_u32(slave, DM_RID_MST_ID, &readback, 150u)) {
        printf("[DM] readback MST_ID = 0x%02lX\r\n",
               (unsigned long)(readback & 0xFFu));
        if ((readback & 0xFFu) == master) {
            printf("[DM] MasterID set and saved: 0x%02X\r\n", (unsigned)master);
            return true;
        }
        printf("[DM] readback mismatch; motor may not have saved the value\r\n");
        return false;
    }

    printf("[DM] readback timeout; check the motor after a power cycle\r\n");
    return false;
}

void dm_config_menu(void)
{
    dm_info_t info;
    char line[32];
    uint8_t default_master;
    uint8_t new_master;

    printf("\r\n=== DM4340 scan / MasterID setup (CAN1, no motion) ===\r\n");

    if (!dm_scan(&info)) {
        return;
    }

    printf("\r\n[DM] found: SlaveID=0x%02X MasterID=0x%02X mode=%u\r\n",
           (unsigned)info.slave_id, (unsigned)info.master_id,
           (unsigned)info.control_mode);
    printf("[DM] limits: PMAX=%.2f VMAX=%.2f TMAX=%.2f\r\n",
           (double)info.pmax, (double)info.vmax, (double)info.tmax);

    if (info.pmax > 0.0f &&
        (info.pmax < 12.0f || info.pmax > 13.0f ||
         info.vmax < 7.0f || info.vmax > 11.0f)) {
        printf("[DM] warning: limits are not the expected DM4340 24V/48V range\r\n");
    }

    default_master = (uint8_t)(info.slave_id + 0x10u);
    if (default_master > 0x7Fu) {
        default_master = 0x11u;
    }

    printf("\r\nCurrent MasterID = 0x%02X\r\n", (unsigned)info.master_id);
    printf("New MasterID [hex/dec, Enter=0x%02X, q=cancel]: ",
           (unsigned)default_master);

    {
        const int n = read_line(line, (int)sizeof(line), 15000u);

        if (n < 0) {
            printf("[DM] cancelled (timeout)\r\n");
            return;
        }
        if (n == 0) {
            new_master = default_master;
        } else if (line[0] == 'q' || line[0] == 'Q') {
            printf("[DM] cancelled\r\n");
            return;
        } else {
            char *end = NULL;
            const unsigned long requested = strtoul(line, &end, 0);

            if (end == line || requested == 0u || requested > 0x7Fu) {
                printf("[DM] invalid MasterID; use 0x01..0x7F\r\n");
                return;
            }
            if ((uint8_t)requested == info.slave_id) {
                printf("[DM] MasterID must differ from SlaveID\r\n");
                return;
            }
            new_master = (uint8_t)requested;
        }
    }

    printf("\r\nWrite MasterID 0x%02X to SlaveID 0x%02X and save to flash?\r\n"
           "Type 'yes' to confirm: ",
           (unsigned)new_master, (unsigned)info.slave_id);
    if (read_line(line, (int)sizeof(line), 15000u) < 0 ||
        strcmp(line, "yes") != 0) {
        printf("[DM] cancelled\r\n");
        return;
    }

    (void)dm_set_master(info.slave_id, new_master);
}
