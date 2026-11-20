/**
  ******************************************************************************
  * @file    dm_config.h
  * @brief   Read-only DM4340 scan + MasterID configuration over CAN1.
  *
  * Uses the Damiao parameter protocol on standard ID 0x7FF:
  *   read  : data = {slave_lo, slave_hi, 0x33, RID, 0,0,0,0}
  *   write : data = {slave_lo, slave_hi, 0x55, RID, value[0..3]}
  *   save  : data = {slave_lo, slave_hi, 0xAA, 0x01, 0,0,0,0}
  *
  * The scan itself only reads registers (no enable, no motion).  The menu then
  * asks before writing MST_ID (register 7) and saving it to flash.
  ******************************************************************************
  */
#ifndef DM_CONFIG_H
#define DM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Interactive DM4340 scan + MasterID setup on CAN1.
  *        Prints every step; does not send enable or motion commands. */
void dm_config_menu(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_CONFIG_H */
