/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt service routines for the CAN bus test.
  ******************************************************************************
  */
#include "main.h"

#include "fdcan.h"

/* Cortex-M4 core exceptions */
void NMI_Handler(void)
{
    for (;;) {
    }
}

void HardFault_Handler(void)
{
    for (;;) {
    }
}

void MemManage_Handler(void)
{
    for (;;) {
    }
}

void BusFault_Handler(void)
{
    for (;;) {
    }
}

void UsageFault_Handler(void)
{
    for (;;) {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* FDCAN1 interrupt line 0: RX FIFO0 new message (and, if enabled, errors). */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}
