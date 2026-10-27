/*
 * hal_include.h
 *
 * Platform seam for the candleLight firmware core on CH32L103 + ch32fun.
 *
 * Same contract as the HPMicro port: the firmware sources call STM32-flavoured
 * HAL names (HAL_GPIO_WritePin, HAL_GetTick, ...) and this header declares them
 * on top of ch32fun's register definitions.  Only the vendor underneath changes.
 *
 * CH32L103 specifics that matter here:
 *   - a pin is addressed by ch32fun's port*16 + number encoding, so a
 *     (GPIO_TypeDef *, pin) pair is folded into that form by CH32_PIN()
 *   - the CAN controller is bxCAN compatible (CAN_TypeDef, CTLR/STATR/BTIMR),
 *     so candleLight is built with CONFIG_BXCAN and there is no FDCAN handle
 *   - the unique id sits at 0x1FFFF7E8 (12 bytes), read by HAL_Init()
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "ch32fun.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

/* ------------------------------------------------------------------ */
/* GPIO                                                               */
/* ------------------------------------------------------------------ */
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET,
} GPIO_PinState;

/* ch32fun's GPIO_TypeDef is a real port (GPIOA..GPIOE at consecutive
 * addresses); the pin number is separate because the LED table stores the two
 * halves of the STM32 style pair. */
#define CH32_PIN(port, pin)  ((uint32_t)(((port) == GPIOA ? 0U : \
                                          (port) == GPIOB ? 1U : \
                                          (port) == GPIOC ? 2U : \
                                          (port) == GPIOD ? 3U : 4U) << 4) | ((pin) & 0xFU))

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);

/* ------------------------------------------------------------------ */
/* Tick (1 ms) / busy wait                                            */
/* ------------------------------------------------------------------ */
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

/* Board/SoC bring-up, called by candleLight_fw/src/main.c */
void HAL_Init(void);

/* ------------------------------------------------------------------ */
/* CAN                                                                */
/*                                                                    */
/* CH32L103's CAN is bxCAN compatible, so candleLight is built with   */
/* CONFIG_BXCAN and addresses CAN_TypeDef directly; this alias keeps  */
/* the shared headers happy where they mention the FD name.           */
/* ------------------------------------------------------------------ */
typedef CAN_TypeDef FDCAN_GlobalTypeDef;

/* ------------------------------------------------------------------ */
/* Unique id: the USB iSerialNumber must differ between boards         */
/* ------------------------------------------------------------------ */
extern uint8_t ch32_uid[12];
#define UID_BASE ((uint32_t)(uintptr_t)ch32_uid)
