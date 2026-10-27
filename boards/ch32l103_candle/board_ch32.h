/*
 * board.h
 *
 * Board primitive interface.  Chip and board agnostic code (the USB-CAN
 * firmware) only ever talks to the board through these functions, so porting to
 * another board means writing a board.c and a board_config.h, nothing else.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring the board up: GPIO clocks, LEDs off, termination in its default state,
 * SysTick running.  Call once, after SystemInit(). */
void board_init(void);

/* ---- LEDs ---- */
void board_led1_write(uint8_t on);
void board_led2_write(uint8_t on);
void board_led1_toggle(void);
void board_led2_toggle(void);

/* ---- CAN transceiver termination (120 ohm switch) ---- */
void board_set_termination(bool on);
bool board_get_termination(void);

/* ---- time base ---- */
/* Milliseconds since board_init(), from SysTick. */
uint32_t board_time_ms(void);
/* Busy wait.  The board owns SysTick, so ch32fun's Delay_Ms()/Delay_Us() (which
 * drive it too) must not be used. */
void board_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
