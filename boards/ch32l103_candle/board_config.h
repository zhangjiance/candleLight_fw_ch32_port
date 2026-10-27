/*
 * board_config.h - "ch32l103_candle"
 *
 * Board Support Package for the CH32L103 USB-CANFD adapter.
 *
 * Each board lives in its own, self-contained folder under boards/:
 *     boards/<board>/board_config.h   pins / polarity / features (this file)
 *     boards/<board>/board.h          board primitive interface (copy as is)
 *     boards/<board>/board.c          board primitive implementation (copy as is)
 *     boards/<board>/board.cmake      crystal + clock tree
 *
 * Adding a board should be nothing more than copying this folder, editing the
 * pins here and the crystal in board.cmake, and selecting it with
 * -DBOARD=<board>: nothing outside boards/ has to change.
 *
 * Pins are spelled the way ch32fun addresses them, port * 16 + number
 * (ch32l103hw.h: PA0, PB9, ...); board.c turns that into CFGLR/BSHR writes.
 *
 *   USB          : DM = PA11, DP = PA12 (fixed alternate function)
 *   CAN          : RX = PB8, TX = PB9   (AFIO remap, not the EVT's PA11/PA12)
 *   LED1         : PA0
 *   LED2         : PA1
 *   termination  : PA2, active low (drives the 120 ohm switch)
 *   crystal      : 8 MHz HSE
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_NAME              "ch32l103_candle"

/* ---- LEDs ----
 * BOARD_LED_ACTIVE_LOW: 1 = LED on when the pin is low (LED to 3V3 through a
 * resistor, pin sinks the current).  Flip both here if the hardware drives the
 * other way. */
#define BOARD_HAS_LED1          1
#define BOARD_LED1_PIN          PA0
#define BOARD_HAS_LED2          1
#define BOARD_LED2_PIN          PA1
#define BOARD_LED_ACTIVE_LOW    1

/* ---- CAN transceiver termination ----
 * PA2 drives the 120 ohm switch; active low means "termination engaged". */
#define BOARD_HAS_TERMINATION   1
#define BOARD_TERM_PIN          PA2
#define BOARD_TERM_ACTIVE_LOW   1

/* ---- CAN pins ---- */
#define BOARD_CAN_RX_PIN        PB8
#define BOARD_CAN_TX_PIN        PB9

/* ---- GPIO port clocks this board needs ----
 * RCC->APB2PCENR bits, one per port named above (PA0/PA1/PA2, PB8/PB9).
 * PA11/PA12 need no clock setup: the USBFS takes them over itself. */
#define BOARD_GPIO_RCC  (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

#endif /* BOARD_CONFIG_H */
