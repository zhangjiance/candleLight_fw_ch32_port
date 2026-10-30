/*
 * NOTE: this copy is for the local bench board, which only shares the CH32L103
 * chip with the candle adapter (USBFS + bxCAN/CANFD are the same, the wiring is
 * not): both LEDs are on PB10/PB11 and are active high, and there is no
 * transceiver, so use the gs_usb loopback flag when starting the channel (the
 * controller is put in internal loopback only if the host asks for it).
 * boards/ch32l103_candle
 * in ch32_dfu_boot and ch32_hello_world is the real candle adapter (PA0/PA1,
 * active low) - do not copy this file over it, and do not copy that one back.
 */

/*
 * board_config.h - "ch32l103_candle"
 *
 * Board Support Package for the CH32L103 USB-CANFD adapter.
 *
 * The board facts (pins, polarity, crystal) are the ones
 * candleLight_fw_ch32_port/boards/ch32l103_candle uses for the application, so
 * the bootloader and the application agree on the hardware; the bootloader only
 * needs a subset of them.
 *
 * Each board lives in its own, self-contained folder under boards/:
 *     boards/<board>/board_config.h   pins / polarity / features (this file)
 *     boards/<board>/board.h          board primitive interface (copy as is)
 *     boards/<board>/board.c          board primitive implementation (copy as is)
 *     boards/<board>/board.cmake      crystal + clock tree, optional extras
 *
 * Adding a board should be nothing more than copying this folder, editing this
 * file and the crystal in board.cmake, and selecting it with -DBOARD=<board>:
 * no file outside boards/ has to change.
 *
 * Pins are spelled the way ch32fun addresses them, port * 16 + number
 * (ch32l103hw.h: PA0, PB9, ...); board.c turns that into the CFGLR/BSHR writes.
 *
 *   USBFS       : DM = PA11, DP = PA12 (fixed alternate function, no GPIO setup)
 *   LED_RX      : PB10  (receive indicator, active high)
 *   LED_TX      : PB11  (transmit indicator, active high)
 *   termination : PA2, active low (drives the 120 ohm switch)
 *   CAN         : RX = PB8, TX = PB9   (application only)
 *   BOOT switch : wired to BOOT0 (pin 44), not to a GPIO - see below
 *   crystal     : 8 MHz HSE
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_NAME              "ch32l103_candle"

/* ---- BOOT button ----
 * The board has a BOOT switch, but it is wired to BOOT0 (pin 44), which on this
 * part is a dedicated boot-mode input rather than a GPIO: the chip samples it in
 * hardware reset and either starts the flash image (BOOT0 low) or boot-modes
 * into the system memory holding WCH's ISP loader (BOOT0 high).  Firmware can
 * not read it back, so it can not be used as a "stay in DFU" request, and this
 * board is button-less as far as the bootloader is concerned.
 *
 * DFU is therefore entered from the application (dfu-util -e -> DFU_DETACH ->
 * BKP marker + reset) or automatically when no valid application is found; the
 * switch remains the way to reach the WCH ISP loader for the very first flash
 * of this bootloader.
 *
 * On a board whose button *is* a GPIO, set BOARD_HAS_BOOT_BUTTON to 1 and name
 * the pin and its polarity:
 *     #define BOARD_BOOT_PIN          PA6
 *     #define BOARD_BOOT_ACTIVE_LOW   1
 */
#define BOARD_HAS_BOOT_BUTTON   0

/* ---- LEDs ----
 * The bootloader drives LED1 as its status LED (toggled per flash block).
 * BOARD_LED_ACTIVE_LOW: 1 = LED on when the pin is low (LED to 3V3 through a
 * resistor, pin sinks the current). */
#define BOARD_HAS_LED           1
#define BOARD_LED_PIN           PB10
#define BOARD_LED_ACTIVE_LOW    0

#define BOARD_HAS_LED2          1
#define BOARD_LED2_PIN          PB11

/* ---- CAN transceiver termination ----
 * PA2 drives the 120 ohm switch; active low means "termination engaged".
 * board_init() leaves it off; the application owns it from there on. */
#define BOARD_HAS_TERMINATION   1
#define BOARD_TERM_PIN          PA2
#define BOARD_TERM_ACTIVE_LOW   1

/* ---- CAN pins ----
 * Not used by the bootloader (a download never touches the CAN cell); kept
 * here so the board file describes the whole board. */
#define BOARD_CAN_RX_PIN        PB8
#define BOARD_CAN_TX_PIN        PB9

/* ---- application UART ----
 * None on this board.  BOOT_PRINTF goes out on ch32fun's USART1 (PA9), which
 * SystemInit() brings up (see user/funconfig.h). */
#define BOARD_HAS_APP_UART      0

/* ---- GPIO port clocks this board needs ----
 * RCC->APB2PCENR bits, one per port named above (PA0/PA1/PA2, PB8/PB9).
 * PA11/PA12 need no clock setup: the USBFS takes them over itself. */
#define BOARD_GPIO_RCC  (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

/* ---- periodic tick ----
 * BOARD_TICK_TIM_ISR has to be the handler name ch32fun's vector table uses
 * for that timer, otherwise the tick interrupt never arrives (TIM3_IRQHandler,
 * TIM2_IRQHandler, ... are all in it). */
#define BOARD_TICK_TIM        TIM3
#define BOARD_TICK_TIM_RCC    RCC_APB1Periph_TIM3
#define BOARD_TICK_TIM_IRQ    TIM3_IRQn
#define BOARD_TICK_TIM_ISR    TIM3_IRQHandler

#endif /* BOARD_CONFIG_H */
