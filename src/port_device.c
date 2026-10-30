/*
 * port_device.c - CH32L103 HAL shim for the candleLight firmware core.
 *
 * The counterpart of the HPMicro port's port_device.c: it implements the
 * HAL_* names declared in port/hal_include.h on top of ch32fun and this
 * project's board package.  The firmware sources above it are unchanged.
 */
#include <stdint.h>

#include "ch32fun.h"

#include "board_ch32.h"
#include "board_config.h"
#include "board_layout.h"
#include "boot_usb_port.h"
#include "hal_include.h"
#include "timer.h"

/* USB iSerialNumber source: the 96 bit chip unique id at 0x1FFFF7E8.  Copied
 * out at init so the descriptor code can read it like any other buffer. */
uint8_t ch32_uid[12];

static void ch32_uid_load(void)
{
    const volatile uint8_t *uid = (const volatile uint8_t *)0x1FFFF7E8UL;

    for (uint32_t i = 0; i < sizeof(ch32_uid); i++) {
        ch32_uid[i] = uid[i];
    }
}

/*
 * Put SYSCLK back on HSI before SystemInit() rebuilds the clock tree.
 *
 * The application is entered from the DFU bootloader by a jump, with SYSCLK
 * already on the bootloader's PLL, and ch32fun's SystemInit() stops PLLON with
 * its very first write - without this the core would run out of clock in the
 * middle of SystemInit().  Safe on a power-on reset: SW already selects HSI and
 * the loops below exit at once.
 */
static void clock_fallback_to_hsi(void)
{
    RCC->CTLR |= RCC_HSION;
    while ((RCC->CTLR & RCC_HSIRDY) == 0U) {
    }

    RCC->CFGR0 &= ~RCC_SW;                          /* SW = HSI             */
    while ((RCC->CFGR0 & RCC_SWS) != 0U) {          /* wait for SWS = HSI   */
    }
}

void HAL_Init(void)
{
    /*
     * ch32fun's handle_reset() jumps straight to main() with the clock still on
     * HSI, and candleLight's main() never calls SystemInit() itself, so it has
     * to happen here - before anything that depends on the clock tree.  Without
     * it SYSCLK stays on HSI, the PLL never starts, and the USBFS gets no 48 MHz
     * (PLL/2), i.e. the device never enumerates; the millisecond tick and the
     * CAN bit timing would be wrong by the same factor.
     */
    /* Two blinks = main() is running.  Placed before anything that can hang
     * and before USB comes up, so it cannot disturb enumeration. */

    clock_fallback_to_hsi();
    SystemInit();
    /* GPIO and the millisecond tick come from the board layer. */
    board_init();
    ch32_uid_load();

    /* Hand-over from the bootloader: the host has to see this device go away
     * before the application attaches, or it keeps the bootloader's instance
     * and the new device never enumerates.  Release D+ (which on this part also
     * drives the line low), wait past the host's 10 ms disconnect window, then
     * attach below. */
    boot_usb_port_deinit();
    board_delay_ms(20);

    /* USBFS clock/PLL and the pins are brought up by the controller driver's
     * low-level hook when the stack is initialised (see usb_dc_low_level_init
     * in port/ch32l103/boot_usb_ch32l103.c). */
}

void device_sysclock_config(void)
{
    /* board_init() / SystemInit() already configured the clock tree. */
}

/* ------------------------------------------------------------------ */
/* GPIO                                                               */
/* ------------------------------------------------------------------ */
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == NULL) {
        return;
    }

    funDigitalWrite(CH32_PIN(port, pin), (state == GPIO_PIN_SET) ? 1U : 0U);
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == NULL) {
        return GPIO_PIN_RESET;
    }

    return funDigitalRead(CH32_PIN(port, pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == NULL) {
        return;
    }

    const uint32_t p = CH32_PIN(port, pin);

    funDigitalWrite(p, funDigitalRead(p) ? 0U : 1U);
}

/* ------------------------------------------------------------------ */
/* Tick                                                               */
/* ------------------------------------------------------------------ */
uint32_t HAL_GetTick(void)
{
    return board_time_ms();
}

void HAL_Delay(uint32_t ms)
{
    board_delay_ms(ms);
}
