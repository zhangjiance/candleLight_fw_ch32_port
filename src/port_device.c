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

void HAL_Init(void)
{
    /* Clock tree, GPIO and the millisecond tick come from the board layer; main()
     * has already run SystemInit() before this. */
    board_init();

    ch32_uid_load();

    /* USBFS clock/PLL and the pins are brought up by the controller driver's
     * low-level hook when the stack is initialised (see usb_dc_low_level_init
     * in port/ch32/ch32usb.c). */
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
