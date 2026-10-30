/*
 * boot_usb_ch32l103.c
 *
 * CH32L103 USBFS device-controller port glue:
 *   - the CherryUSB low-level hooks (usb_dc_low_level_init / _deinit)
 *   - the vector entry the controller driver calls into
 *   - the teardown used before jumping to the application
 *
 * The controller driver itself is CherryUSB's WCH USBFS port
 * (third_party_components/CherryUSB/port/wch/usbfs/usb_dc_usbfs.c, which lists
 * CH32L103 among the parts it supports); everything the driver cannot know -
 * how this board clocks the cell and what its vector is called - lives here.
 *
 * CH32L103 facts (WCH EVT USBFS device examples, ch32l103hw.h):
 *   - the USBFS cell needs its AHB clock enabled (RCC_AHBPeriph_USBFS)
 *   - the USB clock must be 48 MHz: with SYSCLK = PLL = 96 MHz that is PLL / 2,
 *     i.e. CFGR0.USBPRE = 01
 *   - DP/DM are the dedicated PA12/PA11 USB pins; the controller drives them,
 *     they need no GPIO mode setup
 *   - this is a full-speed cell: no CONFIG_USB_HS for this chip port
 */
#include "boot_usb_port.h"

#include "ch32fun.h"

/* CherryUSB's driver entry point (port/wch/usbfs/usb_dc_usbfs.c). */
void USBD_IRQHandler(uint8_t busid);

void usb_dc_low_level_init(uint8_t busid)
{
    (void)busid;

    /* USB clock = PLL / 2 = 48 MHz at SYSCLK 96 MHz. */
    RCC->CFGR0 = (RCC->CFGR0 & ~(3U << 22)) | RCC_USBCLKSource_PLLCLK_Div2;

    /* Enable the USBFS peripheral clock. */
    RCC->AHBPCENR |= RCC_AHBPeriph_USBFS;

    /* Put the serial interface engine in reset first, exactly like WCH's own
     * USBFS_Device_Init( ENABLE ): it clears whatever state a previous session
     * left behind (the bootloader hands over with the cell already configured),
     * and the 10 us let the SIE settle before the driver programs it. */
    USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    for (volatile uint32_t wait = 0U; wait < 1000U; ++wait) {
    }
    USBFSD->BASE_CTRL = 0;

    NVIC_EnableIRQ(USBFS_IRQn);
}

void usb_dc_low_level_deinit(uint8_t busid)
{
    (void)busid;
    NVIC_DisableIRQ(USBFS_IRQn);
}

void boot_usb_port_deinit(void)
{
    NVIC_DisableIRQ(USBFS_IRQn);

    /* Clock the cell while it is being shut down, so the writes below land. */
    RCC->AHBPCENR |= RCC_AHBPeriph_USBFS;

    /* WCH's "USB device off" sequence (USBFS_Device_Init( DISABLE ) in the
     * SDK): reset the serial interface engine and clear the FIFOs, and wait the
     * 10 us it needs to settle.  Per R8_USB_CTRL.RB_UC_RST_SIE the pins PA11/
     * PA12 become ordinary GPIOs while the SIE is in reset - that is what the
     * step below uses. */
    USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    for (volatile uint32_t wait = 0U; wait < 1000U; ++wait) {
    }
    USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE;   /* keep the pins on the GPIOs */
    USBFSD->INT_EN = 0;
    USBFSD->UDEV_CTRL = 0;          /* port off, internal pull-downs on */

    /* Force D+ low instead of only releasing the internal pull-up: a floated
     * line is not a disconnect the host is guaranteed to notice, and a host
     * that misses it keeps this device instance and ignores whatever attaches
     * next (the "device listed but dead" state). */
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;
    funDigitalWrite(PA12, FUN_LOW);
    funPinMode(PA12, GPIO_CFGLR_OUT_10Mhz_PP);

    /* >10 ms so the host definitely samples the disconnect.  With D+ held low
     * by the GPIO that is the only window that matters, so it is kept close to
     * the minimum: 500000 iterations of the empty loop below are about 15 ms at
     * 96 MHz (3 cycles each). */
    for (volatile uint32_t settle = 0U; settle < 500000U; ++settle) {
    }

    /* Hand the pin back and close the USB block; the next attach turns both
     * back on. */
    funPinMode(PA12, GPIO_CFGLR_IN_FLOAT);
    USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    RCC->AHBPCENR &= ~RCC_AHBPeriph_USBFS;
}

uintptr_t boot_usb_reg_base(void)
{
    return (uintptr_t)USBFS_BASE;
}

/* ch32fun's vector table entry for the USBFS interrupt.  It belongs to the chip
 * port, not to the controller driver: a driver that ships one of its own (the
 * port/ch32/ch32fs copy candleLight uses) has to leave it out. */
void USBFS_IRQHandler(void) INTERRUPT_DECORATOR;
void USBFS_IRQHandler(void)
{
    USBD_IRQHandler(0);
}
