/*
 * usb_lowlevel_ch32.c
 *
 * Overrides the weak hooks in CherryUSB's CH32 USBFS device driver
 * (port/ch32/ch32fs/usb_dc_usbfs.c).  The driver itself is controller logic;
 * what it cannot know is how this board clocks the block and where the PHY
 * pins sit.
 *
 * CH32L103 facts taken from the EVT (EXAM/USB/USBFS/.../ch32l103_usbfs_host.c,
 * USBFS_RCC_Init) and ch32l103hw.h:
 *   - the USBFS cell needs its AHB clock enabled (RCC->AHBPCENR bit 0x1000)
 *   - the USB clock must be 48 MHz: with SYSCLK = 96 MHz that is PLL / 2,
 *     i.e. CFGR0.USBPRE = 01
 *   - DP/DM are the dedicated PA12/PA11 USB pins; the controller drives them,
 *     they need no GPIO mode setup (the same as the EVT device examples)
 */
#include "ch32fun.h"

void usb_dc_low_level_init(void);

void usb_dc_low_level_init(void)
{
    /* USB clock = PLL / 2 = 48 MHz at SYSCLK 96 MHz. */
    RCC->CFGR0 = (RCC->CFGR0 & ~(3U << 22)) | RCC_USBCLKSource_PLLCLK_Div2;

    /* Enable the USBFS peripheral clock. */
    RCC->AHBPCENR |= RCC_AHBPeriph_USBFS;

    NVIC_EnableIRQ(USBFS_IRQn);
}

void usb_dc_low_level_deinit(void);

void usb_dc_low_level_deinit(void)
{
    NVIC_DisableIRQ(USBFS_IRQn);
    RCC->AHBPCENR &= ~RCC_AHBPeriph_USBFS;
}
