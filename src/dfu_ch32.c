/*
 * dfu_ch32.c - hand over to the DFU bootloader.
 *
 * The application runs behind the 12 KB bootloader in the first flash pages
 * (ch32_dfu_boot).  A gs_usb detach request has to land there, so set the same
 * BKP hand-shake marker the bootloader reads and reset: NVIC_SystemReset does
 * not clear the backup domain, so the marker survives.
 */
#include "ch32fun.h"

/* Same contract as ch32_dfu_boot/shared/boot_protocol.h. */
#define BOOT_TRIGGER_MAGIC  ((uint16_t)0xDF01)

void dfu_run_bootloader(void)
{
    /* Backup domain write enable, then the marker in BKP->DATAR1. */
    RCC->APB1PCENR |= RCC_APB1Periph_PWR | RCC_APB1Periph_BKP;
    /* Backup domain write enable (DBP, PWR_CTLR bit 8 on this part). */
    PWR->CTLR |= (1U << 8);

    BKP->DATAR1 = BOOT_TRIGGER_MAGIC;

    NVIC_SystemReset();
}
