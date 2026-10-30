/*
 * fault_ch32.c - recover from a hard fault.
 *
 * ch32fun aliases every vector it does not know to DefaultIRQHandler(), which
 * spins forever, so without this a hard fault leaves the board dead until it is
 * power cycled.  Resetting instead brings the bootloader/application back up,
 * which the host sees as a re-enumeration rather than a hung device.
 */
#include "ch32fun.h"

void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)
{
    NVIC_SystemReset();
}
