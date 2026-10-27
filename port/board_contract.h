/*
 * board_contract.h
 *
 * What a board package tells the platform layer: how many CAN channels it
 * routes out, which controllers they are, and where the USB controller lives.
 *
 * Unlike the HPMicro port this file carries working defaults - CH32L103 has one
 * USBFS at a fixed address with one interrupt line and one CAN controller - so a
 * board only has to say something when it differs.  A board that wants different
 * values defines them in its board_config.h (or on the command line) before this
 * header is reached.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ch32fun.h"

/* USB device controller.  USBFS_BASE/USBFS_IRQn come from ch32l103hw.h. */
#ifndef BOARD_USB_BASE
#define BOARD_USB_BASE   USBFS_BASE
#endif
#ifndef BOARD_USB_IRQn
#define BOARD_USB_IRQn   USBFS_IRQn
#endif

/* CAN: one bxCAN-compatible controller on this part. */
#ifndef BOARD_CAN_COUNT
#define BOARD_CAN_COUNT  1
#endif

/* The candleLight status LEDs double as the system LEDs. */
#ifndef BOARD_HAS_SYS_LED
#define BOARD_HAS_SYS_LED  1
#endif
