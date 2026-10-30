/*
 * board_ch32.h - access to the shared board package's primitives.
 *
 * The board package (boards/<board>/, the same files the bootloader and the
 * hello_world application use) calls its interface header board.h; on this
 * port that name belongs to candleLight's board *table* (the struct the core
 * fills in with the CAN channels and LEDs), so the package is pulled in under
 * its path in the source tree instead.  BOARD_PRIMITIVE_HEADER is set by
 * CMakeLists.txt and selects the board.
 */
#ifndef BOARD_CH32_H
#define BOARD_CH32_H

#include BOARD_PRIMITIVE_HEADER

#endif /* BOARD_CH32_H */
