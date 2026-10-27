cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# candleLight_fw_ch32_port - candleLight/gs_usb USB-CANFD firmware for CH32L103
#
# Layering (kept deliberately decoupled so new boards are additive):
#   shared/boot_protocol.h  DFU partition + hand-shake contract (with the boot)
#   boards/<board>/         board_config.h + board.h + board.c + board.cmake
#   port/                   chip port: flash/CAN/USB register level, per chip
#   src/                    the firmware: gs_usb bridge, CAN driver, LEDs
#
# ch32fun supplies startup, clock and printf; USB goes through CherryUSB with
# its CH32 USBFS device controller.
#
# The image links at 0x00003000 behind the 12 KB DFU bootloader (the same one
# the other CH32 projects use), so it is flashed with dfu-util.
# ==============================================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_PROJECT_NAME candle_usb2canfd)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
set(CH32FUN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party_components/ch32fun"
    CACHE PATH "ch32fun checkout (the submodule, or a path of your own)")
set(CHERRYUSB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party_components/CherryUSB"
    CACHE PATH "CherryUSB submodule root")
set(CANDLE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party_components/candleLight_fw"
    CACHE PATH "candleLight_fw submodule root (the gs_usb application)")

if(NOT EXISTS "${CH32FUN_ROOT}/ch32fun/ch32fun.c")
    message(FATAL_ERROR
        "ch32fun not found at ${CH32FUN_ROOT}.  Run "
        "git submodule update --init --recursive, or point -DCH32FUN_ROOT=... at it.")
endif()
if(NOT EXISTS "${CANDLE_ROOT}/src/usbd_desc.c")
    message(FATAL_ERROR
        "candleLight_fw not found at ${CANDLE_ROOT}.  Run "
        "git submodule update --init --recursive.")
endif()
if(NOT EXISTS "${CHERRYUSB_ROOT}/core/usbd_core.c")
    message(FATAL_ERROR
        "CherryUSB not found at ${CHERRYUSB_ROOT}.  Run "
        "git submodule update --init --recursive.")
endif()

# CherryUSB: device stack core only.  The gs_usb class is candleLight's own code
# (src/usbd_bridge.c + the submodule's usbd_gs_can.c), so no class from the
# stack is selected here.
set(CONFIG_CHERRYUSB_DEVICE 1)

# ------------------------------------------------------------------------------
# Board selection
# ------------------------------------------------------------------------------
set(BOARD "ch32l103_candle" CACHE STRING "Target board directory under boards/")
set(BOARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}")

if(NOT EXISTS "${BOARD_DIR}/board_config.h")
    file(GLOB _board_cfgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/boards"
         "${CMAKE_CURRENT_SOURCE_DIR}/boards/*/board_config.h")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'. Available: ${_board_cfgs}")
endif()
if(NOT EXISTS "${BOARD_DIR}/board.c")
    message(FATAL_ERROR "Board '${BOARD}' has no board.c")
endif()

# Optional per-board compile definitions (crystal, clock tree, ...).
if(EXISTS "${BOARD_DIR}/board.cmake")
    include("${BOARD_DIR}/board.cmake")
endif()

# The candle selector macro the shared firmware expects, e.g. BOARD_ch32l103_candle.
string(REPLACE "-" "_" _board_macro "${BOARD}")
message(STATUS "Board: ${BOARD} (BOARD_${_board_macro})")
message(STATUS "  board src: ${BOARD_DIR}/board.c")
message(STATUS "  ch32fun:   ${CH32FUN_ROOT}")

# Toolchain before project() (sets the riscv-wch-elf- compilers).
include(cmake/riscv.cmake)
include("${CHERRYUSB_ROOT}/cherryusb.cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
enable_language(C ASM)

project(${CMAKE_PROJECT_NAME})

set(PROJECT_TARGET ${CMAKE_PROJECT_NAME}.elf)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/src/Link.ld)

add_link_options(
    -T ${LINKER_SCRIPT}
    -Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map
)

add_executable(${PROJECT_TARGET})

target_sources(${PROJECT_TARGET} PRIVATE
    # ch32fun: startup, clock tree, SysTick and printf
    ${CH32FUN_ROOT}/ch32fun/ch32fun.c
    # CherryUSB core + the CH32 USBFS device controller (our copy of the
    # upstream port, see port/ch32/ch32fs/usb_dc_usbfs.c)
    ${cherryusb_srcs}
    ${CMAKE_CURRENT_SOURCE_DIR}/port/ch32/ch32fs/usb_dc_usbfs.c
    # board primitives
    ${BOARD_DIR}/board.c
    # candleLight core, verbatim from the submodule
    ${CANDLE_ROOT}/src/led.c
    ${CANDLE_ROOT}/src/util.c
    ${CANDLE_ROOT}/src/usbd_desc.c
    # candleLight application, copied with an include seam
    ${CMAKE_CURRENT_SOURCE_DIR}/src/candle/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/candle/usbd_gs_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/candle/can_common.c
    # CH32 platform layer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usbd_bridge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/port_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/port_board.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/timer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/status_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dfu_ch32.c
    # overrides the weak USBFS clock/pin hooks of the CherryUSB CH32 driver
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_lowlevel_ch32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can_ch32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can_ch32_adapter.c
)

target_include_directories(${PROJECT_TARGET} PRIVATE
    ${CH32FUN_ROOT}/ch32fun
    ${CMAKE_CURRENT_SOURCE_DIR}/port
    ${CANDLE_ROOT}/include
    ${CANDLE_ROOT}/src
    ${CANDLE_ROOT}/libs/STM32_USB_Device_Library/config
    ${cherryusb_incs}
    # register map for the vendored USBFS driver (upstream copy)
    ${CHERRYUSB_ROOT}/port/ch32/ch32fs
    ${BOARD_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/shared
)

target_compile_definitions(${PROJECT_TARGET} PRIVATE
    BOARD_${_board_macro}=1
    GIT_HASH="ch32l103-port"
    $<$<CONFIG:Debug>:BOOT_ENABLE_LOG=1>
)

# ch32fun.c is one big translation unit that also carries the whole mini libc
# (printf/snprintf/sprintf/strstr/setjmp/...), all of it as weak symbols.  In a
# -flto build LTO keeps those weak definitions alive and merges them into a
# single .text, where --gc-sections can no longer drop them, so ~1.3 KB of code
# the firmware never calls is linked anyway.  Compiling just this file without
# LTO gives every function its own section again.
set_source_files_properties(${CH32FUN_ROOT}/ch32fun/ch32fun.c
    PROPERTIES COMPILE_OPTIONS "-fno-lto")

add_custom_command(TARGET ${PROJECT_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf> ${CMAKE_PROJECT_NAME}.bin
)
