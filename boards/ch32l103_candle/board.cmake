# ==============================================================================
# boards/ch32l103_candle/board.cmake
#
# Board-specific build definitions, included by the top level CMakeLists.txt
# when BOARD selects this directory.
# ==============================================================================

# ------------------------------------------------------------------------------
# Crystal and clock tree
#
# ch32fun derives the PLL setup, every delay, every baud rate and the USB clock
# from these numbers, and they describe the hardware fitted on the board - which
# is why they live here and not in funconfig.h (that file only keeps the ch32fun
# behaviour switches and repeats these values as fallbacks).
#
#     HSE 8 MHz  ->  PLL x12  ->  SYSCLK = HCLK = 96 MHz, APB1 = HCLK/2 = 48 MHz
#
# 96 MHz is the CH32L103 maximum; ch32fun also picks the matching flash latency
# (FLASH_ACTLR_LATENCY_2 above 72 MHz).
#
# CAN bit timing is derived from PCLK1 = 48 MHz, so a board with a different
# crystal changes HSE_VALUE here and the CAN prescalers follow automatically.
# ------------------------------------------------------------------------------
add_compile_definitions(
    HSE_VALUE=8000000
    FUNCONF_PLL_MULTIPLIER=12
    FUNCONF_SYSTEM_CORE_CLOCK=96000000
)
