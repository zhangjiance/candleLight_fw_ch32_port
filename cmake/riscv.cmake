# ==============================================================================
# Toolchain and flags for the CH32L103 candleLight port (ch32fun based)
#
# CH32L103 builds as plain rv32imac.  The hardware prologue (HPE) is available
# on this part - WCH's own startup_ch32l103.S sets INTSYSCR = 0x2 and the EVT
# handlers use interrupt("WCH-Interrupt-fast") - but the *attribute* does not
# depend on the xw ISA string: it just suppresses the compiler's prologue and
# emits a bare mret, so it needs HPE enabled at run time, not -march=...xw.
# FUNCONF_ENABLE_HPE (src/funconfig.h) picks which style the handlers use.
#
# Entry point is InterruptVector: ch32fun's vector table lives in .init at
# address 0, there is no _start stub.
# ==============================================================================
set(CMAKE_SYSTEM_NAME               Generic)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# riscv-wch-elf- must be part of path environment
set(TOOLCHAIN_PREFIX                riscv-wch-elf-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_AR                        ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O3 -flto -g0")

add_compile_options(
        -fmessage-length=0
        -fsigned-char
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fno-ident
        -Wunused
        -Wuninitialized
)

add_compile_options(-march=rv32imac -mabi=ilp32)

add_link_options(
        -march=rv32imac -mabi=ilp32
        -msmall-data-limit=8 -msave-restore
        -nostartfiles
        -Xlinker
        --gc-sections
        -lm
        -fno-ident
        -Wl,--print-memory-usage
        --specs=nano.specs --specs=nosys.specs
)
