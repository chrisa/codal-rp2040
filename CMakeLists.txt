cmake_minimum_required(VERSION 3.13)
enable_language(ASM)

project( codal-rp2040 )

include("${CODAL_UTILS_LOCATION}")

set(HAL_RP2040_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk/src/rp2040")
set(HAL_RP2_COMMON_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk/src/rp2_common")
set(HAL_COMMON_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk/src/common")
set(HAL_HOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk/src/host")

# RECURSIVE_FIND_DIR(PROCESSOR_DIR_INCLUDES "${PROCESSOR_INCLUDE_ROOT}", "*.h")
# RECURSIVE_FIND_DIR(HAL_DIR_INCLUDES "${PROCESSOR_INCLUDES}", "*.h")
RECURSIVE_FIND_DIR(TOP_LEVEL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/inc" "*.h")
RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE0 "${CMAKE_CURRENT_SOURCE_DIR}/source" "*.cpp")
RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE1 "${CMAKE_CURRENT_SOURCE_DIR}/source" "*.c")

list(APPEND TOP_LEVEL_SOURCE0 "asm/CortexContextSwitch.s")

set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk")
set(PICO_PLATFORM "RP2040")
set(PICO_TARGET "PICO")
set(PICO_CONFIG_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/inc/sdk_config.h")

# set(PICO_DEFAULT_BOOT_STAGE2 "boot2_w25q080") # second stage bootloader, read from flash chip

include(pico-util.cmake)
include(${HAL_COMMON_ROOT}/pico_base/generate_config_header.cmake)

include(RF24/CMakeLists.txt)

add_subdirectory(${HAL_COMMON_ROOT})
add_subdirectory(${HAL_RP2_COMMON_ROOT})
add_subdirectory(${HAL_RP2040_ROOT})

set(TARGET_DEPS
    RF24
    cmsis_core
    hardware_sync
    hardware_irq
    hardware_gpio
    hardware_spi
    hardware_i2c
    hardware_uart
    hardware_pwm
    hardware_flash
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_regs
    hardware_structs
    hardware_exception
    hardware_divider
    pico_runtime
    pico_float
    pico_double
    pico_stdlib)

include_directories(${HAL_INCLUDES})

# create our target
add_library(codal-rp2040 ${TOP_LEVEL_SOURCE} ${TOP_LEVEL_SOURCE0} ${TOP_LEVEL_SOURCE1} ${HAL_SOURCE})

target_link_libraries(codal-rp2040 codal-core ${OBJECT_FILES} ${ARCHIVE_FILES} ${TARGET_DEPS})

target_compile_definitions(codal-rp2040
    PUBLIC PICO_DEFAULT_SPI=1 # can only be 0 or 1 (as in `spi0` or `spi1`)
    PUBLIC PICO_DEFAULT_SPI_SCK_PIN=26 # depends on which SPI bus (0 or 1) is being used
    PUBLIC PICO_DEFAULT_SPI_TX_PIN=27  # depends on which SPI bus (0 or 1) is being used
    PUBLIC PICO_DEFAULT_SPI_RX_PIN=28  # depends on which SPI bus (0 or 1) is being used
)

message("${PROCESSOR_DIR_INCLUDES} ${TOP_LEVEL_INCLUDE_DIRS}")
# expose it to parent cmake.
target_include_directories(codal-rp2040 PUBLIC ${TOP_LEVEL_INCLUDE_DIRS} ${PROCESSOR_DIR_INCLUDES} ${HAL_INCLUDES})