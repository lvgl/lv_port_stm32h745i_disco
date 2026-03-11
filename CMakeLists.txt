cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-toolchain.cmake)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# These are required to prevent build platform-specific libraries from being automagically added
# to the ld command line.
set(CMAKE_C_STANDARD_LIBRARIES "")
set(CMAKE_CXX_STANDARD_LIBRARIES "")

project(lv_port_stm32h745i_disco_cmake C ASM)

set(CM4_BINARY_DIR ${CMAKE_BINARY_DIR}/CM4)
set(CM7_BINARY_DIR ${CMAKE_BINARY_DIR}/CM7)

add_subdirectory(CM7 ${CM7_BINARY_DIR} )
add_subdirectory(CM4 ${CM4_BINARY_DIR} )

set(EXTERNAL_LOADER "${CMAKE_SOURCE_DIR}/MT25TL01G_STM32H745I-DISCO.stldr")

add_custom_target(FLASH_CM4
        COMMAND STM32_Programmer_CLI --connect port=SWD --write ${CMAKE_BINARY_DIR}/CM4/${TARGET_CM4}.elf --start
        DEPENDS ${TARGET_CM4}
)

add_custom_target(FLASH_CM7
        COMMAND STM32_Programmer_CLI --connect port=SWD -el "${EXTERNAL_LOADER}" --write ${CMAKE_BINARY_DIR}/CM7/${TARGET_CM7}.elf --start
        DEPENDS ${TARGET_CM7}
)
