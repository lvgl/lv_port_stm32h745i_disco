cmake_minimum_required(VERSION 3.15)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(lv_port_stm32h745i_disco_cmake_cm4 C ASM)

set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

set(TARGET_CM4 lv_port_stm32h745i_disco_cmake_cm4)
set(TARGET_CM4 ${TARGET_CM4} PARENT_SCOPE)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(COMPILE_DEFS
        STM32H745xx
        USE_STM32H745I_DISCO
        USE_HAL_DRIVER
        USE_PWR_DIRECT_SMPS_SUPPLY
        CORE_CM4
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -std=gnu11 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static -ffunction-sections -fdata-sections -Wall -fstack-usage ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity -MMD -MP")

set(SOURCE_DIR_CM4 ${CMAKE_CURRENT_SOURCE_DIR})

set(COMMON_DIR "${SOURCE_DIR_CM4}/../Common")
set(COMMON_DRIVERS_DIR "${SOURCE_DIR_CM4}/../Drivers")
set(CMSIS_DIR "${COMMON_DRIVERS_DIR}/CMSIS")

set(CM4_CORE_DIR "${SOURCE_DIR_CM4}/Core")
set(CM4_DRIVERS_DIR "${SOURCE_DIR_CM4}/Drivers")

set(STARTUP_FILE "${CM4_CORE_DIR}/Startup/startup_stm32h745xihx.s")
set(LD_FILE ${SOURCE_DIR_CM4}/STM32H745XIHX_FLASH.ld)

set(HAL_DIR "${COMMON_DRIVERS_DIR}/STM32H7xx_HAL_Driver")
set(HAL_SRC_DIR "${HAL_DIR}/Src")

file(GLOB_RECURSE CM4_CORE_SOURCES ${CM4_CORE_DIR}/Src/*.c)
file(GLOB COMMON_SOURCES ${COMMON_DIR}/Src/*.c)
file(GLOB HAL_SOURCES ${HAL_SRC_DIR}/*.c)

set(CM4_SOURCES
    ${COMMON_SOURCES}
    ${CM4_CORE_SOURCES}
    ${HAL_SOURCES}
)

set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${DEBUG_FLAGS} -mcpu=cortex-m4 -x assembler-with-cpp --specs=nano.specs")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -MMD -MP")

add_library(CM4_STARTUP_OBJ OBJECT ${STARTUP_FILE})

add_executable(${TARGET_CM4}
        ${CM4_SOURCES}
        $<TARGET_OBJECTS:CM4_STARTUP_OBJ>
)

target_compile_definitions(${TARGET_CM4} PUBLIC ${COMPILE_DEFS})

set(INCLUDE_DIRS
        ${SOURCE_DIR_CM4}
        ${CM4_CORE_DIR}/Inc
        ${CMSIS_DIR}/Include
        ${CMSIS_DIR}/Device/ST/STM32H7xx/Include
        ${HAL_DIR}/Inc
)

target_include_directories(${TARGET_CM4} PUBLIC ${INCLUDE_DIRS})

target_link_libraries(${TARGET_CM4}
        -T${LD_FILE}
        -Wl,-Map=${TARGET_CM4}.map
        --specs=nosys.specs
        -Wl,--gc-sections
        -Wl,--start-group -lc -lm -Wl,--end-group
)
