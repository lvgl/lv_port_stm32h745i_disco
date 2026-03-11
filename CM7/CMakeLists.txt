cmake_minimum_required(VERSION 3.15)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(lv_port_stm32h745i_disco_cmake_cm7 C ASM)

set(CMAKE_SYSTEM_PROCESSOR cortex-m7)

set(TARGET_CM7 lv_port_stm32h745i_disco_cmake_cm7)
set(TARGET_CM7 ${TARGET_CM7} PARENT_SCOPE)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(COMPILE_DEFS
        STM32H745xx
        USE_STM32H745I_DISCO
        USE_HAL_DRIVER
        USE_PWR_DIRECT_SMPS_SUPPLY
        CORE_CM7
        TS_MULTI_TOUCH_SUPPORTED
        LV_DRV_NO_CONF
        LV_CONF_INCLUDE_SIMPLE
)

set(CMAKE_C_FLAGS_DEBUG "-g3 -O0")

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug" )
    add_definitions(-DDEBUG)
    set(OPTIMISATION_FLAGS "-O0")
else()
    set(OPTIMISATION_FLAGS "-Os")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m7 -std=gnu11 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard --specs=nano.specs")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static -ffunction-sections -fdata-sections -Wall -fstack-usage ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity -MMD -MP ${OPTIMISATION_FLAGS}")

set(SOURCE_DIR_CM7 ${CMAKE_CURRENT_SOURCE_DIR})

set(COMMON_DIR "${SOURCE_DIR_CM7}/../Common")
set(COMMON_DRIVERS_DIR "${SOURCE_DIR_CM7}/../Drivers")
set(CM7_DRIVERS_DIR "${SOURCE_DIR_CM7}/Drivers")
set(CM7_CORE_DIR "${SOURCE_DIR_CM7}/Core")

set(BSP_DIR "${CM7_DRIVERS_DIR}/BSP")
set(BSP_PLATFORM_DIR "${BSP_DIR}/STM32H745I-DISCO")
set(BSP_COMPONENTS_DIR "${BSP_DIR}/Components")
set(CMSIS_DIR "${COMMON_DRIVERS_DIR}/CMSIS")
set(HAL_DIR "${COMMON_DRIVERS_DIR}/STM32H7xx_HAL_Driver")
set(HAL_SRC_DIR "${HAL_DIR}/Src")
set(LVGL_DIR "${SOURCE_DIR_CM7}/lvgl")
set(APP_DIR "${SOURCE_DIR_CM7}/Application")


set(STARTUP_FILE "${CM7_CORE_DIR}/Startup/startup_stm32h745xihx.s")
set(LD_FILE ${SOURCE_DIR_CM7}/STM32H745XIHX_FLASH.ld)

file(GLOB_RECURSE CM7_CORE_SOURCES CONFIGURE_DEPENDS ${CM7_CORE_DIR}/Src/*.c)
file(GLOB COMMON_SOURCES CONFIGURE_DEPENDS ${COMMON_DIR}/Src/*.c)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS ${HAL_SRC_DIR}/*.c)
file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS ${BSP_DIR}/*.c)
file(GLOB_RECURSE LVGL_SOURCES CONFIGURE_DEPENDS ${LVGL_DIR}/src/*.c)
file(GLOB_RECURSE LVGL_DEMO_SOURCES CONFIGURE_DEPENDS ${LVGL_DIR}/demos/*.c)

set(CM7_SOURCES
    ${COMMON_SOURCES}
    ${CM7_CORE_SOURCES}
    ${HAL_SOURCES}
    ${BSP_SOURCES}
    ${LVGL_SOURCES}
    ${LVGL_DEMO_SOURCES}
        lv_conf.h
)

set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${DEBUG_FLAGS} -mcpu=cortex-m7 -x assembler-with-cpp --specs=nano.specs")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -MMD -MP")

add_library(CM7_STARTUP_OBJ OBJECT ${STARTUP_FILE})

add_executable(${TARGET_CM7}
        ${CM7_SOURCES}
        $<TARGET_OBJECTS:CM7_STARTUP_OBJ>
)

target_compile_definitions(${TARGET_CM7} PUBLIC ${COMPILE_DEFS})

set(INCLUDE_DIRS
        ${SOURCE_DIR_CM7}
        ${CM7_CORE_DIR}/Inc
        ${BSP_DIR}/Components/Common
        ${BSP_PLATFORM_DIR}
        ${CMSIS_DIR}/Include
        ${CMSIS_DIR}/Device/ST/STM32H7xx/Include
        ${HAL_DIR}/Inc
        ${LVGL_DIR}
        ${LVGL_DIR}/src
)

target_include_directories(${TARGET_CM7} PUBLIC ${INCLUDE_DIRS})

target_link_libraries(${TARGET_CM7}
        -T${LD_FILE}
        -Wl,-Map=${TARGET_CM7}.map
        --specs=nosys.specs
        -Wl,--gc-sections
        -Wl,--start-group -lc -lm -Wl,--end-group
)
