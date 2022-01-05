# LVGL ported to the STM32H745I Discovery.

The [STM32H745I Discovery kit](https://www.st.com/en/evaluation-tools/stm32h745i-disco.html) is an affordable development board with:
* 480x272 RGB interface LCD with capacitive multi-touch panel
* Arm® Cortex®-M7 480MHz and Arm® Cortex®-M4 240MHz dual-core
* 2 Mbytes Flash Memory
* 1 Mbyes RAM
* 2 x 512-Mbit Quad-SPI NOR Flash Memory
* 128-Mbit SDRAM
* GPU.

The CubeMX drivers are used to initilaize the peripherals. 

The project was created with CubeIDE.

## Get started
- Clone the project: `git clone https://github.com/lvgl/lv_port_stm32h745i_disco.git --recurse-submodules`
- Import into CubeIDE

## Notes

- STM32H745I-Disco uses 
    - TFT 
        - TFT model's RK043FN48H
        - BSP/stm32h745i_discovery_lcd.c initialize LTDC, DMA2D, TIM8 Channel 3
        - BSP/stm32h745i_discovery_lcd.c drive RK043FN48H 
        - RK043FN48H has touch control capability and uses FT5336QQ Capacitive Touch Panel Controller
            - BSP/stm32h745i_discovery_bus.c initialize I2C4 and control it
    - SPI NOR Flash
        - Flash model's MT25TL01G
        - BSP/stm32h745i_discovery_qspi.c initialize FLASH
    - SDRAM
        - SDRAM model's MT48LC4M32B2 
        - BSP/stm32h745i_discovery_sdram.c initialize SDRAM

BSP initialize necessary peripherals  so we must disable the CubeMX generated codes.

- CubeMX integration with BSP
    - Disable CubeMX generated Functions
        - HAL_LTDC_MspInit,  HAL_LTDC_MspDeInit, MX_LTDC_Init
        - MX_DMA2D_Init, HAL_DMA2D_MspInit, HAL_DMA2D_MspDeInit
        - MX_TIM8_Init, HAL_TIM_PWM_MspInit, HAL_TIM_MspPostInit, HAL_TIM_PWM_MspDeInit
        - MX_I2C4_Init, HAL_I2C_MspInit, HAL_I2C_MspDeInit
        - MX_QUADSPI_Init, HAL_QSPI_MspInit, HAL_QSPI_MspDeInit, QUADSPI_IRQHandler
        - MX_MDMA_Init, MDMA_IRQHandler
        - MX_FMC_Init, HAL_FMC_MspInit, HAL_FMC_MspDeInit
        - HAL_SDRAM_MspInit, HAL_SDRAM_MspDeInit
        - MX_GPIO_Init, EXTI2_IRQHandler
        
 SDRAM Memory Map

 - SDRAM Start Address : 0xD0000000
    - LCD Buffer   : 0xD0000000 - 0xD007F800 : Size 480*272*4=522.240B 
    - LVGL Buffer  : 0xD007F810 - 0xD00FF010 : Size 480*272*4=522.240B
    - LVGL Buffer 2: 0xD00FF020 - 0xD017E820 : Size 480*272*4=522.240B