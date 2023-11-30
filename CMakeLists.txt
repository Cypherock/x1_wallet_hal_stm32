cmake_minimum_required(VERSION 3.15.3)
set(CMAKE_MESSAGE_LOG_LEVEL debug)

# !! IMPORTANT !!
# This is an empty project to organise source & includes list. These are
# set into parent-project scope with PARENT_SCOPE
project(stm32-hal C)

message(STATUS "Configure (${PROJECT_NAME}): populating sources & includes")

# specify compile definitions
set(HAL_COMPILE_DEFS STM32L486xx PARENT_SCOPE)

# Specify the source files
set(HAL_SOURCES
BSP/board.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_firewall.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c
Peripherals/Buzzer/buzzer.c
Peripherals/display/SSD1306/Adafruit_GFX.c
Peripherals/display/SSD1306/SSD1306.c
Peripherals/display/SSD1306/glcdfont.c
Peripherals/nfc/adafruit_pn532.c
Src/stm32l4xx_hal_msp.c
Src/stm32l4xx_it.c
Src/syscalls.c
Src/sysmem.c
Src/system_stm32l4xx.c
build/CMakeFiles/3.22.1/CompilerIdC/CMakeCCompilerId.c
libusb/libusb.c
libusb/src/usbd_core.c
libusb/src/usbd_stm32l476_otgfs.c
porting/lv_port_disp.c
porting/lv_port_fs.c
porting/lv_port_indev.c
PARENT_SCOPE)

set(HAL_INCLUDES
${CMAKE_CURRENT_SOURCE_DIR}/BSP/
${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include/
${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include/
${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/
${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc/
${CMAKE_CURRENT_SOURCE_DIR}/Inc/
${CMAKE_CURRENT_SOURCE_DIR}/Peripherals/Buzzer/
${CMAKE_CURRENT_SOURCE_DIR}/Peripherals/display/SSD1306/
${CMAKE_CURRENT_SOURCE_DIR}/Peripherals/nfc/
${CMAKE_CURRENT_SOURCE_DIR}/libusb/inc/
${CMAKE_CURRENT_SOURCE_DIR}/libusb/
${CMAKE_CURRENT_SOURCE_DIR}/porting/
PARENT_SCOPE)
