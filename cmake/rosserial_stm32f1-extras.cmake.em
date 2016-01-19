cmake_minimum_required(VERSION 2.8.3)

@[if DEVELSPACE]@
set(ROSSERIAL_STM32_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/stm32-cmake/gcc_stm32.cmake")
@[else]@
set(ROSSERIAL_STM32_TOOLCHAIN "${rosserial_stm32f1_DIR}/../stm32-cmake/gcc_stm32.cmake")
@[end if]@
