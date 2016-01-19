# rosserial_stm32f1
rosserial package for STM32F10x MCUs using stm32_cmake and ST's StdPeriph library.

## Instructions:

### Download rosserial_stm32f1
You must get this package into your workspace, and the respective stm32_cmake branch for StdPeriph library.

    cd <your_ws_dir>/src/
    git clone https://github.com/robosavvy/rosserial_stm32f1.git
    git submodule init
    git submodule update
    
### Prepare stm32's StdPeriph library
TODO: For now, follow stm32_cmake's instructions.

### Build and install rosserial_stm32f1
Now go to your workspace, build and install the package.

    cd <your_ws_dir>
    catkin_make
    catkin_make install
