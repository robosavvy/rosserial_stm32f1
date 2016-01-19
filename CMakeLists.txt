cmake_minimum_required(VERSION 2.8.3)
project(rosserial_stm32f1)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS rosserial_stm32f1-extras.cmake
)

install(
  DIRECTORY
    src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(
  DIRECTORY stm32-cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS 
    src/rosserial_stm32f1/make_libraries
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
