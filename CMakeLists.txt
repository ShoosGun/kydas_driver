cmake_minimum_required(VERSION 3.0.2)
project(kydas_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
## DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/kydas_driver.cpp
                         src/kydas_driver_reading_serial.cpp
                         src/kydas_driver_sending_to_serial.cpp
                         src/rs232.c)

##add_executable(kydas_driver src/kydas_driver.cpp
##                            src/kydas_driver_interface.cpp
##                            src/kydas_driver_reading_serial.cpp
##                            src/kydas_driver_sending_to_serial.cpp
##                            src/rs232.c)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
