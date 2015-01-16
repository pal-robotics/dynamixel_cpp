cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_cpp)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/DynControl.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/dynamixel_device.cpp
  src/dynamixel.c
  src/dxl_hal.c
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(dynamixel_node src/dynamixel_node.cpp)
target_link_libraries(dynamixel_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(dynamixel_node ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

install(TARGETS dynamixel_cpp dynamixel_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
