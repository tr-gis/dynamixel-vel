################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation roscpp)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(FILES joint_control.msg)
################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

generate_messages(
  DEPENDENCIES std_msgs)
################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk
CATKIN_DEPENDS message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamixel_sdk
  src/dynamixel_sdk/packet_handler.cpp
  src/dynamixel_sdk/protocol1_packet_handler.cpp
  src/dynamixel_sdk/protocol2_packet_handler.cpp
  src/dynamixel_sdk/group_sync_read.cpp
  src/dynamixel_sdk/group_sync_write.cpp
  src/dynamixel_sdk/group_bulk_read.cpp
  src/dynamixel_sdk/group_bulk_write.cpp
  src/dynamixel_sdk/port_handler.cpp
  src/dynamixel_sdk/port_handler_linux.cpp
  src/dynamixel_sdk/controller_dynamixel_sdk.cpp

)

file(GLOB SOURCES "src/dynamixel_sdk/*.cpp")


add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_sdk ${catkin_LIBRARIES})

add_executable(new_rw src/dynamixel_sdk/main.cpp ${SOURCES})
target_link_libraries(new_rw ${Boost_LIBRARIES} ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
