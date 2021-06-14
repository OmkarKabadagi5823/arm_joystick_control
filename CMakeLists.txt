cmake_minimum_required(VERSION 2.8.3)
project(arm_joystick_control)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_core
  roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  
)

add_executable(arm_joystick_control src/arm_joystick_control.cpp)
target_link_libraries(arm_joystick_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS arm_joystick_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})