cmake_minimum_required(VERSION 3.8)
project(fixed_wing_formation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# XTDrone's formation law and its ECL TECS copy, built as they are: their warnings are not this port's to fix.
add_library(formation_law STATIC
  src/formation_controller/formation_controller.cpp
  src/formation_controller/abs_formation_controller.cpp
  src/fixed_wing_lib/vertical_controller/tecs.cpp)
target_include_directories(formation_law SYSTEM PUBLIC src)
include_directories(include)

set(PORT_WARNINGS -Wall -Wextra -Wpedantic)

add_executable(fw_formation_follower src/fw_formation_follower.cpp)
target_compile_options(fw_formation_follower PRIVATE ${PORT_WARNINGS})
target_link_libraries(fw_formation_follower formation_law)
ament_target_dependencies(fw_formation_follower rclcpp px4_msgs std_msgs)

install(TARGETS fw_formation_follower DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_law test/test_control_law.cpp)
  target_compile_options(test_control_law PRIVATE ${PORT_WARNINGS})
  target_link_libraries(test_control_law formation_law)
  ament_target_dependencies(test_control_law px4_msgs)
endif()

ament_package()
