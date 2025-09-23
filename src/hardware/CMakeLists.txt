cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(var_dependencies 
  ros2_interface 
  ros2_utils 
  rclcpp
  pcl_conversions
  rclcpp
  rclpy
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)

foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()

add_executable(keyboard_input src/keyboard_input.cpp)
ament_target_dependencies(keyboard_input rclcpp std_msgs)

add_executable(io_stm32 src/io_stm32.cpp)
ament_target_dependencies(io_stm32 ${var_dependencies})

add_executable(io_lslidar_n301 src/io_lslidar_n301.cpp)
ament_target_dependencies(io_lslidar_n301 ${var_dependencies})
target_link_libraries(io_lslidar_n301 ${PCL_LIBRARIES})

install(TARGETS
  keyboard_input
  io_stm32
  io_lslidar_n301
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
