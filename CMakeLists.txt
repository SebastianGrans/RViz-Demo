cmake_minimum_required(VERSION 3.5)
project(rviz_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)



#############
# axes_demo #
#############
add_executable(axes_demo
  src/axes_demo.cpp
)
# Specify dependencies for this specific target
ament_target_dependencies(axes_demo
  rclcpp
  tf2_ros
)

# Install in /base_ws/install/ so that `ros2 run` can find it. 
install(TARGETS
  axes_demo
  DESTINATION lib/${PROJECT_NAME}
)


##############
# image_demo #
##############
add_executable(image_demo
  src/image_demo.cpp
)
# Specify dependencies for this specific target
ament_target_dependencies(image_demo
  rclcpp
  cv_bridge
  sensor_msgs
)

# Install in /base_ws/install/ so that `ros2 run` can find it. 
install(TARGETS
  image_demo
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
