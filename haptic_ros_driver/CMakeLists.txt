cmake_minimum_required(VERSION 3.8)
project(haptic_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(DHDLIB libdhd.so lib)
find_library(USB1LIB  libusb-1.0.so)

add_executable(haptic_ros_driver
  src/haptic_ros_driver.cpp
  src/HapticDevice.cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(haptic_ros_driver
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(haptic_ros_driver
  Eigen3::Eigen
  ${DHDLIB}
  ${USB1LIB}
)

install(TARGETS haptic_ros_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY lib/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN ".so*"
)

set_target_properties(haptic_ros_driver PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
