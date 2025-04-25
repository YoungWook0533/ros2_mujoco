cmake_minimum_required(VERSION 3.5.0)
project(rqt_dyros_gui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(rqt_dyros_gui_SRCS
  src/rqt_dyros_gui.cpp
)

set(rqt_dyros_gui_HDRS
  include/rqt_dyros_gui/rqt_dyros_gui.h
)

set(rqt_dyros_gui_UIS
  src/rqt_dyros_gui.ui
)

set(rqt_dyros_gui_INCLUDE_DIRECTORIES
include
${rclcpp_INCLUDE_DIRS}
${qt_gui_cpp_INCLUDE_DIRS}
${rqt_gui_cpp_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${trajectory_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${Qt5Widgets_INCLUDE_DIRS}
)

message("{rqt_gui_cpp_INCLUDE_DIRS}")
message(${rqt_gui_cpp_INCLUDE_DIRS})

qt5_wrap_cpp(rqt_dyros_gui_MOCS ${rqt_dyros_gui_HDRS})
qt5_wrap_ui(rqt_dyros_gui_UIS_H ${rqt_dyros_gui_UIS})

include_directories(
  ${rqt_dyros_gui_INCLUDE_DIRECTORIES}
  
)

add_library(${PROJECT_NAME} SHARED
  ${rqt_dyros_gui_SRCS}
  ${rqt_dyros_gui_MOCS}
  ${rqt_dyros_gui_UIS_H}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs trajectory_msgs sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${qt_gui_cpp_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  Qt5::Widgets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include)


install(PROGRAMS scripts/rqt_dyros_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()