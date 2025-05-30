cmake_minimum_required(VERSION 3.5)
project(dyros_robot_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3 -march=native)

# 1) 패키지 설정
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(eigenpy REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(qpOASES REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${eigenpy_INCLUDE_DIRS}
)

# 2) 서브 디렉토리들
# fr3_controller, husky_controller, etc.
add_subdirectory(fr3_controller)
add_subdirectory(husky_controller)
add_subdirectory(fr3_husky_controller)

# 3) (옵션) Python 설치
# ament_python_install_package(${PROJECT_NAME})
# => 만약 상위 레벨 Python 모듈이 있다면 설치 가능 (없으면 빼도 됨)

# 4) ament_package

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
