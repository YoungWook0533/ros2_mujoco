# fr3_husky_controller/CMakeLists.txt

# 상위 CMake에서 find_package(...)를 이미 했다고 가정.
# 필요하다면 중복으로 find_package(...)를 호출해도 무방.

#-----------------------------------------
# include 디렉토리
#-----------------------------------------
include_directories(
  include
)

#-----------------------------------------
# 1) C++ 라이브러리 (로봇 로직)
#-----------------------------------------
add_library(fr3_husky_controller SHARED
  src/controller.cpp
  src/robot_data.cpp
)

target_link_libraries(fr3_husky_controller
  Eigen3::Eigen
  rclcpp::rclcpp
  qpOASES
  yaml-cpp
  # 등 필요한 라이브러리
)

#-----------------------------------------
# 2) Python wrapper 라이브러리 (Boost.Python)
#-----------------------------------------
add_library(fr3_husky_controller_wrapper_cpp SHARED
  src/bindings.cpp
)

set_target_properties(fr3_husky_controller_wrapper_cpp PROPERTIES
  OUTPUT_NAME "fr3_husky_controller_wrapper_cpp"  # 실제 so 파일 이름
  PREFIX ""                                 # (리눅스에서 'lib' 제거)
)

target_link_libraries(fr3_husky_controller_wrapper_cpp
  fr3_husky_controller
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  Python3::Python
  eigenpy::eigenpy
  pinocchio::pinocchio
  rclcpp::rclcpp
)

#-----------------------------------------
# 3) 헤더(선택) 설치
#-----------------------------------------
# install(DIRECTORY include/
#   DESTINATION include/
# )

#-----------------------------------------
# 4) Python 모듈 설치 (해당 폴더 안에 __init__.py 등 있다면)
#-----------------------------------------
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
ament_python_install_package(fr3_husky_controller)
# => fr3_husky_controller/ 디렉토리를 파이썬 모듈로 설치 (setup.py는 상위에서 통합되거나 여기에 둘 수도 있음)

#-----------------------------------------
# 5) Boost.Python wrapper 라이브러리 설치
#    => 최종적으로 python3.x/site-packages/ 밑에 "fr3_husky_controller_wrapper_cpp.so"가 위치
#-----------------------------------------
install(TARGETS fr3_husky_controller fr3_husky_controller_wrapper_cpp
    LIBRARY DESTINATION lib
)

install(TARGETS fr3_husky_controller_wrapper_cpp
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)


install(DIRECTORY robot/ DESTINATION share/${PROJECT_NAME}/fr3_husky_controller/robot)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/fr3_husky_controller/config)  # yaml path
#-----------------------------------------
# NOTE: 여기서는 ament_package()를 호출하지 않음.
#       상위(루트) CMakeLists에서만 ament_package() 호출.
#-----------------------------------------
