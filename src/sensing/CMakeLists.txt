cmake_minimum_required(VERSION 3.5)

#### Project name ####
project(sensing)

#### Default to C99 ####
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

#### Default to C++14 ####
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

#### Find dependencies ####
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL COMPONENTS io REQUIRED)
find_package(geometry_msgs REQUIRED)

# Set SmartEye dependency
set(SmartEye_FOUND true)
set(SmartEye_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(SmartEye_LIBRARY_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/lib/smarteye/lib")
set(SmartEye_LIBRARIES "${SmartEye_LIBRARY_DIRS}/libSmarteyeAPI.so")

#### Add library ####
add_library(sensing SHARED
  src/sensing/SensingServer.cpp 
  src/sensing/SmartEyeCameraController.cpp
  src/sensing/FakeCameraController.cpp
  src/sensing/BoxLocalizationAlgo.cpp
)

#### Include DIR ####
target_include_directories(sensing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/pcl-1.10
  /usr/include/python3.8)

#### Target package dependencies ####
ament_target_dependencies(sensing
  rclcpp
  rcutils
  geometry_msgs
  PCL_IO
  SmartEye
)

#### Target link libraries ####
target_link_libraries(sensing
  python3.8
)

#### Add test ####
add_executable(test_sensing 
  src/test.cpp
)

target_link_libraries(test_sensing
  sensing
)

#### Install ####
install(TARGETS sensing
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS test_sensing
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install python script
install(
  DIRECTORY
  script/box
  DESTINATION
  script/${PROJECT_NAME}
)

# Install smart eye libraries
install(
  PROGRAMS
  lib/smarteye/lib/libSmarteyeAPI.so
  lib/smarteye/lib/libicui18n.so.60
  lib/smarteye/lib/libicuuc.so.60
  lib/smarteye/lib/libgxiapi.so
  lib/smarteye/lib/libicudata.so.60
  DESTINATION
  lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
