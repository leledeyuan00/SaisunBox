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
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(saisun_msgs REQUIRED)
find_package(Eigen REQUIRED)
find_package(rclcpp_components REQUIRED)

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
  /usr/include/python3.8
  /usr/include/opencv4)

#### Target package dependencies ####
ament_target_dependencies(sensing
  rclcpp
  rcutils
  geometry_msgs
  PCL_IO
  SmartEye
)
ament_export_include_directories(include)
#### Target link libraries ####
target_link_libraries(sensing
  python3.8
  opencv_imgcodecs
)

#### Add test ####
add_executable(vision_node 
  src/vision_node.cpp
)
target_link_libraries(vision_node
  sensing
)
ament_target_dependencies(vision_node
  "rclcpp" 
  "rclcpp_action" 
  "saisun_msgs"
  "Eigen"
  "rclcpp_components"
  "cv_bridge"
)

#### Install ####
install(TARGETS sensing
  # DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS vision_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/
)
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
  lib/smarteye/lib/GxU3VTL.cti
  DESTINATION lib
  # lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
