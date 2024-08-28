cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(ZJUDancer)

#找到ros专用cmake包和rclcpp
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#找到gz的注册包和系统包
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

#列出等下要编译的文件
set(ZJUDANCER_CXX_SOURCES
  src/MotionController.cpp
  src/IO.cpp 
  src/Parameters.cpp
  src/InverseKinematics.cpp
  src/OneFootLanding.cpp
  src/ThreeInterpolation.cpp
 )

add_library(ZJUDancer SHARED ${ZJUDANCER_CXX_SOURCES})
target_include_directories(ZJUDancer PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)
set_property(TARGET ZJUDancer PROPERTY CXX_STANDARD 17)
#链接到ros库
ament_target_dependencies(
  ZJUDancer
  "rclcpp"
  "std_msgs"
)
#链接到gazebo的系统库
target_link_libraries(ZJUDancer
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})