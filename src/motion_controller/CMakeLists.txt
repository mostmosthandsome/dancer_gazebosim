cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(ZJUDancer)

#找到ros专用cmake包和rclcpp
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#找到gz的注册包和系统包
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
gz_find_package(gz-sensors8 REQUIRED COMPONENTS imu)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

#和仿真交互的IO库
set(IO_CXX_SOURCES
  src/IO.cpp
)
add_library(IO SHARED ${IO_CXX_SOURCES})
target_include_directories(IO PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_property(TARGET IO PROPERTY CXX_STANDARD 17)#需要用C++17或以上编译
target_link_libraries(IO
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-sensors${GZ_SENSORS_VER}::imu
)

#dmotion库
set(MOTION_CXX_SOURCES
  src/Parameters.cpp
  src/InverseKinematics.cpp
  src/OneFootLanding.cpp
  src/ThreeInterpolation.cpp
  src/PendulumWalk.cpp
  src/NewMotion.cpp
  src/Climb.cpp
  src/ForwardKinematics.cpp
  src/Planner.cpp
)
add_library(dmotion SHARED ${MOTION_CXX_SOURCES})
target_include_directories(dmotion PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)
#链接到ros
ament_target_dependencies(
  dmotion
  "rclcpp"
  "geometry_msgs"
)
set_property(TARGET dmotion PROPERTY CXX_STANDARD 17)


#列出等下要编译的文件
set(ZJUDANCER_CXX_SOURCES
  src/MotionController.cpp
 )

add_library(ZJUDancer SHARED ${ZJUDANCER_CXX_SOURCES})
target_include_directories(ZJUDancer PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)
set_property(TARGET ZJUDancer PROPERTY CXX_STANDARD 17)
#链接到gazebo的系统库
target_link_libraries(ZJUDancer IO dmotion)