cmake_minimum_required(VERSION 3.8)
project(gazebo_env)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})

ament_package()