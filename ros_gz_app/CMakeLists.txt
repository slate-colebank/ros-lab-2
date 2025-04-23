cmake_minimum_required(VERSION 3.5)

project(ros_gz_app)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


install(
  PROGRAMS
    src/robot_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
