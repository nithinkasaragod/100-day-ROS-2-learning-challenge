cmake_minimum_required(VERSION 3.5)
project(two_wheel_bot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files (only if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install worlds if exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  install(DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()

