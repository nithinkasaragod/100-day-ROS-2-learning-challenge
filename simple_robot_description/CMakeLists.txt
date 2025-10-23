cmake_minimum_required(VERSION 3.5)
project(simple_robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install folders so they appear in install/share/<pkg>
install(
  DIRECTORY launch urdf meshes resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
