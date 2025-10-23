cmake_minimum_required(VERSION 3.8)
project(custom_interfaces2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Action files
set(action_files
  "action/MoveAndRotate.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# Export dependencies to downstream packages
ament_export_dependencies(rosidl_default_runtime)

# Enable tests (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

