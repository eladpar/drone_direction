cmake_minimum_required(VERSION 3.5)
project(drone_direction)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executables
install(
  PROGRAMS
  accurate_model/accurate_model.py
  heuristic_model/heuristic_model.py
  controller/controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
