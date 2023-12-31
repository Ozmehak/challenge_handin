cmake_minimum_required(VERSION 3.8)
project(challenge_handin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY  launch include
  DESTINATION share/${PROJECT_NAME}
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/velocity_converter.py
  scripts/robot_plotting.py
  DESTINATION lib/${PROJECT_NAME}/
)
ament_package()
