cmake_minimum_required(VERSION 3.5)
project(yahboom_dog_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# Install URDF, launch, config
install(DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

# Chỉ copy meshes nếu tồn tại
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/xacro_watcher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

