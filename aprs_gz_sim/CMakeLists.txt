cmake_minimum_required(VERSION 3.5)
project(aprs_gz_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclpy
  rclcpp
  aprs_interfaces
  ros_gz_interfaces
)

find_package(gz-transport13 REQUIRED)

find_package(gz-msgs10 REQUIRED)

# set(GZ_TARGET_PREFIX gz)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(DIRECTORY
  launch
  gz_models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  nodes/environment_startup_node.py
  nodes/combined_load_controllers.py
  nodes/seperate_load_controllers.py
  nodes/clone_real_world_node.py
  nodes/dummy_motoman_joints_publisher.py
  nodes/dummy_trays_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()