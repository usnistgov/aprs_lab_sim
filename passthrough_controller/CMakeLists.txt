cmake_minimum_required(VERSION 3.8)
project(passthrough_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  hardware_interface
  controller_interface
  pluginlib
  generate_parameter_library
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  passthrough_controller_parameters # cmake target name for the parameter library
  src/passthrough_controller_parameters.yaml # path to input yaml file
)

# Build gripper controller
add_library(passthrough_controller SHARED
  src/passthrough_controller.cpp
)

target_include_directories(passthrough_controller PUBLIC
  include
)

target_link_libraries(passthrough_controller PUBLIC
passthrough_controller_parameters
)

ament_target_dependencies(passthrough_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface passthrough_controller.xml)

install(
  TARGETS
    passthrough_controller
    passthrough_controller_parameters
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()