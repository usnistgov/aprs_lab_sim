cmake_minimum_required(VERSION 3.8)
project(aprs_gz_plugins)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(conveyor_interfaces REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  gz-plugin2
  gz-cmake3
  gz-sim8
  rclcpp
  conveyor_interfaces
)

# Conveyor plugin

add_library(ConveyorPlugin SHARED src/conveyor_plugin.cpp)
target_include_directories(ConveyorPlugin PUBLIC include)

ament_target_dependencies(ConveyorPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_library(GearPlugin SHARED src/gear_plugin.cpp)
target_include_directories(GearPlugin PUBLIC include)

ament_target_dependencies(GearPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_libraries(
  ConveyorPlugin
  GearPlugin
)

install(
  TARGETS
    ConveyorPlugin
    GearPlugin
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()