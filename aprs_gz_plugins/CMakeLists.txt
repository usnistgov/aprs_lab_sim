cmake_minimum_required(VERSION 3.8)
project(aprs_gz_plugins)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  conveyor_interfaces
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(ament_index_cpp REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})


add_library(ConveyorPlugin SHARED src/conveyor_plugin.cpp)
target_include_directories(ConveyorPlugin PUBLIC include)

target_link_libraries(ConveyorPlugin
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  ament_index_cpp::ament_index_cpp
)

ament_target_dependencies(ConveyorPlugin 
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_libraries(ConveyorPlugin)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()


install(TARGETS
  ConveyorPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)