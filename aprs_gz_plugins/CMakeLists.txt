cmake_minimum_required(VERSION 3.8)
project(aprs_gz_plugins)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(conveyor_interfaces REQUIRED)
find_package(aprs_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  gz-plugin2
  gz-cmake3
  gz-sim8
  rclcpp
  tf2_ros
  conveyor_interfaces
  aprs_interfaces
)

# Conveyor plugin

add_library(ConveyorPlugin SHARED src/conveyor_plugin.cpp)
target_include_directories(ConveyorPlugin PUBLIC include)

ament_target_dependencies(ConveyorPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Gear plugin

add_library(GearPlugin SHARED src/gear_plugin.cpp)
target_include_directories(GearPlugin PUBLIC include)

ament_target_dependencies(GearPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Slot plugin

add_library(SlotPlugin SHARED src/slot_plugin.cpp)
target_include_directories(SlotPlugin PUBLIC include)

ament_target_dependencies(SlotPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# State saver plugin

add_library(StateSaverPlugin SHARED src/state_saver_plugin.cpp)
target_include_directories(StateSaverPlugin PUBLIC include)

ament_target_dependencies(StateSaverPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Spawn part

add_library(SpawnPart SHARED src/spawn_part.cpp)
target_include_directories(SpawnPart PUBLIC include)

ament_target_dependencies(SpawnPart
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Tf publisher plugin

add_library(TFPublisherPlugin SHARED src/tf_publisher_plugin.cpp)
target_include_directories(TFPublisherPlugin PUBLIC include)

ament_target_dependencies(TFPublisherPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_libraries(
  ConveyorPlugin
  GearPlugin
  SlotPlugin
  SpawnPart
  StateSaverPlugin
  TFPublisherPlugin
)

install(
  TARGETS
    ConveyorPlugin
    GearPlugin
    SlotPlugin
    SpawnPart
    StateSaverPlugin
    TFPublisherPlugin
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()