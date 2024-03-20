cmake_minimum_required(VERSION 3.8)
project(aprs_plugins)

# set(THIS_PACKAGE_INCLUDE_DEPENDS

# )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})


add_library(ConveyorPlugin SHARED src/conveyor_plugin.cpp)
target_include_directories(ConveyorPlugin PUBLIC include)
# set_property(TARGET AriacCameraPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(ConveyorPlugin
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim8::gz-sim8)
# ament_target_dependencies(AriacCameraPlugin

# )

ament_export_libraries(ConveyorPlugin)
ament_package()


install(TARGETS
  ConveyorPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)