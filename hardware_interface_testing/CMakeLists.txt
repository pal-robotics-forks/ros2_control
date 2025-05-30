cmake_minimum_required(VERSION 3.16)
project(hardware_interface_testing LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  hardware_interface
  lifecycle_msgs
  pluginlib
  rclcpp_lifecycle
  ros2_control_test_assets
  fmt
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(test_components SHARED
test/test_components/test_actuator.cpp
test/test_components/test_sensor.cpp
test/test_components/test_system.cpp
test/test_components/test_actuator_exclusive_interfaces.cpp)
ament_target_dependencies(test_components hardware_interface pluginlib ros2_control_test_assets)
install(TARGETS test_components
DESTINATION lib
)
pluginlib_export_plugin_description_file(
hardware_interface test/test_components/test_components.xml)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_resource_manager test/test_resource_manager.cpp)
  target_link_libraries(test_resource_manager)
  ament_target_dependencies(test_resource_manager hardware_interface ros2_control_test_assets)

  ament_add_gmock(test_resource_manager_prepare_perform_switch test/test_resource_manager_prepare_perform_switch.cpp)
  ament_target_dependencies(test_resource_manager_prepare_perform_switch hardware_interface ros2_control_test_assets)

endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
