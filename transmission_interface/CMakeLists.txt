cmake_minimum_required(VERSION 3.16)
project(transmission_interface LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  fmt
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(transmission_interface SHARED
  src/simple_transmission_loader.cpp
  src/four_bar_linkage_transmission_loader.cpp
  src/differential_transmission_loader.cpp
)
target_include_directories(transmission_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/transmission_interface>
)
target_link_libraries(transmission_interface PUBLIC
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      fmt::fmt)
pluginlib_export_plugin_description_file(transmission_interface ros2_control_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_simple_transmission
    test/simple_transmission_test.cpp
  )
  target_link_libraries(test_simple_transmission transmission_interface)

  ament_add_gmock(test_differential_transmission
    test/differential_transmission_test.cpp
  )
  target_link_libraries(test_differential_transmission transmission_interface)

  ament_add_gmock(test_four_bar_linkage_transmission
    test/four_bar_linkage_transmission_test.cpp
  )
  target_link_libraries(test_four_bar_linkage_transmission transmission_interface)

  ament_add_gmock(test_simple_transmission_loader
    test/simple_transmission_loader_test.cpp
  )
  target_link_libraries(test_simple_transmission_loader
                        transmission_interface
                        ros2_control_test_assets::ros2_control_test_assets)

  ament_add_gmock(test_four_bar_linkage_transmission_loader
    test/four_bar_linkage_transmission_loader_test.cpp
  )
  target_link_libraries(test_four_bar_linkage_transmission_loader
                        transmission_interface
                        ros2_control_test_assets::ros2_control_test_assets)

  ament_add_gmock(test_differential_transmission_loader
    test/differential_transmission_loader_test.cpp
  )
  target_link_libraries(test_differential_transmission_loader
                        transmission_interface
                        ros2_control_test_assets::ros2_control_test_assets)

  ament_add_gmock(
    test_utils
    test/utils_test.cpp
  )
  target_include_directories(test_utils PUBLIC include hardware_interface)
  target_link_libraries(test_utils hardware_interface::hardware_interface)
endif()

install(
  DIRECTORY include/
  DESTINATION include/transmission_interface
)
install(TARGETS transmission_interface
  EXPORT export_transmission_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_transmission_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
