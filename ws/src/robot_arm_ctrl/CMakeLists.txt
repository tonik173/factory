cmake_minimum_required(VERSION 3.8)
project(robot_arm_ctrl LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclpy
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# compile
add_library(
  robot_arm_ctrl
  SHARED
  hardware/robot_arm_driver.cpp
)
target_compile_features(robot_arm_ctrl PUBLIC cxx_std_17)
target_include_directories(robot_arm_ctrl PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/robot_arm_ctrl>
)
ament_target_dependencies(
  robot_arm_ctrl PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "robot_arm_ctrl_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robot_arm_ctrl.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/robot_arm_ctrl
)
install(
  DIRECTORY launch urdf rviz config meshes worlds
  DESTINATION share/robot_arm_ctrl
)
install(TARGETS robot_arm_ctrl
  EXPORT export_robot_arm_ctrl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  robot_arm_ctrl/scripts/forward_position.py
  robot_arm_ctrl/scripts/joint_trajectory_position.py
  robot_arm_ctrl/scripts/trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_robot_arm_ctrl HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
