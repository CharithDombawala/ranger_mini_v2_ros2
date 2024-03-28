cmake_minimum_required(VERSION 3.8)
project(four_wheel_steering_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
  
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  nav_msgs
  parameter_traits
  ackermann_msgs
  tf2_msgs
  geometry_msgs
  control_msgs
  tf2
  tf2_geometry_msgs
  gazebo_msgs
)    

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()


generate_parameter_library(four_wheel_steering_controller_parameters src/four_wheel_steering_controller_parameters.yaml)
  

add_library(
  four_wheel_steering_controller
  SHARED
  src/four_wheel_steering_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

#add_executable(minimal_node src/four_wheel_steering_controller.cpp)

target_compile_features(four_wheel_steering_controller PUBLIC cxx_std_17)
target_include_directories(four_wheel_steering_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(four_wheel_steering_controller PUBLIC
  rclcpp::rclcpp
  four_wheel_steering_controller_parameters
)


ament_target_dependencies(four_wheel_steering_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(four_wheel_steering_controller PRIVATE "FOUR_WHEEL_STEERING_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface four_wheel_steering_controller.xml)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS four_wheel_steering_controller four_wheel_steering_controller_parameters
  EXPORT export_four_wheel_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_four_wheel_steering_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
