cmake_minimum_required(VERSION 3.5)
project(urdf_geometry_parser)

# Load required dependencies for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

# Declare ament package


###########
## Build ##
###########

# Specify header include paths
include_directories(include)

# URDF geometry parser library
add_library(${PROJECT_NAME}
  include/urdf_geometry_parser/urdf_geometry_parser.h
  src/urdf_geometry_parser.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  urdf
)

#############
## Testing ##
#############



#############
## Install ##
#############

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
