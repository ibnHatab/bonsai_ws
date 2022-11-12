cmake_minimum_required(VERSION 3.0.2)
project(position_transform)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES geonav_transform
   CATKIN_DEPENDS
    roscpp
    cmake_modules
    geometry_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

###########
## Build ##
###########

include_directories(
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(position_transform
#    src/position_transform.cpp
# )

## Declare a C++ executable
add_executable(position_transform_node src/position_transform_node.cpp)

# target_link_libraries(position_transform
#    ${catkin_LIBRARIES}
# )
target_link_libraries(position_transform_node #position_transform
   ${catkin_LIBRARIES}
 )
