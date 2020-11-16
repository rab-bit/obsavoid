cmake_minimum_required(VERSION 2.8.3)
project(p3at_tutorial)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  amcl
  base_local_planner
  base_local_planner
  gmapping
  move_base
  navfn
  roslaunch
  tf
  urdf
  xacro
)
catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES radom_walk
 CATKIN_DEPENDS amcl base_local_planner base_local_planner gmapping move_base navfn roslaunch tf urdf xacro
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(random_walk src/random_walk.cpp)
# target_link_libraries(random_walk ${catkin_LIBRARIES})

# Declare a C++ executable
# add_executable(random_walk_node src/random_walk_node.cpp)
# target_link_libraries(random_walk_node random_walk ${catkin_LIBRARIES})

# Declara a C++ de VFH
add_library(VFH src/VFH.cpp)
target_link_libraries(VFH ${catkin_LIBRARIES})

# Declara o C++ de obstacle avoidance
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance VFH ${catkin_LIBRARIES})
#target_compile_features(p3at_tutorial PRIVATE cxx_range_for)
#add_dependencies(obstacle_avoidance beginner_tutorials_generate_messages_cpp)
