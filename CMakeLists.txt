cmake_minimum_required(VERSION 3.0.2)
project(rs_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/rs_planner_node.cpp src/reeds_shepp.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
