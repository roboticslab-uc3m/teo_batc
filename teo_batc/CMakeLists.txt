cmake_minimum_required(VERSION 2.8.3)
project(teo_batc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  moveit_msgs
  message_generation
)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
  DIRECTORY srv 
  FILES
  trajectory_gen_srv.srv
  move_group_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    message_runtime
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(central_node_trajectory src/central_node_trajectory.cpp)
add_dependencies(central_node_trajectory ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(central_node_trajectory ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS central_node_trajectory DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
