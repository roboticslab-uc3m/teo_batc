cmake_minimum_required(VERSION 2.8.3)
project(teo_moveit)

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
  pick_srv.srv 
  place_srv.srv
  move_group_srv.srv
  communication_srv.srv
  base_solution_srv.srv
  teo_task_template_srv.srv
)

add_message_files(
  DIRECTORY msg
  FILES
  param.msg
  param_array.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  moveit_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
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

#add_executable(pick_place_TEO src/pick_place_TEO.cpp)
#target_link_libraries(pick_place_TEO ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS pick_place_TEO DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(pick_place_libro src/pick_place_libro.cpp)
#target_link_libraries(pick_place_libro ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS pick_place_libro DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(environment_setup src/environment_setup.cpp)
target_link_libraries(environment_setup ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS environment_setup DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(tomar_cafe src/tomar_cafe.cpp)
#target_link_libraries(tomar_cafe ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS tomar_cafe DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(drink_coffee_task src/manipulation_examples/drink_coffee_task.cpp)
add_dependencies(drink_coffee_task ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(drink_coffee_task ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS drink_coffee_task DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(atomic_task_memory src/atomic_task_memory.cpp)
add_dependencies(atomic_task_memory ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(atomic_task_memory ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS atomic_task_memory DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(water_plant_task src/manipulation_examples/water_plant_task.cpp)
add_dependencies(water_plant_task ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(water_plant_task ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS water_plant_task DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(central_node src/central_node.cpp)
add_dependencies(central_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(central_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS central_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(communication src/communication/communication.cpp)
add_dependencies(communication ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(communication ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS communication DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(teo_task_template src/teo_task_template/teo_task_template.cpp)
add_dependencies(teo_task_template ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(teo_task_template ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS teo_task_template DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(publisher_test src/teo_task_template/publisher_test.cpp)
add_dependencies(publisher_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(publisher_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS publisher_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(base_solution src/base_solution/base_solution.cpp)
add_dependencies(base_solution ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(base_solution ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS base_solution DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(move_group src/move_group.cpp)
#target_link_libraries(move_group ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS move_group DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(pick src/pick.cpp)
#target_link_libraries(pick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS pick DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

