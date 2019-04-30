cmake_minimum_required(VERSION 2.8.3)
project(flexbe_loop_nodes)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  tough_controller_interface
  tough_common
  tough_moveit_planners
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

#generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 	  roscpp
                    rospy
                    std_msgs
                    tough_controller_interface
                    tough_common
                    tough_moveit_planners
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)

##############
## EXAMPLES ##
##############

## example_node
add_executable(chest_listener_node src/chest_node.cpp)
add_dependencies(chest_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chest_listener_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS chest_listener_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
