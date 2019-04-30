cmake_minimum_required(VERSION 2.8.3)
project(simple_test)

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
add_executable(cn src/chest_node.cpp)
add_dependencies(cn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cn ${catkin_LIBRARIES})

add_executable(example_node src/atlas_example_node.cpp)
add_dependencies(example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS example_node cn
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
