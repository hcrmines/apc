cmake_minimum_required(VERSION 2.8.3)
project(apc)

find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
	pcl_ros
	roscpp
	rospy
	sensor_msgs
	geometry_msgs
	std_msgs
    image_transport
  	cv_bridge
	message_generation
    actionlib_msgs
    actionlib
 moveit_core
  moveit_ros_planning_interface
)
find_package(OpenCV REQUIRED)

#catkin_python_setup()

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
    FILES
    MatInfo.msg
    RecInfo.msg
    objRec.msg
    recRes.msg
    Recognized.msg
    Recognition.msg
    AddScoop.msg
)

add_service_files(
    FILES
    recognizeObjects.srv
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   MoveTo.action
#   Action2.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs

)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apc
    CATKIN_DEPENDS moveit_core moveit_ros_planning_interface geometry_msgs openni_camera openni_launch roscpp std_msgs actionlib_msgs message_generation message_runtime
    DEPENDS system_lib
#  DEPENDS system_lib
)




###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(next_object_node src/next_object.cpp)
 add_executable(move_arm_node src/move_arm.cpp)
 add_executable(add_objects_node src/add_objects.cpp)
 add_executable(move_server src/move_server.cpp)
 add_executable(test_move_node src/test_move.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(move_server apc_generate_messages_cpp)
 add_dependencies(test_move_node apc_generate_messages_cpp)
 add_dependencies(add_objects_node apc_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(next_object_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(move_arm_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(move_server
   ${catkin_LIBRARIES}
 )
 target_link_libraries(test_move_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(add_objects_node
   ${catkin_LIBRARIES}
 )

add_executable(AnalyzeMask src/AnalyzeMask.cpp)
add_executable(ActionServer src/ActionBridge.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(threeD_node threeD_generate_messages_cpp)
add_dependencies(ActionServer apc_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(AnalyzeMask
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

target_link_libraries(ActionServer
 ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS threeD threeD_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_threeD.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
