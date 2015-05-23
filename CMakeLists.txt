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
)
find_package(OpenCV REQUIRED)

catkin_python_setup()

add_message_files(
    FILES
    MatInfo.msg
    RecInfo.msg
    objRec.msg
    recRes.msg
    Recognized.msg
    Recognition.msg
)

add_service_files(
    FILES
    recognizeObjects.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
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
    CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable(AnalyzeMask src/AnalyzeMask.cpp)

target_link_libraries(AnalyzeMask
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

