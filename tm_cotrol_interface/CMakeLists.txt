cmake_minimum_required(VERSION 2.8.3)
project(tm_cotrol_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(BOOST_FLAG TRUE)
if(BOOST_FLAG)
    add_definitions( -DUSE_BOOST )
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  apriltags_ros
  # visualization_msgs
  roslib
  
  # tm_description
  tm_msgs
  tm_driver
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# find_package(Boost REQUIRED COMPONENTS system thread chrono)

# if(BOOST_FLAG)
# catkin_package(
#     INCLUDE_DIRS include
#     # LIBRARIES TM_REFLEXXESII_LIB RMLTYPEII_LIB   
#     CATKIN_DEPENDS geometry_msgs message_filters roscpp rospy sensor_msgs std_msgs tf visualization_msgs tm_driver tm_kinematics 
#     DEPENDS boost
# )
# else()
# catkin_package(
#     INCLUDE_DIRS include
#     #LIBRARIES TM_REFLEXXESII_LIB RMLTYPEII_LIB   
#     CATKIN_DEPENDS geometry_msgs message_filters roscpp rospy sensor_msgs std_msgs tf visualization_msgs tm_driver tm_kinematics 
#     # DEPENDS boost
# )
# endif()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  do_unseen.srv
#   Service2.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  # sensor_msgs#   std_msgs#   tm_msgs
  std_msgs
)
# set(CMAKE_CXX_FLAGS "-std=c++11")
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES tm_cotrol_interface
 CATKIN_DEPENDS 
  geometry_msgs 
  message_filters 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  tf 
  tm_driver 
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
  # ${Boost_INCLUDE_DIRS}
)

add_executable (tag_tf_test src/tag_tf_test.cpp)
target_link_libraries (tag_tf_test ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (tag_tf_hole src/tag_tf_hole.cpp)
target_link_libraries (tag_tf_hole ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (find_target src/find_target.cpp)
target_link_libraries (find_target ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (find_target_bone src/find_target_bone.cpp)
target_link_libraries (find_target_bone ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (find_target_bone0524 src/find_target_bone0524.cpp)
target_link_libraries (find_target_bone0524 ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (tm_pose_test src/tm_pose_test.cpp)
# add_dependencies(tm_pose_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (tm_pose_test ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})
# add_executable (tm_move src/tm_move.cpp)
# target_link_libraries (tm_move TM_kinematics TM_print ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (start_move src/start_move.cpp)
target_link_libraries (start_move ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

# add_executable (tm_move2target src/tm_move2target.cpp)
# target_link_libraries (tm_move2target TM_kinematics TM_print ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (tm_move2target_line src/tm_move2target_line.cpp)
target_link_libraries (tm_move2target_line ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (bt300 src/bt300.cpp)
target_link_libraries (bt300 ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (target_pub_test src/target_pub_test.cpp)
target_link_libraries (target_pub_test ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (settarget src/settarget.cpp)
target_link_libraries (settarget ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

# add_executable (tm_move2target_csv src/tm_move2target_csv.cpp)
# target_link_libraries (tm_move2target_csv TM_kinematics TM_print ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (find_target_qr2bone src/find_target_qr2bone.cpp)
target_link_libraries (find_target_qr2bone ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_executable (find_target_qr2bone1118 src/find_target_qr2bone1118.cpp)
target_link_libraries (find_target_qr2bone1118 ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})