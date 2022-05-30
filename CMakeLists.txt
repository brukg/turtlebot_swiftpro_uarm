cmake_minimum_required(VERSION 2.8.3)
project(swift_uarm)

## Compile as C++11, supported in ROS Kinetic and newer
 #add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS} 
  ${OPENCV_INCLUDE_DIRS} #for aruco detection
)
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(robot_control src/robot_control.cpp src/CNTRL.cpp src/ROBOT_TASK.cpp)
add_executable(task_sequencer src/task_sequencer.cpp src/SQNCR.cpp)
add_executable(aruco_detector src/aruco_detector.cpp src/ARUCO.cpp)

target_link_libraries(robot_control
                  ${catkin_LIBRARIES}  )
target_link_libraries(task_sequencer
    ${catkin_LIBRARIES}
    )

target_link_libraries(aruco_detector
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    )

add_dependencies(robot_control 
    ${catkin_EXPORTED_TARGETS}
    )
add_dependencies(task_sequencer 
    ${catkin_EXPORTED_TARGETS}
    )
add_dependencies(aruco_detector 
    ${catkin_EXPORTED_TARGETS}
    )

