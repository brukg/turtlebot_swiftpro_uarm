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
)
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(PCL REQUIRED)
# find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
# find_package(ZED 3 REQUIRED)
# find_package(OpenCV REQUIRED)
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
  INCLUDE_DIRS include
#  LIBRARIES rgbd_risk
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
)

add_executable(robot_control src/robot_control.cpp src/CNTRL.cpp src/ROBOT_TASK.cpp)
add_executable(task_sequencer src/task_sequencer.cpp src/SQNCR.cpp)
add_executable(aruco_detector src/aruco_detector.cpp src/ARUCO.cpp)

target_link_libraries(robot_control
                  ${catkin_LIBRARIES}  )
target_link_libraries(task_sequencer
                  ${catkin_LIBRARIES}  )
target_link_libraries(aruco_detector
                  ${catkin_LIBRARIES}  )

add_dependencies(robot_control ${catkin_EXPORTED_TARGETS})
add_dependencies(task_sequencer ${catkin_EXPORTED_TARGETS})
add_dependencies(aruco_detector ${catkin_EXPORTED_TARGETS})

