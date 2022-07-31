cmake_minimum_required(VERSION 2.8.3)
project(mca_pr2_set)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  turtlebot3_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

## RPi GPIO setup
# set (wiringPi_include "/use/local/include")

find_package(JetsonGPIO)

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    actionlib_msgs
    move_base_msgs
    turtlebot3_msgs
    message_runtime
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_masterpc src/robot_masterpc.cpp)
target_link_libraries(robot_masterpc ${catkin_LIBRARIES})

add_executable(robot_setpose src/robot_setpose.cpp)
target_link_libraries(robot_setpose ${catkin_LIBRARIES})

add_executable(robot_main src/robot_main.cpp)
target_link_libraries(robot_main JetsonGPIO::JetsonGPIO ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS robot_masterpc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############