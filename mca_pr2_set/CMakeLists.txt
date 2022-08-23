cmake_minimum_required(VERSION 2.8.3)
project(mca_pr2_set)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  turtlebot3_msgs
  message_generation
  message_runtime
)

find_package(Boost REQUIRED COMPONENTS system)

####################
## RPi GPIO setup ##
####################
# set (wiringPi_include "/use/local/include")

find_package(JetsonGPIO)

add_service_files(
  FILES
  setlocation.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   std_srvs
   geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES
    mca_pr2_set
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    geometry_msgs
    actionlib_msgs
    move_base_msgs
    turtlebot3_msgs
    message_generation
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

add_executable(robot_masterpc2 src/robot_masterpc2.cpp)
target_link_libraries(robot_masterpc2 ${catkin_LIBRARIES})

add_executable(robot_setpose2 src/robot_setpose2.cpp)
target_link_libraries(robot_setpose2 ${catkin_LIBRARIES})

add_executable(robot_server src/robot_server.cpp)
target_link_libraries(robot_server ${catkin_LIBRARIES})

add_executable(robot_service src/robot_service.cpp)
add_dependencies(robot_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_service ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS robot_masterpc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS robot_masterpc2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############