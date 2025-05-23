cmake_minimum_required(VERSION 3.0.2)
project(servo_pixhawk)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  custom_msgs
  mavros_msgs
  # VISP
  # OpenCV
  cv_bridge
  image_transport
  # VISP_ROS
)
find_package(VISP REQUIRED PATHS /opt/ros/noetic) 
find_package(OpenCV REQUIRED PATHS /usr/include/opencv4) 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES servo_pixhawk
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ./include
  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  # ${VISP_ROS_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/servoPixhawkNode.cpp # 或选择 src/servoPixhawkDroneIBVS_sim.cpp
  src/servoPixhawk.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${VISP_LIBRARIES}
  # ${VISP_ROS_LIBRARIES}
)
