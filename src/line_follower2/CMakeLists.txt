cmake_minimum_required(VERSION 3.0.2)
project(line_follower2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  dynamic_reconfigure
  tf2
  tf2_geometry_msgs
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/HSVThreshold.cfg
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line_follower2
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge image_transport tf2 tf2_geometry_msgs tf2_ros dynamic_reconfigure
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_follower2_node src/line_follower2_node.cpp)
target_link_libraries(line_follower2_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_hsv src/image_hsv.cpp)

add_dependencies(image_hsv ${PROJECT_NAME}_gencfg)
add_dependencies(line_follower2_node ${PROJECT_NAME}_gencfg)

target_link_libraries(image_hsv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables for installation
install(TARGETS line_follower2_node image_hsv
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Mark script files for installation
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)