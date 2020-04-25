cmake_minimum_required(VERSION 2.8.3)
project(MultiUAV)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_runtime
  roscpp
  rospy
  std_msgs
  message_generation  
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
  cv_bridge
  image_transport
  actionlib_msgs
)

find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES MultiUAV
#  CATKIN_DEPENDS gazebo_ros message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
)


add_executable(multi_uav 
               src/uav/multi_uav.cpp
               src/uav/single_uav.cpp)

add_executable(camera_uav 
               src/uav/camera_uav.cpp
               src/uav/single_uav.cpp)

add_executable(show_image 
               src/image/image_show.cpp)



target_link_libraries(multi_uav
  ${catkin_LIBRARIES}
)


target_link_libraries(camera_uav
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(show_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark executables and/or libraries for installation
install(TARGETS multi_uav camera_uav show_image 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)