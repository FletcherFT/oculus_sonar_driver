cmake_minimum_required(VERSION 2.8.3)
project(oculus_sonar_ros)
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)

set( CATKIN_PACKAGES
  roscpp
  std_msgs
  imaging_sonar_msgs
  g3log_catkin
  active_object
  liboculus REQUIRED
  rospy
  serdp_common
)

set(OpenCV_DIR /usr/share/OpenCV/)

# Standard dependencies
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ${CATKIN_PACKAGES}
  OpenCV
  dynamic_reconfigure
  
  
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/OculusSonar.cfg
)

find_package( OpenCV REQUIRED )


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS liboculus g3log_catkin active_object serdp_common 
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  thirdparty/
  include/
)


add_library( oculus_sonar_lib
             lib/OculusSonarBase.cpp
             lib/OculusSonarPlayback.cpp
             lib/OculusSonarReal.cpp)
             #thirdparty/gpmf-parser/GPMF_parser.c )

target_link_libraries(oculus_sonar_lib ${catkin_LIBRARIES})

add_executable(oculus_sonar_driver app/OculusSonarNode.cpp)
target_link_libraries(oculus_sonar_driver oculus_sonar_lib ${catkin_LIBRARIES} ${liboculus_catkin_LIBRARIES})

add_executable(oculus_sonar_playback_driver app/OculusSonarPlaybackNode.cpp)
target_link_libraries(oculus_sonar_playback_driver oculus_sonar_lib ${catkin_LIBRARIES})

add_executable(oculus_publisher src/oculus_publisher.cpp)
target_link_libraries(oculus_publisher oculus_sonar_lib ${catkin_LIBRARIES})

add_dependencies(oculus_publisher ${oculus_sonar_ros}_gencfg )

add_executable(oculus_subscriber src/oculus_subscriber.cpp)
target_link_libraries(oculus_subscriber oculus_sonar_lib ${catkin_LIBRARIES})















