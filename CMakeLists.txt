cmake_minimum_required(VERSION 3.0.2)
project(oculus_sonar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)

set(OpenCV_DIR /usr/share/OpenCV/)

# Standard dependencies
find_package(catkin REQUIRED COMPONENTS
  acoustic_msgs
  cmake_modules
  cv_bridge
  sonar_image_proc
  dynamic_reconfigure
  g3log_ros
  image_transport
  liboculus REQUIRED
  message_generation
  nodelet
  nodelet_topic_tools
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Build messages
# add_message_files(
#   FILES
# )

generate_messages(DEPENDENCIES std_msgs sensor_msgs)


#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/OculusSonar.cfg
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS liboculus g3log_ros image_transport
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include/
  ${catkin_INCLUDE_DIRS}
)


add_library(oculus_driver_nodelet src/oculus_driver_nodelet.cpp)
target_link_libraries(oculus_driver_nodelet ${catkin_LIBRARIES})
class_loader_hide_library_symbols(oculus_driver_nodelet)

add_executable(oculus_driver src/oculus_node.cpp)
target_link_libraries(oculus_driver
  ${catkin_LIBRARIES}
)
add_dependencies(oculus_driver
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

install(TARGETS oculus_driver
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

install(TARGETS oculus_driver_nodelet
	      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install( DIRECTORY launch/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
install( DIRECTORY rqt_config/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rqt_config )

install( FILES nodelet_plugins.xml
  		   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(oculus_sonar_test test/unit/main.cpp)
  target_link_libraries(oculus_sonar_test ${catkin_LIBRARIES})
endif()
