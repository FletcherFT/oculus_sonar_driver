cmake_minimum_required(VERSION 2.8.3)
project(oculus_sonar_ros)
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)

set( CATKIN_PACKAGES
  cv_bridge
  roscpp
  std_msgs
  imaging_sonar_msgs
  g3log_catkin
  active_object
  liboculus REQUIRED
  rospy
  gpmf
)

set(OpenCV_DIR /usr/share/OpenCV/)
set( PI "3.14159265358979")
add_definitions( -DPI=${PI} )

set( SCALE_FACTOR "250")
add_definitions( -DSCALE_FACTOR=${SCALE_FACTOR} )

OPTION(THETA_SHIFT_DEGREES "Define " ON) # Enabled by default
if(THETA_SHIFT_DEGREES)
	add_definitions( -DTHETA_SHIFT=90 )
else()
	add_definitions( -DTHETA_SHIFT=${PI} )
endif()


# Standard dependencies
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ${CATKIN_PACKAGES}
  OpenCV
  dynamic_reconfigure
  image_transport


)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/OculusSonar.cfg
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS liboculus g3log_catkin active_object gpmf image_transport
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  thirdparty/
  include/
  ${catkin_INCLUDE_DIRS}
)


add_executable(oculus_publisher src/oculus_publisher.cpp)
target_link_libraries(oculus_publisher ${catkin_LIBRARIES})
add_dependencies(oculus_publisher ${oculus_sonar_ros}_gencfg )

add_executable(oculus_subscriber src/oculus_subscriber.cpp)
target_link_libraries(oculus_subscriber  ${catkin_LIBRARIES})

add_executable(sonar_img_view src/sonar_img_view.cpp)
target_link_libraries(sonar_img_view  ${catkin_LIBRARIES})

install(TARGETS oculus_publisher oculus_subscriber
	      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install( DIRECTORY launch/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
install( DIRECTORY rqt_config/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rqt_config )
