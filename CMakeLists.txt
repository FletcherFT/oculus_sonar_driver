cmake_minimum_required(VERSION 2.8.3)
project(oculus_sonar_ros)

set( CATKIN_PACKAGES
  roscpp
  std_msgs
  imaging_sonar_msgs
  g3log_catkin
  active_object )

# Standard dependencies
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGES}
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## System dependencies are found with CMake's conventions
# find_package( Boost COMPONENTS filesystem system REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  LIBRARIES oculus_sonar_lib
  CATKIN_DEPENDS ${CATKIN_PACKAGES}
)


###########
## Build ##
###########

# Check out the lsd-slam submodule if necessary
if( NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../liboculus/CMakeLists.txt )
  message(FATAL_ERROR "Did not find liboculus in src/ directory")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ../liboculus/include/
  ../liboculus/thirdparty/
  thirdparty/
  include/
)


add_library( oculus
             ../liboculus/lib/SimpleFireMessage.cpp
             ../liboculus/lib/SonarStatus.cpp
             ../liboculus/lib/DataRx.cpp
             ../liboculus/lib/SonarPlayer.cpp
             ../liboculus/lib/StatusRx.cpp )
target_link_libraries(oculus ${catkin_LIBRARIES})

add_library( oculus_sonar_lib
             lib/OculusSonarBase.cpp
             lib/OculusSonarPlayback.cpp
             lib/OculusSonarReal.cpp
             thirdparty/gpmf-parser/GPMF_parser.c )
target_link_libraries(oculus_sonar_lib oculus ${catkin_LIBRARIES})


## Build the applications and nodes
add_executable(oculus_sonar_driver app/OculusSonarNode.cpp)
target_link_libraries(oculus_sonar_driver oculus_sonar_lib ${catkin_LIBRARIES})

add_executable(oculus_sonar_playback_driver app/OculusSonarPlaybackNode.cpp)
target_link_libraries(oculus_sonar_playback_driver oculus_sonar_lib ${catkin_LIBRARIES})

install(TARGETS oculus_sonar_lib oculus_sonar_driver oculus_sonar_playback_driver
	      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

##== Unit tests ==

file(GLOB LIBOCULUS_TESTS ../liboculus/test/unit/*.cpp )

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(oculus_sonar_test
                  ${LIBOCULUS_TESTS}
                  test/unit/main.cpp)
  target_link_libraries(oculus_sonar_test oculus_sonar_lib ${catkin_LIBRARIES})
endif()
