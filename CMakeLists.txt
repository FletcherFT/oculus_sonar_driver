cmake_minimum_required(VERSION 2.8.3)
project(oculus_sonar_ros)

set( CATKIN_PACKAGES
  std_msgs
  imaging_sonar_msgs )

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

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)


###########
## Build ##
###########

## Check out the lsd-slam submodule if necessary
# if( NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lsd-slam/README.md )
#   execute_process(COMMAND git submodule update --init --force
#                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#                   OUTPUT_VARIABLE cout
#                   ERROR_VARIABLE cerr
#                   RESULT_VARIABLE rv)
#   #message("rv='${rv}'")
#   message("cout='${cout}'")
#   message("cerr='${cerr}'")
# endif()


add_library(oculus_sonar_lib lib/Foo.cpp)
target_link_libraries(oculus_sonar_lib ${catkin_LIBRARIES})


## Build the applications and nodes
add_executable(oculus_sonar_driver app/OculusSonarDriver.cpp)
target_link_libraries(oculus_sonar_driver oculus_sonar_lib)


install(TARGETS oculus_sonar_lib oculus_sonar_driver
	      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
