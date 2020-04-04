# oculus_sonar_ros

[![Build Status](https://gitlab.drone.camhd.science/api/badges/apl-ocean-engineering/oculus_sonar_ros/status.svg)](https://gitlab.drone.camhd.science/apl-ocean-engineering/oculus_sonar_ros)

ROS nodes that use [liboculus](https://github.com/apl-ocean-engineering/liboculus) to publish sonar data to ROS topics.
Currently runs on ROS Melodic, Ubuntu 18.04  

NOTE: Images are scaled by SCALE_FACTOR, a macro defaulted to 250 (to help with display)  

## Installation
  1. Clone [liboculus](https://github.com/apl-ocean-engineering/liboculus), [active_object](https://gitlab.com/apl-ocean-engineering/hmi-lsd-slam-transition/active_object), [imaging_sonar_msgs](https://gitlab.com/apl-ocean-engineering/imaging_sonar_msgs), and [g3log_catkin](https://gitlab.com/apl-ocean-engineering/lsd-slam/g3log_catkin) to <catkin_ws>/src
  2. Run *./fips build* from <catkin_ws>/src/liboculus
  3. Run *catkin_make* from <catkin_ws>
  4. Run *source ./devel/setup.bash* from <catkin_ws>

## Usage
Running the command *roslaunch oculus_sonar_ros default_ros.launch* will start both the publisher and subscriber nodes, as well as rqt_reconfigure gui.
Parameters within default_ros.launch can be modified to change the starting configuration of the sonar.  

Running the command *roslaunch oculus_sonar_ros sonar_and_cameras.launch* will start the publisher and subscriber nodes, the publisher node from [blackmagic_ros](https://gitlab.com/apl-ocean-engineering/blackmagic_ros), and a dashboard in rqt_gui displaying the sonar imagery, blackmagic cameras, and dynamic reconfigure.  

Code for the nodes used can be found in oculus_sonar_ros/src  
Note: errors with running *catkin_make* in step 2 of Installation may be fixed by switching to the "tanner_working" branch in whatever repo is causing an error.

# Licensing

This repository is covered by the [BSD License](LICENSE.txt).
