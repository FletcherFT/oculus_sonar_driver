# oculus_sonar_ros
ROS nodes that uses [liboculus](https://github.com/apl-ocean-engineering/liboculus) to publish sonar data to ROS topics.

## Installation
  1. Clone liboculus to <catkin_ws>/src
  2. Run *./fips build* from <catkin_ws>/src/liboculus
  3. Run *catkin_make* from <catkin_ws>
  4. Run *source ./devel/setup.bash* from <catkin_ws>

## Usage
Running the command *roslaunch oculus_sonar_ros default_ros.launch* will start both the publisher and subscriber nodes, as well as rqt_reconfigure gui.
Parameters within default_ros.launch can be modified to change the starting configuration of the sonar.

Code for the nodes used can be found in oculus_sonar_ros/src
