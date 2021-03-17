# oculus_sonar_driver

[![Build Status](https://gitlab.drone.camhd.science/api/badges/apl-ocean-engineering/oculus_sonar_driver/status.svg)](https://gitlab.drone.camhd.science/apl-ocean-engineering/oculus_sonar_driver)

ROS nodes that use [liboculus](https://github.com/apl-ocean-engineering/liboculus) to interface with a [Blueprint Subsea Oculus sonar](https://www.blueprintsubsea.com/oculus/index.php) sonar and publish to ROS topics.
Currently runs on ROS Melodic, Ubuntu 18.04.

At present we have only tested with the [Oculus M1200d](https://www.blueprintsubsea.com/pages/product.php?PN=BP01042) sonar.

NOTE: Images are scaled by SCALE_FACTOR, a macro defaulted to 250 (to help with display)

## Contents

This package defines the `oculus_sonar/driver` nodelet that interfaces with the sonar
over ethernet and publishes `acoustic_msgs::SonarImage` and
`oculus_sonar_driver/OculusSonarRawMsg` messages.

The package also builds a conventional node `oculus_driver` which runs the
`oculus_sonar/driver` nodelet and an instance of the `draw_sonar` nodelet from
[sonar_image_proc](https://github.com/apl-ocean-engineering/sonar_image_proc).
It subscribes to `acoustic_msgs::SonarImage` and produces a `sensor_msgs/Image` 
of the projected sonar data for easy preview (e.g. in `rqt`).


## Installation

  1. Either:
     1. Use [wstool](http://wiki.ros.org/wstool):
        1. Clone this repo to `<catkin_ws>/src`
        1. `cd <catkin_src>`
        1. `wstool init src src/oculus_sonar_driver/oculus_sonar_driver.rosinstall` (or `wstool merge -t src src/oculus_sonar_driver/oculus_sonar_driver.rosinstall` if you're already using wstool )
     1. Or install dependencies manually. Clone [liboculus](https://github.com/apl-ocean-engineering/liboculus), [active_object](https://gitlab.com/apl-ocean-engineering/hmi-lsd-slam-transition/active_object), [imaging_sonar_msgs](https://gitlab.com/apl-ocean-engineering/imaging_sonar_msgs), and [g3log_catkin](https://gitlab.com/apl-ocean-engineering/lsd-slam/g3log_catkin) to ``<catkin_ws>/src`
  1. Run `catkin_make` or `catkin build` from ``<catkin_ws>``
  1. Run `source ./devel/setup.bash` from ``<catkin_ws>``

## Usage
Running the command `roslaunch oculus_sonar_driver default_ros.launch` will start both the publisher and subscriber nodes, as well as rqt_reconfigure gui.
Parameters within `default_ros.launch` can be modified to change the starting configuration of the sonar.

Running the command `roslaunch oculus_sonar_driver sonar_and_cameras.launch` will start the publisher and subscriber nodes, the publisher node from [blackmagic_ros](https://gitlab.com/apl-ocean-engineering/blackmagic_ros), and a dashboard in rqt_gui displaying the sonar imagery, blackmagic cameras, and dynamic reconfigure.

# License

This repository is covered by the [BSD 3-Clause License](LICENSE).
