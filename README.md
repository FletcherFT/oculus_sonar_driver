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
     1. Use [vcstool](http://wiki.ros.org/vcstool):
        1. Clone this repo to `<catkin_ws>/src`
        1. `cd <catkin_src>`
        1. `vcs import --input oculus_sonar_driver/oculus_sonar_driver.repos`
     1. Or install dependencies manually. Clone [liboculus](https://github.com/apl-ocean-engineering/liboculus), [hydrographic_msgs](https://github.com/apl-ocean-engineering/hydrographic_msgs.git) and [g3log_ros](https://gitlab.com/apl-ocean-engineering/g3log_ros) to `<catkin_ws>/src`
  1. Run `catkin_make` or `catkin build` from ``<catkin_ws>``
  1. Run `source ./devel/setup.bash` from ``<catkin_ws>``

## Usage
Running the command `roslaunch oculus_sonar_driver default_ros.launch` will start the sonar driver.
See the comments in [`default_ros.launch`](launch/default_ros.launch) to set sonar parameters at startup.   Sonar params can also be set
through dynamic reconfigure (below).

## dynamic_reconfigure

The sonar params can be modified on the fly using dynamic reconfigure.  The launchfile [`default_ros.launch`](launch/default_ros.launch) starts the nodes in a nodelet manager `/nodelet_manager` to the commands are:

```
$ rosrun dynamic_reconfigure dynparam get /nodelet_manager           
{'send_range_as_meters': True, 'send_gain': True, 'send_simple_return': True, 'gain_assistance': False, 'all_beams': True, 'num_beams': 1, 'gamma': 127, 'ping_rate': 0, 'data_size': 0, 'freq_mode': 2, 'range': 2.0, 'gain': 50.0, 'groups': {'id': 0, 'parent': 0, 'name': 'Default', 'type': '', 'state': True, 'groups': {}, 'parameters': {}, 'send_range_as_meters': True, 'send_gain': True, 'send_simple_return': True, 'gain_assistance': False, 'num_beams': 1, 'all_beams': True, 'range': 2.0, 'gain': 50.0, 'gamma': 127, 'ping_rate': 0, 'data_size': 0, 'freq_mode': 2}}
```

For example, for 32-bit mode:

```
$ rosrun dynamic_reconfigure dynparam set /nodelet_manager data_size 2
```

----
# A note about sonar resolution

(This is inferred from experimental data, it's not from Blueprint)

The sonar appears to have a fixed maximum number of range bins, approx 720.  It also has fixed quantized range resolutions:  ~2.8mm, ~5.6mm, etc.  So the number of range bins N present in data at a given range R in meters is:

```
N = R / (2^k * 2.8mm)  minimizing k s.t. N <= 720
```

This means, for example:

| Range (m) | Bins | Resolution |
|-----------|------|------------|
| 2.0 | 709 | 2.8mm |
| 2.02 | 720 | 2.8mm |
| 3 | 532 | 5.6mm |
| 4 | 709 | 5.6mm |
| 4.065 | 720 | 5.6mm |
| 5 | 443 | 11.2mm |
| 6 | 532 | 11.2mm |

etc.   This effect seems to be invariant of 256 v 512 beams, and 8/16/32 bit data.  Haven't tested in the lower frequency mode


----
# License

This repository is covered by the [BSD 3-Clause License](LICENSE).
