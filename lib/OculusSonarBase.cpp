
#include <ros/console.h>

#include "oculus_sonar_ros/OculusSonar.h"
#include <imaging_sonar_msgs/imagingSonarMsg.h>

namespace oculus_sonar_ros {

  using namespace imaging_sonar_msgs;

  OculusSonar::OculusSonar( ros::NodeHandle &nh )
    : _nh(nh),
      _simplePingPub( _nh.advertise<ImagingSonarMsg>("ping", 1) )
  {
    ;
  }

}
