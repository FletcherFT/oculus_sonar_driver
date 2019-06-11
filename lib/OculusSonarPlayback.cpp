
#include <ros/console.h>

#include "oculus_sonar_ros/OculusSonarPlayback.h"
#include <imaging_sonar_msgs/imagingSonarMsg.h>

namespace oculus_sonar_ros {

  using namespace imaging_sonar_msgs;

  OculusSonarPlayback::OculusSonarPlayback( ros::NodeHandle &nh )
    : OculusSonarBase( nh )
  {
    ;
  }

  OculusSonarPlayback::~OculusSonarPlayback()
  {;}

  bool OculusSonarPlayback::open( const std::string &filename )
  {
    _filename = filename;
  }


}
