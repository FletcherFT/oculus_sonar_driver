
#include <ros/console.h>

#include "oculus_sonar_ros/OculusSonarReal.h"

namespace oculus_sonar_ros {

  OculusSonarReal::OculusSonarReal( ros::NodeHandle &nh )
    : OculusSonarBase( nh )
  {
    ;
  }

  OculusSonarReal::~OculusSonarReal()
  {;}

  bool OculusSonarReal::open( const std::string &filename )
  {
    _filename = filename;
  }


}
