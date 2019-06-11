#pragma once

#include <ros/ros.h>


namespace oculus_sonar_ros {

  class OculusSonar {
  public:

    OculusSonar() = delete;
    OculusSonar( const OculusSonar & ) = delete;

    OculusSonar(ros::NodeHandle &nh);

  private:

    ros::NodeHandle _nh;

    ros::Publisher _simplePingPub;

  };

}
