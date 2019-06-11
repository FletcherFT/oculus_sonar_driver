#pragma once

#include <ros/ros.h>

#include "oculus_sonar_ros/OculusSonarBase.h"

namespace oculus_sonar_ros {

  class OculusSonarReal : public OculusSonarBase {
  public:

    OculusSonarReal() = delete;
    OculusSonarReal( const OculusSonarReal & ) = delete;

    OculusSonarReal(ros::NodeHandle &nh );
    virtual ~OculusSonarReal();

    bool open( const std::string &filename );

  private:

    std::string _filename;

  };

}
