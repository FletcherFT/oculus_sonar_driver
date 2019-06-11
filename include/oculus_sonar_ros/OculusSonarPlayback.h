#pragma once

#include <ros/ros.h>


namespace oculus_sonar_ros {

  class OculusSonarPlayback : public OculusSonarBase {
  public:

    OculusSonarPlayback() = delete;
    OculusSonarPlayback( const OculusSonarPlayback & ) = delete;

    OculusSonarPlayback(ros::NodeHandle &nh );
    virtual ~OculusSonarPlayback();

    bool open( const std::string &filename )

  private:

    std::string _filename

  };

}
