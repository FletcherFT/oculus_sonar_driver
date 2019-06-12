#pragma once

#include <ros/ros.h>

#include "oculus_sonar_ros/OculusSonarBase.h"
#include "liboculus/SonarPlayer.h"

namespace oculus_sonar_ros {

  class OculusSonarPlayback : public OculusSonarBase {
  public:

    OculusSonarPlayback() = delete;
    OculusSonarPlayback( const OculusSonarPlayback & ) = delete;

    OculusSonarPlayback(ros::NodeHandle &nh, float dt = 0.1 );
    virtual ~OculusSonarPlayback();

    bool open( const std::string &filename );

    void doLoop( bool dl ) { _doLoop = dl; }

    void nextPacket( const ros::TimerEvent& );

  private:

    std::string _filename;

    liboculus::SonarPlayer _player;

    ros::Timer _timer;
    bool _doLoop;
    float _dt;
  };

}
