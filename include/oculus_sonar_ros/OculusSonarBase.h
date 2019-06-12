#pragma once

#include <ros/ros.h>

#include "imaging_sonar_msgs/ImagingSonarMsg.h"
#include "liboculus/SimplePingResult.h"

namespace oculus_sonar_ros {

  using liboculus::SimplePingResult;
  using imaging_sonar_msgs::ImagingSonarMsg;

  class OculusSonarBase {
  public:

    OculusSonarBase() = delete;
    OculusSonarBase( const OculusSonarBase & ) = delete;

    OculusSonarBase(ros::NodeHandle &nh);
    virtual ~OculusSonarBase();

    ImagingSonarMsg pingToMessage( const std::shared_ptr<SimplePingResult> &ping );

  protected:

    ros::NodeHandle _nh;

    ros::Publisher _simplePingPub;

    uint32_t _seq;

  };

}
