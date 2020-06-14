#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <sstream>
using std::string;

#include <memory>

// Sonar ROS message that aaron made
#include <imaging_sonar_msgs/ImagingSonarMsg.h>

#include "oculus_sonar_ros/OculusSonarRawMsg.h"

// Used to get sonar ping info
#include "liboculus/SimplePingResult.h"

// SonarClient wraps all of the functionality into one class
#include "liboculus/SonarClient.h"

// For modifying sonar parameters
#include "liboculus/SonarConfiguration.h"

// Allow dynamic reconfigure of sonar parameters
#include <dynamic_reconfigure/server.h>
#include <oculus_sonar_ros/OculusSonarConfig.h>

using namespace liboculus;

class OculusPublisher {

  std::unique_ptr< SonarClient > _sonarClient;

  ros::Publisher _imagingSonarPub, _oculusRawPub;

  int initRange, initGainPercent, initGamma, initPingRate, initMasterMode;
  std::string _ipAddress;

  SonarConfiguration sonarConfig;

public:

  OculusPublisher();
  ~OculusPublisher();

  // will have publisher, dataRx, statusRx, all as class fields
  // -> don't need to pass things in methods

  void pingCallback(const shared_ptr<SimplePingResult> &ping);

  void configCallback(oculus_sonar_ros::OculusSonarConfig &config, uint32_t level);
  void reconfigListener();
  void run();

};
