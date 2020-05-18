#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <sstream>
using std::string;
// Sonar ROS message that aaron made
#include <imaging_sonar_msgs/ImagingSonarMsg.h>

#include "oculus_sonar_ros/OculusSonarRawMsg.h"

// Used to get sonar ping info
#include "liboculus/SimplePingResult.h"

// DataRx recieves pings from the sonar
#include "liboculus/DataRx.h"
// IoServiceThread allows for network communication
#include "liboculus/IoServiceThread.h"
// Pretty sure StatusRx validates sonar, might get ip address
#include "liboculus/StatusRx.h"

// For modifying sonar parameters
#include "liboculus/SimpleFireMessage.h"

// Allow dynamic reconfigure of sonar parameters
#include <dynamic_reconfigure/server.h>
#include <oculus_sonar_ros/OculusSonarConfig.h>
#include <thread>

using namespace liboculus;

class OculusPublisher {

  std::unique_ptr<DataRxQueued> dataRx_;
  ros::Publisher oculus_pub_, oculus_raw_pub_;
  int initRange, initGainPercent, initGamma, initPingRate, initMasterMode;
  SimpleFireMessage initialConfig;
  SimpleFireMessage updateFireMsg;


public:

  OculusPublisher();
  ~OculusPublisher();

  // will have publisher, dataRx, statusRx, all as class fields
  // -> don't need to pass things in methods
  void pingCallback(shared_ptr<SimplePingResult> ping);
  void configCallback(oculus_sonar_ros::OculusSonarConfig &config, uint32_t level);
  void reconfigListener();
  void run();

};
