#pragma once

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <sstream>
using std::string;

#include <memory>

// Used to get sonar ping info
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarClient.h"
#include "liboculus/SonarConfiguration.h"

// Allow dynamic reconfigure of sonar parameters
#include <dynamic_reconfigure/server.h>

// Auto-generated files
#include "oculus_sonar_driver/OculusSonarRawMsg.h"
#include "oculus_sonar_driver/OculusSonarConfig.h"

namespace oculus_sonar {

using namespace liboculus;

class OculusDriver : public nodelet::Nodelet {
public:

  OculusDriver();
  virtual ~OculusDriver();

  void pingCallback(const SimplePingResult &ping);
  void configCallback(oculus_sonar_driver::OculusSonarConfig &config, uint32_t level);

private:

  void onInit() override;

  std::unique_ptr< SonarClient > _sonarClient;

  ros::Publisher _imagingSonarPub;
  ros::Publisher _oculusRawPub;
  std::string _ipAddress;

  SonarConfiguration _sonarConfig;

  dynamic_reconfigure::Server<oculus_sonar_driver::OculusSonarConfig> _reconfigureServer;

};

}
