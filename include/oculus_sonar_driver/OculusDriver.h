#pragma once

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <sstream>
#include <string>

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

  std::unique_ptr< SonarClient > sonar_client_;

  ros::Publisher imaging_sonar_pub_;
  ros::Publisher oculus_raw_pub_;
  std::string ip_address_;
  std::string frame_id_;

  SonarConfiguration sonar_config_;

  dynamic_reconfigure::Server<oculus_sonar_driver::OculusSonarConfig> reconfigure_server_;

};

}
