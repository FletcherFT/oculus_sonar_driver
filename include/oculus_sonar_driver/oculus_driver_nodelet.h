// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#pragma once


#include <cstdlib>
#include <sstream>
#include <string>
#include <memory>

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>


// Used to get sonar ping info
#include "liboculus/SimplePingResult.h"
#include "liboculus/StatusRx.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/SonarConfiguration.h"

#include "oculus_sonar_driver/publishing_data_rx.h"
#include "oculus_sonar_driver/ping_to_sonar_image.h"

// Auto-generated files
#include "oculus_sonar_driver/OculusSonarConfig.h"
#include "oculus_sonar_driver/OculusMetadata.h"

namespace oculus_sonar_driver {

class OculusDriver : public nodelet::Nodelet {
 public:
  OculusDriver();
  virtual ~OculusDriver();

  // Translate SimplePingResult to SonarImage and publish
  template <typename Ping_t>
  void pingCallback(const Ping_t &ping) {
    // Publish message parsed into the image format
    acoustic_msgs::SonarImage sonar_msg = pingToSonarImage(ping);

    sonar_msg.header.seq = ping.ping()->pingId;
    sonar_msg.header.stamp = ros::Time::now();
    sonar_msg.header.frame_id = frame_id_;
    imaging_sonar_pub_.publish(sonar_msg);

    oculus_sonar_driver::OculusMetadata meta;
    meta.header = sonar_msg.header;

    // \todo Make this cleaner...
    for (unsigned int i = 0; i < ping.gains().size(); i++) {
      meta.tvg.push_back(ping.gains().at(i));
    }

    oculus_meta_pub_.publish(meta);
  }

  // Update configuration based on command from dynamic_reconfigure
  void configCallback(const oculus_sonar_driver::OculusSonarConfig &config,
                      uint32_t level);

 private:
  // Set up all ROS interfaces and start the sonarClient
  void onInit() override;

  liboculus::IoServiceThread io_srv_;
  PublishingDataRx data_rx_;
  liboculus::StatusRx status_rx_;

  ros::Publisher imaging_sonar_pub_;
  ros::Publisher oculus_meta_pub_;
  ros::Publisher raw_data_pub_;

  std::string ip_address_;
  std::string frame_id_;

  liboculus::SonarConfiguration sonar_config_;

  dynamic_reconfigure::Server<oculus_sonar_driver::OculusSonarConfig> reconfigure_server_;
};

}  // namespace oculus_sonar
