// Copyright 2020 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#include <boost/asio.hpp>

#include <acoustic_msgs/SonarImage.h>
#include <apl_msgs/RawData.h>

#include "liboculus/Constants.h"
#include "oculus_sonar_driver/oculus_driver.h"
#include "oculus_sonar_driver/publishing_data_rx.h"
#include "oculus_sonar_driver/ping_to_sonar_image.h"

namespace oculus_sonar_driver {

OculusDriver::OculusDriver()
  : Nodelet(),
    io_srv_(),
    data_rx_(io_srv_.context()),
    status_rx_(io_srv_.context()),
    reconfigure_server_()
{;}

OculusDriver::~OculusDriver() {
  io_srv_.stop();
  io_srv_.join();
}

void OculusDriver::onInit() {
  reconfigure_server_.setCallback(boost::bind(&OculusDriver::configCallback,
                                              this, _1, _2));

  ros::NodeHandle n_(getMTNodeHandle());
  ros::NodeHandle pn_(getMTPrivateNodeHandle());

  NODELET_DEBUG_STREAM("Advertising topics in namespace " << n_.getNamespace());
  NODELET_DEBUG_STREAM("Private namespace would be:" << pn_.getNamespace());

  imaging_sonar_pub_ = n_.advertise<acoustic_msgs::SonarImage>("sonar_image", 100);
  raw_data_pub_ = n_.advertise<apl_msgs::RawData>("raw_data", 100);

  // NB: Params set in the launch file go to /raven/oculus's namespace,
  //     rather than /raven/oculus/driver. For normal nodes, should definitely
  //     use the private namespace, but I'm not sure how to do that when
  //     nodelets have been compiled into the executable, rather than
  //     configured in a launch file.
  n_.param<std::string>("ipAddress", ip_address_, "auto");

  n_.param<std::string>("frameId", frame_id_, "");
  NODELET_INFO_STREAM("Publishing data with frame = " << frame_id_);

  data_rx_.setRawPublisher(raw_data_pub_);

  data_rx_.setSimplePingCallback(std::bind(&OculusDriver::pingCallback,
                                             this, std::placeholders::_1));

  // When the node connects, start the sonar pinging by sending
  // a OculusSimpleFireMessage current configuration.
  data_rx_.setOnConnectCallback( [&]() {
    data_rx_.sendSimpleFireMessage(sonar_config_);
  });

  // It is not necessary to load any of the parameters controlled by
  // dynamic reconfigure, since dynamic reconfigure will read them from
  // the launch file and immediately publish an update message at launch.

  if (ip_address_ == "auto") {
    NODELET_INFO_STREAM("Attempting to auto-detect sonar");
    status_rx_.setCallback( [&](const liboculus::SonarStatus &status, bool is_valid) {
      if (!is_valid || data_rx_.isConnected()) return;
      data_rx_.connect(status.ipAddr());
    });
  } else {
    NODELET_INFO_STREAM("Opening sonar at " << ip_address_);
    data_rx_.connect(ip_address_);
  }

  io_srv_.start();
}


// Processes and publishes sonar pings to a ROS topic
void OculusDriver::pingCallback(const liboculus::SimplePingResult &ping) {
  // Publish message parsed into the image format
  acoustic_msgs::SonarImage sonar_msg = pingToSonarImage(ping);

  sonar_msg.header.seq = ping.ping()->pingId;
  sonar_msg.header.stamp = ros::Time::now();
  sonar_msg.header.frame_id = frame_id_;
 
  // sonar_msg.frequency = ping.ping()->frequency;

  // // \todo This is actually frequency dependent
  // if (sonar_msg.frequency > 2000000) {
  //   sonar_msg.azimuth_beamwidth = liboculus::Oculus_2100MHz::AzimuthBeamwidthRad;
  //   sonar_msg.elevation_beamwidth = liboculus::Oculus_2100MHz::ElevationBeamwidthRad;
  // } else if ((sonar_msg.frequency > 1100000) && (sonar_msg.frequency < 1300000)) {
  //   sonar_msg.azimuth_beamwidth = liboculus::Oculus_1200MHz::AzimuthBeamwidthRad;
  //   sonar_msg.elevation_beamwidth = liboculus::Oculus_1200MHz::ElevationBeamwidthRad;
  // } else {
  //   ROS_ERROR_STREAM("Unsupported frequency received from oculus: "
  //                    << sonar_msg.frequency << ". Not publishing SonarImage "
  //                    << "for seq# " << sonar_msg.header.seq);
  //   return;
  // }

  // const int num_bearings = ping.ping()->nBeams;
  // const int num_ranges = ping.ping()->nRanges;

  // sonar_msg.azimuth_angles.resize(num_bearings);
  // for (unsigned int b = 0; b < num_bearings; b++) {
  //   sonar_msg.azimuth_angles[b] = ping.bearings().at_rad(b);
  // }

  // // QUESTION(lindzey): Is this actually right?
  // //    Do their ranges start at 0, or at the min range of 10 cm?
  // //
  // // (Aaron):  We don't actually know.  Given there's no way to
  // //    set "minimum range", and it's not in the data struct, we
  // //    have to assume is starts from zero, though as you say, it
  // //    could actually be another arbitrary constant.
  // sonar_msg.ranges.resize(num_ranges);
  // for (unsigned int i = 0; i < num_ranges; i++) {
  //   sonar_msg.ranges[i] = static_cast<float>(i+0.5) 
  //                           * ping.ping()->rangeResolution;
  // }

  // sonar_msg.is_bigendian = false;
  // sonar_msg.data_size = ping.dataSize();

  // for (unsigned int r = 0; r < num_ranges; r++) {
  //   for (unsigned int b = 0; b < num_bearings; b++) {
  //     const uint16_t data = ping.image().at_uint16(b, r);

  //     if (ping.dataSize() == 1) {
  //       sonar_msg.intensities.push_back(data & 0xFF);
  //     } else if (ping.dataSize() == 2) {
  //       // Data is stored little-endian (lower byte first)
  //       sonar_msg.intensities.push_back(data & 0xFF);
  //       sonar_msg.intensities.push_back((data & 0xFF00) >> 8);
  //     }
  //   }
  // }
  imaging_sonar_pub_.publish(sonar_msg);
}

// Updates sonar parameters
void OculusDriver::configCallback(const oculus_sonar_driver::OculusSonarConfig &config,
                                  uint32_t level) {
  ROS_INFO_STREAM("Setting sonar range to " << config.range << " m");
  sonar_config_.setRange(config.range);

  ROS_INFO_STREAM("Setting gain to " << config.gain << " pct");
  sonar_config_.setGainPercent(config.gain);

  ROS_INFO_STREAM("Setting gamma to " << config.gamma);
  sonar_config_.setGamma(config.gamma);

  ROS_INFO_STREAM("Setting ping rate to (" << config.pingRate << "): "
                  << liboculus::PingRateToHz(config.pingRate) << " Hz");
  sonar_config_.setPingRate(static_cast<PingRateType>(config.pingRate));

  ROS_INFO_STREAM("Setting freq mode to " << liboculus::FreqModeToString(config.freqMode));
  sonar_config_.setFreqMode(static_cast<liboculus::SonarConfiguration::OculusFreqMode>(config.freqMode));

  // I would prefer to just or some consts together, but didn't see a clean
  // way to do that within dynamic reconfig. Ugly works.
  sonar_config_.flags().setRangeAsMeters(config.range_as_meters)
                        .setData16Bit(config.data_16bit)
                        .setSendGain(config.send_gain)
                        .setSimpleReturn(config.send_simple_return)
                        .setGainAssistance(config.gain_assistance)
                        .set512Beams(config.all_beams);

  ROS_INFO_STREAM("Setting flags: 0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned int>(sonar_config_.flags()())
            << std::dec << std::setw(0)
            << "\n   range is meters " << sonar_config_.flags().getRangeAsMeters()
            << "\n   data is 16 bit  " << sonar_config_.flags().getData16Bit()
            << "\n   send gain       " << sonar_config_.flags().getSendGain()
            << "\n   simple return   " << sonar_config_.flags().getSimpleReturn()
            << "\n   gain assistance " << sonar_config_.flags().getGainAssistance()
            << "\n   use 512 beams   " << sonar_config_.flags().get512Beams());

  // Update the sonar with new params
  if (data_rx_.isConnected()) {
    data_rx_.sendSimpleFireMessage(sonar_config_);
  }
};

}  // namespace oculus_sonar_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar_driver::OculusDriver, nodelet::Nodelet);
