// Copyright 2020 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#include <boost/asio.hpp>

#include <acoustic_msgs/SonarImage.h>
#include <apl_msgs/RawData.h>

#include "oculus_sonar_driver/OculusDriver.h"

namespace oculus_sonar_driver {

OculusDriver::OculusDriver()
  : Nodelet(),
    io_srv_(),
    data_rx_(io_srv_.context()),
    //status_rx_(io_srv_.context()),
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

  NODELET_INFO_STREAM("Advertising topics in namespace " << n_.getNamespace());
  NODELET_INFO_STREAM("Private namespace would be:" << pn_.getNamespace());

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
    // status_rx_.setCallback( [&](const liboculus::SonarStatus &status, bool is_valid) {
    //   if (!is_valid || data_rx_.isConnected()) return;
    //   data_rx_.connect(status.ipAddr());
    // });
  } else {
    NODELET_INFO_STREAM("Opening sonar at " << ip_address_);
    data_rx_.connect(ip_address_);
  }

  io_srv_.start();

}


// Processes and publishes sonar pings to a ROS topic
void OculusDriver::pingCallback(const liboculus::SimplePingResult &ping) {
  NODELET_INFO_STREAM("In ping callback");

  apl_msgs::RawData raw_msg;
  // NOTE(lindzey): I don't think we're supposed to use seq this way, but
  //     I also don't know that it breaks anything.
  raw_msg.header.seq = ping.oculusPing()->pingId;
  raw_msg.header.stamp = ros::Time::now();
  raw_msg.header.frame_id = frame_id_;
  raw_msg.direction = apl_msgs::RawData::DATA_IN;
  auto raw_size = ping.size();
  raw_msg.data.resize(raw_size);
  memcpy(raw_msg.data.data(), ping.ptr(), raw_size);
  raw_data_pub_.publish(raw_msg);

  // Publish message parsed into the image format
  acoustic_msgs::SonarImage sonar_msg;
  sonar_msg.header.seq = ping.oculusPing()->pingId;
  sonar_msg.header.stamp = raw_msg.header.stamp;
  sonar_msg.header.frame_id = frame_id_;
  sonar_msg.frequency = ping.oculusPing()->frequency;

  // \todo This is actually frequency dependent
  if (sonar_msg.frequency > 2000000) {
    sonar_msg.azimuth_beamwidth = 0.4*M_PI/180;
    sonar_msg.elevation_beamwidth = 12*M_PI/180;
  } else if ((sonar_msg.frequency > 1100000) && (sonar_msg.frequency < 1300000)) {
    sonar_msg.azimuth_beamwidth = 0.6*M_PI/180;
    sonar_msg.elevation_beamwidth = 20*M_PI/180;
  } else {
    ROS_ERROR_STREAM("Unsupported frequency received from oculus: "
                     << sonar_msg.frequency << ". Not publishing SonarImage "
                     << "for seq# " << raw_msg.header.seq);
    return;
  }

  const int num_bearings = ping.oculusPing()->nBeams;
  const int num_ranges = ping.oculusPing()->nRanges;

  // QUESTION(lindzey): it looks like bearings is commented out of the OculusSimplePingResult...
  // ANSWER(aaron): It's not commented out, the "bearing[]" is a ghost
  // indicating "there's a variable length array of shorts here but we're
  // not going to actually assign it a variable in the struct"
  // This is then followed by the block of sonar data...
  for (unsigned int b = 0; b < num_bearings; b++) {
    sonar_msg.azimuth_angles.push_back(ping.bearings().at(b) * M_PI/180);
  }

  // QUESTION(lindzey): Is this actually right?
  //    Do their ranges start at 0, or at the min range of 10 cm?
  for (unsigned int i = 0; i < num_ranges; i++) {
    sonar_msg.ranges.push_back(float(i+0.5) * ping.oculusPing()->rangeResolution);
  }

  // Only handle single-byte data for right now
  sonar_msg.is_bigendian = false;
  sonar_msg.data_size = 1;

  for (unsigned int r = 0; r < num_ranges; r++) {
    for (unsigned int b = 0; b < num_bearings; b++) {
      sonar_msg.intensities.push_back(ping.image().at(b, r));
    }
  }
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
  uint8_t flags = 0;
  if (config.range_as_meters)    flags += 1;
  if (config.data_16bit)         flags += 2;
  if (config.send_gain)          flags += 4;
  if (config.send_simple_return) flags += 8;
  if (config.gain_assistance)    flags += 16;
  if (config.all_beams)          flags += 64;  // 0x40
  ROS_INFO_STREAM("Setting flags: " << 1.0 * flags
                  << "\n   range is meters " << config.range_as_meters
                  << "\n   data is 16 bit  " << config.data_16bit
                  << "\n   send gain       " << config.send_gain
                  << "\n   simple return   " << config.send_simple_return
                  << "\n   gain assistance " << config.gain_assistance
                  << "\n   use 512 beams   " << config.all_beams);
  sonar_config_.setFlags(flags);

  // Update the sonar with new params
  if (data_rx_.isConnected()) {
    data_rx_.sendSimpleFireMessage(sonar_config_);
  }
};

}  // namespace oculus_sonar_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar_driver::OculusDriver, nodelet::Nodelet);
