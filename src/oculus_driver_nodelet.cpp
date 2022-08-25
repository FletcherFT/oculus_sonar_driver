// Copyright 2020 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#include <boost/asio.hpp>

#include <acoustic_msgs/SonarImage.h>
#include <apl_msgs/RawData.h>

#include "liboculus/Constants.h"
#include "liboculus/SonarConfiguration.h"
#include "oculus_sonar_driver/oculus_driver_nodelet.h"
#include "oculus_sonar_driver/publishing_data_rx.h"

namespace oculus_sonar_driver {

using liboculus::SimplePingResultV1;
using liboculus::SimplePingResultV2;

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

  imaging_sonar_pub_ = n_.advertise<acoustic_msgs::SonarImage>("sonar_image", 10);
  oculus_meta_pub_ = n_.advertise<oculus_sonar_driver::OculusMetadata>("oculus_metadata",10);
  raw_data_pub_ = n_.advertise<apl_msgs::RawData>("raw_data", 100);

  // NB: Params set in the launch file go to /raven/oculus's namespace,
  //     rather than /raven/oculus/driver. For normal nodes, should definitely
  //     use the private namespace, but I'm not sure how to do that when
  //     nodelets have been compiled into the executable, rather than
  //     configured in a launch file.
  n_.param<std::string>("ip_address", ip_address_, "auto");

  n_.param<std::string>("frame_id", frame_id_, "oculus");
  NODELET_INFO_STREAM("Publishing data with frame = " << frame_id_);

  data_rx_.setRawPublisher(raw_data_pub_);

  data_rx_.setCallback<SimplePingResultV1>(std::bind(&OculusDriver::pingCallback<SimplePingResultV1>,
                                            this, std::placeholders::_1));

  data_rx_.setCallback<SimplePingResultV2>(std::bind(&OculusDriver::pingCallback<SimplePingResultV2>,
                                            this, std::placeholders::_1));

  // When the node connects, start the sonar pinging by sending
  // a OculusSimpleFireMessage current configuration.
  data_rx_.setOnConnectCallback([&]() {
    data_rx_.sendSimpleFireMessage(sonar_config_);
  });

  // It is not necessary to load any of the parameters controlled by
  // dynamic reconfigure, since dynamic reconfigure will read them from
  // the launch file and immediately publish an update message at launch.

  if (ip_address_ == "auto") {
    NODELET_INFO_STREAM("Attempting to auto-detect sonar");
    status_rx_.setCallback([&](const liboculus::SonarStatus &status, bool is_valid) {
      if (!is_valid || data_rx_.isConnected()) return;
      data_rx_.connect(status.ipAddr());
    });
  } else {
    NODELET_INFO_STREAM("Opening sonar at " << ip_address_);
    data_rx_.connect(ip_address_);
  }

  io_srv_.start();
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

  ROS_INFO_STREAM("Setting ping rate to (" << config.ping_rate << "): "
                  << liboculus::PingRateToHz(config.ping_rate) << " Hz");
  switch (config.ping_rate) {
    case OculusSonar_Normal:
      sonar_config_.setPingRate(pingRateNormal);
      break;
    case OculusSonar_High:
      sonar_config_.setPingRate(pingRateHigh);
      break;
    case OculusSonar_Highest:
      sonar_config_.setPingRate(pingRateHighest);
      break;
    case OculusSonar_Low:
      sonar_config_.setPingRate(pingRateLow);
      break;
    case OculusSonar_Lowest:
      sonar_config_.setPingRate(pingRateLowest);
      break;
    case OculusSonar_Standby:
      sonar_config_.setPingRate(pingRateStandby);
      break;
    default:
      ROS_WARN_STREAM("Unknown ping rate " << config.ping_rate);
  }

  ROS_INFO_STREAM("Setting freq mode to " << liboculus::FreqModeToString(config.freq_mode));
  switch (config.freq_mode) {
    case OculusSonar_LowFrequency:
      sonar_config_.setFreqMode(liboculus::OCULUS_LOW_FREQ);
      break;
    case OculusSonar_HighFrequency:
      sonar_config_.setFreqMode(liboculus::OCULUS_HIGH_FREQ);
      break;
    default:
      ROS_WARN_STREAM("Unknown frequency mode " << config.freq_mode);
  }

  sonar_config_.sendRangeAsMeters(config.send_range_as_meters)
                .setSendGain(config.send_gain)
                .setSimpleReturn(config.send_simple_return)
                .setGainAssistance(config.gain_assistance);

  if (config.num_beams == OculusSonar_256beams) {
    sonar_config_.use256Beams();
  } else {
    sonar_config_.use512Beams();
  }

  // Rather than trust that the .cfg and Oculus enums are the same,
  // do an explicit mapping
  ROS_INFO_STREAM("config.data_size: " << config.data_size);
  switch (config.data_size) {
    case OculusSonar_8bit:
      sonar_config_.setDataSize(dataSize8Bit);
      break;
    case OculusSonar_16bit:
      sonar_config_.setDataSize(dataSize16Bit);
      break;
    case OculusSonar_32bit:
      sonar_config_.setDataSize(dataSize32Bit);
      break;
    default:
      ROS_WARN_STREAM("Unknown data size " << config.data_size);
  }

  ROS_INFO_STREAM("Setting flags: 0x"
            // << std::hex << std::setw(2) << std::setfill('0')
            // << static_cast<unsigned int>(sonar_config_.flags()())
            // << std::dec << std::setw(0)
            << "\n send range in meters " << sonar_config_.getSendRangeAsMeters()
            << "\n            data size " << liboculus::DataSizeToString(sonar_config_.getDataSize())
            << "\n      send gain       " << sonar_config_.getSendGain()
            << "\n      simple return   " << sonar_config_.getSimpleReturn()
            << "\n      gain assistance " << sonar_config_.getGainAssistance()
            << "\n        use 512 beams " << sonar_config_.get512Beams());

  // Update the sonar with new params
  if (data_rx_.isConnected()) {
    data_rx_.sendSimpleFireMessage(sonar_config_);
  }
};

}  // namespace oculus_sonar_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar_driver::OculusDriver, nodelet::Nodelet);
