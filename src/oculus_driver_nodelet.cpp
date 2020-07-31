#include "oculus_sonar_ros/OculusDriver.h"
#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include <boost/asio.hpp>


namespace oculus_sonar {

OculusDriver::OculusDriver()
  : Nodelet(),
    _sonarClient(),
    _reconfigureServer()
 {;}

OculusDriver::~OculusDriver() {
    _sonarClient->stop();
    _sonarClient->join();
}

void OculusDriver::onInit() {

  _reconfigureServer.setCallback( boost::bind(&OculusDriver::configCallback, this, _1, _2) );

  ros::NodeHandle n_(getMTNodeHandle());
  ros::NodeHandle pn_(getMTPrivateNodeHandle());

  NODELET_INFO_STREAM("Advertising topics in namespace " << n_.getNamespace() );
  //const std::string nodelet_name( getName() );

  _imagingSonarPub = n_.advertise<imaging_sonar_msgs::ImagingSonarMsg>( "imaging_sonar", 100);
  _oculusRawPub = n_.advertise<oculus_sonar_ros::OculusSonarRawMsg>( "oculus_raw", 100);

  // This should be unnecessary, we should get a dynamic reconfigure callback
  // almost immediately with the params values.
  //
  // Get parameter values from launch file.
  // if no launch file was used, set equal to default values
  // int range, gainPercent, gamma, pingRate, freqMode;
  //
  // ni_.param<int>("range", range, 2);
  // ni_.param<int>("gainPercent", gainPercent, 50);
  // ni_.param<int>("gamma", gamma, 127);
  // ni_.param<int>("pingRate", pingRate, 0);
  // ni_.param<int>("freqMode", freqMode, 2);

  pn_.param<string>("ipAddress", _ipAddress, "auto");

  NODELET_INFO_STREAM("Opening sonar at " << _ipAddress );

  _sonarClient.reset( new SonarClient( sonarConfig, _ipAddress) );
  _sonarClient->setDataRxCallback( std::bind( &OculusDriver::pingCallback, this, std::placeholders::_1 ) );
  _sonarClient->start();

}


// Processes and publishes sonar pings to a ROS topic
void OculusDriver::pingCallback(const SimplePingResult &ping) {

  imaging_sonar_msgs::ImagingSonarMsg sonar_msg;
  oculus_sonar_ros::OculusSonarRawMsg raw_msg;

  // from aaron's OculusSonarBase.cpp
  sonar_msg.header.seq = ping.oculusPing()->pingId;
  sonar_msg.header.stamp = ros::Time::now();

  raw_msg.header.seq = sonar_msg.header.seq;
  raw_msg.header.stamp = sonar_msg.header.stamp;

  sonar_msg.frequency = ping.oculusPing()->frequency;

  const int nBearings = ping.oculusPing()->nBeams;
  const int nRanges = ping.oculusPing()->nRanges;

  for( unsigned int b = 0; b < nBearings; b++ ) {
    sonar_msg.bearings.push_back( ping.bearings().at( b ) );
  }

  for( unsigned int i = 0; i < nRanges; i++ ) {
    sonar_msg.ranges.push_back( float(i+0.5) * ping.oculusPing()->rangeResolution );
  }

  for( unsigned int r = 0; r < nRanges; r++ ) {
    for( unsigned int b = 0; b < nBearings; b++ ) {
      sonar_msg.v2intensities.push_back( ping.image().at(b,r) );
    }
  }
  _imagingSonarPub.publish(sonar_msg);

  auto rawSize = ping.size();
  raw_msg.data.resize( rawSize );
  memcpy( raw_msg.data.data(), ping.ptr(), rawSize );
  _oculusRawPub.publish( raw_msg );
}

// Updates sonar parameters
void OculusDriver::configCallback(oculus_sonar_ros::OculusSonarConfig &config, uint32_t level) {

  sonarConfig.postponeCallback();

  ROS_INFO_STREAM("Setting sonar range to " << config.range << " m");
  sonarConfig.setRange( config.range);

  ROS_INFO_STREAM("Setting gain to " << config.gain << " pct");
  sonarConfig.setGainPercent(config.gain);

  ROS_INFO_STREAM("Setting gamma to " << config.gamma);
  sonarConfig.setGamma(config.gamma);

  ROS_INFO_STREAM("Setting ping rate to (" << config.pingRate << "): " << PingRateToHz(config.pingRate) << " Hz" );
  sonarConfig.setPingRate( static_cast<PingRateType>(config.pingRate) );

  ROS_INFO_STREAM("Setting freq mode to " << FreqModeToString( config.freqMode) );
  sonarConfig.setFreqMode( static_cast<liboculus::SonarConfiguration::OculusFreqMode>(config.freqMode) );

  sonarConfig.sendCallback();
}

// // Set up dynamic reconfigure server in separate thread
// void OculusDriver::reconfigListener() {
//   dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig> server;
//   dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig>::CallbackType f;
//   f = boost::bind(&OculusDriver::configCallback, this, _1, _2);
//   server.setCallback(f);
//   ros::spin();
// }

// int main(int argc, char **argv) {
//   libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
//   logWorker.logBanner();
//   logWorker.verbose(2);
//
//   ros::init(argc, argv, "oculus_node");
//
//   _imagingSonarPubnode.run();
//
//   return 0;
// }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar::OculusDriver, nodelet::Nodelet);
