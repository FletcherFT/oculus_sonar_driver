#include "oculus_sonar_driver/OculusDriver.h"
#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include <boost/asio.hpp>

#include <acoustic_msgs/SonarImage.h>


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

  _imagingSonarPub = n_.advertise<acoustic_msgs::SonarImage>( "sonar_image", 100);
  _oculusRawPub = n_.advertise<oculus_sonar_driver::OculusSonarRawMsg>( "oculus_raw", 100);

  pn_.param<string>("ipAddress", _ipAddress, "auto");
  NODELET_INFO_STREAM("Opening sonar at " << _ipAddress );

  // It is not necessary to load any of the parameters controlled by
  // dynamic reconfigure, since dynamic reconfigure will read them from
  // the launch file and immediately publish an update message at launch.

  _sonarClient.reset( new SonarClient( sonarConfig, _ipAddress) );
  _sonarClient->setDataRxCallback( std::bind( &OculusDriver::pingCallback, this, std::placeholders::_1 ) );
  _sonarClient->start();

}


// Processes and publishes sonar pings to a ROS topic
void OculusDriver::pingCallback(const SimplePingResult &ping) {
  // TODO: It might make sense to have a generic "raw data" message type,
  //       used for all relevant hardware.
  oculus_sonar_driver::OculusSonarRawMsg raw_msg;

  raw_msg.header.seq = ping.oculusPing()->pingId;
  raw_msg.header.stamp = ros::Time::now();
  auto rawSize = ping.size();
  raw_msg.data.resize( rawSize );
  memcpy( raw_msg.data.data(), ping.ptr(), rawSize );
  _oculusRawPub.publish( raw_msg );

  // Publish message parsed into the image format
  acoustic_msgs::SonarImage sonar_msg;
  sonar_msg.header.seq = ping.oculusPing()->pingId;
  sonar_msg.header.stamp = raw_msg.header.stamp;
  sonar_msg.frequency = ping.oculusPing()->frequency;

  // \todo This is actually frequency dependent
  if( sonar_msg.frequency > 2000000 ) {
    sonar_msg.azimuth_beamwidth = 0.4*M_PI/180;
    sonar_msg.elevation_beamwidth = 12*M_PI/180;
  } else if( (sonar_msg.frequency > 1100000) && (sonar_msg.frequency < 1300000) ){
    sonar_msg.azimuth_beamwidth = 0.6*M_PI/180;
    sonar_msg.elevation_beamwidth = 20*M_PI/180;
  } else {
    ROS_ERROR_STREAM("Unsupported frequency received from oculus: "
		     << sonar_msg.frequency << ". Not publishing SonarImage "
		     << "for seq# " << raw_msg.header.seq);
    return;
  }

  const int nBearings = ping.oculusPing()->nBeams;
  const int nRanges = ping.oculusPing()->nRanges;

  for( unsigned int b = 0; b < nBearings; b++ ) {
    sonar_msg.azimuth_angles.push_back( ping.bearings().at( b ) * M_PI/180 );
  }

  for( unsigned int i = 0; i < nRanges; i++ ) {
    sonar_msg.ranges.push_back( float(i+0.5) * ping.oculusPing()->rangeResolution );
  }

  // Bytes for right now
  sonar_msg.is_bigendian = false;
  sonar_msg.data_size = 1;

  for( unsigned int r = 0; r < nRanges; r++ ) {
    for( unsigned int b = 0; b < nBearings; b++ ) {
      sonar_msg.intensities.push_back( ping.image().at(b,r) );
    }
  }
  _imagingSonarPub.publish(sonar_msg);

}

// Updates sonar parameters
void OculusDriver::configCallback(oculus_sonar_driver::OculusSonarConfig &config,
	                          uint32_t level) {

  sonarConfig.postponeCallback();

  ROS_INFO_STREAM("Setting sonar range to " << config.range << " m");
  sonarConfig.setRange(config.range);

  ROS_INFO_STREAM("Setting gain to " << config.gain << " pct");
  sonarConfig.setGainPercent(config.gain);

  ROS_INFO_STREAM("Setting gamma to " << config.gamma);
  sonarConfig.setGamma(config.gamma);

  ROS_INFO_STREAM("Setting ping rate to (" << config.pingRate << "): "
		  << PingRateToHz(config.pingRate) << " Hz" );
  sonarConfig.setPingRate( static_cast<PingRateType>(config.pingRate) );

  ROS_INFO_STREAM("Setting freq mode to " << FreqModeToString( config.freqMode) );
  sonarConfig.setFreqMode( static_cast<liboculus::SonarConfiguration::OculusFreqMode>(config.freqMode) );

  // I would prefer to just or some consts together, but didn't see a clean
  // way to do that within dynamic reconfig. Ugly works.
  uint8_t flags = 0;
  if (config.range_as_meters)    flags += 1;
  if (config.data_16bit)         flags += 2;
  if (config.send_gain)          flags += 4;
  if (config.send_simple_return) flags += 8;
  if (config.gain_assistance)    flags += 16;
  ROS_INFO_STREAM("Setting flags: "
                  << "\n   range is meters " << config.range_as_meters
                  << "\n   data is 16 bit  " << config.data_16bit
                  << "\n   send gain       " << config.send_gain
                  << "\n   simple return   " << config.send_simple_return
                  << "\n   gain assistance " << config.gain_assistance);
  sonarConfig.setFlags(flags);

  sonarConfig.enableCallback();
  // No need for sendCallback() because it is called by enableCallback.
}

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar::OculusDriver, nodelet::Nodelet);
