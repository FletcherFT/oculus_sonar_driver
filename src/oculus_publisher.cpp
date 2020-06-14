#include "oculus_sonar_ros/OculusPublisher.h"
#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include <boost/asio.hpp>



OculusPublisher::OculusPublisher()
  : _sonarClient() {
  ros::NodeHandle n_;
  _imagingSonarPub = n_.advertise<imaging_sonar_msgs::ImagingSonarMsg>("imaging_sonar", 100);
  _oculusRawPub = n_.advertise<oculus_sonar_ros::OculusSonarRawMsg>("oculus_raw", 100);

  //std::thread thread_obj(std::bind(&OculusPublisher::reconfigListener, this));
  // Get parameter values from launch file.
  // if no launch file was used, set equal to default values
  n_.param<int>("initRange", initRange, 2);
  n_.param<int>("initGainPercent", initGainPercent, 50);
  n_.param<int>("initGamma", initGamma, 127);
  n_.param<int>("initPingRate", initPingRate, 2);
  n_.param<int>("initMasterMode", initMasterMode, 2);

  n_.param<string>("ipAddress", _ipAddress, "auto");

  // Set up SimpleFireMessage for initial sonar configuration
  sonarConfig.setRange(initRange);
  sonarConfig.setGainPercent(initGainPercent);
  sonarConfig.setGamma(initGamma);
  //sonarConfig.setPingRate(initPingRate);
  //sonarConfig.setFreqMode(initMasterMode);

  //run();
}

OculusPublisher::~OculusPublisher() {
}


// Processes and publishes sonar pings to a ROS topic
void OculusPublisher::pingCallback(const shared_ptr<SimplePingResult> &ping) {

  imaging_sonar_msgs::ImagingSonarMsg sonar_msg;
  oculus_sonar_ros::OculusSonarRawMsg raw_msg;

  // from aaron's OculusSonarBase.cpp
  sonar_msg.header.seq = ping->oculusPing()->pingId;
  sonar_msg.header.stamp = ros::Time::now();

  raw_msg.header.seq = sonar_msg.header.seq;
  raw_msg.header.stamp = sonar_msg.header.stamp;

  sonar_msg.frequency = ping->oculusPing()->frequency;

  const int nBearings = ping->oculusPing()->nBeams;
  const int nRanges = ping->oculusPing()->nRanges;

  for( unsigned int b = 0; b < nBearings; b++ ) {
    sonar_msg.bearings.push_back( ping->bearings().at( b ) );
  }

  for( unsigned int i = 0; i < nRanges; i++ ) {
    sonar_msg.ranges.push_back( float(i+0.5) * ping->oculusPing()->rangeResolution );
  }

  // trying out array of uint8 instead of float32
  for( unsigned int r = 0; r < nRanges; r++ ) {
    for( unsigned int b = 0; b < nBearings; b++ ) {
      sonar_msg.v2intensities.push_back( ping->image().at(b,r) );
    }
  }
  _imagingSonarPub.publish(sonar_msg);

  auto rawSize = ping->buffer()->size();
  raw_msg.data.resize( rawSize );
  memcpy( raw_msg.data.data(), ping->buffer()->ptr(), rawSize );
  _oculusRawPub.publish( raw_msg );
}

// Updates sonar parameters
void OculusPublisher::configCallback(oculus_sonar_ros::OculusSonarConfig &config, uint32_t level) {

  sonarConfig.postponeCallback();

  sonarConfig.setRange( config.range);
  sonarConfig.setGainPercent(config.gain);
  sonarConfig.setGamma(config.gamma);
  //sonarConfig.setPingRate(config.ping_rate);
  //sonarConfig.setFreqMode(config.master_mode);
  sonarConfig.sendCallback();
}

// // Set up dynamic reconfigure server in separate thread
// void OculusPublisher::reconfigListener() {
//   dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig> server;
//   dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig>::CallbackType f;
//   f = boost::bind(&OculusPublisher::configCallback, this, _1, _2);
//   server.setCallback(f);
//   ros::spin();
// }

void OculusPublisher::run() {

  try {

    _sonarClient.reset( new SonarClient( sonarConfig, _ipAddress) );

    _sonarClient->setDataRxCallback( std::bind( &OculusPublisher::pingCallback, this, std::placeholders::_1 ) );

    _sonarClient->start();

    ros::spin();

    // // Run loop while ROS is up
    // while( ros::ok() ) {
    //
    //
    //   //
    //   //
    //   // // Set up dataRx with correct ip address of sonar
    //   // while( !dataRx_ ) {
    //   //
    //   //
    //   //   LOG(DEBUG) << "Need to find the sonar.  Waiting for sonar...";
    //   //
    //   //   if( statusRx->status().wait_for(std::chrono::seconds(1)) ){
    //   //     boost::asio::ip::address addr;
    //   //     LOG(INFO) << "Got sonar message";
    //   //     if( statusRx->status().valid() ) {
    //   //       addr = statusRx->status().ipAddr();
    //   //
    //   //       LOG(DEBUG) << "Using detected sonar at IP address " << addr;
    //   //
    //   //       //statusRx->status().dump();
    //   //
    //   //       dataRx_.reset( new DataRxQueued( ioSrv.service(), addr ) );
    //   //
    //   //     } else {
    //   //         LOG(WARNING) << "IP address " << addr << " wasn't valid";
    //   //     }
    //   //   } else {
    //   //     LOG(WARNING) << "Failed to get status, trying again";
    //   //   }
    //   // }
    //   // if (init) {
    //   //   init = false;
    //   //   dataRx_->updateFireMessage(initialConfig);
    //   // }
    //   //
    //   // // Wait for pings from sonar
    //   // shared_ptr<SimplePingResult> ping;
    //   // dataRx_->queue().wait_and_pop( ping );
    //   //
    //   // // Process and publish ping once recieved
    //   // this->pingCallback(ping);
    //   ros::spinOnce();
    //
    // }

    _sonarClient->stop();
    _sonarClient->join();

  }
  catch (std::exception& e) {
    LOG(WARNING) << e.what();
  }
}

int main(int argc, char **argv) {
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  ros::init(argc, argv, "oculus_node");

   dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig> server;
   dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig>::CallbackType f;

  OculusPublisher _imagingSonarPubnode;
   f = boost::bind(&OculusPublisher::configCallback, &_imagingSonarPubnode, _1, _2);
   server.setCallback(f);


  _imagingSonarPubnode.run();

  return 0;
}
