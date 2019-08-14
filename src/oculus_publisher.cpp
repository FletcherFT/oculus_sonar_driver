#include "oculus_sonar_ros/OculusPublisher.h"
#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include <boost/asio.hpp>



OculusPublisher::OculusPublisher() : dataRx_( nullptr ) {
  ros::NodeHandle n_;
  oculus_pub_ = n_.advertise<imaging_sonar_msgs::ImagingSonarMsg>("sonar_info", 1000);
  std::thread thread_obj(std::bind(&OculusPublisher::reconfigListener, this));
  // Get parameter values from launch file.
  // if no launch file was used, set equal to default values
  n_.param<int>("initRange", initRange, 2);
  n_.param<int>("initGainPercent", initGainPercent, 50);
  n_.param<int>("initGamma", initGamma, 127);
  n_.param<int>("initPingRate", initPingRate, 2);
  n_.param<int>("initMasterMode", initMasterMode, 2);

  run();
}

OculusPublisher::~OculusPublisher() {
}


// Processes and publishes sonar pings to a ROS topic
void OculusPublisher::pingCallback(shared_ptr<SimplePingResult> ping) {

  imaging_sonar_msgs::ImagingSonarMsg sonar_msg;
  // from aaron's OculusSonarBase.cpp
  sonar_msg.header.seq = ping->ping()->pingId;;
  sonar_msg.header.stamp = ros::Time::now();
  sonar_msg.frequency = ping->ping()->frequency;

  const int nBearings = ping->ping()->nBeams;
  const int nRanges = ping->ping()->nRanges;

  for( unsigned int b = 0; b < nBearings; b++ ) {
    sonar_msg.bearings.push_back( ping->bearings().at( b ) );
  }

  for( unsigned int i = 0; i < nRanges; i++ ) {
    sonar_msg.ranges.push_back( float(i+0.5) * ping->ping()->rangeResolution );
  }

  // trying out array of uint8 instead of float32
  for( unsigned int r = 0; r < nRanges; r++ ) {
    for( unsigned int b = 0; b < nBearings; b++ ) {
      sonar_msg.v2intensities.push_back( ping->image().at(b,r) );
    }
  }
  oculus_pub_.publish(sonar_msg);
}

// Updates sonar parameters
void OculusPublisher::configCallback(oculus_sonar_ros::OculusSonarConfig &config, uint32_t level) {
  if ( dataRx_ ) {
    SimpleFireMessage updateFireMsg;
    updateFireMsg.setRange(config.range);
    updateFireMsg.setGainPercent(config.gain);
    updateFireMsg.setGamma(config.gamma);
    updateFireMsg.setPingRate(config.ping_rate);
    updateFireMsg.setMasterMode(config.master_mode);
    dataRx_->updateFireMessage(updateFireMsg);
  }
}

// Set up dynamic reconfigure server in separate thread
void OculusPublisher::reconfigListener() {
  dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig> server;
  dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig>::CallbackType f;
  f = boost::bind(&OculusPublisher::configCallback, this, _1, _2);
  server.setCallback(f);
  ros::spin();
}

void OculusPublisher::run() {
  bool init = true;
  SimpleFireMessage initialConfig;
    // Set up SimpleFireMessage for initial sonar configuration
  initialConfig.setRange(initRange);
  initialConfig.setGainPercent(initGainPercent);
  initialConfig.setGamma(initGamma);
  initialConfig.setPingRate(initPingRate);
  initialConfig.setMasterMode(initMasterMode);
  try {
    IoServiceThread ioSrv;
    std::unique_ptr<StatusRx> statusRx( new StatusRx( ioSrv.service() ) );
    ioSrv.fork();
    // Run loop while ROS is up
    while( ros::ok() ) {
      // Set up dataRx with correct ip address of sonar
      while( !dataRx_ ) {


        LOG(DEBUG) << "Need to find the sonar.  Waiting for sonar...";

        if( statusRx->status().wait_for(std::chrono::seconds(1)) ){
          boost::asio::ip::address addr;
          LOG(INFO) << "Got sonar message";
          if( statusRx->status().valid() ) {
            addr = statusRx->status().ipAddr();

            LOG(DEBUG) << "Using detected sonar at IP address " << addr;

            //statusRx->status().dump();

            dataRx_.reset( new DataRxQueued( ioSrv.service(), addr ) );

          } else {
              LOG(WARNING) << "IP address " << addr << " wasn't valid";
          }
        } else {
          LOG(WARNING) << "Failed to get status, trying again";
        }
      }
      if (init) {
        init = false;
        dataRx_->updateFireMessage(initialConfig);
      }

      // Wait for pings from sonar
      shared_ptr<SimplePingResult> ping;
      dataRx_->queue().wait_and_pop( ping );

      // Process and publish ping once recieved
      this->pingCallback(ping);

    }

    ioSrv.stop();

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
  OculusPublisher oculus_pub_node;

  return 0;
}
