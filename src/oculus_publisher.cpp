#include "oculus_sonar_ros/oculus_publisher.h"

//std::unique_ptr<DataRxQueued> dataRx( nullptr );

// This node is basically the same as running client.cpp from liboculus/tools,
// just adapted to a ROS publisher node

// Processes and publishes sonar pings to a ROS topic
void OculusPublisher::pingCallback(auto ping) {

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

/*
  for( unsigned int b = 0; b < nBearings; b++ ) {
    for( unsigned int r = 0; r < nRanges; r++ ) {
      sonar_msg.intensities.push_back( ping->image().at(b,r) );
    }
  }
*/
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

void OculusPublisher::loop() {
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

        std::cout << "Need to find the sonar.  Waiting for sonar..." << std::endl;
        if( statusRx->status().wait_for(std::chrono::seconds(1)) ) {

          std::cout << "   ... got status message" << std::endl;
          if( statusRx->status().valid() ) {
            auto addr( statusRx->status().ipAddr() );

            std::cout << "Using detected sonar at IP address " << addr << std::endl;

            //statusRx->status().dump();

            dataRx_.reset( new DataRxQueued( ioSrv.service(), addr ) );

          } else {
            std::cout << "   ... but it wasn't valid" << std::endl;
          }
        } else {
          // Failed to get status, try again.
        }
      }
      // DataRx is now working

      // Set up initial sonar config if this is the first loop
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
    std::cout << "Exception: " << e.what();
  }
}

OculusPublisher::OculusPublisher(int argc, char **argv) : dataRx_( nullptr ) {
  ros::init(argc, argv, "oculus_node");
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
  this->loop();
}

OculusPublisher::~OculusPublisher() {
  // not sure what to do here
  //ros::shutdown();
}

int main(int argc, char **argv) {
  OculusPublisher oculus_pub_node(argc, argv);
  //ros::init(argc, argv, "oculus_node");
  //ros::NodeHandle n;
  //ros::Publisher oculus_pub = n.advertise<imaging_sonar_msgs::ImagingSonarMsg>("sonar_info", 1000);

  // Need to use threading in order to listen for reconfigure requests
  // and sonar packets at the same time


  // Setting up initial sonar config according to launch file
  // bool init = true;
  // SimpleFireMessage initialConfig;
  // int initRange, initGainPercent, initGamma, initPingRate, initMasterMode;
  //   // Get parameter values from launch file.
  //   // if no launch file was used, set equal to default values
  // n.param<int>("initRange", initRange, 2);
  // n.param<int>("initGainPercent", initGainPercent, 50);
  // n.param<int>("initGamma", initGamma, 127);
  // n.param<int>("initPingRate", initPingRate, 2);
  // n.param<int>("initMasterMode", initMasterMode, 2);
  //   // Set up SimpleFireMessage for initial sonar configuration
  // initialConfig.setRange(initRange);
  // initialConfig.setGainPercent(initGainPercent);
  // initialConfig.setGamma(initGamma);
  // initialConfig.setPingRate(initPingRate);
  // initialConfig.setMasterMode(initMasterMode);
  //
  // try {
  //   IoServiceThread ioSrv;
  //   std::unique_ptr<StatusRx> statusRx( new StatusRx( ioSrv.service() ) );
  //   ioSrv.fork();
  //   // Run loop while ROS is up
  //   while( ros::ok() ) {
  //     // Set up dataRx with correct ip address of sonar
  //     while( !dataRx ) {
  //
  //       std::cout << "Need to find the sonar.  Waiting for sonar..." << std::endl;
  //       if( statusRx->status().wait_for(std::chrono::seconds(1)) ) {
  //
  //         std::cout << "   ... got status message" << std::endl;
  //         if( statusRx->status().valid() ) {
  //           auto addr( statusRx->status().ipAddr() );
  //
  //           std::cout << "Using detected sonar at IP address " << addr << std::endl;
  //
  //           //statusRx->status().dump();
  //
  //           dataRx.reset( new DataRxQueued( ioSrv.service(), addr ) );
  //
  //         } else {
  //           std::cout << "   ... but it wasn't valid" << std::endl;
  //         }
  //       } else {
  //         // Failed to get status, try again.
  //       }
  //     }
  //     // DataRx is now working
  //
  //     // Set up initial sonar config if this is the first loop
  //     if (init) {
  //       init = false;
  //       dataRx->updateFireMessage(initialConfig);
  //     }
  //
  //     // Wait for pings from sonar
  //     shared_ptr<SimplePingResult> ping;
  //     dataRx->queue().wait_and_pop( ping );
  //
  //     // Process and publish ping once recieved
  //     pingCallback(ping, oculus_pub);
  //
  //   }
  //
  //   ioSrv.stop();
  //
  // }
  // catch (std::exception& e) {
  //   std::cout << "Exception: " << e.what();
  // }

  return 0;
}
