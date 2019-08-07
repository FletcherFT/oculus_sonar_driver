#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <sstream>
using std::string;
// Sonar ROS message that aaron made
#include <imaging_sonar_msgs/ImagingSonarMsg.h>
// Used to get sonar ping info
#include "liboculus/SimplePingResult.h"

// DataRx recieves pings from the sonar
#include "liboculus/DataRx.h"
// IoServiceThread allows for network communication
#include "liboculus/IoServiceThread.h"
// Pretty sure StatusRx validates sonar, might get ip address
#include "liboculus/StatusRx.h"

// For modifying sonar parameters
#include "liboculus/SimpleFireMessage.h"

// Allow dynamic reconfigure of sonar parameters
#include <dynamic_reconfigure/server.h>
#include <oculus_sonar_ros/OculusSonarConfig.h>
#include <thread>

using namespace liboculus;

std::unique_ptr<DataRxQueued> dataRx( nullptr );

// This node is basically the same as running client.cpp from liboculus/tools,
// just adapted to a ROS publisher node

// Processes and publishes sonar pings to a ROS topic
void pingCallback(auto ping, auto oculus_pub) {

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

  // Oculus data is natively bearing-major, so we need to
  // reshape it manually.  Too bad.
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
  oculus_pub.publish(sonar_msg);
}

// Updates sonar parameters
void configCallback(oculus_sonar_ros::OculusSonarConfig &config, uint32_t level) {
  if ( dataRx ) {
    SimpleFireMessage updateFireMsg;
    updateFireMsg.setRange(config.range);
    updateFireMsg.setGainPercent(config.gain);
    updateFireMsg.setGamma(config.gamma);
    updateFireMsg.setPingRate(config.ping_rate);
    updateFireMsg.setMasterMode(config.master_mode);
    dataRx->updateFireMessage(updateFireMsg);
  }
}

// Set up dynamic reconfigure server in separate thread
void reconfigListener() {
  dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig> server;
  dynamic_reconfigure::Server<oculus_sonar_ros::OculusSonarConfig>::CallbackType f;
  f = boost::bind(configCallback, _1, _2);
  server.setCallback(f);
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "oculus_node");
  ros::NodeHandle n;
  ros::Publisher oculus_pub = n.advertise<imaging_sonar_msgs::ImagingSonarMsg>("sonar_info", 1000);

  // Need to use threading in order to listen for reconfigure requests
  // and sonar packets at the same time
  std::thread thread_obj(reconfigListener);

  // Setting up initial sonar config according to launch file
  bool init = true;
  SimpleFireMessage initialConfig;
  int initRange, initGainPercent, initGamma, initPingRate, initMasterMode;
    // Get parameter values from launch file.
    // if no launch file was used, set equal to default values
  n.param<int>("initRange", initRange, 2);
  n.param<int>("initGainPercent", initGainPercent, 50);
  n.param<int>("initGamma", initGamma, 127);
  n.param<int>("initPingRate", initPingRate, 4);
  n.param<int>("initMasterMode", initMasterMode, 2);
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
      while( !dataRx ) {

        std::cout << "Need to find the sonar.  Waiting for sonar..." << std::endl;
        if( statusRx->status().wait_for(std::chrono::seconds(1)) ) {

          std::cout << "   ... got status message" << std::endl;
          if( statusRx->status().valid() ) {
            auto addr( statusRx->status().ipAddr() );

            std::cout << "Using detected sonar at IP address " << addr << std::endl;

            //statusRx->status().dump();

            dataRx.reset( new DataRxQueued( ioSrv.service(), addr ) );

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
        dataRx->updateFireMessage(initialConfig);
      }

      // Wait for pings from sonar
      shared_ptr<SimplePingResult> ping;
      dataRx->queue().wait_and_pop( ping );

      // Process and publish ping once recieved
      pingCallback(ping, oculus_pub);

    }

    ioSrv.stop();

  }
  catch (std::exception& e) {
    std::cout << "Exception: " << e.what();
  }

  return 0;
}
