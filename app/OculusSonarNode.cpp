
#include <arpa/inet.h>

#include <memory>
#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "oculus_sonar_ros/OculusSonarReal.h"
#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"


using namespace oculus_sonar_ros;

void timerCallback(const ros::TimerEvent&) {
  ROS_DEBUG("  .... waiting");
}


int main(int argc, char **argv) {

  // Setup loggers
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  ros::init(argc, argv, "oculus_sonar" );
  ros::NodeHandle nh( "oculus_sonar" );

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }


  std::string sonarIp;
  if(!nh.getParam("sonar_ip", sonarIp) ) {
    ROS_DEBUG("Unable to load parameter \"sonar_ip\"");
  }

  std::unique_ptr<OculusSonarReal> oculus( nullptr );

  if( sonarIp.size() > 0 ) {
    uint32_t ip = inet_addr( sonarIp.c_str() );

    ip = ntohl(ip);

    ROS_DEBUG_STREAM( "Launching node with IP address: " << sonarIp << "(" << ip << ")" );

    if( ip == -1 ) {
      ROS_FATAL_STREAM("Unable to parse IP address: " << sonarIp );
    }

    oculus.reset( new OculusSonarReal( nh, ip ) );
  } else {

    ROS_DEBUG("Attempting to automatically detect sonar");
    oculus.reset( new OculusSonarReal( nh ) );
  }

ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);

  ROS_DEBUG("Spin!");
  ros::spin();

  return 0;

}
