

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>


#include "oculus_sonar_ros/OculusSonarPlayback.h"

using namespace oculus_sonar_ros;

int main(int argc, char **argv) {

  ros::init(argc, argv, "oculus_sonar");
  ros::NodeHandle nh("oculus_sonar");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }

  std::string pointcloud_topic;
  bool do_fusion;

  OculusSonarPlayback oculus( nh );

  ros::spin();

  return 0;

}