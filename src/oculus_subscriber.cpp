#include "ros/ros.h"
#include "imaging_sonar_msgs/ImagingSonarMsg.h"
#include "serdp_common/DrawSonar.h"

using namespace serdp_common;

// Subscribes to sonar message topic, displays using opencv

void msgCallback(const imaging_sonar_msgs::ImagingSonarMsg& msg) {

  serdp_common::drawSonar(msg);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "oculus_sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sonar_info", 100, msgCallback);
  ros::spin();
  return 0;
}
