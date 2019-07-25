#include "ros/ros.h"
#include "imaging_sonar_msgs/ImagingSonarMsg.h"
#include "serdp_common/DrawSonar.h"
// not sure if this next one is needed
#include "serdp_common/OpenCVDisplay.h"
// to concert ImagingSonarMsg back into SimplePingResult
#include "liboculus/SimplePingResult.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "std_msgs/String.h"
#include <cstdlib>
#include <sstream>
using std::string;

#include "/home/tanner/code/oculus_ws/src/libg3logger/include/libg3logger/g3logger.h"

using namespace liboculus;

using std::ofstream;
using std::ios_base;

// Subscribes to sonar message topic, displays using opencv

void drawSonar(const shared_ptr<imaging_sonar_msgs::ImagingSonarMsg>& msg_ptr);

void msgCallback(const imaging_sonar_msgs::ImagingSonarMsg& msg) {
  // does not currently work
  //auto msg_ptr = msg;
  //drawSonar(msg);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "oculus_sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sonar_info", 100, msgCallback);
  ros::spin();
  return 0;
}
