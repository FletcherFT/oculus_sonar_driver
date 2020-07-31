
#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "oculus");

  nodelet::Loader nodelet(true);
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name + "/driver", "oculus_sonar/driver", remap, nargv);
  nodelet.load(nodelet_name + "/draw", "oculus_sonar/draw", remap, nargv);

  ros::spin();
  return 0;
}
