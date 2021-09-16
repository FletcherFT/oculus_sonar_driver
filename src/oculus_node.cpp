#include "ros/ros.h"
#include "nodelet/loader.h"

#include "g3log_catkin/ROSLogSink.h"
#include "g3log_catkin/g3logger.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "oculus");

  libg3logger::G3Logger<ROSLogSink> log_worker("oculus_node");

  nodelet::Loader nodelet(true);
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name + "/driver", "oculus_sonar/driver", remap, nargv);
  nodelet.load(nodelet_name + "/draw", "draw_sonar", remap, nargv);

  ros::spin();
  return 0;
}
