

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

  std::string sonarFilename;
  if(!nh.getParam("/sonar_filename", sonarFilename)) {
    ROS_FATAL("Parameter \"sonar_filename\" not defined");
  }

  float sonarFps;
  nh.param<float>("/sonar_fps", sonarFps, 10.0f);

  bool doLoop;
  nh.param("/do_loop", doLoop, false );

  if( sonarFps <= 0 ) {
    ROS_FATAL("sonarFps less than zero");
  }

  OculusSonarPlayback oculus( nh, 1.0/sonarFps );
  oculus.doLoop( doLoop );
  if(!oculus.open( sonarFilename )) {
    ROS_FATAL_STREAM("Unable to open sonar filename \"" << sonarFilename << "\"");
  }

  ros::spin();

  return 0;

}
