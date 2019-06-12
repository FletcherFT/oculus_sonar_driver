
#include <ros/console.h>

#include "oculus_sonar_ros/OculusSonarBase.h"
#include <imaging_sonar_msgs/imagingSonarMsg.h>

namespace oculus_sonar_ros {

  using namespace imaging_sonar_msgs;

  OculusSonarBase::OculusSonarBase( ros::NodeHandle &nh )
    : _nh(nh),
      _simplePingPub( _nh.advertise<ImagingSonarMsg>("ping", 1) ),
      _seq(0)
  {
    ;
  }

  OculusSonarBase::~OculusSonarBase()
  {;}

  ImagingSonarMsg OculusSonarBase::pingToMessage( const std::shared_ptr<SimplePingResult> &ping )
  {

    ImagingSonarMsg msg;

    msg.header.seq = ping->ping()->pingId;;
    msg.header.stamp = ros::Time::now();

    msg.frequency = ping->ping()->frequency;

    const int nBearings = ping->ping()->nBeams;
    const int nRanges = ping->ping()->nRanges;

    for( unsigned int b = 0; b < nBearings; b++ ) {
      msg.bearings.push_back( ping->bearings().at( b ) );
    }

    for( unsigned int i = 0; i < nRanges; i++ ) {
      msg.ranges.push_back( float(i+0.5) * ping->ping()->rangeResolution );
    }

    // Oculus data is natively bearing-major, so we need to
    // reshape it manually.  Too bad.
    for( unsigned int b = 0; b < nBearings; b++ ) {
      for( unsigned int r = 0; r < nRanges; r++ ) {
        msg.intensities.push_back( ping->image().at(b,r) );
      }
    }



    return msg;
  }


}
