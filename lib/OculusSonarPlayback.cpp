
#include <ros/console.h>

#include "oculus_sonar_ros/OculusSonarPlayback.h"

namespace oculus_sonar_ros {

  OculusSonarPlayback::OculusSonarPlayback( ros::NodeHandle &nh, float dt )
    : OculusSonarBase( nh ),
      _player(),
      _doLoop(false),
      _dt(dt)
  {
  }

  OculusSonarPlayback::~OculusSonarPlayback()
  {;}

  bool OculusSonarPlayback::open( const std::string &filename )
  {
    _filename = filename;
    bool success =  _player.open(filename);

    if( success ) {
      _timer = _nh.createTimer(ros::Duration(_dt), &OculusSonarPlayback::nextPacket, this );
    }

    return success;
  }

  void OculusSonarPlayback::nextPacket( const ros::TimerEvent& )
  {
    if( !_player.isOpen() ) {
      ROS_WARN("Attempting to send next packet, but player is not open");
      return;
    }

    std::shared_ptr<SimplePingResult> ping( _player.nextPing() );

    ROS_WARN_COND(!bool(ping), "Unable to read next ping from sonar file");

    if( _doLoop && _player.eof() ) {
      ROS_DEBUG("Sonar file reached EOF, rewinding.");
      _player.rewind();
    }

    _simplePingPub.publish( pingToMessage( ping ));

  }

}
