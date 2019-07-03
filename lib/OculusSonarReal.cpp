
#include <arpa/inet.h>

#include <ros/console.h>

#include "oculus_sonar_ros/OculusSonarReal.h"

namespace oculus_sonar_ros {

  using namespace liboculus;

  OculusSonarReal::OculusSonarReal( ros::NodeHandle &nh )
    : OculusSonarBase( nh ),
      _ioSrv(),
      _statusRx( new StatusRx( _ioSrv.service()) ),
      _dataRx( nullptr )
  {
    // Todo, handle non-auto sonar detection
    _ioSrv.fork();
  }

  OculusSonarReal::OculusSonarReal( ros::NodeHandle &nh, uint32_t ip )
    : OculusSonarBase( nh ),
      _ioSrv(),
      _statusRx( nullptr ),
      _dataRx( new DataRx( _ioSrv.service(), ip ) )
  {
    in_addr addr;
    addr.s_addr = ip;
    ROS_DEBUG_STREAM("Configuring sonar driver to look for IP address " << inet_ntoa( addr ) );

    _dataRx->setCallback( std::bind( &OculusSonarReal::simplePingResultCallback, this, std::placeholders::_1 ));

    // Todo, handle non-auto sonar detection
    _ioSrv.fork();
  }


  OculusSonarReal::~OculusSonarReal()
  {
    _ioSrv.stop();
  }

  void OculusSonarReal::sonarStatusCallback( const std::shared_ptr<SonarStatus> &status ) {
    ROS_DEBUG("   ... got status message");

    if( status && status->valid() ) {
      auto addr( status->ipAddr() );

      ROS_DEBUG_STREAM("Using detected sonar at IP address " << addr);

      //if( verbosity > 0 ) statusRx->status().dump();

      _dataRx.reset( new DataRx( _ioSrv.service(), addr ) );
      _dataRx->setCallback( std::bind( &OculusSonarReal::simplePingResultCallback, this, std::placeholders::_1 ));

    } else {
      ROS_DEBUG( "   ... but it wasn't valid" );
    }
  }

  void OculusSonarReal::simplePingResultCallback( const std::shared_ptr<SimplePingResult> &ping ) {
    ROS_DEBUG("   ... got ping");

    _simplePingPub.publish( pingToMessage( ping ));


  }


}
