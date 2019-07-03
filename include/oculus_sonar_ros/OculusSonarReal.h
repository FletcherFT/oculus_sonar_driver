#pragma once

#include <ros/ros.h>

#include "oculus_sonar_ros/OculusSonarBase.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/DataRx.h"
#include "liboculus/StatusRx.h"

namespace oculus_sonar_ros {

  class OculusSonarReal : public OculusSonarBase {
  public:

    OculusSonarReal() = delete;
    OculusSonarReal( const OculusSonarReal & ) = delete;

    // Attempt auto-detection
    OculusSonarReal(ros::NodeHandle &nh );

    // With IP address
    OculusSonarReal(ros::NodeHandle &nh, uint32_t ip );

    virtual ~OculusSonarReal();

    void initialize();

  private:

    liboculus::IoServiceThread _ioSrv;
    std::unique_ptr<liboculus::StatusRx> _statusRx;
    std::unique_ptr<liboculus::DataRx> _dataRx;

    void sonarStatusCallback( const std::shared_ptr<liboculus::SonarStatus> & );
    void simplePingResultCallback( const std::shared_ptr<liboculus::SimplePingResult> & );


  };

}
