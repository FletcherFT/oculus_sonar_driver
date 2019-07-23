#include "ros/ros.h"
#include "std_msgs/String.h"
// sonar message that aaron made
#include <cstdlib>
#include <sstream>
using std::string;
#include <imaging_sonar_msgs/ImagingSonarMsg.h>
// used to get ping info
#include "liboculus/SimplePingResult.h"



// inclusions from client.cpp
#include <boost/asio.hpp>
#include <boost/bind.hpp>

// originally was #include "libg3logger/g3logger.h" but kept getting errors
#include "/home/tanner/code/oculus_ws/src/libg3logger/include/libg3logger/g3logger.h"

// DataRx recieves pings from the sonar
#include "liboculus/DataRx.h"
// IoServiceThread allows for internet communication
#include "liboculus/IoServiceThread.h"
// pretty sure StatusRx validates sonar, might get ip address
#include "liboculus/StatusRx.h"

using namespace liboculus;

using std::ofstream;
using std::ios_base;

// Right now the approach is to basically copy client.cpp from the liboculus library
// to understand how to wait until a packet is recieved, then publish info

int main(int argc, char **argv) {
  ros::init(argc, argv, "oculus_node");
  ros::NodeHandle n;
  //ros::Publisher oculus_pub = ;// to be filled in with custom message info later

  try {
    IoServiceThread ioSrv;
    std::unique_ptr<StatusRx> statusRx( new StatusRx( ioSrv.service() ) );
    std::unique_ptr<DataRxQueued> dataRx( nullptr );


    ioSrv.fork();

    while( ros::ok() ) {
      // set up dataRx with correct ip address of sonar
      while( !dataRx ) {

        std::cout << "Need to find the sonar.  Waiting for sonar..." << std::endl;
        if( statusRx->status().wait_for(std::chrono::seconds(1)) ) {

          std::cout << "   ... got status message" << std::endl;
          if( statusRx->status().valid() ) {
            auto addr( statusRx->status().ipAddr() );

            std::cout << "Using detected sonar at IP address " << addr << std::endl;

            // around here client.cpp has:
            // if( verbosity > 0 ) statusRx->status().dump()
            // at lines 36, 37 of client.cpp we have:
            // int verbosity = 0;
            // app.add_flag("-v,--verbose", verbosity, "Additional output (use -vv for even more!)");
            // so I think it is !=0 if -v or -vv is passed. we want it to run in verbose by default
            statusRx->status().dump();

            dataRx.reset( new DataRxQueued( ioSrv.service(), addr ) );

          } else {
            std::cout << "   ... but it wasn't valid" << std::endl;
          }
        } else {
          // Failed to get status, try again.
        }
      }
      // dataRx should now be set up, can start doing things with packets
      shared_ptr<SimplePingResult> ping;
      dataRx->queue().wait_and_pop( ping );
      // just doing basic tests for now
      // TODO: make a callback method where ping is processed and published
      std::cout << "Got ping with ID " << ping->ping()->pingId << std::endl;

    }

    ioSrv.stop();

  }
  catch (std::exception& e) {
    std::cout << "Exception: " << e.what();
  }

  return 0;
}
