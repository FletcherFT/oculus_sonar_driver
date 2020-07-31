#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "imaging_sonar_msgs/ImagingSonarMsg.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

// For uploading drawn sonar images to Image topic
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"
#include <iostream>

#include "draw_sonar/ColorMaps.h"
#include "draw_sonar/DataStructures.h"
#include "draw_sonar/DrawSonar.h"


// Subscribes to sonar message topic, draws using opencv then publishes result

namespace oculus_sonar {

  struct ImagingSonarMsgInterface : public draw_sonar::AbstractSonarInterface {

    ImagingSonarMsgInterface( const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &ping )
      : _ping(ping) {;}

    virtual int nBearings() const         { return _ping->bearings.size(); }

    // Remember that the _original_ ImagingSonarMsg stored bearings as degrees
    virtual float bearing( int n ) const  { return _ping->bearings[n] * M_PI/180; }

    virtual int nRanges() const           { return _ping->ranges.size(); }
    virtual float range( int n ) const    { return _ping->ranges[n]; }

    virtual uint8_t intensity( int i ) const { return _ping->v2intensities[i]; }

    imaging_sonar_msgs::ImagingSonarMsg::ConstPtr _ping;
  };


  class OculusDrawNodelet : public nodelet::Nodelet {
  public:

    // NB Color Maps are in the draw_sonar package

    OculusDrawNodelet()
      : Nodelet(),
        _counter(0),
        _height(0), _width(0), _pixPerRangeBin(2),
        _colorMap( new draw_sonar::InfernoColorMap )
    {;}

    virtual ~OculusDrawNodelet()
    {;}

  private:

    virtual void onInit() {
      ros::NodeHandle nh = getMTNodeHandle();
      ros::NodeHandle pnh = getMTPrivateNodeHandle();
      //const std::string nodelet_name( getName() );

      pnh.param<int>("width", _width, 0);
      pnh.param<int>("height", _height, 0);
      pnh.param<int>("pix_per_range_bin", _pixPerRangeBin, 2 );

      sub_ = nh.subscribe("imaging_sonar", 10, &OculusDrawNodelet::imagingSonarCallback, this );

      pub_ = nh.advertise<sensor_msgs::Image>("drawn_sonar", 10);
    }

    void imagingSonarCallback(const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg) {

      ImagingSonarMsgInterface interface( msg );
      cv::Size sz = draw_sonar::calculateImageSize( interface, cv::Size( _width, _height), _pixPerRangeBin );
      cv::Mat mat( sz, CV_8UC3 );
      draw_sonar::drawSonar( interface, mat, *_colorMap );

      // Convert and publish drawn sonar images
      std_msgs::Header header;
      header.seq = _counter;
      header.stamp = ros::Time::now();

      cv_bridge::CvImage img_bridge(header,
                                    sensor_msgs::image_encodings::RGB8,
                                    mat);

      sensor_msgs::Image output_msg;
      img_bridge.toImageMsg(output_msg);
      pub_.publish(output_msg);

      _counter++;
    }


    ros::Subscriber sub_;
    ros::Publisher pub_;
    int _counter;

    int _height, _width, _pixPerRangeBin;

    std::unique_ptr< draw_sonar::SonarColorMap > _colorMap;

  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar::OculusDrawNodelet, nodelet::Nodelet);
