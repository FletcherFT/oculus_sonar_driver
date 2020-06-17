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

#include "serdp_common/ColorMaps.h"
#include "serdp_common/DataStructures.h"
#include "serdp_common/DrawSonar.h"


// Subscribes to sonar message topic, draws using opencv then publishes result

namespace oculus_sonar {

  class OculusDrawNodelet : public nodelet::Nodelet {
  public:

    // NB Color Maps are in the serdp_common package

    OculusDrawNodelet()
      : Nodelet(),
        _counter(0),
        _height(0), _width(0), _pixPerRangeBin(2),
        _colorMap( new serdp_common::InfernoColorMap )
    {;}

    virtual ~OculusDrawNodelet()
    {;}

  private:

    virtual void onInit() {
      ros::NodeHandle nh = getNodeHandle();

      nh.param<int>("width", _width, 0);
      nh.param<int>("height", _height, 0);
      nh.param<int>("pix_per_range_bin", _pixPerRangeBin, 2 );

      sub_ = nh.subscribe("imaging_sonar", 10, &OculusDrawNodelet::imagingSonarCallback, this );

      pub_ = nh.advertise<sensor_msgs::Image>("drawn_sonar", 10);
    }

    // For now, does a copy.  Will improve later...
    serdp_common::AbstractSonarData msgToAbstractSonarData( const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg ) {
        return serdp_common::AbstractSonarData( msg->bearings, msg->ranges, msg->v2intensities );
    }

    void imagingSonarCallback(const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg) {

      serdp_common::AbstractSonarData d( msgToAbstractSonarData(msg) );
      cv::Size sz = serdp_common::calculateImageSize( d, cv::Size( _width, _height), _pixPerRangeBin );
      cv::Mat mat( sz, CV_8UC3 );
      serdp_common::drawSonar( d, mat, *_colorMap );

      // Convert and publish drawn sonar images
      std_msgs::Header header;
      header.seq = _counter;
      header.stamp = ros::Time::now();

      cv_bridge::CvImage img_bridge(header,
                                    sensor_msgs::image_encodings::BGR8,
                                    mat);

      sensor_msgs::Image output_msg;
      img_bridge.toImageMsg(output_msg);
      pub_.publish(output_msg);

      _counter++;
    }

    // cv::Size calculateImageSize( const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg ) {
    //
    //   int h = _height, w = _width;
    //
    //   if( _width <= 0 ) {
    //     const int height = _height;
    //
    //     if( _height <= 0 ) {
    //         h = msg->ranges.size() * _pixPerRangeBin;
    //     }
    //
    //     // Assume bearings are symmetric plus and minus
    //     // Also assumes bearings are degrees
    //     w = 2*fabs(h*sin( M_PI/180 * msg->bearings[0] ));
    //
    //   } else if( _height <= 0 ) {
    //     h = (w/2) / fabs(sin( M_PI/180 * msg->bearings[0]));
    //   }
    //
    //   return Size(w,h);
    // }
    //
    // cv::Mat drawSonar(const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg) {
    //
    //   const int nRanges = msg->ranges.size();
    //   const int nBeams = msg->bearings.size();
    //
    //   const cv::Size sz = calculateImageSize( msg );
    //   cv::Mat mat( sz, CV_8UC3, Scalar(0.0, 0.0, 0.0));
    //
    //   const unsigned int radius = mat.size().height;
    //   const cv::Point origin(mat.size().width/2, mat.size().height);
    //
    //   const float binThickness = 3 * ceil(radius / nRanges);
    //
    //   // Current ImagingSonarMsg data is in _degrees_
    //   struct BearingEntry {
    //       float begin, center, end;
    //
    //       BearingEntry( float b, float c, float e )
    //         : begin( b ), center(c), end(e)
    //           {;}
    //   };
    //
    //   vector<BearingEntry> angles;
    //   angles.reserve( nBeams );
    //
    //   for (unsigned int b = 0; b < nBeams; ++b) {
    //     const float center = msg->bearings[b];
    //     float begin = 0.0, end = 0.0;
    //
    //     if (b == 0) {
    //
    //       end = (msg->bearings[b + 1] + center) / 2.0;
    //       begin = 2 * center - end;
    //
    //     } else if (b == nBeams - 1) {
    //
    //       begin = angles[b - 1].end;
    //       end = 2 * center - begin;
    //
    //     } else {
    //
    //       begin = angles[b - 1].end;
    //       end = (msg->bearings[b + 1] + center) / 2.0;
    //     }
    //
    //     angles.push_back( BearingEntry(begin, center, end) );
    //   }
    //
    //   for (unsigned int r = 0; r < nRanges; ++r) {
    //     for (unsigned int b = 0; b < nBeams; ++b) {
    //
    //       const float range = msg->ranges[r];
    //       const uint8_t intensity = msg->v2intensities[(r * nBeams) + b];
    //
    //       const float begin = angles[b].begin + ThetaShift,
    //                   end = angles[b].end + ThetaShift;
    //
    //       const float rad = float(radius * r) / nRanges;
    //
    //       const float fudge = 0; // in degrees
    //
    //       // Assume angles are in image frame x-right, y-down
    //       cv::ellipse(mat, origin, cv::Size(rad, rad), 0,
    //                   begin, end,
    //                   255*_colorMap->color( angles[b].center, range, intensity ),
    //                   binThickness * 1.4);
    //     }
    //   }
    //
    //   return mat;
    //
    // }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    int _counter;

    int _height, _width, _pixPerRangeBin;

    std::unique_ptr< serdp_common::SonarColorMap > _colorMap;

  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar::OculusDrawNodelet, nodelet::Nodelet);
