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

// Subscribes to sonar message topic, draws using opencv then publishes result

namespace oculus_sonar {

  class OculusDrawNodelet : public nodelet::Nodelet {
  public:

    const float ThetaShift = 270;
    const int PixelsPerRangeBin = 2;  // Only used if _height == _width == 0

    // TODO.  _height and _width should be params

    OculusDrawNodelet()
      : Nodelet(),
        _counter(0),
        _height(0), _width(0),
        _colorMap( new MitchellColorMap )
    {;}

    virtual ~OculusDrawNodelet()
    {;}

  private:

    virtual void onInit() {
      ros::NodeHandle nh = getNodeHandle();

      nh.param<int>("width", _width, 0);
      nh.param<int>("height", _height, 0);


      sub_ = nh.subscribe("imaging_sonar", 10, &OculusDrawNodelet::imagingSonarCallback, this );

      pub_ = nh.advertise<sensor_msgs::Image>("drawn_sonar", 10);
    }

    // void timerCb(const ros::TimerEvent& event){
    // // Using timers is the preferred 'ROS way' to manual threading
    // NODELET_INFO_STREAM("The time is now " << event.current_real);
    // }


    void imagingSonarCallback(const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg) {

      cv::Mat mat = drawSonar( msg );

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

    cv::Size calculateImageSize( const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg ) {

      int h = _height, w = _width;

      if( _width <= 0 ) {
        const int height = _height;

        if( _height <= 0 ) {
            h = msg->ranges.size() * PixelsPerRangeBin;
        }

        // Assume bearings are symmetric plus and minus
        // Also assumes bearings are degrees
        w = 2*fabs(h*sin( M_PI/180 * msg->bearings[0] ));

      } else if( _height <= 0 ) {
        h = (w/2) / fabs(sin( M_PI/180 * msg->bearings[0]));
      }

      return Size(w,h);
    }

    cv::Mat drawSonar(const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg) {

      const int nRanges = msg->ranges.size();
      const int nBeams = msg->bearings.size();
      const int nIntensities = msg->v2intensities.size();

      const cv::Size sz = calculateImageSize( msg );
      cv::Mat mat( sz, CV_8UC3, Scalar(0.0, 0.0, 0.0));

      const unsigned int radius = mat.size().height;
      const cv::Point origin(mat.size().width/2, mat.size().height);

      const float binThickness = 3 * ceil(radius / nRanges);

      // Current ImagingSonarMsg data is in _degrees_
      struct BearingEntry {
          float begin, center, end;

          BearingEntry( float b, float c, float e )
            : begin( b ), center(c), end(e)
              {;}
      };

      vector<BearingEntry> angles;
      angles.reserve( nBeams );

      for (unsigned int b = 0; b < nBeams; ++b) {
        const float center = msg->bearings[b];
        float begin = 0.0, end = 0.0;

        if (b == 0) {

          end = (msg->bearings[b + 1] + center) / 2.0;
          begin = 2 * center - end;

        } else if (b == nBeams - 1) {

          begin = angles[b - 1].end;
          end = 2 * center - begin;

        } else {

          begin = angles[b - 1].end;
          end = (msg->bearings[b + 1] + center) / 2.0;
        }

        angles.push_back( BearingEntry(begin, center, end) );
      }

      for (unsigned int r = 0; r < nRanges; ++r) {
        for (unsigned int b = 0; b < nBeams; ++b) {

          const float range = msg->ranges[r];
          const uint8_t intensity = msg->v2intensities[(r * nBeams) + b];

          const float begin = angles[b].begin + ThetaShift,
                      end = angles[b].end + ThetaShift;

          const float rad = float(radius * r) / nRanges;

          const float fudge = 0; // in degrees

          // Assume angles are in image frame x-right, y-down
          cv::ellipse(mat, origin, cv::Size(rad, rad), 0,
                      begin - fudge,
                      end + fudge,
                      _colorMap->color( angles[b].center, range, intensity ),
                      binThickness * 1.4);
        }
      }

      return mat;

    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    int _counter;

    int _height, _width;


    struct ColorMap {
      virtual cv::Scalar color( float bearing, float range, uint8_t intensity ) = 0;
    };

    struct MitchellColorMap : public ColorMap {
      virtual cv::Scalar color( float bearing, float range, uint8_t intensity ) {
        return Scalar( 256-intensity, intensity, intensity );
      }
    };

    std::unique_ptr< ColorMap > _colorMap;

  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar::OculusDrawNodelet, nodelet::Nodelet);
