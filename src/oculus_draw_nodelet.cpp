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

    const float ThetaShift = M_PI;


    OculusDrawNodelet()
      : Nodelet(),
        _counter(0),
        _colorMap( new MitchellColorMap )
    {;}

    virtual ~OculusDrawNodelet()
    {;}

  private:

    virtual void onInit() {
      ros::NodeHandle nh = getNodeHandle();

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

      cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR16,
                                    mat);

      sensor_msgs::Image output_msg;
      img_bridge.toImageMsg(output_msg);
      pub_.publish(output_msg);

      _counter++;
    }

    cv::Mat drawSonar(const imaging_sonar_msgs::ImagingSonarMsg::ConstPtr &msg) {

      const int nRanges = msg->ranges.size();
      const int nBeams = msg->bearings.size();
      const int nIntensities = msg->v2intensities.size();

      cv::Mat mat(500, 1000, CV_16SC3, Scalar(0.0, 0.0, 0.0));

      const unsigned int radius = mat.size().width / 2;
      const cv::Point origin(radius, mat.size().height);

      const float binThickness = 3 * ceil(radius / nRanges);

      // LOG(DEBUG) << "binThickness is " << binThickness;

      // Build vector of start and end angles (in degrees, but still in sonar 0 ==
      // straight ahead frame)

      vector<pair<float, float>> angles(nBeams, make_pair(0.0f, 0.0f));

      for (unsigned int b = 0; b < nBeams; ++b) {
        float begin = 0.0, end = 0.0;

        // LOG(DEBUG) << "Bearing " << b << " is " << ping->bearings().at(b);
        if (b == 0) {

          // originally: end = (ping->bearings().at(b+1) +
          // ping->bearings().at(b))/2.0;
          end = (msg->bearings[b + 1] + msg->bearings[b]) / 2.0;
          // originally: begin = 2*ping->bearings().at(b) - end;
          begin = 2 * msg->bearings[b] - end;

        } else if (b == nBeams - 1) {

          begin = angles[b - 1].second;
          // originally: end = 2*ping->bearings().at(b) - begin;
          end = 2 * msg->bearings[b] - begin;

        } else {

          begin = angles[b - 1].second;
          // originally: end = (ping->bearings().at(b+1) +
          // ping->bearings().at(b))/2.0;
          end = (msg->bearings[b + 1] + msg->bearings[b]) / 2.0;
        }

        angles[b] = make_pair(begin, end);
      }
      // Bearings
      std::vector<float> bearings;
      std::vector<float> ranges;
      for (unsigned int i = 0; i < nBeams; i++) {
        bearings.push_back(msg->bearings[i] + ThetaShift);
      }
      // Ranges
      for (unsigned int i = 0; i < nRanges; i++) {
        ranges.push_back(msg->ranges[i]);
      }
      int bearing_count(0);

      for (unsigned int r = 0; r < nRanges; ++r) {
        for (unsigned int b = 0; b < nBeams; ++b) {

          float bearing = bearings.at(b);
          float range = ranges.at(r);
          uint8_t intensity = msg->v2intensities[(r * nBeams) + b];
          // cv::Vec3s color(bearing * SCALE_FACTOR, range * SCALE_FACTOR,
          //                 intensity * SCALE_FACTOR);

          //
          const float begin = angles[b].first + 270, end = angles[b].second + 270;

          const float rad = float(radius * r) / nRanges;

          const float fudge = 0.7;

          // Assume angles are in image frame x-right, y-down
          cv::ellipse(mat, origin, cv::Size(rad, rad), 0,
                      begin - fudge,
                      end + fudge,
                      _colorMap->color( bearing, range, intensity ),
                      binThickness * 1.4);
        }
      }


    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    int _counter;


    struct ColorMap {
      virtual cv::Scalar color( float bearing, float range, uint8_t intensity ) = 0;
    };

    struct MitchellColorMap : public ColorMap {
      virtual cv::Scalar color( float bearing, float range, uint8_t intensity ) {
        return Scalar( intensity, intensity, intensity );
      }
    };

    std::unique_ptr< ColorMap > _colorMap;

  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar::OculusDrawNodelet, nodelet::Nodelet);
