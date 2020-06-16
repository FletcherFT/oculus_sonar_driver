#include "imaging_sonar_msgs/ImagingSonarMsg.h"
#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef THETA_SHIFT
#define THETA_SHIFT PI;
#endif

using namespace std;
using namespace cv;

// For uploading drawn sonar images to Image topic
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"
#include <iostream>

// Subscribes to sonar message topic, draws using opencv then publishes result

int counter;
ros::Publisher oculus_drawn_pub;

// draw sonar using ImagingSonarMsg rather than a SimplePingResult
void drawSonar(const imaging_sonar_msgs::ImagingSonarMsg &msg) {

  int nRanges = msg.ranges.size();
  int nBeams = msg.bearings.size();
  int nIntensities = msg.v2intensities.size();
  // int nIntensities = msg.intensities.size();
  // how to recreate bearings() and image()
  // seems like bearings().at(x) is a 1D array, translates to values in
  // msg.bearings image.at(x,y) is a 2D array, maps to values in msg.intensities
  cv::Mat mat(500, 1000, CV_16SC3);
  mat.setTo(cv::Vec3s(0.0, 0.0, 0.0));
  mat.create(mat.size(), CV_16SC3);

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
      end = (msg.bearings[b + 1] + msg.bearings[b]) / 2.0;
      // originally: begin = 2*ping->bearings().at(b) - end;
      begin = 2 * msg.bearings[b] - end;

    } else if (b == nBeams - 1) {

      begin = angles[b - 1].second;
      // originally: end = 2*ping->bearings().at(b) - begin;
      end = 2 * msg.bearings[b] - begin;

    } else {

      begin = angles[b - 1].second;
      // originally: end = (ping->bearings().at(b+1) +
      // ping->bearings().at(b))/2.0;
      end = (msg.bearings[b + 1] + msg.bearings[b]) / 2.0;
    }

    angles[b] = make_pair(begin, end);
  }
  // Bearings
  std::vector<float> bearings;
  std::vector<float> ranges;
  for (unsigned int i = 0; i < nBeams; i++) {
    bearings.push_back(msg.bearings[i] + THETA_SHIFT);
  }
  // Ranges
  for (unsigned int i = 0; i < nRanges; i++) {
    ranges.push_back(msg.ranges[i]);
  }
  int bearing_count(0);

  for (unsigned int r = 0; r < nRanges; ++r) {
    for (unsigned int b = 0; b < nBeams; ++b) {

      float bearing = bearings.at(b);
      float range = ranges.at(r);
      float intensity = msg.v2intensities[(r * nBeams) + b];
      cv::Vec3s color(bearing * SCALE_FACTOR, range * SCALE_FACTOR,
                      intensity * SCALE_FACTOR);

      //
      const float begin = angles[b].first + 270, end = angles[b].second + 270;

      const float rad = float(radius * r) / nRanges;

      const float fudge = 0.7;

      // Assume angles are in image frame x-right, y-down
      cv::ellipse(mat, origin, cv::Size(rad, rad), 0, begin - fudge,
                  end + fudge, color, binThickness * 1.4);
    }
  }
  // for (unsigned int r = 0; r < nRanges; ++r) {
  //   for (unsigned int b = 0; b < nBeams; ++b) {
  //     cv::Vec3s intensity = mat.at<cv::Vec3s>(b, r);
  //     std::cout << intensity << std::endl;
  //   }
  // }

  // cv::imshow("ROS sonar", mat);
  // cv::waitKey(1);

  // Convert and publish drawn sonar images
  std_msgs::Header header;
  header.seq = counter;
  header.stamp = ros::Time::now();

  cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR16,
                                mat);

  sensor_msgs::Image output_msg;
  img_bridge.toImageMsg(output_msg);
  oculus_drawn_pub.publish(output_msg);
  counter++;
}

/*
void msgCallback(const imaging_sonar_msgs::ImagingSonarMsg& msg) {

  serdp_common::drawSonar(msg);

}
*/

int main(int argc, char **argv) {
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  ros::init(argc, argv, "oculus_sub");
  ros::NodeHandle n;

  oculus_drawn_pub = n.advertise<sensor_msgs::Image>("drawn_sonar", 100);

  //  ros::Subscriber sub = n.subscribe("sonar_info", 100, std::bind(&drawSonar,
  //  std::placeholders::_1, oculus_drawn_pub));
  ros::Subscriber sub = n.subscribe("imaging_sonar", 100, drawSonar);

  counter = 0;
  ros::spin();
  return 0;
}
