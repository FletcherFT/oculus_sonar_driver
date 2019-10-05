#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter() : it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ =
        it_.subscribe("/drawn_sonar", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter() { cv::destroyWindow(OPENCV_WINDOW); }

  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR16);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    cv::Mat im16(cv_ptr->image.rows, cv_ptr->image.cols, CV_16SC3);

    cv_ptr->image.convertTo(im16, CV_16SC3);
    cv::Vec3s intensity = im16.at<cv::Vec3s>(0, 0);
    std::cout << intensity << std::endl;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
