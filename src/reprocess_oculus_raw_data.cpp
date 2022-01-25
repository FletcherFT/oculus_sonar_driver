// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#include <memory>

#include "oculus_sonar_driver/reprocess_oculus_raw_data.h"
#include "oculus_sonar_driver/ping_to_sonar_image.h"


#include "acoustic_msgs/SonarImage.h"
#include "liboculus/Constants.h"
#include "liboculus/MessageHeader.h"
#include "liboculus/SimplePingResult.h"

namespace oculus_sonar_driver {

ReprocessOculusRawData::ReprocessOculusRawData()
{;}

ReprocessOculusRawData::~ReprocessOculusRawData() {
  ;
}

void ReprocessOculusRawData::onInit() {
  ros::NodeHandle n_(getMTNodeHandle());
  ros::NodeHandle pn_(getMTPrivateNodeHandle());

  NODELET_DEBUG_STREAM("Advertising topics in namespace " << n_.getNamespace());
  NODELET_DEBUG_STREAM("Private namespace would be:" << pn_.getNamespace());

  sonar_image_pub_ = n_.advertise<acoustic_msgs::SonarImage>("sonar_image", 100);
  raw_data_sub_ = n_.subscribe("raw_data", 1000, &ReprocessOculusRawData::rawDataCallback, this);
}


// Processes and publishes sonar pings to a ROS topic
void ReprocessOculusRawData::rawDataCallback(const apl_msgs::RawData::ConstPtr &raw_data) {
  ROS_DEBUG_STREAM("Got raw data seq " << raw_data->header.seq);
  
  // Ignore data out
  if (raw_data->direction != apl_msgs::RawData::DATA_IN) return;

  // Check for Oculus header bytes
  if ((raw_data->data[0] != liboculus::PacketHeaderLSB) ||
      (raw_data->data[1] != liboculus::PacketHeaderMSB)) return;

  // n.b. this is a deep copy.  Not sure if it's avoidable
  std::shared_ptr<liboculus::ByteVector> buffer(std::make_shared<liboculus::ByteVector>(raw_data->data));

  liboculus::MessageHeader header(buffer);
  if (!header.valid()) {
    ROS_INFO_STREAM("Header not valid, ignoring...");
    return;
  }

  const auto msgId = header.msgId();
  if (msgId == messageSimplePingResult) {
    if (header.msgVersion() == 2) {
      liboculus::SimplePingResultV2 ping(buffer);

      // Publish message parsed into the image format
      acoustic_msgs::SonarImage sonar_image = pingToSonarImage(ping);

      // Overwrite the header with info from the incoming packet
      sonar_image.header = raw_data->header;
      sonar_image_pub_.publish(sonar_image);
    } else {
      liboculus::SimplePingResultV1 ping(buffer);

      // Publish message parsed into the image format
      acoustic_msgs::SonarImage sonar_image = pingToSonarImage(ping);

      // Overwrite the header with info from the incoming packet
      sonar_image.header = raw_data->header;
      sonar_image_pub_.publish(sonar_image);
    }
  } else if (msgId == messageLogs) {
    // If it's a log message, just stream to the ROS log
    ROS_INFO_STREAM(std::string(buffer->begin()+sizeof(OculusMessageHeader), buffer->end()));
  }
}

}  // namespace oculus_sonar_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(oculus_sonar_driver::ReprocessOculusRawData, nodelet::Nodelet);
