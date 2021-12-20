// Copyright 2020 UW-APL
// Authors: Aaron Marburg

#pragma once

#include <apl_msgs/RawData.h>

#include "liboculus/DataRx.h"

namespace oculus_sonar_driver {

class PublishingDataRx : public liboculus::DataRx {
 public:
  PublishingDataRx(const std::shared_ptr<boost::asio::io_context> &iosrv) 
          : DataRx(iosrv),
          count_(0) 
  {
  }

  ~PublishingDataRx() {}

  // Assume there are copiable
  void setRawPublisher(ros::Publisher pub) {
      raw_data_pub_ = pub;
  }

  // Implement data read / data written hooks as virtual functions rather
  // rather than callbacks
  // The override *must* call this function for the data to be written
  // to the networ
  virtual void haveWritten(const std::vector<uint8_t> &bytes) override {
    doPublish(bytes,apl_msgs::RawData::DATA_OUT);
  }

  virtual void haveRead(const std::vector<uint8_t> &bytes) override {
    doPublish(bytes,apl_msgs::RawData::DATA_IN);
  }

  void doPublish(const std::vector<uint8_t> &bytes, uint8_t direction) {
    if (bytes.size() == 0) return;

    apl_msgs::RawData raw_msg;
    // NOTE(lindzey): I don't think we're supposed to use seq this way, but
    //     I also don't know that it breaks anything.
    raw_msg.header.seq = count_++;
    raw_msg.header.stamp = ros::Time::now();
    raw_msg.direction = direction; 
    auto raw_size = bytes.size();
    raw_msg.data.resize(raw_size);
    memcpy(raw_msg.data.data(), bytes.data(), raw_size);
    raw_data_pub_.publish(raw_msg);
  }

  ros::Publisher raw_data_pub_;

  unsigned int count_;

};

}
