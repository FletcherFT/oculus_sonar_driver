#pragma once

#include <g3log/g3log.hpp>

#include <ros/console.h>

struct ROSLogSink {

  // This sink is a thin wrapper around the ROS logging functionality.
  // This class doesn't do any filtering ... let ROS handle it.
  ROSLogSink(const LEVELS threshold = INFO) { ; }

  ~ROSLogSink() { ; }

  void setThreshold(const LEVELS t) { ; }

  void ReceiveLogMessage(g3::LogMessageMover logEntry) {
    auto level = logEntry.get()._level;

    if (level == WARNING) {
      ROS_WARN_STREAM(logEntry.get().toString());
    } else if (level == DEBUG) {
      ROS_DEBUG_STREAM(logEntry.get().toString());
    } else {
      ROS_INFO_STREAM(logEntry.get().toString());
    }
  }

private:
};
