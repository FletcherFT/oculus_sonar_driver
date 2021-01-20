#pragma once

#include <g3log/g3log.hpp>

#include <ros/console.h>

struct ROSLogSink {

  // This sink is a thin wrapper around the ROS logging functionality.
  // This class doesn't do any filtering ... let ROS handle it.
  ROSLogSink(const LEVELS threshold = INFO) { ; }

  ~ROSLogSink() { ; }

  // Level is ignored, uses ROS filtering instead
  void setThreshold(const LEVELS t) { ; }

  void ReceiveLogMessage(g3::LogMessageMover logEntry) {
    auto level = logEntry.get()._level;

    std::string entry = logEntry.get().toString();

    // Strip the trailing newline, otherwise we get conflicting/redundant newlines
    entry.pop_back();

    if (level == WARNING) {
      ROS_WARN_STREAM(entry);
    } else if (level == DEBUG) {
      ROS_DEBUG_STREAM(entry);
    } else {
      ROS_INFO_STREAM(entry);
    }
  }

private:
};
