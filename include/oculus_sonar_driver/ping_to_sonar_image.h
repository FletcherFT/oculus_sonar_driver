// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#pragma once

#include "ros/ros.h"

#include "acoustic_msgs/SonarImage.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/Constants.h"

namespace oculus_sonar_driver {

// Packs an acoustic_msgs::SonarImage from the contents of a SimplePingResult
//
// \todo Currently has no way to indicate failure...
template<typename PingT>
acoustic_msgs::SonarImage pingToSonarImage(const liboculus::SimplePingResult<PingT> &ping) {
  acoustic_msgs::SonarImage sonar_image;

  sonar_image.frequency = ping.ping()->frequency;

  // These fields are frequency dependent
  if (sonar_image.frequency > 2000000) {
    sonar_image.azimuth_beamwidth   = liboculus::Oculus_2100MHz::AzimuthBeamwidthRad;
    sonar_image.elevation_beamwidth = liboculus::Oculus_2100MHz::ElevationBeamwidthRad;
  } else if ((sonar_image.frequency > 1100000)
          && (sonar_image.frequency < 1300000)) {
    sonar_image.azimuth_beamwidth   = liboculus::Oculus_1200MHz::AzimuthBeamwidthRad;
    sonar_image.elevation_beamwidth = liboculus::Oculus_1200MHz::ElevationBeamwidthRad;
  } else if ((sonar_image.frequency > 650000)
          && (sonar_image.frequency < 850000)) {
    sonar_image.azimuth_beamwidth   = liboculus::Oculus_750kHz::AzimuthBeamwidthRad;
    sonar_image.elevation_beamwidth = liboculus::Oculus_750kHz::ElevationBeamwidthRad;
  } else {
    ROS_ERROR_STREAM("Unsupported frequency received from oculus: "
                     << sonar_image.frequency << ". Not publishing SonarImage "
                     << "for seq# " << sonar_image.header.seq);
  }

  const int num_bearings = ping.ping()->nBeams;
  const int num_ranges = ping.ping()->nRanges;

  sonar_image.azimuth_angles.resize(num_bearings);
  for (unsigned int b = 0; b < num_bearings; b++) {
    sonar_image.azimuth_angles[b] = ping.bearings().at_rad(b);
  }

  // QUESTION(lindzey): Is this actually right?
  //    Do their ranges start at 0, or at the min range of 10 cm?
  //
  // (Aaron):  We don't actually know.  Given there's no way to
  //    set "minimum range", and it's not in the data struct, we
  //    have to assume is starts from zero, though as you say, it
  //    could actually start at an arbitrary offset.
  sonar_image.ranges.resize(num_ranges);
  for (unsigned int i = 0; i < num_ranges; i++) {
    sonar_image.ranges[i] = static_cast<float>(i+0.5)
                            * ping.ping()->rangeResolution;
  }

  // \todo  Why am I byte-swapping the data below.  Why not set
  // is_bigendian to true?
  sonar_image.is_bigendian = false;
  sonar_image.data_size = ping.dataSize();

  for (unsigned int r = 0; r < num_ranges; r++) {
    for (unsigned int b = 0; b < num_bearings; b++) {
      if (ping.dataSize() == 1) {
        const uint8_t data = ping.image().at_uint8(b, r);
        sonar_image.intensities.push_back(data & 0xFF);
      } else if (ping.dataSize() == 2) {
        // Data is stored little-endian (lower byte first)
        const uint16_t data = ping.image().at_uint16(b, r);
        sonar_image.intensities.push_back(data & 0xFF);
        sonar_image.intensities.push_back((data & 0xFF00) >> 8);
      } else if (ping.dataSize() == 4) {
        // Data is stored in the sonar_image little-endian (lower byte first)
        const uint32_t data = ping.image().at_uint32(b, r);
        sonar_image.intensities.push_back(data & 0x000000FF);
        sonar_image.intensities.push_back((data & 0x0000FF00) >> 8);
        sonar_image.intensities.push_back((data & 0x00FF0000) >> 16);
        sonar_image.intensities.push_back((data & 0xFF000000) >> 24);
      }

    }
  }

    return sonar_image;
}

}  // namespace oculus_sonar_driver
