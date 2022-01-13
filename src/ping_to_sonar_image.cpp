// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#include "oculus_sonar_driver/ping_to_sonar_image.h"
#include "liboculus/Constants.h"

namespace oculus_sonar_driver {

using liboculus::SimplePingResult;

acoustic_msgs::SonarImage pingToSonarImage(const SimplePingResult &ping) {
  acoustic_msgs::SonarImage sonar_image;

  sonar_image.frequency = ping.ping()->frequency;

  // \todo This is actually frequency dependent
  if (sonar_image.frequency > 2000000) {
    sonar_image.azimuth_beamwidth = liboculus::Oculus_2100MHz::AzimuthBeamwidthRad;
    sonar_image.elevation_beamwidth = liboculus::Oculus_2100MHz::ElevationBeamwidthRad;
  } else if ((sonar_image.frequency > 1100000) && (sonar_image.frequency < 1300000)) {
    sonar_image.azimuth_beamwidth = liboculus::Oculus_1200MHz::AzimuthBeamwidthRad;
    sonar_image.elevation_beamwidth = liboculus::Oculus_1200MHz::ElevationBeamwidthRad;
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
  //    could actually be another arbitrary constant.
  sonar_image.ranges.resize(num_ranges);
  for (unsigned int i = 0; i < num_ranges; i++) {
    sonar_image.ranges[i] = static_cast<float>(i+0.5) 
                            * ping.ping()->rangeResolution;
  }

  sonar_image.is_bigendian = false;
  sonar_image.data_size = ping.dataSize();

  for (unsigned int r = 0; r < num_ranges; r++) {
    for (unsigned int b = 0; b < num_bearings; b++) {
      const uint16_t data = ping.image().at_uint16(b, r);

      if (ping.dataSize() == 1) {
        sonar_image.intensities.push_back(data & 0xFF);
      } else if (ping.dataSize() == 2) {
        // Data is stored little-endian (lower byte first)
        sonar_image.intensities.push_back(data & 0xFF);
        sonar_image.intensities.push_back((data & 0xFF00) >> 8);
      }
    }
  }

    return sonar_image;
}

}  // namespace oculus_sonar_driver