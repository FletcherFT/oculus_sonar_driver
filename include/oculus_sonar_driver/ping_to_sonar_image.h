// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#pragma once

#include "ros/ros.h"

#include "acoustic_msgs/SonarImage.h"
#include "liboculus/SimplePingResult.h"

namespace oculus_sonar_driver {

    // Packs an acoustic_msgs::SonarImage from the contents of a SimplePingResult
    //
    // \todo Currently has no way to indicate failure...
    acoustic_msgs::SonarImage pingToSonarImage(const liboculus::SimplePingResult &ping);

}  // namespace oculus_sonar_driver