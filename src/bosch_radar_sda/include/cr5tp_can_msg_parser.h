#pragma once

#include <vector>
#include <string>
#include <gflags/gflags.h>
#include "const_vars.h"
#include "can_node.h"

namespace driver {
namespace radar {
using CANFDArray = drive::common::can::CANFDArray;

bool ParseReport_RadarState(int sensor_id, const SetupMap& setup_map, const CANFDArray& data);
bool ParseReport_Diagnostic(int sensor_id, const SetupMap& setup_map, const CANFDArray& data);

/* modify the interface to support unit test */
bool ParseRadarTrackInfo(int sensor_id, const CANFDArray& data, RadarTrackObject& radarTrackObject);
bool ParseRadarParsedInfo(int sensor_id, const CANFDArray& data, RadarParsedObject& radarParsedObject);
bool ParseReport_ObjectInfo(int sensor_id, const SetupMap& setup_, const CANFDArray& data,
                            RadarTrackObject& radarTrackObject, RadarParsedObject& radarParsedObject);


}  // namespace radar
}  // namespace driver
