#pragma once

#include <monitor/status_report_msg.pb.h>

using drive::common::monitor_msg::SystemState;

namespace driver {
namespace radar {

struct RadarSystemState {
    SystemState HW_MAIN_BOARD_ERROR;
    SystemState DSP_ERROR;
    SystemState BLINDNESS_ERROR;
    SystemState ANTENNA_ERROR;
    SystemState MIS_ALIGNMENT_ERROR;
    SystemState CALIB_ERROR;
    SystemState CALIB_STATUS_OK;
    SystemState HW_MCU_ERROR;
    SystemState INIT_ERROR;
    SystemState CONNECTION_TIMEOUT;
};

const RadarSystemState& GetSystemState(const std::string& radar_frame_id);

} // namespace radar
} // namespace driver
