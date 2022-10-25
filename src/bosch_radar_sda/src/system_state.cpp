#include "system_state.h"

#include <map>
#include <glog/logging.h>

namespace driver {
namespace radar {

const RadarSystemState& GetSystemState(const std::string& radar_frame_id) {
    static std::map<std::string, RadarSystemState> kSystemState = {
        {
            "/side_left_radar",
            {
                SystemState::SIDE_LEFT_RADAR_HW_MAIN_BOARD_ERROR,
                SystemState::SIDE_LEFT_RADAR_DSP_ERROR,
                SystemState::SIDE_LEFT_RADAR_BLINDNESS_ERROR,
                SystemState::SIDE_LEFT_RADAR_ANTENNA_ERROR,
                SystemState::SIDE_LEFT_RADAR_MIS_ALIGNMENT_ERROR,
                SystemState::SIDE_LEFT_RADAR_CALIB_ERROR,
                SystemState::SIDE_LEFT_RADAR_CALIB_STATUS_OK,
                SystemState::SIDE_LEFT_RADAR_HW_MCU_ERROR,
                SystemState::SIDE_LEFT_RADAR_INIT_ERROR,
                SystemState::SIDE_LEFT_RADAR_CONNECTION_TIMEOUT,
            }
        },
        {
            "/side_right_radar",
            {
                SystemState::SIDE_RIGHT_RADAR_HW_MAIN_BOARD_ERROR,
                SystemState::SIDE_RIGHT_RADAR_DSP_ERROR,
                SystemState::SIDE_RIGHT_RADAR_BLINDNESS_ERROR,
                SystemState::SIDE_RIGHT_RADAR_ANTENNA_ERROR,
                SystemState::SIDE_RIGHT_RADAR_MIS_ALIGNMENT_ERROR,
                SystemState::SIDE_RIGHT_RADAR_CALIB_ERROR,
                SystemState::SIDE_RIGHT_RADAR_CALIB_STATUS_OK,
                SystemState::SIDE_RIGHT_RADAR_HW_MCU_ERROR,
                SystemState::SIDE_RIGHT_RADAR_INIT_ERROR,
                SystemState::SIDE_RIGHT_RADAR_CONNECTION_TIMEOUT,
            }
        }
    };

    if (radar_frame_id.empty()) {
        return kSystemState["/side_left_radar"];
    }

    auto iter = kSystemState.find(radar_frame_id);
    if(iter != kSystemState.end()) {
        return iter->second;
    }else {
        LOG(ERROR) << "invalid sensor frame id...";
        return kSystemState["/side_left_radar"];
    }
}

}  // namespace radar
}  // namespace driver