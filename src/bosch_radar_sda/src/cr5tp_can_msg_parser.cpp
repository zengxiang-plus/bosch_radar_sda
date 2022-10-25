#include "cr5tp_can_msg_parser.h"
#include <vector>

#include <glog/logging.h>
#include <geometry_msgs/Point32.h>
#include <monitor/status_reporter.h>

// DECLARE_bool(enable_pub_parsed_info);

namespace driver {
namespace radar {
PLUSAI_DECLARE_bool(enable_pub_parsed_info);
PLUSAI_DECLARE_int32(error_report_frequency);
PLUSAI_DEFINE_bool(enable_pub_parsed_info, false, "flag to enable publish parsed topic or not");
PLUSAI_DEFINE_int32(error_report_frequency, 5, "the frequency of error report");

enum ERRORCode {HW_MAIN_BOARD_ERROR, DSP_ERROR, BLINDNESS_ERROR, ANTENNA_ERROR, MIS_ALIGNMENT_ERROR, CALIB_ERROR, CALIB_OK, HW_MCU_ERROR};
constexpr unsigned int side_left_radar_id = 4;
constexpr unsigned int side_right_radar_id = 2;
constexpr int side_radar_raw_frequency = 50;

inline bool ShouldLog(int sensor_id, ERRORCode ec) {
    static int sl_board_err_num{0};
    static int sr_board_err_num{0};
    static int sl_dsp_err_num{0};
    static int sr_dsp_err_num{0};
    static int sl_blindness_err_num{0};
    static int sr_blindness_err_num{0};
    static int sl_antenna_err_num{0};
    static int sr_antenna_err_num{0};
    static int sl_misalign_err_num{0};
    static int sr_misalign_err_num{0};
    static int sl_calib_status_num{0};
    static int sr_calib_status_num{0};
    static int sl_calib_err_num{0};
    static int sr_calib_err_num{0};
    static int sl_mcu_err_num{0};
    static int sr_mcu_err_num{0};
    bool publish_error{false};
    
    const int continuous_err_num = side_radar_raw_frequency / FLAGS_error_report_frequency;

    switch (ec) {
    case HW_MAIN_BOARD_ERROR:
        sensor_id == side_left_radar_id ? sl_board_err_num++ : sr_board_err_num++;
        if (sl_board_err_num == continuous_err_num || sr_board_err_num == continuous_err_num) {
            publish_error = true;
            sl_board_err_num == continuous_err_num? sl_board_err_num = 0 : sr_board_err_num = 0;
        }
        break;
    case DSP_ERROR:
        sensor_id == side_left_radar_id ? sl_dsp_err_num++ : sr_dsp_err_num++;
        if (sl_dsp_err_num == continuous_err_num || sr_dsp_err_num == continuous_err_num) {
            publish_error = true; 
            sl_dsp_err_num == continuous_err_num ? sl_dsp_err_num = 0 : sr_dsp_err_num = 0; 
        }
        break;
    case BLINDNESS_ERROR:
        sensor_id == side_left_radar_id ? sl_blindness_err_num++ : sr_blindness_err_num++;
        if (sl_blindness_err_num == continuous_err_num || sr_blindness_err_num == continuous_err_num) {
            publish_error = true; 
            sl_blindness_err_num == continuous_err_num ? sl_blindness_err_num = 0 : sr_blindness_err_num = 0;
        }
        break;
    case ANTENNA_ERROR:
        sensor_id == side_left_radar_id ? sl_antenna_err_num++ : sr_antenna_err_num++;
        if (sl_antenna_err_num == continuous_err_num || sr_antenna_err_num == continuous_err_num) {
            publish_error = true; 
            sl_antenna_err_num == continuous_err_num ? sl_antenna_err_num = 0 : sr_antenna_err_num = 0;
        }
        break;
    case MIS_ALIGNMENT_ERROR:
        sensor_id == side_left_radar_id ? sl_misalign_err_num++ : sr_misalign_err_num++;
        if (sl_misalign_err_num == continuous_err_num || sr_misalign_err_num == continuous_err_num) {
            publish_error = true; 
            sl_misalign_err_num == continuous_err_num ? sl_misalign_err_num = 0 : sr_misalign_err_num = 0;
        }
        break;
    case CALIB_ERROR:
        sensor_id == side_left_radar_id ? sl_calib_err_num++ : sr_calib_err_num++;
        if (sl_calib_err_num == side_radar_raw_frequency || sr_calib_err_num == side_radar_raw_frequency) {
            publish_error = true;
            sl_calib_err_num == side_radar_raw_frequency ? sl_calib_err_num = 0 : sr_calib_err_num = 0;
        }
        break;
    case CALIB_OK:
        sensor_id == side_left_radar_id ? sl_calib_status_num++ : sr_calib_status_num++;
        if (sl_calib_status_num == continuous_err_num || sr_calib_status_num == continuous_err_num) {
            publish_error = true; 
            sl_calib_status_num == continuous_err_num ? sl_calib_status_num = 0 : sr_calib_status_num = 0;
        }
        break;
    case HW_MCU_ERROR:
        sensor_id == side_left_radar_id ? sl_mcu_err_num++ : sr_mcu_err_num++;
        if (sl_mcu_err_num == continuous_err_num || sr_mcu_err_num == continuous_err_num) {
            publish_error = true;
            sl_mcu_err_num == continuous_err_num ? sl_mcu_err_num = 0 : sr_mcu_err_num = 0;
        }
        break;
    default:
        break;
    }
    return publish_error;
}

bool ParseReport_RadarState(int sensor_id, const SetupMap& setup_map, const CANFDArray& data) {
    const auto setup_it = setup_map.find(sensor_id);
    const auto sensor_frame_id = setup_it == setup_map.end() ? "" : setup_it->second.sensor_frame_id;

    bool hw_main_board_error = (data[6] >> 6) & 0x03;
    if (hw_main_board_error && ShouldLog(sensor_id, HW_MAIN_BOARD_ERROR)) {
        LOG(ERROR) << sensor_frame_id.data() << sensor_id << " hw_main_board_error";
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).HW_MAIN_BOARD_ERROR);
    }

    bool dsp_error = (data[6] >> 4) & 0x03;
    if (dsp_error && ShouldLog(sensor_id, DSP_ERROR)) {
        LOG(ERROR) << sensor_frame_id.data() << sensor_id << " dsp_error";
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).DSP_ERROR);
    }

    bool blindness_error = (data[6] >> 2) & 0x03;
    if (blindness_error && ShouldLog(sensor_id, BLINDNESS_ERROR)) {
        LOG(ERROR) << sensor_frame_id.data() << sensor_id << " blindness_error";
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).BLINDNESS_ERROR);
    }

    bool antenna_error = data[6] & 0x03;
    if (antenna_error && ShouldLog(sensor_id, ANTENNA_ERROR)) {
        LOG(ERROR) << sensor_frame_id.data() << sensor_id << " antenna_error";
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).ANTENNA_ERROR);
    }

    bool mis_alignment_error = (data[7] >> 6) & 0x03;
    if (mis_alignment_error && ShouldLog(sensor_id, MIS_ALIGNMENT_ERROR)) {
        LOG(ERROR) << sensor_frame_id.data() << sensor_id << " mis_alignment_error";
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).MIS_ALIGNMENT_ERROR);
    }

    bool calib_error = (data[7] >> 4) & 0x03;
    if (calib_error && ShouldLog(sensor_id, CALIB_ERROR)) {
        LOG(ERROR) << sensor_frame_id.data() << sensor_id << " calib_error";
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).CALIB_ERROR);
    }
    if (!calib_error && ShouldLog(sensor_id, CALIB_OK)) {
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).CALIB_STATUS_OK);
    }

    bool hw_mcu_error = (data[7] >> 2) & 0x03;
    if (hw_mcu_error && ShouldLog(sensor_id, HW_MCU_ERROR)) {
        LOG(ERROR) << sensor_frame_id.data() << sensor_id << " hw_mcu_error";
        drive::common::monitor::StatusReporter::getInstance().reportSystemState(
                GetSystemState(sensor_frame_id).HW_MCU_ERROR);
    }
    
    return true;
}

bool ParseRadarTrackInfo(int sensor_id, const CANFDArray& data, RadarTrackObject& radarTrackObject) {
    int obj_id      = (data[20] & 0x3F) + CR5TP_OBJECT_ID_MIN;
    float long_acc  = (((data[23] & 0x01) << 8) + data[22]) * CR5TP_OBJECT_ACCEL_RES + CR5TP_OBJECT_ACCEL_MIN;
    float long_vel  = ((data[25] << 5) + (data[24] >> 3)) * CR5TP_OBJECT_VEL_RES + CR5TP_OBJECT_VEL_MIN;
    float lat_vel   = (((data[27] & 0x1F) << 8) + data[26]) * CR5TP_OBJECT_VEL_RES + CR5TP_OBJECT_VEL_MIN;
    float lat_pos   = (((data[29] & 0x03) << 11) + (data[28] << 3) + (data[27] >> 5)) * CR5TP_OBJECT_POSITION_RES + CR5TP_OBJECT_POSITION_MIN;
    float long_pos  = (((data[30] & 0x7F) << 6) + (data[29] >> 2)) *  CR5TP_OBJECT_POSITION_RES + CR5TP_OBJECT_POSITION_MIN;
    float lat_acc   = (((data[34] & 0x0F) << 5) + (data[33] >> 3)) * CR5TP_OBJECT_ACCEL_RES + CR5TP_OBJECT_ACCEL_MIN;

    radarTrackObject.track_id = obj_id;
    radarTrackObject.linear_velocity.x = long_vel;
    radarTrackObject.linear_velocity.y = lat_vel;
    radarTrackObject.linear_velocity.z = 0;
    radarTrackObject.linear_acceleration.x = long_acc;
    radarTrackObject.linear_acceleration.y = lat_acc;
    radarTrackObject.linear_acceleration.z = 0;

    // Fliter some object info according to the road test result
    if (sensor_id == side_left_radar_id) {
        if (lat_pos <= 0) return false;
    } else {
        if (lat_pos >= 0) return false;
    }
    geometry_msgs::Point32 point;
    point.x = long_pos;
    point.y = lat_pos;
    point.z = 0;
    radarTrackObject.track_shape.points.assign(4, point);

    return true;
}

bool ParseRadarParsedInfo(int sensor_id, const CANFDArray& data, RadarParsedObject& radarParsedObject) {
    int obj_id      = (data[20] & 0x3F) + CR5TP_OBJECT_ID_MIN;
    float long_acc  = (((data[23] & 0x01) << 8) + data[22]) * CR5TP_OBJECT_ACCEL_RES + CR5TP_OBJECT_ACCEL_MIN;
    float long_vel  = ((data[25] << 5) + (data[24] >> 3)) * CR5TP_OBJECT_VEL_RES + CR5TP_OBJECT_VEL_MIN;
    float lat_vel   = (((data[27] & 0x1F) << 8) + data[26]) * CR5TP_OBJECT_VEL_RES + CR5TP_OBJECT_VEL_MIN;
    float lat_pos   = (((data[29] & 0x03) << 11) + (data[28] << 3) + (data[27] >> 5)) * CR5TP_OBJECT_POSITION_RES + CR5TP_OBJECT_POSITION_MIN;
    float long_pos  = (((data[30] & 0x7F) << 6) + (data[29] >> 2)) *  CR5TP_OBJECT_POSITION_RES + CR5TP_OBJECT_POSITION_MIN;
    float lat_acc   = (((data[34] & 0x0F) << 5) + (data[33] >> 3)) * CR5TP_OBJECT_ACCEL_RES + CR5TP_OBJECT_ACCEL_MIN;
    int crc                     = (data[1] << 8) + data[0];
    int counter                 = data[2];
    float timestamp             = ((data[4] << 8) + data[3]) * CR5TP_TIME_STAMP_RES;
    float long_vel_var          = data[5] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float long_pos_var          = data[6] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float long_ext_front_var    = data[7] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float long_ext_back_var     = data[8] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float long_acc_var          = data[9] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float lat_vel_var           = data[10] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float lat_pos_var           = data[11] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float lat_ext_right_var     = data[12] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float lat_ext_left_var      = data[13] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float lat_acc_var           = data[14] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float heading_var           = data[15] * CR5TP_OBJECT_VAR_RES + CR5TP_OBJECT_VAR_MIN;
    float life_time             = data[16] * CR5TP_OBJECT_LIFE_TIME_RES;
    float height                = data[17] * CR5TP_OBJECT_HEIGHT_RES + CR5TP_OBJECT_HEIGHT_MIN;
    float heading               = (((data[19] & 0x07) << 8) + data[18]) * CR5TP_OBJECT_HEADING_RES + CR5TP_OBJECT_HEADING_MIN;
    float coast_index           = data[19] >> 3;
    int obj_dyn_class           = data[20] >> 6;
    float obj_class_conf        = (data[21] & 0x0F) * CR5TP_OBJECT_CLASS_CONF_RES;
    int obj_class               = data[21] >> 4;
    float lat_ext_right         = (data[23] >> 1) * CR5TP_OBJECT_RIGHT_EXT_RES + CR5TP_OBJECT_RIGHT_EXT_MIN;;
    int track_status            = data[24] & 0x07;
    float long_ext_front        = (((data[32] & 0x01) << 9) + (data[31] << 1) + (data[30] >> 7)) * CR5TP_OBJECT_FRONT_EXT_RES + CR5TP_OBJECT_FRONT_EXT_MIN;
    float long_ext_back         = (((data[33] & 0x07) << 7) + (data[32] >> 1)) * CR5TP_OBJECT_BACK_EXT_RES + CR5TP_OBJECT_BACK_EXT_MIN;
    float lat_ext_left          = (((data[35] & 0x07) << 4) + (data[34] >> 4)) * CR5TP_OBJECT_LEFT_EXT_RES + CR5TP_OBJECT_LEFT_EXT_MIN;
    float exist_conf            = (((data[36] & 0x03) << 5) + (data[35] >> 3)) * CR5TP_OBJECT_EXIST_CONF_RES + CR5TP_OBJECT_EXIST_CONF_MIN;

    // Fliter some object info according to the road test result
    if (sensor_id == side_left_radar_id) {
        if (lat_pos <= 0) return false;
    } else {
        if (lat_pos >= 0) return false;
    }

    /* fill in the radar data structure*/  
    radarParsedObject.crc = crc;
    radarParsedObject.counter = counter;
    radarParsedObject.timestamp = timestamp;
    radarParsedObject.long_vel_var = long_vel_var;
    radarParsedObject.long_pos_var = long_pos_var;
    radarParsedObject.long_ext_front_var = long_ext_front_var;
    radarParsedObject.long_ext_back_var = long_ext_back_var;
    radarParsedObject.long_acc_var = long_acc_var;
    radarParsedObject.lat_vel_var = lat_vel_var;
    radarParsedObject.lat_pos_var = lat_pos_var;
    radarParsedObject.lat_ext_right_var = lat_ext_right_var;
    radarParsedObject.lat_ext_left_var = lat_ext_left_var;
    radarParsedObject.lat_acc_var = lat_acc_var;
    radarParsedObject.heading_var = heading_var;
    radarParsedObject.life_time = life_time;
    radarParsedObject.height = height;
    radarParsedObject.heading = heading;
    radarParsedObject.coast_index = coast_index;
    radarParsedObject.obj_id = obj_id;
    radarParsedObject.obj_dyn_class = obj_dyn_class;
    radarParsedObject.obj_class_conf = obj_class_conf;
    radarParsedObject.obj_class = obj_class;
    radarParsedObject.long_acc = long_acc;
    radarParsedObject.lat_ext_right = lat_ext_right;
    radarParsedObject.track_status = track_status;
    radarParsedObject.long_vel = long_vel;
    radarParsedObject.lat_vel = lat_vel;
    radarParsedObject.lat_pos = lat_pos;
    radarParsedObject.long_pos = long_pos;
    radarParsedObject.long_ext_front = long_ext_front;
    radarParsedObject.long_ext_back = long_ext_back;
    radarParsedObject.lat_acc = lat_acc;
    radarParsedObject.lat_ext_left = lat_ext_left;
    radarParsedObject.exist_conf = exist_conf;

    return true;
}

bool ParseReport_ObjectInfo(int sensor_id, const SetupMap& setup_map, const CANFDArray& data, 
                            RadarTrackObject& radarTrackObject, RadarParsedObject& radarParsedObject){
    const auto setup_it = setup_map.find(sensor_id);
    const auto report_msgs_manager_ptr = setup_it->second.report_msgs_manager;

    if(FLAGS_enable_pub_parsed_info) {
        bool parse_success = ParseRadarParsedInfo(sensor_id, data, radarParsedObject);
        RadarParsedArray& radar_parsed_array = report_msgs_manager_ptr->GetRadarParsedArray();
        if (parse_success) {
            radar_parsed_array.tracks.emplace_back(radarParsedObject);
        }
    }

    bool parse_success = ParseRadarTrackInfo(sensor_id, data, radarTrackObject);
    RadarTrackArray& radar_track_array = report_msgs_manager_ptr->GetRadarTrackArray();
    if (parse_success) {
        radar_track_array.tracks.emplace_back(radarTrackObject);
    }
    
    return true;
}

bool ParseReport_Diagnostic(int sensor_id, const SetupMap& setup_map, const CANFDArray&) {
    return true;
}

} // namespace radar
} // namespace driver
