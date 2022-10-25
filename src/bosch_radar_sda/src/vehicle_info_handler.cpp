#include "vehicle_info_handler.h"
#include "const_vars.h"
#include <algorithm>

namespace driver {
namespace radar {

bool VehicleInfoHandler::Init(ros::NodeHandle &node, float factor){
    speed_factor = factor;

    std::string imu_topic_name;
    std::string imu_topic_param = "ros_imu_topic_name";
    std::string dbw_topic_name;
    std::string dbw_topic_param = "ros_dbw_topic_name";
    int ros_sub_queue_length;
    std::string ros_sub_queue_length_param = "ros_subscriber_queue_length";
    
    node.getParam(imu_topic_param, imu_topic_name);
    node.getParam(dbw_topic_param, dbw_topic_name);
    node.getParam(ros_sub_queue_length_param, ros_sub_queue_length);
    
    _imu_subscriber = ::drive::common::ipc::subscribe(
                        node, imu_topic_name, ros_sub_queue_length, &VehicleInfoHandler::HandleImuMsg, this);
    _dbw_reports_subscriber = ::drive::common::ipc::subscribe(
                        node, dbw_topic_name, ros_sub_queue_length, &VehicleInfoHandler::HandleDbwReports, this);
    
    if (_imu_subscriber && _dbw_reports_subscriber) {
        return true;
    } else {
        return false;
    }
};

void VehicleInfoHandler::HandleImuMsg(const sensor_msgs::Imu::Ptr& imu_msg) {
    _ego_motion_data.yaw_rate = imu_msg->angular_velocity.z;
    _ego_motion_data.lat_accel = imu_msg->linear_acceleration.x;
};

void VehicleInfoHandler::HandleDbwReports(const drive::common::control::DbwReports& dbw_reports) {
    float ego_motion_raw_data_array[4] = {0.0, 0.0, 0.0, 0.0};
    static const float M_TO_KM = 3.6;
    static const float KM_TO_M = 1 / M_TO_KM; 
    _ego_motion_raw_data.front_axle_left_wheel_speed = dbw_reports.wheel_speed_report().front_left() * M_TO_KM * speed_factor;
    _ego_motion_raw_data.front_axle_right_wheel_speed = dbw_reports.wheel_speed_report().front_right() * M_TO_KM * speed_factor ;
    _ego_motion_raw_data.rear_axle_left_wheel_speed = dbw_reports.wheel_speed_report().rear_left() * M_TO_KM * speed_factor;
    _ego_motion_raw_data.rear_axle_right_wheel_speed = dbw_reports.wheel_speed_report().rear_right() * M_TO_KM * speed_factor;
    ego_motion_raw_data_array[0] = _ego_motion_raw_data.front_axle_left_wheel_speed;
    ego_motion_raw_data_array[1] = _ego_motion_raw_data.front_axle_right_wheel_speed;
    ego_motion_raw_data_array[2] = _ego_motion_raw_data.rear_axle_left_wheel_speed;
    ego_motion_raw_data_array[3] = _ego_motion_raw_data.rear_axle_right_wheel_speed;
    std::sort(ego_motion_raw_data_array, ego_motion_raw_data_array + 4);
    _ego_motion_data.long_speed_local = (ego_motion_raw_data_array[1] + ego_motion_raw_data_array[2]) * 0.5 * KM_TO_M;
    _ego_motion_data.long_accel_local = dbw_reports.vehicle_dynamic().linear_acceleration().y();
    _ego_motion_raw_data.steering_wheel_angle = dbw_reports.steering_report().steering_wheel_angle();
    _ego_motion_raw_data.front_wheel_angle_local = convertSteeringToFrontWheelAngle(_ego_motion_raw_data.steering_wheel_angle);
};


float VehicleInfoHandler::convertSteeringToFrontWheelAngle(float steering_wheel_angle) {
    float front_wheel_angle{0.0};
    float front_wheel_angle_offset{0.0};
    float multi_factor{0.0};
    float steering_wheel_angle_offset{0.0};
    int is_value_greater_than_zero{0};
    
    if ( 0 > steering_wheel_angle ) {
        is_value_greater_than_zero = 0;
        steering_wheel_angle = 0 - steering_wheel_angle;
    } else {
        is_value_greater_than_zero = 1;
    }
                                                                
    const float split_value[12] = {0.42193335, 0.75110244, 1.14336519, 1.39545182, 1.81473227, 3.67147461, 5.43015564, 7.47509683, 9.64032612, 11.81151571, 13.51539340, 100};
    const float lookup_table[12][3] = {{0.00000000, 0.00000000,  24.175}, {0.01745329, 0.42193335,  18.86      }, {0.03490659, 0.75110244,  22.475     }, {0.05235988, 1.14336519,  14.4435},
                                       {0.06981317, 1.39545182,  24.023}, {0.08726646, 1.81473227,  21.2767    }, {0.17453293, 3.67147461,  20.153     }, {0.26179939, 5.43015564,  23.4333},
                                       {0.34906585, 7.47509683,  24.8117},{0.43633231, 9.64032612,  24.88      }, {0.52359878, 11.81151571, 32.54166667}, {0.57595865, 13.51539340, 36.8125}};

    for(size_t i = 0; i < 12; i++) {
        if (split_value[i] > steering_wheel_angle) {
            front_wheel_angle_offset = lookup_table[i][0];
            steering_wheel_angle_offset = lookup_table[i][1];
            multi_factor = lookup_table[i][2];
            break; 
        }
    }

    front_wheel_angle = front_wheel_angle_offset + ( steering_wheel_angle - steering_wheel_angle_offset ) / multi_factor;

    if( 0 == is_value_greater_than_zero ) {
        front_wheel_angle = 0 - front_wheel_angle;
    }

    return front_wheel_angle;
};
}  // namespace radar
}  // namespace driver