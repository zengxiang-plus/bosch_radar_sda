#pragma once

#include <ros/duration.h>
#include <ipc/subscriber.h>

#include "can_common/can_interface.h"
#include "sensor_msgs/Imu.h"
#include "control/dbw_reports.pb.h"

namespace driver {
namespace radar {

typedef struct {
    float long_accel_local;
    float refer_point_x_local;
    float lat_accel;
    float long_speed_local;
    float yaw_rate;
    float lat_speed_local;
    uint8_t long_motion_dir_local;
} EgoMotionData;

typedef struct {
    float front_axle_right_wheel_speed;
    float front_axle_left_wheel_speed;
    float front_wheel_angle_local;
    float rear_axle_right_wheel_speed;
    float rear_axle_left_wheel_speed;
    float steering_wheel_angle;
} EgoMotionRawData;

typedef struct {
    bool vehicle_mode_ub;
    bool wiper_mode_status_ub;
    uint8_t vehicle_mode;
    uint8_t wiper_mode_status;
    uint8_t lift_axle1_pos_status;
    uint8_t lift_axle2_pos_status;
    uint8_t side_sensor_power_mode;
    uint8_t trailer_connected;
    uint8_t anti_lock_braking_active;
} Dacu01PData;

typedef struct {
    uint8_t trailer1_lift_axle1_pos;
    uint8_t trailer1_lift_axle2_pos;
    uint8_t trailer2_lift_axle1_pos;
    uint8_t trailer2_lift_axle2_pos;
    uint8_t trailer3_lift_axle1_pos;
    uint8_t trailer3_lift_axle2_pos;
    uint8_t trailer4_lift_axle1_pos;
    uint8_t trailer4_lift_axle2_pos;
    uint8_t trailer5_lift_axle1_pos;
    uint8_t trailer5_lift_axle2_pos;
    uint8_t trailers_connected;
    bool trailes_without_abs_ub;
} Dacu02PData; 

class VehicleInfoHandler {
  public:
    VehicleInfoHandler() = default;
    VehicleInfoHandler(const VehicleInfoHandler&) = delete;
    VehicleInfoHandler& operator=(const VehicleInfoHandler&) = delete;
    ~VehicleInfoHandler() = default;
    
    bool Init(ros::NodeHandle &node, float factor);

    /* interfaces for unit test */
    void SetEgoMotionData(const EgoMotionData& test_ego_motion_data) { 
      _ego_motion_data = test_ego_motion_data;
    };
    void SetEgoMotionRawData(const EgoMotionRawData& test_ego_motion_raw_data) {
      _ego_motion_raw_data = test_ego_motion_raw_data;
    };

  private:
    void HandleImuMsg(const sensor_msgs::Imu::Ptr& imu_msg);
    void HandleDbwReports(const drive::common::control::DbwReports& dbw_reports);
    float convertSteeringToFrontWheelAngle(float steering_wheel_angle);
    drive::common::ipc::Subscriber _imu_subscriber;
    drive::common::ipc::Subscriber _dbw_reports_subscriber; 
    friend class RadarConf;
  
  private:
    EgoMotionData _ego_motion_data{};
    EgoMotionRawData _ego_motion_raw_data{};
    Dacu01PData _dacu_01p_data{};
    Dacu02PData _dacu_02p_data{};
    float speed_factor;
};

}  // namespace radar
}  // namespace driver