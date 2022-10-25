#pragma once

namespace driver {
namespace radar {

const double CR5TP_OBJECT_HEADING_RES = 0.005818;
const double CR5TP_OBJECT_HEADING_MIN = -4.71;
const double CR5TP_OBJECT_HEIGHT_RES = 0.02;
const double CR5TP_OBJECT_HEIGHT_MIN = 0;
const double CR5TP_OBJECT_ACCEL_RES = 0.08;
const double CR5TP_OBJECT_ACCEL_MIN = -20;
const double CR5TP_OBJECT_VAR_RES = 0.1;
const double CR5TP_OBJECT_VAR_MIN = 0;

const double CR5TP_OBJECT_LEFT_EXT_RES = 0.05;
const double CR5TP_OBJECT_LEFT_EXT_MIN = 0;
const double CR5TP_OBJECT_RIGHT_EXT_RES = 0.05;
const double CR5TP_OBJECT_RIGHT_EXT_MIN = -5;
const double CR5TP_OBJECT_POSITION_RES = 0.05;
const double CR5TP_OBJECT_POSITION_MIN = -180;
const double CR5TP_OBJECT_VEL_RES = 0.032;
const double CR5TP_OBJECT_VEL_MIN = -80;
const double CR5TP_OBJECT_BACK_EXT_RES = 0.05;
const double CR5TP_OBJECT_BACK_EXT_MIN = -50;
const double CR5TP_OBJECT_FRONT_EXT_RES = 0.05;
const double CR5TP_OBJECT_FRONT_EXT_MIN = 0;
const double CR5TP_OBJECT_CLASS_CONF_RES = 0.1;
const double CR5TP_OBJECT_STATE_LATENCY_RES = 5;
const int CR5TP_OBJECT_ID_MIN = 0;

const double CR5TP_WHEEL_SPEED_MIN = 0;
const double CR5TP_WHEEL_SPEED_RES = 0.00390625;
const double CR5TP_WHEEL_ANGLE_LOCAL_MIN = -1.6;
const double CR5TP_WHEEL_ANGLE_LOCAL_RES = 0.00005;
const double CR5TP_REL_LEVEL_LOCAL_MIN = -3200;
const double CR5TP_REL_LEVEL_LOCAL_RES = 0.1;
const double CR5TP_ACCEL_MIN = -15.687;
const double CR5TP_ACCEL_RES = 0.000488281;
const double CR5TP_SPEED_GROUND_MIN = 0;
const double CR5TP_SPEED_GROUND_RES = 0.00108507;
const double CR5TP_YAW_RATE_MIN = -3.92;
const double CR5TP_YAW_RATE_RES = 0.000122;
const double CR5TP_ANGLE_MIN = -50;
const double CR5TP_ANGLE_RES = 0.01;
const double CR5TP_STEERING_WHEEL_ANGLE_LOCAL_MIN = -31.374;
const double CR5TP_STEERING_WHEEL_ANGLE_LOCAL_RES = 0.0009765625;

const double CR5TP_TIME_STAMP_RES = 0.1;
const double CR5TP_OBJECT_LIFE_TIME_RES = 0.1;
const double CR5TP_OBJECT_EXIST_CONF_RES = 0.01;
const double CR5TP_OBJECT_EXIST_CONF_MIN = 0;

const double CR5TP_LAT_SPEED_GROUND_MIN = -35;
const double CR5TP_LAT_SPEED_GROUND_RES = 0.00108507;

const double CR5TP_REFER_POINT_X_MIN = -32;
const double CR5TP_REFER_POINT_X_RES = 0.001;
}  // namespace radar
}  // namespace driver