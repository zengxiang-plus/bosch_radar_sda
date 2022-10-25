#include <gtest/gtest.h>
#include "cr5tp_can_msg_parser.h"

using namespace driver::radar;
TEST(RadarCodecTest, test_parsed_decoder) {
  int sensor_id = 4;
  const int canfd_msg_len = 64;
  RadarParsedObject test_parsed_object;
  drive::common::can::CANFDArray data;
  data.resize(canfd_msg_len, 0xFF);

  ParseRadarParsedInfo(sensor_id, data, test_parsed_object);

  EXPECT_EQ(test_parsed_object.crc, 65535);
  EXPECT_EQ(test_parsed_object.counter, 255);
  EXPECT_EQ(test_parsed_object.timestamp, 6553.5);
  EXPECT_EQ(test_parsed_object.long_vel_var, 25.5);
  EXPECT_EQ(test_parsed_object.long_pos_var, 25.5);
  EXPECT_EQ(test_parsed_object.long_ext_front_var, 25.5);
  EXPECT_EQ(test_parsed_object.long_ext_back_var, 25.5);
  EXPECT_EQ(test_parsed_object.long_acc_var, 25.5);
  EXPECT_EQ(test_parsed_object.lat_vel_var, 25.5);
  EXPECT_EQ(test_parsed_object.lat_pos_var, 25.5);
  EXPECT_EQ(test_parsed_object.lat_ext_right_var, 25.5);
  EXPECT_EQ(test_parsed_object.lat_ext_left_var, 25.5);
  EXPECT_EQ(test_parsed_object.lat_acc_var, 25.5);
  EXPECT_EQ(test_parsed_object.heading_var, 25.5);
  EXPECT_EQ(test_parsed_object.life_time, 25.5); 
  ASSERT_NEAR(test_parsed_object.height, 5.1, 0.001);
  ASSERT_NEAR(test_parsed_object.heading, 7.19945, 0.001);
  EXPECT_EQ(test_parsed_object.coast_index, 31);
  EXPECT_EQ(test_parsed_object.obj_id, 63);
  EXPECT_EQ(test_parsed_object.obj_dyn_class, 3);
  EXPECT_EQ(test_parsed_object.obj_class_conf, 1.5);
  EXPECT_EQ(test_parsed_object.obj_class, 15);
  ASSERT_NEAR(test_parsed_object.long_acc, 20.88, 0.001);
  ASSERT_NEAR(test_parsed_object.lat_ext_right, 1.35, 0.001);
  EXPECT_EQ(test_parsed_object.track_status, 7);
  ASSERT_NEAR(test_parsed_object.long_vel, 182.112, 0.001);
  ASSERT_NEAR(test_parsed_object.lat_vel, 182.112, 0.001);
  ASSERT_NEAR(test_parsed_object.lat_pos, 229.55, 0.001);
  ASSERT_NEAR(test_parsed_object.long_pos, 229.55, 0.001);
  ASSERT_NEAR(test_parsed_object.long_ext_front, 51.15, 0.001);
  ASSERT_NEAR(test_parsed_object.long_ext_back, 1.15, 0.001);
  ASSERT_NEAR(test_parsed_object.lat_acc, 20.88, 0.001);
  ASSERT_NEAR(test_parsed_object.lat_ext_left, 6.35, 0.001);
  ASSERT_NEAR(test_parsed_object.exist_conf, 1.27, 0.001);
}

TEST(RadarCodecTest, test_track_decoder) {
  int sensor_id = 4;
  const int canfd_msg_len = 64;
  RadarTrackObject test_track_object;
  drive::common::can::CANFDArray data;
  data.resize(canfd_msg_len, 0xFF);
  
  ParseRadarTrackInfo(sensor_id, data, test_track_object);

  EXPECT_EQ(test_track_object.track_id, 63);
  ASSERT_NEAR(test_track_object.linear_velocity.x, 182.112, 0.001);
  ASSERT_NEAR(test_track_object.linear_velocity.y, 182.112, 0.001);
  EXPECT_EQ(test_track_object.linear_velocity.z, 0);
  ASSERT_NEAR(test_track_object.linear_acceleration.x, 20.88, 0.001);
  ASSERT_NEAR(test_track_object.linear_acceleration.y, 20.88, 0.001);
  EXPECT_EQ(test_track_object.linear_acceleration.z, 0);
  ASSERT_NEAR(test_track_object.track_shape.points[0].x, 229.55, 0.001);
}