#include <gtest/gtest.h>
#include "cr5tp_radar_conf.h"

namespace driver{
namespace radar{
class RadarCodecTest : public ::testing::Test {
  protected:
    void SetUp() override { radar_conf_ = std::make_unique<RadarConf>(); }

  protected:
    std::unique_ptr<RadarConf> radar_conf_;
};

TEST_F(RadarCodecTest, TestEgoMotionEncoder) {
    const int canfd_msg_len = 64;
    EgoMotionData test_ego_motion_data{5, 0, 0, 10, 0, 2, 1};
    radar_conf_->vehicle_info_handler.SetEgoMotionData(test_ego_motion_data);
    drive::common::can::CANFDArray test_result;
    test_result.resize(canfd_msg_len);
    radar_conf_->EncodeDACUEgoMotionV1c4(test_result);

    EXPECT_EQ(test_result[2], 0);
    EXPECT_EQ(test_result[3], 0x7E);
    EXPECT_EQ(test_result[4], 0xA5);
    EXPECT_EQ(test_result[5], 0);
    EXPECT_EQ(test_result[6], 0x7D);
    EXPECT_EQ(test_result[7], 0x7E);
    EXPECT_EQ(test_result[8], 0x7D);
    EXPECT_EQ(test_result[9], 0);
    EXPECT_EQ(test_result[10], 0);
    EXPECT_EQ(test_result[11], 0);
    EXPECT_EQ(test_result[12], 0);
    EXPECT_EQ(test_result[13], 0xFF);
    EXPECT_EQ(test_result[14], 0x23);
    EXPECT_EQ(test_result[15], 0);
    EXPECT_EQ(test_result[16], 0);
    EXPECT_EQ(test_result[17], 0x83);
    EXPECT_EQ(test_result[18], 0x7D);
    EXPECT_EQ(test_result[19], 0x33);
    EXPECT_EQ(test_result[20], 0x85);
    EXPECT_EQ(test_result[21], 0);
    EXPECT_EQ(test_result[22], 0);
    EXPECT_EQ(test_result[23], 0x8);
}

TEST_F(RadarCodecTest, TestEgoMotionRawEncoder) {
    const int canfd_msg_len = 64;
    EgoMotionRawData test_ego_motion_raw_data{10, 10, 10, 10, 10, 2};
    radar_conf_->vehicle_info_handler.SetEgoMotionRawData(test_ego_motion_raw_data);
    drive::common::can::CANFDArray test_result;
    test_result.resize(canfd_msg_len);
    radar_conf_->EncodeDACUEgoMotionRawV1c4(test_result);

    EXPECT_EQ(test_result[2], 0);
    EXPECT_EQ(test_result[3], 0);
    EXPECT_EQ(test_result[4], 0xA);
    EXPECT_EQ(test_result[5], 0x0);
    EXPECT_EQ(test_result[6], 0xA);
    EXPECT_EQ(test_result[7], 0x3F);
    EXPECT_EQ(test_result[8], 0x8A);
    EXPECT_EQ(test_result[9], 0);
    EXPECT_EQ(test_result[10], 0xA);
    EXPECT_EQ(test_result[11], 0);
    EXPECT_EQ(test_result[12], 0xA);
    EXPECT_EQ(test_result[13], 0x7E);
    EXPECT_EQ(test_result[14], 0x85);
}

} // namespace radar
} // namespace driver