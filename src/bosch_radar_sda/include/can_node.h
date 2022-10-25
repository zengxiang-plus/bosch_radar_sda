#pragma once

#include <atomic>
#include <thread>
#include <gflags/gflags.h>
#include <ros/node_handle.h>

#include <monitor/status_reporter.h>
#include "can_common/can_interface.h"
#include "periodics/scheduler.h"

#include "report_msgs_manager.h"
#include "system_state.h"

#include "radar_sda_control_flow.h"

namespace driver {
namespace radar {

struct RadarSetup {
    std::string sensor_frame_id;
    std::shared_ptr<ReportMsgsManager> report_msgs_manager = nullptr;
    int parse_failure = 0;
};

using SensorID = unsigned int;
using SetupMap = std::unordered_map<SensorID, RadarSetup>;

class CanNode {
  public:
    CanNode();
    CanNode(const CanNode&) = delete;
    CanNode& operator=(const CanNode&) = delete;
    ~CanNode() = default;

    bool Init();
    /* CANFd common interface for receiving and dealing with CAN message */
    void Run();

  private:
    /* config radar-related info */
    bool InitSetupMap();
    /* config dbc-related send info and send them to radar */
    bool InitSdaConf();
    /* register radar message processing interface */
    void RegisterSdaCanFDMsgHandler();

    /* interface from can_common for parsing CAN data and publishing topic */
    void HandleCanFDSdaMsg(int result,
                      const drive::common::can::MessageID& msg_id,
                      const drive::common::can::CANFDArray& data);
    
    void PublishRadarTopic(unsigned int sensor_id);

    bool IsFrameAvailable(const drive::common::can::CANFDArray& data);
    /* status_report related help function */
    void ReportSystemStateForAll(const SystemState RadarSystemState::* state) const;
    std::string AllSensorIDs() const;
    void TimeOutStatusReport() const;

  private:
    const int object_frame_size{10};
     
    ros::NodeHandle node_handle_{"~"};

    SetupMap _setup_map;
    RadarSdaControlFlow _radar_sda_flow;
    drive::common::CanInterface can0_{0};
    
    drive::common::periodics::Scheduler _scheduler{1};
    drive::common::monitor::StatusReporter& _status_reporter;
};

}  // namespace radar
}  // namespace driver