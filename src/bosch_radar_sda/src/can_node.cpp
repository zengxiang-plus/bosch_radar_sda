#include "can_node.h"

#include <glog/logging.h>
#include "periodics/scheduler.h"
#include "system_state.h"
#include "cr5tp_can_msg_parser.h"

#include <radar_msgs/RadarTrackArray.h>
#include <radar_msgs/RadarTrack.h>
#include "bosch_radar_sda_msgs/CR5TPRadarParsedArray.h"
#include "bosch_radar_sda_msgs/CR5TPRadarParsed.h"


#define CR5TP_OBJETCT_NUMS  12

namespace driver {
namespace radar {

CanNode::CanNode()
    : can0_(0), _radar_sda_flow(SideRadar_Left, can0_), _status_reporter(drive::common::monitor::StatusReporter::getInstance()) {
}

void CanNode::Run() {
    can0_.StartCanFdComm();
    LOG(INFO) << "BoschRadar SDA started...";
    // _radar_sda_flow.StartSdaFlow();
    {
        _radar_sda_flow.SendChangeSession(SideRadar_Left, can0_);
        _radar_sda_flow.SetSdaFlowStatus(changeExtendMode);
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    ros::spin();
    can0_.Stop();
    LOG(INFO) << "BoschRadar SDA stopped...";
}

bool CanNode::Init() {
    if (InitSetupMap() && InitSdaConf()) {
        RegisterSdaCanFDMsgHandler();
        return true;
    } else {
        return false;
    }
}

bool CanNode::InitSetupMap() {
    _status_reporter.init(node_handle_);

    int max_radar_num = 2; //side left radar and side right radar
    node_handle_.getParam("max_radar_num", max_radar_num);
	for (int i = 0; i < max_radar_num; i++){
		std::string frame_param = "sensor_frame_id" + std::to_string(i);
        std::string sensor_id_param = "sensor_id" + std::to_string(i);

        std::string frame;
        int sensor_id;

        node_handle_.getParam(frame_param, frame);
        node_handle_.getParam(sensor_id_param, sensor_id);

        if (frame.length() > 0) {
            auto& setup = _setup_map[sensor_id];
            setup.sensor_frame_id = frame;
        }
    }

    for (auto& setup : _setup_map) {
        setup.second.report_msgs_manager = std::make_shared<ReportMsgsManager>(
                                               setup.second.sensor_frame_id);
        const auto sensor_id = setup.first;
        auto init_result = setup.second.report_msgs_manager->Init();
        if (0 != init_result) {
            LOG(ERROR) << "Cannot init ReportMsgsManager for"
                       << setup.second.sensor_frame_id.data() << sensor_id;
            _status_reporter.reportSystemState(
                GetSystemState(setup.second.sensor_frame_id).INIT_ERROR);
            _status_reporter.pub();
            return false;
        }
    }

    return true;
}

bool CanNode::InitSdaConf() {
    
    auto id = _scheduler.add(
            "send_heart_beat",
            [&](){ _status_reporter.pub();
                   TimeOutStatusReport(); },
            std::chrono::milliseconds(int(1.0 / 10.0 * 1000)));
    if (id.is_nil()) {
            LOG(ERROR) << "Failed to create heart beat timer" << AllSensorIDs().data();
            ReportSystemStateForAll(&RadarSystemState::INIT_ERROR);
            return false;
    }

    _scheduler.start();
    
    return true;
}

void CanNode::RegisterSdaCanFDMsgHandler() {
    can0_.RegisterCanFdReader([this](const int result, const drive::common::can::MessageID & msg_id,
                            const drive::common::can::CANFDArray& data) {
        HandleCanFDSdaMsg(result, msg_id, data);
    });
}



void CanNode::HandleCanFDSdaMsg(const int result,
                           const drive::common::can::MessageID& msg_id,
                           const drive::common::can::CANFDArray& data) {
    if (result <= 0) {
        return;
    }
    if ((msg_id  == _radar_sda_flow.GetRadarSdaPar_leftresid()) && (_radar_sda_flow.GetSensorID() == SideRadar_Left)){
        std::cout<< "INFO: RADAR SDA FLOW IS "<<_radar_sda_flow.GetSdaFlowStatus()<<std::endl;
        for(int i=0; i<8; i++){
            std::cout<<std::hex<<(data[i] & 0xff)<<"    ";
        }
        std::cout<<std::endl;
        switch(_radar_sda_flow.GetSdaFlowStatus()){
            case changeExtendMode:
                if ((!_radar_sda_flow.CheckChangeSessionResponse(data))){
                    std::cout << "ERROE: SDA FAIL: change session  "<<std::endl;
                }else{
                    _radar_sda_flow.SendSecurityAccess(SideRadar_Left, can0_);
                    _radar_sda_flow.SetSdaFlowStatus(securityAccess1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                break;
            case securityAccess1:
                if ((!_radar_sda_flow.CheckSecurityAccessResponse(data))){
                    std::cout << "ERROE: SDA FAIL: security access  "<<std::endl;
                }else{
                }
            case testerPresent:
            case startSda:
            case sdaStatus:
            case stopSda:
            default:
                break;
        }
        
        
    }
    // if ((msg_id  == _radar_sda_flow.GetRadarSdaPar_leftresid()) && (_radar_sda_flow.GetSensorID() == SideRadar_Left)){
    //     _radar_sda_flow.SetSdaRspBuffer(data);
    // }
    // if ((msg_id  == _radar_sda_flow.GetRadarSdaPar_rightesid()) && _radar_sda_flow.GetSensorID() == SideRadar_Right){
    //     _radar_sda_flow.SetSdaRspBuffer(data);
    // }
}

bool CanNode::IsFrameAvailable(const drive::common::can::CANFDArray& data) {
    for (auto i = 0; i < object_frame_size; i++) {
        if ( (data[i] ^ 0xFF) != 0 ) { return true; }
    }
    return false;
}

void CanNode::PublishRadarTopic(unsigned int sensor_id) {
    const auto setup_it = _setup_map.find(sensor_id); 
    setup_it->second.report_msgs_manager->addOneObject();
    if (setup_it->second.report_msgs_manager->isFrameComplete()) {
        setup_it->second.report_msgs_manager->publishData();
    }
}

inline void CanNode::ReportSystemStateForAll(
    const SystemState RadarSystemState::* state) const {
    for (const auto& setup : _setup_map) {
        _status_reporter.reportSystemState(
            GetSystemState(setup.second.sensor_frame_id).*state);
    }
}

inline std::string CanNode::AllSensorIDs() const {
    std::stringstream ss;
    for (const auto& setup : _setup_map) {
        ss << setup.second.sensor_frame_id << std::to_string(setup.first) << " ";
    }
    return ss.str();
}

void CanNode::TimeOutStatusReport() const {
    if (can0_.Timeout()) {
        LOG(ERROR) << AllSensorIDs().data() << "can timeout";
        ReportSystemStateForAll(&RadarSystemState::CONNECTION_TIMEOUT);
    }
}

}  // namespace radar
}  // namespace driver
