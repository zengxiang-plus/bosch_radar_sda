#pragma once

#include <gtest/gtest_prod.h>
#include <ros/duration.h>

#include "can_common/can_interface.h"
#include "sensor_msgs/Imu.h"
#include "control/dbw_reports.pb.h"
#include "vehicle_info_handler.h"


namespace driver {
namespace radar {

enum SensorType {SideRadar_Left, SideRadar_Right};
enum errType {noErr, flowErr, responseErr};
enum timeoutType{noTimeout, changeExtendModeTimeout, securityAccessTimeout, startSdaTimeout, sdaStatusTimeout, stopSdaTimeout, testerPresentTimeout};
enum flowStatusType{fail, init, changeExtendMode, securityAccess, testerPresent, startSda, sdaStatus, stopSda, finish};



using CANFDArray = drive::common::can::CANFDArray;
using CANFDElement = std::pair<const drive::common::can::MessageID, CANFDArray>;
using CANFDDataArray = boost::container::static_vector<CANFDArray, 3>;

class RadarSdaControlFlow{
    public:
        RadarSdaControlFlow() = default;
        RadarSdaControlFlow(SensorType id, drive::common::CanInterface& canInterface);
        RadarSdaControlFlow(const RadarSdaControlFlow&) = delete;
        RadarSdaControlFlow& operator=(const RadarSdaControlFlow&) = delete;
        ~RadarSdaControlFlow() = default;

        SensorType GetSensorID(void){ return _sensor_ID;}

        void SetRadarSdaPar_funcid(drive::common::can::MessageID ID);
        drive::common::can::MessageID GetRadarSdaPar_funcid(void);

        void SetRadarSdaPar_leftphyid(drive::common::can::MessageID ID);
        drive::common::can::MessageID GetRadarSdaPar_leftphyid(void);

        void SetRadarSdaPar_rightphyid(drive::common::can::MessageID ID);
        drive::common::can::MessageID GetRadarSdaPar_rightphyid(void);

        void SetRadarSdaPar_leftresid(drive::common::can::MessageID ID);
        drive::common::can::MessageID GetRadarSdaPar_leftresid(void);

        void SetRadarSdaPar_rightesid(drive::common::can::MessageID ID);
        drive::common::can::MessageID GetRadarSdaPar_rightesid(void);

        void SetSdaFlowStatus(flowStatusType status);
        flowStatusType GetSdaFlowStatus(void);

        bool StartSdaFlow(void);

        void SdaFlow(void);

        void SetSdaRspBuffer(CANFDArray data);

        void SendChangeSession(SensorType sensor_id, drive::common::CanInterface& can);

        bool CheckChangeSessionResponse(errType ec);

        void SendSecurityAccess(SensorType sensor_id, drive::common::CanInterface& can);

        bool CheckSecurityAccessResponse(errType ec);

        void SendSecurityAccess2(SensorType sensor_id, drive::common::CanInterface& can);

        bool CheckSecurityAccessResponse2(errType ec);

        void StartRounteSda(SensorType sensor_id, drive::common::CanInterface& can);

        bool CheckStartRounteSda(errType ec);

        void ReadRounteSda(SensorType sensor_id, drive::common::CanInterface& can);

        bool CheckReadRounteSda(errType ec);

        void StopRounteSda(SensorType sensor_id, drive::common::CanInterface& can);

        bool CheckStopRounteSda(errType ec);

         void SendTesterPresent(SensorType sensor_id, drive::common::CanInterface& can);

        bool CheckTesterPresent(errType ec);

        
    private:

        std::vector<CANFDElement> command_result;

        drive::common::can::MessageID side_radar_left_req;
        drive::common::can::MessageID side_radar_right_req;
        drive::common::can::MessageID side_radar_left_resp;
        drive::common::can::MessageID side_radar_right_resp;
        drive::common::can::MessageID side_radar_func;

        drive::common::can::Byte _ChangeSession_DID;
        drive::common::can::Byte _SecurityAccess_DID;
        drive::common::can::Byte _RouteControl_DID;
        drive::common::can::Byte _Test_DID;
        drive::common::can::Byte _StartRoute_SID;
        drive::common::can::Byte _ReadRoute_SID;
        drive::common::can::Byte _StopRoute_SID;
        drive::common::can::Byte _RadarSDA_H;
        drive::common::can::Byte _RadarSDA_L;
        drive::common::can::Byte _ExtendSession;
        drive::common::can::Byte _ActiveResponse;
        drive::common::can::Byte _SecurityAccessLever1;
        drive::common::can::Byte _SecurityAccessLever2;

        drive::common::can::CANFDArray ChangeSession;
        drive::common::can::CANFDArray SecurityAccess;
        drive::common::can::CANFDArray StartRouteSDA;
        drive::common::can::CANFDArray ReadRouteSDA;
        drive::common::can::CANFDArray StopRouteSDA;
        drive::common::can::CANFDArray TesterPresent;
        

        const  drive::common::can::Byte _ChangeSession_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _SecurityAccess_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _StartRouteSDA_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _ReadRouteSDA_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _StopRouteSDA_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _TesterPresent_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _SDA_Frame_BYTE_NUM;

        std::thread _sda_flow_thread; 
        SensorType _sensor_ID;
        errType  _errCode;
        timeoutType _timeoutCode;
        flowStatusType _flowStatus;
        drive::common::CanInterface& _canInterface;
        CANFDDataArray _canFdBuffer;
};

}
}