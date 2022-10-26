#pragma once

#include <gtest/gtest_prod.h>
#include <ros/duration.h>

#include "can_common/can_interface.h"
#include "sensor_msgs/Imu.h"
#include "control/dbw_reports.pb.h"
#include "vehicle_info_handler.h"

namespace driver {
namespace radar {

const  drive::common::can::Byte SecuritySeedKeyNUM = 5;
const  drive::common::can::Byte SDAStatusFrameNUM = 11;

enum type_Sensor {SideRadar_Left, SideRadar_Right};
enum type_Err {noErr, flowErr, responseErr};
enum type_Timeout{noTimeout, changeExtendModeTimeout, securityAccessTimeout, startSdaTimeout, sdaStatusTimeout, stopSdaTimeout, testerPresentTimeout};
enum type_FlowStatus{fail, init, changeExtendMode, securityAccess1, securityAccess2, testerPresent, startSda, sdaStatus, stopSda, finish};

using CANFDArray = drive::common::can::CANFDArray;
using CANFDElement = std::pair<const drive::common::can::MessageID, CANFDArray>;
using SecurityFramArray = boost::container::static_vector<uint8_t, SecuritySeedKeyNUM>;
using SDAStatusArray = boost::container::static_vector<drive::common::can::Byte, SDAStatusFrameNUM>;
using SDAResponseStatus = boost::container::static_vector<std::string, 10>;

void GenerateKeyEx(const uint8_t *f_SeedArray, uint8_t f_SeedArrarySize, const uint8_t f_SecurityLevel, uint8_t *f_KeyArray);

class RadarSdaControlFlow{
    public:
        RadarSdaControlFlow() = default;
        RadarSdaControlFlow(type_Sensor id, drive::common::CanInterface& canInterface);
        RadarSdaControlFlow(const RadarSdaControlFlow&) = delete;
        RadarSdaControlFlow& operator=(const RadarSdaControlFlow&) = delete;
        ~RadarSdaControlFlow() = default;

        type_Sensor GetSensorID(void){ return _sensor_ID;}

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

        void SetSdaFlowStatus(type_FlowStatus status);
        type_FlowStatus GetSdaFlowStatus(void);
        std::string PrintSdaFlowStatus(void);

        void SendChangeSession(type_Sensor sensor_id, drive::common::CanInterface& can);

        bool CheckChangeSessionResponse(CANFDArray data);

        void SendSecurityAccess(type_Sensor sensor_id, drive::common::CanInterface& can);

        bool CheckSecurityAccessResponse(CANFDArray data);

        void SendSecurityAccess2(type_Sensor sensor_id, drive::common::CanInterface& can,  const SecurityFramArray& gKey);

        bool CheckSecurityAccessResponse2(CANFDArray data);

        void StartRounteSda(type_Sensor sensor_id, drive::common::CanInterface& can);

        bool CheckStartRounteSda(CANFDArray data);

        void ReadRounteSda(type_Sensor sensor_id, drive::common::CanInterface& can);

        void SendContinueFrame(type_Sensor sensor_id, drive::common::CanInterface& can);

        bool GetReadRounteSdaStatus(CANFDArray data);

        bool AnySDAStatus(void);

        void StopRounteSda(type_Sensor sensor_id, drive::common::CanInterface& can);

        bool CheckStopRounteSda(CANFDArray data);

         void SendTesterPresent(type_Sensor sensor_id, drive::common::CanInterface& can);

        
    private:

        type_Sensor _sensor_ID;
        type_Err  _errCode;
        type_Timeout _timeoutCode;
        type_FlowStatus _flowStatus;
        
        bool _sda_Process;
        SDAStatusArray _Sda_Status;

        std::vector<CANFDElement> command_result;

        drive::common::CanInterface& _canInterface;

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
        drive::common::can::CANFDArray ContinueFrame;       

        const  drive::common::can::Byte _ChangeSession_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _SecurityAccess_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _StartRouteSDA_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _ReadRouteSDA_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _StopRouteSDA_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _TesterPresent_COMMAND_BYTE_NUM;
        const  drive::common::can::Byte _SDA_Frame_BYTE_NUM;

        drive::common::can::Byte _SDA_Status_Frame_BYTE_NUM;
        drive::common::can::Byte _Count_BYTE_NUM;

        SDAResponseStatus _RoutineStatus = {"Routine Inactive", "Routine Active", "Routine NVM Write not OK", "Routine Timeout",
                                                                                      "Routine Finished Correctly", "Routine Aborted"};
        SDAResponseStatus _RoutineResult = {"Alignment no Result Avaliable", "Alignement Incorrect Result", "Alignment correct Result"};
        SDAResponseStatus _DrivingProfile = {"Velocity too Slow", "Velocity too Fast", "Yaw Rate too High", "Acceleration too High",
                                                                                        "Location Number Insufficient", "Sensor is Blind"};
        SDAResponseStatus _SdaStatus = {"fail", "init", "changeExtendMode", "securityAccess1", "securityAccess2", "testerPresent", 
                                                                                "startSda", "sdaStatus", "stopSda", "finish"};
                                                                                
        double _Horizontal_Angle = 0;
        double _Vertical_Angle = 0;
};

}
}