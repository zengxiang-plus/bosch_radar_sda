#include "radar_sda_control_flow.h"

namespace driver {
namespace radar {

RadarSdaControlFlow::RadarSdaControlFlow(SensorType id, drive::common::CanInterface& canInterface):
    side_radar_func(0x7F2), side_radar_left_req(0x1F2), side_radar_right_req(0x3F2), side_radar_left_resp(0x569), side_radar_right_resp(0x56B),
    _ChangeSession_DID(0x10), _SecurityAccess_DID(0x27), _RouteControl_DID(0x31), _Test_DID(0x3E), 
    _SecurityAccessLever1(0x01), _SecurityAccessLever2(0x02),
    _StartRoute_SID(0x01), _ReadRoute_SID(0x03), _StopRoute_SID(0x02), 
    _RadarSDA_H(0xF8), _RadarSDA_L(0x08), _ExtendSession(0x03), _SDA_Frame_BYTE_NUM(8), _ActiveResponse(0x40),
    _ChangeSession_COMMAND_BYTE_NUM(2), _SecurityAccess_COMMAND_BYTE_NUM(2), 
    _StartRouteSDA_COMMAND_BYTE_NUM(4), _ReadRouteSDA_COMMAND_BYTE_NUM(4), 
    _StopRouteSDA_COMMAND_BYTE_NUM(4), _TesterPresent_COMMAND_BYTE_NUM(4),
    _errCode(noErr), _timeoutCode(noTimeout), _flowStatus(init),
    _sensor_ID(id),_canInterface(canInterface)
{
    ChangeSession.resize(_SDA_Frame_BYTE_NUM);
    SecurityAccess.resize(_SDA_Frame_BYTE_NUM);
    StartRouteSDA.resize(_SDA_Frame_BYTE_NUM);
    ReadRouteSDA.resize(_SDA_Frame_BYTE_NUM);
    StopRouteSDA.resize(_SDA_Frame_BYTE_NUM);
    TesterPresent.resize(_SDA_Frame_BYTE_NUM);
}

void RadarSdaControlFlow::SetRadarSdaPar_funcid(drive::common::can::MessageID ID){
    side_radar_func = ID;
}

void RadarSdaControlFlow::SetRadarSdaPar_leftphyid(drive::common::can::MessageID ID){
    side_radar_left_req = ID;
}

void RadarSdaControlFlow::SetRadarSdaPar_rightphyid(drive::common::can::MessageID ID){
    side_radar_right_req = ID;
}

void RadarSdaControlFlow::SetRadarSdaPar_leftresid(drive::common::can::MessageID ID){
    side_radar_left_resp = ID;
}

void RadarSdaControlFlow::SetRadarSdaPar_rightesid(drive::common::can::MessageID ID){
    side_radar_right_resp = ID;
}

drive::common::can::MessageID RadarSdaControlFlow::GetRadarSdaPar_funcid(void){
    return side_radar_func;
}

drive::common::can::MessageID RadarSdaControlFlow::GetRadarSdaPar_leftphyid(void){
    return side_radar_left_req;
}

drive::common::can::MessageID RadarSdaControlFlow::GetRadarSdaPar_rightphyid(void){
    return side_radar_right_req;
}

drive::common::can::MessageID RadarSdaControlFlow::GetRadarSdaPar_leftresid(void){
    return side_radar_left_resp;
}

drive::common::can::MessageID RadarSdaControlFlow::GetRadarSdaPar_rightesid(void){
    return side_radar_right_resp;
}

void RadarSdaControlFlow::SetSdaFlowStatus(flowStatusType status){
    _flowStatus = status;
}

flowStatusType RadarSdaControlFlow::GetSdaFlowStatus(void){
    return _flowStatus;
}

bool RadarSdaControlFlow::StartSdaFlow(void){
    _flowStatus = init;
    _sda_flow_thread = std::thread(&RadarSdaControlFlow::SdaFlow, this);
    return true;
}

void RadarSdaControlFlow::SdaFlow(void){
    while(_flowStatus != fail && _errCode == noErr){
        std::cout<< "buffer size"<<_canFdBuffer.size()<<std::endl;
        // Change Session to Extend Session
        SendChangeSession(_sensor_ID, _canInterface);
        SetSdaFlowStatus(changeExtendMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if ((!CheckChangeSessionResponse(_errCode))){
            std::cout << "ERROE: SDA FAIL  "<<"timeout:"<<_timeoutCode<<"  "<<"errcode:"<<_errCode<<std::endl;
            // break;
        }

        // // Security Access1
        // SendSecurityAccess(_sensor_ID, _canInterface);
        // SetSdaFlowStatus(securityAccess);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if ((!CheckSecurityAccessResponse(_errCode))){
        //     std::cout << "ERROE: SDA FAIL  "<<"timeout:"<<_timeoutCode<<"  "<<"errcode:"<<_errCode<<std::endl;
        //     // break;
        // }

        // // Security Access2
        // SendSecurityAccess2(_sensor_ID, _canInterface);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if ((!CheckSecurityAccessResponse2(_errCode))){
        //     std::cout << "ERROE: SDA FAIL  "<<"timeout:"<<_timeoutCode<<"  "<<"errcode:"<<_errCode<<std::endl;
        //     // break;
        // }

        // //Keep Tester Present
        // SendTesterPresent(_sensor_ID, _canInterface);
        // SetSdaFlowStatus(testerPresent);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if ((!CheckTesterPresent(_errCode))){
        //     std::cout << "ERROE: SDA FAIL  "<<"timeout:"<<_timeoutCode<<"  "<<"errcode:"<<_errCode<<std::endl;
        //     // break;
        // }

        // //Start SDA
        // StartRounteSda(_sensor_ID, _canInterface);
        // SetSdaFlowStatus(startSda);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if ((!CheckStartRounteSda(_errCode))){
        //     std::cout << "ERROE: SDA FAIL  "<<"timeout:"<<_timeoutCode<<"  "<<"errcode:"<<_errCode<<std::endl;
        //     // break;
        // }

        //  //Read SDA
        // ReadRounteSda(_sensor_ID, _canInterface);
        // SetSdaFlowStatus(sdaStatus);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if ((!CheckReadRounteSda(_errCode))){
        //     std::cout << "ERROE: SDA FAIL  "<<"timeout:"<<_timeoutCode<<"  "<<"errcode:"<<_errCode<<std::endl;
        //     // break;
        // }

        //  //Stop SDA
        // StopRounteSda(_sensor_ID, _canInterface);
        // SetSdaFlowStatus(stopSda);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // if ((!CheckStopRounteSda(_errCode))){
        //     std::cout << "ERROE: SDA FAIL  "<<"timeout:"<<_timeoutCode<<"  "<<"errcode:"<<_errCode<<std::endl;
        //     // break;
        // }

        // SetSdaFlowStatus(finish);

    }
    std::cout<<"SDA FINISH!"<<std::endl;
}

void RadarSdaControlFlow::SetSdaRspBuffer(CANFDArray data){
    if (_flowStatus == sdaStatus && data.size() < 3){
        _canFdBuffer.push_back(data);
        return;
    }
    if (_canFdBuffer.size() == 0 ){
        _canFdBuffer.push_back(data);
        return;
    }
    _errCode = flowErr;
}

void RadarSdaControlFlow::SendChangeSession(SensorType sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    ChangeSession[0] = _ChangeSession_COMMAND_BYTE_NUM;
    ChangeSession[1] = _ChangeSession_DID;
    ChangeSession[2] = _ExtendSession;
    command_result.emplace_back(side_radar_func, ChangeSession);
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckChangeSessionResponse(errType ec){
    if(ec != noErr) return false;
    if (_canFdBuffer.size() == 0){
        _timeoutCode = changeExtendModeTimeout;
        return false;
    }
    drive::common::can::CANFDArray data = _canFdBuffer[0];
    if ((data[0] != (_ActiveResponse + _ChangeSession_DID)) || (data[1] != _ExtendSession)){
        _errCode = responseErr;
        return false;
    }
    _canFdBuffer.clear();
    std::cout<<"CHANGE SESSION SUCCESS!"<<std::endl;
    return true;
}

bool RadarSdaControlFlow::CheckChangeSessionResponse(CANFDArray data){
    if ((data[1] != (_ActiveResponse + _ChangeSession_DID)) || (data[2] != _ExtendSession)){
        _errCode = responseErr;
        return false;
    }
    std::cout<<"CHANGE SESSION SUCCESS!"<<std::endl;
    return true;
}

void RadarSdaControlFlow::SendSecurityAccess(SensorType sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    SecurityAccess[0] = _SecurityAccess_COMMAND_BYTE_NUM;
    SecurityAccess[1] = _SecurityAccess_DID;
    SecurityAccess[2] = _SecurityAccessLever1;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, SecurityAccess);
    }else{
        command_result.emplace_back(side_radar_right_req, SecurityAccess);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckSecurityAccessResponse(errType ec){
    if(ec != noErr) return false;
    if (_canFdBuffer.size() == 0){
        _timeoutCode = securityAccessTimeout;
        return false;
    }
    drive::common::can::CANFDArray data = _canFdBuffer[0];
    if ((data[0] != (_ActiveResponse + _SecurityAccess_DID)) || (data[1] != _SecurityAccessLever1)){
        _errCode = responseErr;
        return false;
    }
    _canFdBuffer.clear();
    std::cout<<"SECURITY ACCESS SUCCESS!"<<std::endl;
    return true;
}

bool RadarSdaControlFlow::CheckSecurityAccessResponse(CANFDArray data){
    if ((data[1] != (_ActiveResponse + _SecurityAccess_DID)) || (data[2] != _SecurityAccessLever1)){
        _errCode = responseErr;
        for(int i=0; i<8; i++){
            std::cout<<std::hex<<(data[i] & 0xff)<<"    ";
        }
        return false;
    }
    std::cout<<"SECURITY ACCESS SUCCESS!"<<std::endl;
    return true;
}

void RadarSdaControlFlow::SendSecurityAccess2(SensorType sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    SecurityAccess[0] = _SecurityAccess_COMMAND_BYTE_NUM;
    SecurityAccess[1] = _SecurityAccess_DID;
    SecurityAccess[2] = _SecurityAccessLever2;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, SecurityAccess);
    }else{
        command_result.emplace_back(side_radar_right_req, SecurityAccess);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckSecurityAccessResponse2(errType ec){
    if(ec != noErr) return false;
    if (_canFdBuffer.size() == 0){
        _timeoutCode = securityAccessTimeout;
        return false;
    }
    drive::common::can::CANFDArray data = _canFdBuffer[0];
    if ((data[0] != (_ActiveResponse + _SecurityAccess_DID)) || (data[1] != _SecurityAccessLever2)){
        _errCode = responseErr;
        return false;
    }
    _canFdBuffer.clear();
    std::cout<<"SECURITY ACCESS SUCCESS!"<<std::endl;
    return true;
}

void RadarSdaControlFlow::StartRounteSda(SensorType sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    StartRouteSDA[0] = _StartRouteSDA_COMMAND_BYTE_NUM;
    StartRouteSDA[1] = _RouteControl_DID;
    StartRouteSDA[2] = _StartRoute_SID;
    StartRouteSDA[3] = _RadarSDA_H;
    StartRouteSDA[4] = _RadarSDA_L;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, StartRouteSDA);
    }else{
        command_result.emplace_back(side_radar_right_req, StartRouteSDA);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckStartRounteSda(errType ec){
    if(ec != noErr) return false;
    if (_canFdBuffer.size() == 0){
        _timeoutCode = startSdaTimeout;
        return false;
    }
    drive::common::can::CANFDArray data = _canFdBuffer[0];
    if ((data[0] != (_ActiveResponse + _RouteControl_DID)) || (data[1] != _StartRoute_SID)){
        _errCode = responseErr;
        return false;
    }
    _canFdBuffer.clear();
    std::cout<<"START ROUTE SDA SUCCESS!"<<std::endl;
    return true;
}

void RadarSdaControlFlow::ReadRounteSda(SensorType sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    ReadRouteSDA[0] = _ReadRouteSDA_COMMAND_BYTE_NUM;
    ReadRouteSDA[1] = _RouteControl_DID;
    ReadRouteSDA[2] = _ReadRoute_SID;
    ReadRouteSDA[3] = _RadarSDA_H;
    ReadRouteSDA[4] = _RadarSDA_L;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, ReadRouteSDA);
    }else{
        command_result.emplace_back(side_radar_right_req, ReadRouteSDA);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckReadRounteSda(errType ec){
    if(ec != noErr) return false;
    if (_canFdBuffer.size() == 0){
        _timeoutCode = sdaStatusTimeout;
        return false;
    }
    drive::common::can::CANFDArray data = _canFdBuffer[0];
    if ((data[0] != (_ActiveResponse + _RouteControl_DID)) || (data[1] != _ReadRoute_SID)){
        _errCode = responseErr;
        return false;
    }
    _canFdBuffer.clear();
    std::cout<<"Read ROUTE SDA SUCCESS!"<<std::endl;
    return true;
}

void RadarSdaControlFlow::StopRounteSda(SensorType sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    StopRouteSDA[0] = _StopRouteSDA_COMMAND_BYTE_NUM;
    StopRouteSDA[1] = _RouteControl_DID;
    StopRouteSDA[2] = _StopRoute_SID;
    StopRouteSDA[3] = _RadarSDA_H;
    StopRouteSDA[4] = _RadarSDA_L;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, StopRouteSDA);
    }else{
        command_result.emplace_back(side_radar_right_req, StopRouteSDA);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckStopRounteSda(errType ec){
    if(ec != noErr) return false;
    if (_canFdBuffer.size() == 0){
        _timeoutCode = stopSdaTimeout;
        return false;
    }
    drive::common::can::CANFDArray data = _canFdBuffer[0];
    if ((data[0] != (_ActiveResponse + _RouteControl_DID)) || (data[1] != _StopRoute_SID)){
        _errCode = responseErr;
        return false;
    }
    _canFdBuffer.clear();
    std::cout<<"Read ROUTE SDA SUCCESS!"<<std::endl;
    return true;
}

void RadarSdaControlFlow::SendTesterPresent(SensorType sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    TesterPresent[0] = _TesterPresent_COMMAND_BYTE_NUM;
    TesterPresent[1] = _Test_DID;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, TesterPresent);
    }else{
        command_result.emplace_back(side_radar_right_req, TesterPresent);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckTesterPresent(errType ec){
    if(ec != noErr) return false;
    if (_canFdBuffer.size() == 0){
        _timeoutCode = testerPresentTimeout;
        return false;
    }
    drive::common::can::CANFDArray data = _canFdBuffer[0];
    if ((data[0] != (_ActiveResponse + _Test_DID))){
        _errCode = responseErr;
        return false;
    }
    _canFdBuffer.clear();
    std::cout<<"TESTPRESENT SDA SUCCESS!"<<std::endl;
    return true;
}

}
}