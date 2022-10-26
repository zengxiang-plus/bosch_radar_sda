#include "radar_sda_control_flow.h"

namespace driver {
namespace radar {

void GenerateKeyEx(const SecurityFramArray& f_SeedArray, uint8_t f_SeedArrarySize, const uint8_t f_SecurityLevel, SecurityFramArray& f_KeyArray)
{
    const uint64_t SEC_MASK = 0x0919931125;
    const uint16_t l_Sec_Coe_a_L1 = 0x2528;
    const uint16_t l_Sec_Coe_a_L2 = 0x2527;
    const uint16_t l_Sec_Coe_c_L1 = 0x28;
    const uint16_t l_Sec_Coe_c_L2 = 0x27;
    uint64_t l_seed = 0;
    uint64_t l_key = 0;
    l_seed = ((static_cast<uint64_t>(f_SeedArray[0]) << 32) & 0x000000FF00000000);
    l_seed = (l_seed | ((static_cast<uint64_t>(f_SeedArray[1]) << 24) & 0x00000000FF000000));
    l_seed = (l_seed | ((static_cast<uint64_t>(f_SeedArray[2]) << 16) & 0x0000000000FF0000));
    l_seed = (l_seed | ((static_cast<uint64_t>(f_SeedArray[3]) << 8) & 0x000000000000FF00));
    l_seed = (l_seed | ((static_cast<uint64_t>(f_SeedArray[4]) & 0x00000000000000FF)));
    if (f_SeedArrarySize == 5 &&  l_seed != 0 && l_seed != 0x000000FFFFFFFFFF){
        switch (f_SecurityLevel)
        {
            case 0x01:
                l_key = (l_Sec_Coe_a_L1 * (l_seed ^ SEC_MASK) + l_Sec_Coe_c_L1) % 0x0000010000000000;
                break;
            case 0x02:
                l_key = (l_Sec_Coe_a_L2 * (l_seed ^ SEC_MASK) + l_Sec_Coe_c_L2) % 0x0000010000000000;
                break;
            default:
                break;
        }
    }
    f_KeyArray[0] = static_cast<uint64_t>(l_key & 0x00000000000000FF);
    f_KeyArray[1] = static_cast<uint64_t>((l_key & 0x000000000000FF00) >> 8);
    f_KeyArray[2] = static_cast<uint64_t>((l_key & 0x0000000000FF0000) >> 16);
    f_KeyArray[3] = static_cast<uint64_t>((l_key & 0x00000000FF000000) >> 24);
    f_KeyArray[4] = static_cast<uint64_t>((l_key & 0x000000FF00000000) >> 32);
}

RadarSdaControlFlow::RadarSdaControlFlow(type_Sensor id, drive::common::CanInterface& canInterface):
    side_radar_func(0x7F2), side_radar_left_req(0x1F2), side_radar_right_req(0x3F2), side_radar_left_resp(0x569), side_radar_right_resp(0x56B),
    _ChangeSession_DID(0x10), _SecurityAccess_DID(0x27), _RouteControl_DID(0x31), _Test_DID(0x3E), 
    _SecurityAccessLever1(0x01), _SecurityAccessLever2(0x02), _SDA_Status_Frame_BYTE_NUM(SDAStatusFrameNUM), _Count_BYTE_NUM(4),
    _StartRoute_SID(0x01), _ReadRoute_SID(0x03), _StopRoute_SID(0x02), 
    _RadarSDA_H(0xF0), _RadarSDA_L(0x08), _ExtendSession(0x03), _SDA_Frame_BYTE_NUM(8), _ActiveResponse(0x40),
    _ChangeSession_COMMAND_BYTE_NUM(2), _SecurityAccess_COMMAND_BYTE_NUM(2), 
    _StartRouteSDA_COMMAND_BYTE_NUM(4), _ReadRouteSDA_COMMAND_BYTE_NUM(4), 
    _StopRouteSDA_COMMAND_BYTE_NUM(4), _TesterPresent_COMMAND_BYTE_NUM(2),
    _errCode(noErr), _timeoutCode(noTimeout), _flowStatus(init),
    _sensor_ID(id),_canInterface(canInterface)
{
    ChangeSession.resize(_SDA_Frame_BYTE_NUM);
    SecurityAccess.resize(_SDA_Frame_BYTE_NUM);
    StartRouteSDA.resize(_SDA_Frame_BYTE_NUM);
    ReadRouteSDA.resize(_SDA_Frame_BYTE_NUM);
    StopRouteSDA.resize(_SDA_Frame_BYTE_NUM);
    TesterPresent.resize(_SDA_Frame_BYTE_NUM);
    ContinueFrame.resize(_SDA_Frame_BYTE_NUM);
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

void RadarSdaControlFlow::SetSdaFlowStatus(type_FlowStatus status){
    _flowStatus = status;
}

type_FlowStatus RadarSdaControlFlow::GetSdaFlowStatus(void){
    return _flowStatus;
}

std::string RadarSdaControlFlow::PrintSdaFlowStatus(void){
    return _SdaStatus[_flowStatus];
}




// Change Side Radar Mode to Extend Mode (Tx: 0x02 10 03 00 00 00 00 00     Active Response: 0x06 50 03 XX XX XX XX 55)
void RadarSdaControlFlow::SendChangeSession(type_Sensor sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    ChangeSession[0] = _ChangeSession_COMMAND_BYTE_NUM;
    ChangeSession[1] = _ChangeSession_DID;
    ChangeSession[2] = _ExtendSession;
    command_result.emplace_back(side_radar_func, ChangeSession);
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckChangeSessionResponse(CANFDArray data){
    if ((data[1] != (_ActiveResponse + _ChangeSession_DID)) || (data[2] != _ExtendSession)){
        _errCode = responseErr;
        return false;
    }
    std::cout<<"CHANGE SESSION SUCCESS!"<<std::endl;
    return true;
}

//Security Access (Tx: 0x02 27 01 00 00 00 00 00    Active Response: 0x07 67 01 S1 S2 S3 S4 S5     
//Calculate KEY K[1-5] By SEED S[1-5]      Tx 0x07 27 02 K1 K2 K3 K4 K5                Active Response:0x02 67 02 55 55 55 55 55)
void RadarSdaControlFlow::SendSecurityAccess(type_Sensor sensor_id, drive::common::CanInterface& can)
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

bool RadarSdaControlFlow::CheckSecurityAccessResponse(CANFDArray data){
    SecurityFramArray g_SeedArray;
    SecurityFramArray g_KeyArray;
    if ((data[1] != (_ActiveResponse + _SecurityAccess_DID)) || (data[2] != _SecurityAccessLever1)){
        _errCode = responseErr;
        return false;
    }
    std::cout<<"SECURITY ACCESS1 SUCCESS!"<<std::endl;
    for(int i=0; i<SecuritySeedKeyNUM; i++){
        g_SeedArray[i] = static_cast<uint8_t>(data[i+3]); 
    }
    GenerateKeyEx(g_SeedArray, SecuritySeedKeyNUM, _SecurityAccessLever1, g_KeyArray);
    SendSecurityAccess2(_sensor_ID, _canInterface, g_KeyArray);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

void RadarSdaControlFlow::SendSecurityAccess2(type_Sensor sensor_id, drive::common::CanInterface& can, const SecurityFramArray& gKey)
{
    command_result.clear();
    SecurityAccess[0] = _SecurityAccess_COMMAND_BYTE_NUM + SecuritySeedKeyNUM;
    SecurityAccess[1] = _SecurityAccess_DID;
    SecurityAccess[2] = _SecurityAccessLever2;
    for(int i=0; i<SecuritySeedKeyNUM; i++)
    {
        SecurityAccess[i+3] = gKey[i];
    }
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, SecurityAccess);
    }else{
        command_result.emplace_back(side_radar_right_req, SecurityAccess);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::CheckSecurityAccessResponse2(CANFDArray data){
    if ((data[1] != (_ActiveResponse + _SecurityAccess_DID)) || (data[2] != _SecurityAccessLever2)){
        _errCode = responseErr;
        return false;
    }
    std::cout<<"SECURITY ACCESS2 SUCCESS!"<<std::endl;
    return true;
}

//Start SDA (Tx: 0x04 31 01 F0 08 00 00 00                Active Response:0x04 71 01 F0 08 55 55 55)
void RadarSdaControlFlow::StartRounteSda(type_Sensor sensor_id, drive::common::CanInterface& can)
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

bool RadarSdaControlFlow::CheckStartRounteSda(CANFDArray data){
    if ((data[1] != (_ActiveResponse + _RouteControl_DID)) || (data[2] != _StartRoute_SID)){
        _errCode = responseErr;
        return false;
    }
    std::cout<<"START SDA SUCCESS!"<<std::endl;
    return true;
}

//Read SDA (Tx: 0x04 31 03 F0 08 00 00 00               Active Response:0x10 0B 71 03 F0 08 XX XX)
// Send Continue Frame(Tx: 0x30 00 00 00 00 00 00 00)
void RadarSdaControlFlow::ReadRounteSda(type_Sensor sensor_id, drive::common::CanInterface& can)
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

void RadarSdaControlFlow::SendContinueFrame(type_Sensor sensor_id, drive::common::CanInterface& can){
    command_result.clear();
    ContinueFrame[0] = 0x30;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, ContinueFrame);
    }else{
        command_result.emplace_back(side_radar_right_req, ContinueFrame);
    }
    can.WriteToCanFd(command_result);
}

bool RadarSdaControlFlow::GetReadRounteSdaStatus(CANFDArray data){
    if(_SDA_Status_Frame_BYTE_NUM == SDAStatusFrameNUM){
        if ((data[2] != (_ActiveResponse + _RouteControl_DID)) || (data[3] != _ReadRoute_SID) || data[0] != 0x10 || data[1] != SDAStatusFrameNUM){
            _errCode = responseErr;
            return false;
        }
        SendContinueFrame(_sensor_ID, _canInterface);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        _Sda_Status.clear();
        for(int i = 2; i < 8; i++){
            _Sda_Status[i-2] = (data[i]);
            _SDA_Status_Frame_BYTE_NUM--;
        }
        return true;
    }else{
        if (data[0] != 0x21){
            _errCode = responseErr;
            return false;
        }
        for(int i = 0; i < _SDA_Status_Frame_BYTE_NUM; i++){
            _Sda_Status[SDAStatusFrameNUM - _SDA_Status_Frame_BYTE_NUM + i] = data[i+1];
        }
    } 
    
    _sda_Process = AnySDAStatus();

   if(_Count_BYTE_NUM && !_sda_Process){
        SendTesterPresent(_sensor_ID, _canInterface);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        ReadRounteSda(_sensor_ID, _canInterface);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
   }

if(_sda_Process){
    StopRounteSda(_sensor_ID, _canInterface);
    SetSdaFlowStatus(stopSda);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout<<"READ SDA SUCCESS!"<<std::endl;
    return true;
}
    return false;
}

bool RadarSdaControlFlow::AnySDAStatus(void){

    std::cout << "PROCESS: Reading SDA Status"<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
     _SDA_Status_Frame_BYTE_NUM = SDAStatusFrameNUM;
     _Count_BYTE_NUM --;

    drive::common::can::Byte X1 = _Sda_Status[4];
    drive::common::can::Byte X2 = _Sda_Status[5];
    drive::common::can::Byte X3 = _Sda_Status[6];
    drive::common::can::Byte X4 = _Sda_Status[7];
    drive::common::can::Byte X5 = _Sda_Status[8];
    drive::common::can::Byte X6 = _Sda_Status[9];
    drive::common::can::Byte X7 = _Sda_Status[10];

    // Process Routine Status && Results
    std::cout << "INFO: Routine Status is "<<_RoutineStatus[X1 & 0x0F] << std::endl;
    std::cout << "INFO: Routine Result is "<<_RoutineResult[X1 & 0xF0] << std::endl;

    //Process  Driving Profile
    for (int i = 0; i < 6; i++){
        if(X2 & (0x01 << i)){
            std::cout << "INFO: Driving Profile is "<< _DrivingProfile[i] << std::endl;
        }
    }

    //SDA Process
    std::cout << "INFO: SDA Process is "<< (X3 & 0xFF) << std::endl;
    if ((X3 & 0xFF) == 100) return true;

    //Horizontal Angle && Vertical Angle
    uint16_t tmpHorizontal = (static_cast<uint16_t>(X4) << 8) + X5;
    uint16_t tmpVertical = (static_cast<uint16_t>(X6) << 8) + X7;
    _Horizontal_Angle = (static_cast<double>(tmpHorizontal)) * 0.01 -3;
    _Vertical_Angle = (static_cast<double>(tmpVertical)) * 0.01 -3;
    std::cout << "INFO: Horizontal Angle is "<< _Horizontal_Angle << std::endl;
    std::cout << "INFO: Vertical Angle is "<< _Vertical_Angle << std::endl;

    return false;
}

//Stop SDA(Tx: 0x04 31 02 F0 08 00 00 00)
void RadarSdaControlFlow::StopRounteSda(type_Sensor sensor_id, drive::common::CanInterface& can)
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

bool RadarSdaControlFlow::CheckStopRounteSda(CANFDArray data){
    if ((data[1] != (_ActiveResponse + _RouteControl_DID)) || (data[2] != _StopRoute_SID)){
        _errCode = responseErr;
        return false;
    }
    std::cout<<"STOP SDA SUCCESS!"<<std::endl;
    return true;
}

void RadarSdaControlFlow::SendTesterPresent(type_Sensor sensor_id, drive::common::CanInterface& can)
{
    command_result.clear();
    TesterPresent[0] = _TesterPresent_COMMAND_BYTE_NUM;
    TesterPresent[1] = _Test_DID;
    TesterPresent[2] = 0x80;
    if(sensor_id == SideRadar_Left){
        command_result.emplace_back(side_radar_left_req, TesterPresent);
    }else{
        command_result.emplace_back(side_radar_right_req, TesterPresent);
    }
    can.WriteToCanFd(command_result);
}

}
}