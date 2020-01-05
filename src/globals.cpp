#include <globals.hpp>
#include <cfg_car.hpp>
#include <micro/debug/params.hpp>

namespace micro {
namespace globals {

ProgramState programState        = ProgramState(ProgramState::ActiveModule::Labyrinth, 1);
bool useSafetyEnableSignal       = true;
bool indicatorLedsEnabled        = true;
bool startSignalEnabled          = false;
bool lineFollowEnabled           = true;
microsecond_t motorCtrl_Ti       = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Ti;
float motorCtrl_Kc               = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Kc;
float frontLineCtrl_P_slow       = cfg::FRONT_LINE_CONTROLLER_DEFAULT_P_slow;
float frontLineCtrl_D_slow       = cfg::FRONT_LINE_CONTROLLER_DEFAULT_D_slow;
float frontLineCtrl_P_fast       = cfg::FRONT_LINE_CONTROLLER_DEFAULT_P_fast;
float frontLineCtrl_D_fast       = cfg::FRONT_LINE_CONTROLLER_DEFAULT_D_fast;
float rearLineCtrl_P             = cfg::REAR_LINE_CONTROLLER_DEFAULT_P;
float rearLineCtrl_D             = cfg::REAR_LINE_CONTROLLER_DEFAULT_D;
CarProps car                     = CarProps();
bool distServoEnabled            = true;
float distServoTransferRate      = 3.0f;
m_per_sec_t speed_FAST           = m_per_sec_t(3.1f);
m_per_sec_t speed_FAST_UNSAFE    = speed_FAST * 0.8;
m_per_sec_t speed_SLOW           = m_per_sec_t(2.0f);
m_per_sec_t speed_SLOW_UNSAFE    = speed_SLOW * 0.7;
meter_t slowSectionStartOffset   = meter_t(1.2);
m_per_sec_t speed_LAB_FWD        = m_per_sec_t(1.0f);
m_per_sec_t speed_LAB_BWD        = m_per_sec_t(0.7f);
m_per_sec_t speed_LANE_CHANGE    = m_per_sec_t(0.8f);

bool isControlTaskInitialized    = false;
bool isDebugTaskInitialized      = false;
bool isDistSensorTaskInitialized = false;
bool isGyroTaskInitialized       = false;
bool isLineDetectInitialized     = false;

void registerGlobalParams(Params& params) {

#define REGISTER_GLOBAL(name) params.registerParam(#name, &name)

    REGISTER_GLOBAL(motorCtrl_Ti);
    REGISTER_GLOBAL(motorCtrl_Kc);
    REGISTER_GLOBAL(frontLineCtrl_P_slow);
    REGISTER_GLOBAL(frontLineCtrl_D_slow);
    REGISTER_GLOBAL(frontLineCtrl_P_fast);
    REGISTER_GLOBAL(frontLineCtrl_D_fast);
    REGISTER_GLOBAL(rearLineCtrl_P);
    REGISTER_GLOBAL(rearLineCtrl_D);
    REGISTER_GLOBAL(car);
    REGISTER_GLOBAL(distServoEnabled);
    REGISTER_GLOBAL(distServoTransferRate);
    REGISTER_GLOBAL(speed_FAST);
    REGISTER_GLOBAL(speed_FAST_UNSAFE);
    REGISTER_GLOBAL(speed_SLOW);
    REGISTER_GLOBAL(speed_SLOW_UNSAFE);
    REGISTER_GLOBAL(slowSectionStartOffset);
    REGISTER_GLOBAL(speed_LAB_FWD);
    REGISTER_GLOBAL(speed_LAB_BWD);
    REGISTER_GLOBAL(speed_LANE_CHANGE);

#undef REGISTER_GLOBAL
}

}  // namespace globals
}  // namespace micro
