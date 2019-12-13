#include <globals.hpp>
#include <cfg_car.hpp>
#include <micro/debug/params.hpp>

namespace micro {
namespace globals {

ProgramState programState        = ProgramState(ProgramState::ActiveModule::RaceTrack, 3);
bool useSafetyEnableSignal       = true;
bool indicatorLedsEnabled        = true;
bool startSignalEnabled          = false;
bool lineFollowEnabled           = true;
microsecond_t motorController_Ti = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Ti;
float motorController_Kc         = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Kc;
float frontLineController_P_slow = cfg::FRONT_LINE_CONTROLLER_DEFAULT_P_slow;
float frontLineController_D_slow = cfg::FRONT_LINE_CONTROLLER_DEFAULT_D_slow;
float frontLineController_P_fast = cfg::FRONT_LINE_CONTROLLER_DEFAULT_P_fast;
float frontLineController_D_fast = cfg::FRONT_LINE_CONTROLLER_DEFAULT_D_fast;
float rearLineController_P       = cfg::REAR_LINE_CONTROLLER_DEFAULT_P;
float rearLineController_D       = cfg::REAR_LINE_CONTROLLER_DEFAULT_D;
CarProps car                     = CarProps();
m_per_sec_t targetSpeedOverride  = m_per_sec_t(1.25f);
bool targetSpeedOverrideActive   = false;
bool frontDistServoEnabled       = true;
float frontDistServoAngleWheelTf = 3.0f;
m_per_sec_t speed_FAST           = m_per_sec_t(3.5f);
m_per_sec_t speed_FAST_UNSAFE    = speed_FAST * 0.8;
m_per_sec_t speed_SLOW           = m_per_sec_t(1.8f);
m_per_sec_t speed_SLOW_UNSAFE    = speed_SLOW * 0.7;
meter_t slowSectionStartOffset   = meter_t(2);

bool isControlTaskInitialized    = false;
bool isDebugTaskInitialized      = false;
bool isDistSensorTaskInitialized = false;
bool isGyroTaskInitialized       = false;
bool isLineDetectInitialized     = false;

void registerGlobalParams(Params& params) {

#define REGISTER_GLOBAL(name) params.registerParam(#name, &name)

    REGISTER_GLOBAL(useSafetyEnableSignal);
    REGISTER_GLOBAL(indicatorLedsEnabled);
    REGISTER_GLOBAL(startSignalEnabled);
    REGISTER_GLOBAL(lineFollowEnabled);
    REGISTER_GLOBAL(motorController_Ti);
    REGISTER_GLOBAL(motorController_Kc);
    REGISTER_GLOBAL(frontLineController_P_slow);
    REGISTER_GLOBAL(frontLineController_D_slow);
    REGISTER_GLOBAL(frontLineController_P_fast);
    REGISTER_GLOBAL(frontLineController_D_fast);
    REGISTER_GLOBAL(rearLineController_P);
    REGISTER_GLOBAL(rearLineController_D);
    REGISTER_GLOBAL(car);
    REGISTER_GLOBAL(targetSpeedOverride);
    REGISTER_GLOBAL(targetSpeedOverrideActive);
    REGISTER_GLOBAL(frontDistServoEnabled);
    REGISTER_GLOBAL(frontDistServoAngleWheelTf);
    REGISTER_GLOBAL(speed_FAST);
    REGISTER_GLOBAL(speed_FAST_UNSAFE);
    REGISTER_GLOBAL(speed_SLOW);
    REGISTER_GLOBAL(speed_SLOW_UNSAFE);
    REGISTER_GLOBAL(slowSectionStartOffset);

#undef REGISTER_GLOBAL
}

}  // namespace globals
}  // namespace micro
