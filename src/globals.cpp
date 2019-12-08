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
float frontLineController_P_1mps = cfg::FRONT_LINE_CONTROLLER_DEFAULT_P;
float frontLineController_D_1mps = cfg::FRONT_LINE_CONTROLLER_DEFAULT_D;
float rearLineController_P       = cfg::REAR_LINE_CONTROLLER_DEFAULT_P;
float rearLineController_D       = cfg::REAR_LINE_CONTROLLER_DEFAULT_D;
CarProps car                     = CarProps();
m_per_sec_t targetSpeedOverride  = m_per_sec_t(1.25f);
bool targetSpeedOverrideActive   = false;
bool frontDistServoEnabled       = true;
float frontDistServoAngleWheelTf = 3.0f;

bool isControlTaskInitialized    = false;
bool isDebugTaskInitialized      = false;
bool isSensorTaskInitialized     = false;

void initializeGlobalParams(Params& params) {

#define REGISTER_GLOBAL(name) params.registerParam(#name, &name)

    REGISTER_GLOBAL(useSafetyEnableSignal);
    REGISTER_GLOBAL(indicatorLedsEnabled);
    REGISTER_GLOBAL(startSignalEnabled);
    REGISTER_GLOBAL(lineFollowEnabled);
    REGISTER_GLOBAL(motorController_Ti);
    REGISTER_GLOBAL(motorController_Kc);
    REGISTER_GLOBAL(frontLineController_P_1mps);
    REGISTER_GLOBAL(frontLineController_D_1mps);
    REGISTER_GLOBAL(rearLineController_P);
    REGISTER_GLOBAL(rearLineController_D);
    REGISTER_GLOBAL(car);
    REGISTER_GLOBAL(targetSpeedOverride);
    REGISTER_GLOBAL(targetSpeedOverrideActive);

#undef REGISTER_GLOBAL
}

}  // namespace globals
}  // namespace micro
