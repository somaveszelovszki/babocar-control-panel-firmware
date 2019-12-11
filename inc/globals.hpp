#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <ProgramState.hpp>

namespace micro {

class Params;

namespace globals {

extern ProgramState programState;
extern bool useSafetyEnableSignal;
extern bool indicatorLedsEnabled;
extern bool startSignalEnabled;
extern bool lineFollowEnabled;
extern microsecond_t motorController_Ti;
extern float motorController_Kc;
extern float frontLineController_P_1mps;
extern float frontLineController_D_1mps;
extern float rearLineController_P;
extern float rearLineController_D;
extern CarProps car;
extern m_per_sec_t targetSpeedOverride;
extern bool targetSpeedOverrideActive;
extern bool frontDistServoEnabled;
extern float frontDistServoAngleWheelTf;

extern bool isControlTaskInitialized;
extern bool isDebugTaskInitialized;
extern bool isDistSensorTaskInitialized;
extern bool isGyroTaskInitialized;
extern bool isLineDetectInitialized;

void initializeGlobalParams(Params& params);

} // namespace globals
} // namespace micro
