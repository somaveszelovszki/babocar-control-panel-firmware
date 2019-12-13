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
extern float frontLineController_P_slow;
extern float frontLineController_D_slow;
extern float frontLineController_P_fast;
extern float frontLineController_D_fast;
extern float rearLineController_P;
extern float rearLineController_D;
extern CarProps car;
extern m_per_sec_t targetSpeedOverride;
extern bool targetSpeedOverrideActive;
extern bool frontDistServoEnabled;
extern float frontDistServoAngleWheelTf;
extern m_per_sec_t speed_FAST;
extern m_per_sec_t speed_FAST_UNSAFE;
extern m_per_sec_t speed_SLOW;
extern m_per_sec_t speed_SLOW_UNSAFE;
extern meter_t slowSectionStartOffset;

extern bool isControlTaskInitialized;
extern bool isDebugTaskInitialized;
extern bool isDistSensorTaskInitialized;
extern bool isGyroTaskInitialized;
extern bool isLineDetectInitialized;

void registerGlobalParams(Params& params);

} // namespace globals
} // namespace micro
