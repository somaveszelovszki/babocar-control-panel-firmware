#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <ProgramState.hpp>

namespace micro {

class Params;

namespace globals {

extern ProgramState programState;
extern bool linePatternCalcEnabled;
extern bool useSafetyEnableSignal;
extern bool indicatorLedsEnabled;
extern bool startSignalEnabled;
extern bool lineFollowEnabled;
extern microsecond_t motorCtrl_Ti;
extern float motorCtrl_Kc;
extern float frontLineCtrl_P_slow;
extern float frontLineCtrl_D_slow;
extern float frontLineCtrl_P_fast;
extern float frontLineCtrl_D_fast;
extern float rearLineCtrl_P;
extern float rearLineCtrl_D;
extern CarProps car;
extern bool distServoEnabled;
extern float distServoTransferRate;
extern m_per_sec_t speed_FAST;
extern m_per_sec_t speed_FAST_UNSAFE;
extern m_per_sec_t speed_SLOW;
extern m_per_sec_t speed_SLOW_UNSAFE;
extern meter_t slowSectionStartOffset;
extern m_per_sec_t speed_LAB_FWD;
extern m_per_sec_t speed_LAB_BWD;
extern m_per_sec_t speed_LANE_CHANGE;

extern bool isControlTaskInitialized;
extern bool isDebugTaskInitialized;
extern bool isDistSensorTaskInitialized;
extern bool isGyroTaskInitialized;
extern bool isLineDetectInitialized;

void registerGlobalParams(Params& params);

} // namespace globals
} // namespace micro
