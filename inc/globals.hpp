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
extern float motorCtrl_P;
extern float motorCtrl_I;
extern float motorCtrl_integral_max;
extern float frontLineCtrl_P_slow;
extern float frontLineCtrl_D_slow;
extern float frontLineCtrl_P_fast;
extern float frontLineCtrl_D_fast;
extern CarProps car;
extern bool distServoEnabled;
extern float distServoTransferRate;
extern m_per_sec_t speed_FAST;
extern m_per_sec_t speed_SLOW;
extern meter_t slowSectionStartOffset;
extern m_per_sec_t speed_LAB_FWD;
extern m_per_sec_t speed_LAB_BWD;
extern m_per_sec_t speed_LANE_CHANGE;
extern m_per_sec_t speed_OVERTAKE_CURVE;
extern m_per_sec_t speed_OVERTAKE_STRAIGHT;

extern bool isControlTaskOk;
extern bool isDebugTaskOk;
extern bool isDistSensorTaskOk;
extern bool isGyroTaskOk;
extern bool isLineDetectTaskOk;

void registerGlobalParams(Params& params);

} // namespace globals
} // namespace micro
