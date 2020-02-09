#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <ProgramState.hpp>

namespace micro {

class Params;

namespace globals {

extern ProgramState programState;
extern bool useSafetyEnableSignal;
extern bool frontIndicatorLedsEnabled;
extern bool rearIndicatorLedsEnabled;
extern bool lineDetectionEnabled;
extern float motorCtrl_P;
extern float motorCtrl_I;
extern float motorCtrl_integral_max;
extern float frontLineCtrl_P_slow;
extern float frontLineCtrl_D_slow;
extern float frontLineCtrl_P_fast;
extern float frontLineCtrl_D_fast;
extern float frontLineCtrl_P_bwd;
extern float frontLineCtrl_D_bwd;
extern float frontLineCtrl_P_fwd_mul;
extern float frontLineCtrl_D_fwd;
extern CarProps car;
extern bool distSensorEnabled;
extern bool distServoEnabled;
extern float distServoTransferRate;
extern m_per_sec_t speed_FAST1;
extern m_per_sec_t speed_FAST2;
extern m_per_sec_t speed_FAST3;
extern m_per_sec_t speed_FAST4;
extern m_per_sec_t speed_FAST5;
extern m_per_sec_t speed_FAST6;
extern m_per_sec_t speed_SLOW1;
extern m_per_sec_t speed_SLOW2;
extern m_per_sec_t speed_SLOW3;
extern m_per_sec_t speed_SLOW4;
extern m_per_sec_t speed_SLOW5;
extern m_per_sec_t speed_SLOW6;
extern m_per_sec_t speed_LAB_FWD;
extern m_per_sec_t speed_LAB_FWD_FAST;
extern m_per_sec_t speed_LAB_BWD;
extern m_per_sec_t speed_LANE_CHANGE;
extern m_per_sec_t speed_REACH_SAFETY_CAR;
extern m_per_sec_t speed_TURN_AROUND;
extern m_per_sec_t speed_OVERTAKE_CURVE;
extern m_per_sec_t speed_OVERTAKE_STRAIGHT;
extern meter_t dist_OVERTAKE_SIDE;

extern bool isControlTaskOk;
extern bool isDebugTaskOk;
extern bool isDistSensorTaskOk;
extern bool isGyroTaskOk;
extern bool isLineDetectTaskOk;

void registerGlobalParams(Params& params);

inline bool areAllTasksOk(void) {
    return isControlTaskOk &&
           isDebugTaskOk &&
           isDistSensorTaskOk &&
           isGyroTaskOk &&
           isLineDetectTaskOk;
}

} // namespace globals
} // namespace micro
