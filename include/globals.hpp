#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <TrackSpeeds.hpp>
#include <ProgramState.hpp>

constexpr uint8_t NUM_LAPS = 6;

class Params;

namespace globals {

extern ProgramState programState;
extern bool useSafetyEnableSignal;
extern bool indicatorLedsEnabled;
extern uint8_t reducedLineDetectScanRangeRadius;
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
extern micro::CarProps car;
extern bool distServoEnabled;
extern float distServoTransferRate;
extern micro::radian_t frontWheelOffset;
extern micro::radian_t rearWheelOffset;
extern micro::radian_t frontDistSensorServoOffset;

extern TrackSpeeds trackSpeeds[];

extern micro::m_per_sec_t speed_LAB_FWD;
extern micro::m_per_sec_t speed_LAB_BWD;
extern micro::m_per_sec_t speed_LANE_CHANGE;
extern micro::m_per_sec_t speed_REACH_SAFETY_CAR;
extern micro::m_per_sec_t speed_TURN_AROUND;
extern micro::m_per_sec_t speed_OVERTAKE_BEGIN;
extern micro::m_per_sec_t speed_OVERTAKE_STRAIGHT;
extern micro::m_per_sec_t speed_OVERTAKE_END;
extern micro::meter_t dist_OVERTAKE_SIDE;
extern micro::meter_t dist_BRAKE_OFFSET;

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
