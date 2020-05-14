#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>

#include <ProgramState.hpp>
#include <track.hpp>

namespace globals {

extern ProgramState programState;
extern bool useSafetyEnableSignal;
extern bool indicatorLedsEnabled;
extern uint8_t reducedLineDetectScanRangeRadius;
extern float motorCtrl_P;
extern float motorCtrl_I;
extern float motorCtrl_integral_max;
extern float linePosCtrl_P;
extern float linePosCtrl_D;
extern float lineAngleCtrl_P;
extern float lineAngleCtrl_D;
extern micro::CarProps car;
extern bool distServoEnabled;
extern float distServoTransferRate;
extern micro::radian_t frontWheelOffset;
extern micro::radian_t rearWheelOffset;
extern micro::radian_t frontDistSensorServoOffset;

extern TrackSpeeds trackSpeeds[];
extern BrakeOffsets brakeOffsets[];

extern micro::m_per_sec_t speed_LAB_FWD;
extern micro::m_per_sec_t speed_LAB_BWD;
extern micro::m_per_sec_t speed_LANE_CHANGE;
extern micro::m_per_sec_t speed_REACH_SAFETY_CAR;
extern micro::m_per_sec_t speed_OVERTAKE_BEGIN;
extern micro::m_per_sec_t speed_OVERTAKE_STRAIGHT;
extern micro::m_per_sec_t speed_OVERTAKE_END;
extern micro::meter_t dist_OVERTAKE_SIDE;

extern bool isControlTaskOk;
extern bool isDebugTaskOk;
extern bool isDistSensorTaskOk;
extern bool isGyroTaskOk;
extern bool isLineDetectTaskOk;
extern bool isVehicleCanTaskOk;

inline bool areAllTasksOk(void) {
    return isControlTaskOk &&
           isDebugTaskOk &&
           isDistSensorTaskOk &&
           isGyroTaskOk &&
           isLineDetectTaskOk &&
           isVehicleCanTaskOk;
}

} // namespace globals
