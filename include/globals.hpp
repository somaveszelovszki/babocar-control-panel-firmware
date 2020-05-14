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
extern micro::CarProps car;
extern bool distServoEnabled;
extern float distServoTransferRate;

extern TrackSpeeds trackSpeeds[];
extern BrakeOffsets brakeOffsets[];

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
