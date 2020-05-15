#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>

#include <ProgramState.hpp>
#include <track.hpp>

namespace globals {

extern ProgramState programState;
extern micro::CarProps car;

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
