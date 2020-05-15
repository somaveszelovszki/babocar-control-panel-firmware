#include <globals.hpp>
#include <cfg_car.hpp>
#include <cfg_track.hpp>

using namespace micro;

namespace globals {

ProgramState programState                = ProgramState::INVALID;
CarProps car                             = CarProps();

bool isControlTaskOk    = false;
bool isDebugTaskOk      = false;
bool isDistSensorTaskOk = false;
bool isGyroTaskOk       = false;
bool isLineDetectTaskOk = false;
bool isVehicleCanTaskOk = false;

}  // namespace globals
