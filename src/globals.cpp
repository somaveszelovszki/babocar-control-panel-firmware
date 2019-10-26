#include <globals.hpp>
#include <cfg_car.hpp>

namespace micro {
namespace globals {

Params debugParams;

#define REGISTER_GLOBAL(name) debugParams.registerParam(#name, &name)

ProgramState programState        = ProgramState(ProgramState::ActiveModule::Labyrinth, 0);
bool useSafetyEnableSignal       = true;
bool indicatorLedsEnabled        = true;
bool startSignalEnabled          = false;
bool lineFollowEnabled           = true;
microsecond_t motorController_Ti = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Ti;
float motorController_Kc         = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Kc;
float frontLineController_P      = cfg::FRONT_LINE_CONTROLLER_DEFAULT_P;
float frontLineController_D      = cfg::FRONT_LINE_CONTROLLER_DEFAULT_D;
float rearLineController_P       = cfg::REAR_LINE_CONTROLLER_DEFAULT_P;
float rearLineController_D       = cfg::REAR_LINE_CONTROLLER_DEFAULT_D;
CarProps car                     = CarProps();

void initializeGlobalParams() {
    REGISTER_GLOBAL(useSafetyEnableSignal);
    REGISTER_GLOBAL(indicatorLedsEnabled);
    REGISTER_GLOBAL(startSignalEnabled);
    REGISTER_GLOBAL(lineFollowEnabled);
    REGISTER_GLOBAL(motorController_Ti);
    REGISTER_GLOBAL(motorController_Kc);
    REGISTER_GLOBAL(frontLineController_P);
    REGISTER_GLOBAL(frontLineController_D);
    REGISTER_GLOBAL(rearLineController_P);
    REGISTER_GLOBAL(rearLineController_D);
    REGISTER_GLOBAL(car);
}

}  // namespace globals
}  // namespace micro
