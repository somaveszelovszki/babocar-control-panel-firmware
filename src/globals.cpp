#include <globals.hpp>
#include <cfg_os.hpp>
#include <cfg_car.hpp>

namespace micro {
namespace globals {

Params debugParams;

#define REGISTER_GLOBAL(name) debugParams.registerParam(#name, &name)

bool useSafetyEnableSignal = true;
bool indicatorLedsEnabled  = true;
bool startSignalEnabled    = false;
bool lineFollowEnabled     = true;

microsecond_t motorController_Ti = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Ti;
float         motorController_Kc = cfg::DC_MOTOR_CONTROLLER_DEFAULT_Kc;

m_per_sec_t      targetSpeed(0.0f);
atomic<CarProps> car(cfg::mutex_Car);

void initializeGlobalParams() {
    REGISTER_GLOBAL(useSafetyEnableSignal);
    REGISTER_GLOBAL(indicatorLedsEnabled);
    REGISTER_GLOBAL(startSignalEnabled);
    REGISTER_GLOBAL(lineFollowEnabled);

    REGISTER_GLOBAL(car);
    REGISTER_GLOBAL(targetSpeed);
}

}  // namespace globals
}  // namespace micro
