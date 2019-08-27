#include <globals.hpp>
#include <cfg_os.hpp>

namespace micro {
namespace globals {

Params debugParams;

#define REGISTER_GLOBAL(name) debugParams.registerParam(#name, &name)

bool useSafetyEnableSignal = true;
bool indicatorLedsEnabled  = true;
bool startSignalEnabled    = false;
bool lineFollowEnabled     = true;
bool targetSpeedOverride   = false;

m_per_sec_t      targetSpeed(0.0f);
atomic<CarProps> car(cfg::mutex_Car);

void initializeGlobalParams() {
    REGISTER_GLOBAL(useSafetyEnableSignal);
//    REGISTER_GLOBAL(indicatorLedsEnabled);
//    REGISTER_GLOBAL(startSignalEnabled);
//    REGISTER_GLOBAL(lineFollowEnabled);
//    REGISTER_GLOBAL(targetSpeedOverride);
//
//    REGISTER_GLOBAL(logLevel_t);
//    REGISTER_GLOBAL(car);
//    REGISTER_GLOBAL(targetSpeed);
}

}  // namespace globals
}  // namespace micro
