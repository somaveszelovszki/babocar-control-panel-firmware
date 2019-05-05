#include <uns/globals.hpp>
#include <uns/config/cfg_os.hpp>

namespace uns {
namespace globals {

task::Config taskConfig;
atomic<CarProps> car(cfg::mutex_Car);
atomic<CarProps> targetSpeed(cfg::mutex_TargetSpeed);

void setDefaultTaskConfig() {
    taskConfig.useSafetyEnableSignal = true;
    taskConfig.indicatorLedsEnabled = true;
    taskConfig.startSignalEnabled = false;
}

}  // namespace globals
}  // namespace uns
