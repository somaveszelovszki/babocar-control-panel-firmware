#include <uns/globals.hpp>
#include <uns/config/cfg_os.hpp>

namespace uns {
namespace globals {

bool                useSafetyEnableSignal = true;
bool                indicatorLedsEnabled  = true;
bool                startSignalEnabled    = false;
bool                lineFollowEnabled     = true;
LogLevel            logLevel              = LogLevel::Debug;

atomic<CarProps>    car(cfg::mutex_Car);
atomic<m_per_sec_t> targetSpeed(cfg::mutex_TargetSpeed);

}  // namespace globals
}  // namespace uns
