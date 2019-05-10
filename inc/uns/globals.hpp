#pragma once

#include <uns/util/atomic.hpp>
#include <uns/util/units.hpp>
#include <uns/CarProps.hpp>

namespace uns {
namespace globals {

extern bool                 useSafetyEnableSignal;
extern bool                 indicatorLedsEnabled;
extern bool                 startSignalEnabled;
extern bool                 lineFollowEnabled;
extern LogLevel             logLevel;

extern atomic<CarProps>     car;
extern atomic<m_per_sec_t>  targetSpeed;

} // namespace globals
} // namespace uns
