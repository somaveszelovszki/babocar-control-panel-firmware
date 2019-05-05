#pragma once

#include <uns/util/atomic.hpp>
#include <uns/task/Config.hpp>
#include <uns/CarProps.hpp>

namespace uns {
namespace globals {

extern task::Config taskConfig;
extern atomic<CarProps> car;
extern atomic<CarProps> targetSpeed;

void setDefaultTaskConfig();

} // namespace globals
} // namespace uns
