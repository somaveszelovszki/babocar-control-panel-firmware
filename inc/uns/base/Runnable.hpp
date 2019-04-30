#pragma once

#include <uns/bsp/tim.hpp>
#include <uns/util/arrays.hpp>
#include <uns/util/units.hpp>
#include <type_traits>

namespace uns {

class Runnable {

public:
    Runnable() : period(time_t::ZERO()) {}

    explicit Runnable(uns::time_t _period) : period(_period) {}

    /* @brief Updates time and calculates time difference between previous and current run.
     * @note This is an internal method, do not call it explicitly!
     * @returns Time difference between the previous and the current run.
     **/
    uns::time_t updateTimeDiff(){
        uns::time_t prev = this->lastRunTime;
        return (this->lastRunTime = uns::getExactTime()) - prev;
    }

    bool shouldRun() const {
        return uns::getTime() - this->lastRunTime >= this->period;
    }

protected:
    uns::time_t period;
    uns::time_t lastRunTime;   // The time of the last run.
};

} // namespace uns
