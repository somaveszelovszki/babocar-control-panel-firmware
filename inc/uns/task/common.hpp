#pragma once

#include <uns/util/units.hpp>
#include <uns/util/algorithm.hpp>
#include <uns/bsp/task.hpp>
#include <uns/control/PID_Controller.hpp>

namespace uns {
/* @brief Represents a main program task.
 */
enum class ProgramTask {
    LABYRINTH,
    LANE_CHANGE,
    SAFETY_CAR_FOLLOW,
    OVERTAKE,
    RACE_TRACK
};

/* @brief This function is to be called on an error - sets error flag. Error flag can be checked with hasErrorHappened().
 * @note Settings error flag will cause all program tasks to end!
 **/
void setErrorFlag();

/* @brief Checks if an error has happened - checks error flag. Error flag can be set with setErrorFlag().
 * @returns Boolean value indicating if an error has happened.
 **/
bool hasErrorHappened();
} // namespace uns
