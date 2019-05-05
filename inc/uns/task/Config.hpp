#pragma once

namespace uns {
namespace task {

struct Config {
    bool useSafetyEnableSignal;
    bool indicatorLedsEnabled;
    bool startSignalEnabled;
};

/* @brief This function is to be called on an error - sets error flag. Error flag can be checked with hasErrorHappened().
 * @note Settings error flag will cause all program tasks to end!
 **/
void setErrorFlag();

/* @brief Checks if an error has happened - checks error flag. Error flag can be set with setErrorFlag().
 * @returns Boolean value indicating if an error has happened.
 **/
bool hasErrorHappened();

} // namespace task
} // namespace uns
