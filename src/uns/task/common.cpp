#include <uns/task/common.hpp>
#include <uns/util/debug.hpp>

using namespace uns;

uns::ProgramTask PROGRAM_TASK = uns::ProgramTask::LABYRINTH;

namespace {
bool _error = false;    // Indicates if an error has occurred.
} // namespace

void uns::setErrorFlag() {
    _error = true;
}

bool uns::hasErrorHappened() {
    return _error;
}
