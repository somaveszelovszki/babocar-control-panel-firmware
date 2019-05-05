#include <uns/task/Config.hpp>
#include <uns/util/debug.hpp>

namespace {
bool _error = false;    // Indicates if an error has occurred.
} // namespace

namespace uns {
namespace task {

void setErrorFlag() {
    _error = true;
}

bool hasErrorHappened() {
    return _error;
}

}  // namespace task
} // namespace uns
