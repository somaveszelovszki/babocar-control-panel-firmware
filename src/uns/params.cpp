#include <uns/params.hpp>
#include <uns/bsp/task.hpp>
#include <uns/util/debug.hpp>

namespace uns {

void Params::updateParam(uint8_t id, const uint8_t *buf, uint8_t size) {
    Param *param = this->values.get(id);
    if (param) {
        if (param->size == size) {

            if (param->hmutex != nullptr) {
                while (!isOk(uns::mutexTake(param->hmutex, millisecond_t(1)))) {}
                memcpy(param->buf, buf, size);
                uns::mutexRelease(param->hmutex);
            } else {
                uns::taskSuspendAll();
                memcpy(param->buf, buf, size);
                uns::taskResumeAll();
            }

        } else {
            LOG_ERROR_WITH_STATUS(Status::INVALID_DATA, "Invalid Param buffer size: %d (for id: %d)", static_cast<int32_t>(size), static_cast<int32_t>(id));
        }
    } else {
        LOG_ERROR_WITH_STATUS(Status::INVALID_ID, "Invalid Param id: %d", static_cast<int32_t>(id));
    }
}

} // namespace uns
