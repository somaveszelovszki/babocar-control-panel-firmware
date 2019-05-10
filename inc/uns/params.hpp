#pragma once

#include <uns/container/map.hpp>
#include <uns/util/atomic.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/typeinfo.hpp>

#include <cstring>

namespace uns {

enum class DebugCode : uint8_t {
    Log   = 1,
    // add debug codes (DebugCode must be < 128, from 128 it means Param)
    Param = 128
};

struct Param {
    const char name[STR_MAX_LEN_GLOBAL_NAME];
    const char type[STR_MAX_LEN_GLOBAL_TYPE];
    uint8_t * const buf;
    const uint8_t size;
};

class Params {
    template <typename T>
    void registerParam(const char *name, T *value) {

        static_assert(std::is_trivially_copyable<T>::value, "Type must be trivially copyable.");
        static uint8_t id = static_cast<uint8_t>(DebugCode::Param);

        Param p;
        strncpy(const_cast<char*>(p.name), name, STR_MAX_LEN_GLOBAL_NAME);
        //strncpy(const_cast<char*>(p.type), uns::typeinfo<T>::name(), STR_MAX_LEN_GLOBAL_TYPE);
        p.buf = reinterpret_cast<uint8_t*>(value);
        p.size = sizeof(T);
        this->values.put(id++, p);
    }

    void updateParam(uint8_t id, const uint8_t *buf, uint8_t size) {
        Param *param = this->values.get(id);
        if (param) {
            if (param->size == size) {
                memcpy(param->buf, buf, size);
            } else {
                LOG_ERROR_WITH_STATUS(Status::INVALID_DATA, "Invalid Param buffer size: %d (for id: %d)", static_cast<int32_t>(size), static_cast<int32_t>(id));
            }
        } else {
            LOG_ERROR_WITH_STATUS(Status::INVALID_ID, "Invalid Param id: %d", static_cast<int32_t>(id));
        }
    }
private:
    map<uint8_t, Param, MAX_NUM_GLOBAL_PARAMS> values;
};

} // namespace uns
