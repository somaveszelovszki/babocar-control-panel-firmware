#pragma once

#include <uns/container/map.hpp>
#include <uns/util/atomic.hpp>
#include <uns/util/typeinfo.hpp>

#include <cstring>

namespace uns {

enum class DebugCode : uint8_t {
    Log   = 1,
    // add debug codes (DebugCode must be < 128, from 128 it means Param)
    Param = 128
};

struct Param {

    Param(const char *name, const char *type, mutex_handle_t *hmutex, uint8_t *buf, uint8_t size)
        : name("")
        , type("")
        , hmutex(hmutex)
        , buf(buf)
        , size(size) {
        strncpy(const_cast<char*>(this->name), name, STR_MAX_LEN_GLOBAL_NAME);
        strncpy(const_cast<char*>(this->type), type, STR_MAX_LEN_GLOBAL_TYPE);
    }

    Param(const Param& other)
        : name("")
        , type("")
        , hmutex(other.hmutex)
        , buf(other.buf)
        , size(other.size) {
        strncpy(const_cast<char*>(this->name), other.name, STR_MAX_LEN_GLOBAL_NAME);
        strncpy(const_cast<char*>(this->type), other.type, STR_MAX_LEN_GLOBAL_TYPE);
    }

    const char name[STR_MAX_LEN_GLOBAL_NAME];
    const char type[STR_MAX_LEN_GLOBAL_TYPE];
    mutex_handle_t *hmutex;
    uint8_t * const buf;
    const uint8_t size;
};

class Params {

public:

    template <typename T>
    void registerParam(const char *name, T *value) {
        static_assert(std::is_trivially_copyable<T>::value, "Type must be trivially copyable.");
        static uint8_t id = static_cast<uint8_t>(DebugCode::Param);

        this->values.put(id++, this->fillParamStruct(name, value));
    }

    void updateParam(uint8_t id, const uint8_t *buf, uint8_t size);

private:

    template <typename T>
    Param fillParamStruct(const char *name, T *value) {
        return Param(name, uns::typeinfo<T>::name(), nullptr, reinterpret_cast<uint8_t*>(value), sizeof(T));
    }

    template <typename T>
    Param fillParamStruct(const char *name, atomic<T> *value) {
        T *value_ptr = const_cast<T*>(value->wait_ptr());
        value->release_ptr();
        return Param(name, uns::typeinfo<T>::name(), value->getMutex(), reinterpret_cast<uint8_t*>(value_ptr), sizeof(T));
    }

    map<uint8_t, Param, MAX_NUM_GLOBAL_PARAMS> values;
};

} // namespace uns
