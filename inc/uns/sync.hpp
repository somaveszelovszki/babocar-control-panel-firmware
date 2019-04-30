#pragma once

#include <uns/util/units.hpp>
#include "cmsis_os.h"


namespace uns {

template <typename T>
class sync {
private:
    T value;
    bool updated;
    osMutexId mutexId;

public:
    explicit sync(osMutexId _mutexId)
        : mutexId(_mutexId)
        , updated(false) {}

    void set(const T& _value) {
        osMutexWait(this->mutexId, osWaitForever);
        this->value = _value;
        this->updated = true;
        osMutexRelease(this->mutexId);
    }

    bool get(T *pResult) {
        osMutexWait(this->mutexId, osWaitForever);
        *pResult = this->value;
        bool _updated = this->updated;
        this->updated = false;
        osMutexRelease(this->mutexId);
        return _updated;
    }
};
} // namespace uns