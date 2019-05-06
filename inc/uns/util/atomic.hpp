#pragma once

#include <uns/util/storage.hpp>
#include <uns/bsp/mutex.hpp>

#include <algorithm>
#include <type_traits>

namespace uns {

template <typename T>
class atomic {
public:
    template<typename ...Args>
    atomic(mutex_handle_t *_hmutex, Args&&... args)
        : hmutex(_hmutex) {
        this->data.emplace(std::forward<Args>(args)...);
    }

    void operator=(const T& _value) volatile {
        this->data.construct(_value);
    }

    void operator=(T&& _value) volatile {
        this->data.construct(std::move(_value));
    }

    void wait_copy(T& result) const volatile {
        while (!isOk(uns::mutexTake(this->hmutex, millisecond_t(1)))) {}
        result = *const_cast<T*>(this->data.value_ptr());
        uns::mutexRelease(this->hmutex);
    }

    void wait_set(const T& value) volatile {
        while (!isOk(uns::mutexTake(this->hmutex, millisecond_t(1)))) {}
        this->data.construct(value);
        uns::mutexRelease(this->hmutex);
    }

    volatile T* wait_ptr() volatile {
        while (!isOk(uns::mutexTake(this->hmutex, millisecond_t(1)))) {}
        return this->data.value_ptr();
    }

    volatile T* accept_ptr() volatile {
        return isOk(uns::mutexTake_ISR(this->hmutex)) ? this->data.value_ptr() : nullptr;
    }

    void release_ptr() volatile {
        uns::mutexRelease(this->hmutex);
    }

private:
    mutex_handle_t * const hmutex;
    storage_t<T> data;
};

} // namespace uns
