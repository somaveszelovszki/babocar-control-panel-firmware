#pragma once

#include <uns/util/storage.hpp>
#include <uns/bsp/mutex.hpp>

#include <algorithm>
#include <type_traits>

namespace uns {

template <typename T>
class atomic {
public:
    atomic(mutex_handle_t *_hmutex, const T& _value)
        : hmutex(_hmutex) {
        this->data.construct(_value);
    }

    atomic(mutex_handle_t *_hmutex, T&& _value)
        : hmutex(_hmutex) {
        this->data.construct(std::move(_value));
    }

    template<typename ...Args>
    atomic(mutex_handle_t *_hmutex, Args&&... args)
        : hmutex(_hmutex) {
        this->data.emplace(std::forward<Args>(args)...);
    }

    void operator=(const T& _value) {
        this->data.construct(_value);
    }

    void operator=(T&& _value) {
        this->data.construct(std::move(_value));
    }

    T get() const {
        storage_t<T> copy;
        while (!isOk(uns::mutexTake(this->hmutex, millisecond_t::ZERO()))) {}
        copy.construct(this->data.value());
        uns::mutexRelease(this->hmutex);
        return copy.value();
    }

    void set(const T& _value) {
        while (!isOk(uns::mutexTake(this->hmutex, millisecond_t::ZERO()))) {}
        this->data.construct(_value);
        uns::mutexRelease(this->hmutex);
    }

    T* get_ptr_nonsafe() {
        return this->data.value_ptr();
    }

private:
    mutex_handle_t * const hmutex;
    storage_t<T> data;
};

} // namespace uns
