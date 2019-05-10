#pragma once

#include <uns/util/types.hpp>

#include <algorithm>

namespace uns {
template <typename T>
class storage_t {
public:
    void construct(const T& _value) {
        new((void*)&buffer) T(_value);
    }

    void construct(T&& _value) {
        new((void*)&buffer) T(std::move(_value));
    }

    template<typename ...Args>
    void emplace(Args&&... args) {
        new((void*)&buffer) T(std::forward<Args>(args)...);
    }

    const T* value_ptr() const {
        return reinterpret_cast<const T*>(&this->buffer);
    }

    T* value_ptr() {
        return reinterpret_cast<T*>(&this->buffer);
    }

    void destruct() {
        this->value().~T();
    }

    const T& value() const {
        return *this->value_ptr();
    }

    T& value() {
        return *this->value_ptr();
    }

private:
    typename std::aligned_storage<sizeof(T), alignof(T)>::type buffer;
};

template <typename T>
class volatile_storage_t {
public:
    void construct(const T& _value) volatile {
        new((void*)&buffer) T(_value);
    }

    void construct(T&& _value) volatile {
        new((void*)&buffer) T(std::move(_value));
    }

    template<typename ...Args>
    void emplace(Args&&... args) volatile {
        new((void*)&buffer) T(std::forward<Args>(args)...);
    }

    const volatile T* value_ptr() volatile const {
        return reinterpret_cast<const volatile T*>(&this->buffer);
    }

    volatile T* value_ptr() volatile {
        return reinterpret_cast<volatile T*>(&this->buffer);
    }

    void destruct() volatile {
        this->value().~T();
    }

    const volatile T& value() const volatile {
        return *this->value_ptr();
    }

    volatile T& value() volatile {
        return *this->value_ptr();
    }

private:
    typename std::aligned_storage<sizeof(T), alignof(T)>::type buffer;
};

} // namespace uns
