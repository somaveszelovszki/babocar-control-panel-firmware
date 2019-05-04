#pragma once

#include <uns/util/types.hpp>

#include <algorithm>

namespace uns {
template <typename T>
class storage_t {
public:
    void construct(const T& _value) {
        new(&buffer) T(_value);
    }

    void construct(T&& _value) {
        new(&buffer) T(std::move(_value));
    }

    template<typename ...Args>
    void emplace(Args&&... args) {
        new(&buffer) T(std::forward<Args>(args)...);
    }

    const T& value_ptr() const {
        return reinterpret_cast<const T*>(&this->buffer);
    }

    T& value_ptr() {
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

} // namespace uns
