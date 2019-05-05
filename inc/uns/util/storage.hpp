#pragma once

#include <uns/util/types.hpp>

#include <algorithm>

namespace uns {
template <typename T>
class storage_t {
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
