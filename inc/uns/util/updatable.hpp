#pragma once

#include <uns/util/atomic.hpp>

namespace uns {

template <typename T>
class atomic_updatable {
public:
    template<typename ...Args>
    atomic_updatable(mutex_handle_t *_hmutex, Args&&... args)
        : value_(_hmutex, std::forward<Args>(args)...)
        , updated_(false) {
    }

    void wait_copy(T& result) volatile {
        result = *const_cast<T*>(this->value_.wait_ptr());
        this->updated_ = false;
        this->value_.release_ptr();
    }

    volatile T* accept_ptr() volatile {
        return this->value_.accept_ptr();
    }

    void release_ptr() volatile {
        this->updated_ = true;
        this->value_.release_ptr();
    }

    bool is_updated() const volatile {
        return this->updated_;
    }

private:
    atomic<T> value_;
    bool updated_;
};
}  // namespace uns
