#pragma once

#include <uns/util/types.hpp>

namespace uns {

template <typename T>
class span {
public:
    span(T *data_, uint32_t size_)
        : data_(data_)
        , size_(size_) {}
    
    const T* data() const {
        return this->data_;
    }

    T* data() {
        return this->data_;
    }

    uint32_t size() const {
        return this->size_;
    }

private:
    T * const data_;
    const uint32_t size_;
};
} // namespace uns
