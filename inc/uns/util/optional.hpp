#pragma once

#include <uns/util/storage.hpp>

#include <algorithm>

namespace uns {
template <typename T>
class optional {
public:
    optional(const T& _value)
        : has_value_(true) {
        this->data.construct(_value);
    }

    optional(T&& _value)
        : has_value_(true) {
        this->data.construct(std::move(_value));
    }

    template<typename ...Args>
    optional(void*, Args&&... args)
        : has_value_(true) {
        this->data.emplace(std::forward<Args>(args)...);
    }

    optional(const optional& other)
        : has_value_(other.has_value_) {
        this->data.construct(other.value());
    }

    optional(optional&& other)
        : has_value_(other.has_value_) {
        this->data.construct(std::move(other.value()));
    }

    optional() : has_value_(false) {}

    void operator=(const T& _value) {
        this->data.construct(_value);
        this->has_value_ = true;
    }

    void operator=(T&& _value) {
        this->data.construct(std::move(_value));
        this->has_value_ = true;
    }

    void operator=(const optional& other) {
        this->data.construct(other.value());
        this->has_value_ = other.has_value_;
    }

    void operator=(optional&& other) {
        this->data.construct(std::move(other.value()));
        this->has_value_ = other.has_value_;
        other.has_value_ = false;
    }

    const T& value() const { return this->data.value(); }

    T& value() { return this->data.value(); }

    const T& operator*() const { return this->value(); }

    T& operator*() { return this->value(); }

    const T* operator->() const { return this->data.value_ptr(); }

    T* operator->() { return this->data.value_ptr(); }

    bool has_value() const { return this->has_value_; }

    bool operator==(const T& _value) const {
        return this->has_value_ && *this == _value;
    }

    bool operator==(const optional& other) const {
        return this->has_value_ && other.has_value_ && *this == *other;
    }

    void reset() {
        if (this->has_value_) this->data.destruct();
        this->has_value_ = false;
    }

    ~optional() {
        this->reset();
    }

private:
    storage_t<T> data;
    bool has_value_;
};

template<typename T, typename ...Args>
optional<T> make_optional(Args&&... args) {
    return optional<T>(nullptr, std::forward<Args>(args)...);
}

} // namespace uns
