#pragma once

#include <uns/util/types.hpp>
#include <uns/bsp/it.hpp>

#include <algorithm>
#include <type_traits>

namespace uns {

template <typename T>
class swap_xchg {
public:
    /* @brief Constructor - initializes getter and setter pointers.
     * @param value1 Pointer to the first underlying data set
     * @param value2 Pointer to the second underlying data set.
     * @note The life cycles of the underlying data sets should be the same as the SwapExchange object in order to prevent segmentation fault!
     */
    swap_xchg(T *value1, T *value2)
        : value_GET(value1)
        , value_SET(value2) {}

    /* @brief Gets getter pointer.
     * @returns The getter pointer.
     */
    const T* get() const { return this->value_GET; }

    /* @brief Gets setter pointer.
     * @returns The setter pointer.
     */
    T* set() { return this->value_SET; }

    /* @brief Swaps getter and setter pointers.
     * @note This function solves concurrency problems by swapping pointers in a critical section.
     */
    void swap() {
        uns::enterCritical();
        std::swap(this->value_GET, this->value_SET);
        uns::exitCritical();
    }

private:
    T *value_GET, *value_SET;    // Getter and setter pointers to the underlying data sets.
};

template <typename T>
class optional {
public:
    optional(const T& value_)
        : hasValue_(true) {
        this->emplace(value_);
    }

    optional(T&& value_)
        : hasValue_(true) {
        this->emplace(std::move(value_));
    }

    template<typename ...Args>
    optional(void*, Args&&... args)
        : hasValue_(true) {
        this->emplace(std::forward<Args>(args)...);
    }

    optional(const optional& other)
        : hasValue_(other.hasValue_) {
        this->emplace(other.value());
    }

    optional(optional&& other)
        : hasValue_(other.hasValue_) {
        this->emplace(std::move(other.value()));
    }

    optional() : hasValue_(false) {}

    void operator=(const T& value_) {
        this->emplace(value_);
        this->hasValue_ = true;
    }

    void operator=(T&& value_) {
        this->emplace(std::move(value_));
        this->hasValue_ = true;
    }

    void operator=(const optional& other) {
        this->emplace(other.value());
        this->hasValue_ = other.hasValue_;
    }

    void operator=(optional&& other) {
        this->emplace(std::move(other.value()));
        this->hasValue_ = other.hasValue_;
        other.hasValue_ = false;
    }

    const T& value() const { return *reinterpret_cast<const T*>(&this->buffer); }

    T& value() { return *reinterpret_cast<T*>(&this->buffer); }

    const T& operator*() const { return this->value(); }

    T& operator*() { return this->value(); }

    const T* operator->() const { return reinterpret_cast<const T*>(&this->buffer); }

    T* operator->() { return reinterpret_cast<T*>(&this->buffer); }

    bool hasValue() const { return this->hasValue_; }

    bool operator==(const T& value_) const {
        return this->hasValue_ && *this == value_;
    }

    bool operator==(const optional& other) const {
        return this->hasValue_ && other.hasValue_ && *this == *other;
    }

    void reset() {
        if (this->hasValue_) this->value().~T();
        this->hasValue_ = false;
    }

    ~optional() {
        this->reset();
    }

private:

    void emplace(const T& value_) {
        new(&buffer) T(value_);
    }

    void emplace(T&& value_) {
        new(&buffer) T(std::move(value_));
    }

    template<typename ...Args>
    void emplace(Args&&... args) {
        new(&buffer) T(std::forward<Args>(args)...);
    }

    typename std::aligned_storage<sizeof(T), alignof(T)>::type buffer;
    bool hasValue_;
};

template<typename T, typename ...Args>
optional<T> make_optional(Args&&... args) {
    return optional<T>(nullptr, std::forward<Args>(args)...);
}

} // namespace uns
