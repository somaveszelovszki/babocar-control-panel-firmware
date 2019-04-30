#pragma once

#include <uns/util/types.hpp>

namespace uns {

/* @brief Performs test-and-set.
 * @note This function must be called from a task!
 * @param pValue Pointer to the value to test and set.
 * @param valueToSet The new value to set.
 * @returns The previous value of the variable.
 **/
bool testAndSet(bool *pValue, bool valueToSet);

/* @brief Performs test-and-set.
 * @note This function must be called from an interrupt service routine!
 * @param pValue Pointer to the value to test and set.
 * @param valueToSet The new value to set.
 * @returns The previous value of the variable.
 **/
bool testAndSet_ISR(bool *pValue, bool valueToSet);

/* @brief Helper class for concurrent getting and setting of large data set.
 * Usage:   1. call set() to get setter pointer to underlying data set
 *          2. set data under pointer
 *          3. call swap() to exchange pointers above underlying data sets
 *          4. call get() to get getter pointer to underlying data set (this will point to the data set updated in 2.)
 */
class SwapExchange {
public:
    /* @brief Constructor - initializes getter and setter pointers.
     * @param value1 Pointer to the first underlying data set
     * @param value2 Pointer to the second underlying data set.
     * @note The life cycles of the underlying data sets should be the same as the SwapExchange object in order to prevent segmentation fault!
     */
    SwapExchange(void * const value1, void * const value2)
        : value_GET(value1)
        , value_SET(value2) {}

    /* @brief Gets getter pointer.
     * @returns The getter pointer.
     */
    const void* get() const { return this->value_GET; }

    /* @brief Gets setter pointer.
     * @returns The setter pointer.
     */
    void* set() { return this->value_SET; }

    /* @brief Swaps getter and setter pointers.
     * @note This function solves concurrency problems by swapping pointers in a critical section.
     */
    void swap();
private:
    void *value_GET, *value_SET;    // Getter and setter pointers to the underlying data sets.
};

template <typename T>
class optional {
public:
    optional(const T& _value)
        : value_(_value)
        , hasValue_(true) {}

    optional() : hasValue_(false) {}

    void operator=(const T& _value) {
        this->value_ = _value;
        this->hasValue_ = true;
    }

    const T& value() const { return this->value_; }

    T& value() { return this->value_; }

    const T& operator*() const { return this->value_; }

    T& operator*() { return this->value_; }

    const T* operator->() const { return &this->value_; }

    T* operator->() { return &this->value_; }

    bool hasValue() const { return this->hasValue_; }

private:
    T value_;
    bool hasValue_;
};

} // namespace uns
