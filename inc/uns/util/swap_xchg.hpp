#pragma once

#include <uns/util/types.hpp>
#include <uns/bsp/it.hpp>

#include <algorithm>

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

} // namespace uns
