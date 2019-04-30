#pragma once

#include <uns/util/types.hpp>
#include <algorithm>

namespace uns {
/* @brief Copies elements of the source array into the destination array.
 * @tparam size Size of the arrays.
 * @tparam T Type of the elements of the arrays.
 * @param src The source array.
 * @param dest The destination array.
 **/
template <uint32_t size, typename T>
void copy(const T * const src, T * const dest) {
    for (uint32_t i = 0; i < size; ++i) {
        dest[i] = src[i];
    }
}

/* @brief Copies elements of the source array into the destination array.
 * @tparam T Type of the elements of the arrays.
 * @param src The source array.
 * @param dest The destination array.
 * @param size Size of the arrays.
 **/
template <typename T>
void copy(const T * const src, T * const dest, uint32_t size) {
    for (uint32_t i = 0; i < size; ++i) {
        dest[i] = src[i];
    }
}

/* @brief If the array contains the item, return its index, otherwise returns -1.
 * @tparam T Type of the item and the elements of the array.
 * @param item The item to look for.
 * @param ar The array in which the check should be performed.
 * @param arraySize The size of the array.
 * @param startIdx The index where search should start. 0 by default.
 * @returns The index of the item or -1 if the array does not contain the item.
 **/
template <typename T>
int32_t indexOf(const T& item, const T * const ar, uint32_t arraySize, uint32_t startIdx = 0){
    int32_t idx = -1;
    for (int32_t i = startIdx; idx == -1 && i < static_cast<int32_t>(arraySize); ++i) {
        if (ar[i] == item) {
            idx = i;
        }
    }
    return idx;
}

/* @brief Checks if the array contains the given item.
 * @tparam T Type of the item and the elements of the array.
 * @param item The item to look for.
 * @param ar The array in which the check should be performed.
 * @param arraySize The size of the array.
 * @returns Boolean value indicating if the array contains the item.
 **/
template <typename T>
bool contains(const T& item, const T * const ar, uint32_t arraySize) {
    return indexOf(item, ar, arraySize) != -1;
}

/* @brief Concatenates two arrays.
 * @tparam size1 Size of the first array.
 * @tparam size2 Size of the second array.
 * @tparam T Type of the arrays.
 * @param ar1 The first array.
 * @param ar2 The second array.
 * @param res The result array.
 **/
template <uint32_t size1, uint32_t size2, typename T>
void concat(const T * const ar1, const T * const ar2, T * const res) {
    copy<size1>(ar1, res);
    copy<size2>(ar2, &res[size1]);
}

/* @brief Reverses array.
 * @tparam T Type of the array.
 * @param The array.
 **/
template <typename T>
void reverse(T * const ar, uint32_t size) {
    for (uint32_t i = 0; i < size / 2; ++i) {
        std::swap(ar[i], ar[size - i - 1]);
    }
}
} // namespace uns
