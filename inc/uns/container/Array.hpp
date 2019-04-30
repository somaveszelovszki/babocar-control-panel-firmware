#pragma once

#include <uns/util/arrays.hpp>

namespace uns {

/* @brief Stores array of given size and provides helper functions to manipulate elements.
 * @tparam T The type of the stored elements.
 * @tparam size The size of the array.
 **/
template <typename T, uint32_t size>
class Array {

private:
    T data[size];    // Array of the elements.

    /* @brief Copies data from source array.
     * @param values The source array.
     **/
    void setValues(const T values[]) {
        uns::copy<size>(values, this->data);
    }

public:

    /* @brief Default constructor - does not initialize array values.
     **/
    Array() {}

    /* @brief Constructor - copies elements.
     * @param values The source array.
     **/
    explicit Array(const T values[]) {
        this->setValues(values);
    }

    /* @brief Copy constructor - copies elements.
     * @param other The other array.
     **/
    Array(const Array<T, size>& other) {
        this->setValues(other.data);
    }

    /* @brief Copies data from the other array.
     * @param other The other array.
     * @returns This array.
     **/
    Array<T, size>& operator=(const Array<T, size>& other) {
        this->setValues(other.data);
        return *this;
    }

    /* @brief Casts Array object to a C/C++ array.
     * @returns The Array object as a C/C++ array.
     **/
    explicit operator T*() {
        return this->data;
    }

    /* @brief Casts Array object to a C/C++ const array.
     * @returns The Array object as a C/C++ const array.
     **/
    explicit operator const T*() const {
        return this->data;
    }

    /* @brief Gets element at given position.
     * @returns The element at the given position.
     **/
    T& operator[](uint32_t pos) {
        return this->data[pos];
    }

    /* @brief Gets element at given position.
     * @returns The element at the given position.
     **/
    const T& operator[](uint32_t pos) const {
        return this->data[pos];
    }

    /* @brief Checks if two Array objects have equal values.
     * @param other The other Array object.
     * @returns Boolean data indicating if the two Array objects have equal values.
     **/
    bool operator==(const Array<T, size>& other) const {
        bool eq = true;
        for (uint32_t i = 0; eq && i < size; ++i)
            eq = (*this)[i] == other[i];
        return eq;
    }

    /* @brief Checks if two Array objects do not have equal values.
     * @param other The other Array object.
     * @returns Boolean data indicating if the two Array objects do not have equal values.
     **/
    bool operator!=(const Array<T, size>& other) const {
        return !(*this == other);
    }

    /* @brief Concatenates two Array objects.
     * @tparam size2 Size of the other Array object.
     * @param other The other Array object.
     * @param dest The destination Array object.
     **/
    template <uint32_t size2>
    void concat(const Array<T, size2>& other, Array<T, size + size2>& dest) const {
        uns::concat<size, size2>(static_cast<const T*>(*this), static_cast<const T*>(other), static_cast<T*>(dest));
    }

    /* @brief Gets subarray of given length starting at given position.
     * @tparam size2 Size of the subarray.
     * @param pos Start index of subarray.
     * @returns The subarray.
     **/
    template <uint32_t size2>
    Array<T, size2> subArray(uint32_t pos) {
        return Array<T, size2>(this->data + pos);
    }

    /* @brief Appends element at the end of this array, and stores the result in the destination Array object.
     * @tparam c The element to append.
     * @param dest The destination Array object.
     **/
    void append(const T& c, Array<T, size + 1>& dest) const {
        uns::copy<size>(this->data, static_cast<T*>(dest));
        dest[size] = c;
    }

    /* @brief Concats two Array objects.
     * @tparam size2 Size of the other Array object.
     * @param other The other Array object.
     * @returns The result Array object.
     **/
    template <uint32_t size2>
    Array<T, size + size2> operator+(const Array<T, size2>& other) const {
        Array<T, size + size2> result;
        concat(other, result);
        return result;
    }

    /* @brief Appends element at the end of this array, and returns the result Array object.
     * @tparam c The element to append.
     * @returns The result Array object.
     **/
    Array<T, size + 1> operator+(const T& c) const {
        Array<T, size + 1> result;
        this->append(c, result);
        return result;
    }

    /* @brief Appends element at the beginning of the byte array, and returns the result Array object.
     * @tparam c The element to append.
     * @param bytes The source Array object.
     * @returns The result Array object.
     **/
    friend Array<T, 1 + size> operator+(const T& c, const Array<T, size>& bytes) {
        return Array<T, 1>(&c) + bytes;
    }
};
} // namespace uns