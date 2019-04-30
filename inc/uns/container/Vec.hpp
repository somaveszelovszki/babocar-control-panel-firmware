#pragma once

#include <uns/container/Array.hpp>
#include <uns/util/numeric.hpp>

namespace uns {

template <typename T, uint32_t capacity>
class Vec {
private:
    T data[capacity];

public:
    typedef T* iterator;
    typedef const T* const_iterator;

    uint32_t size;

    T& operator[](uint32_t pos) {
        return this->data[pos];
    }

    const T& operator[](uint32_t pos) const {
        return this->data[pos];
    }

    /* @brief Default constructor - Sets size to 0.
     **/
    Vec() : size(0) {}

    /* @brief Copy constructor - copies elements.
     * @param other The other vector.
     **/
    Vec(const Vec<T, capacity>& other)
        : data(other.data)
        , size(other.size) {}

    /* @brief Copies data from the other vector.
     * @param other The other vector.
     * @returns This Vec.
     **/
    Vec<T, capacity>& operator=(const Vec<T, capacity>& other) {
        uns::copy(other.data, this->data, (this->size = other.size));
        return *this;
    }

    /* @brief Casts vector to a C/C++ array.
     * @returns The vector as a C/C++ array.
     **/
    explicit operator T * const () {
        return static_cast<T * const>(this->data);
    }

    /* @brief Casts vector to a C/C++ array.
     * @returns The vector as a C/C++ array.
     **/
    explicit operator const T * const () const {
        return static_cast<const T * const>(this->data);
    }

    /* @brief Appends elements to the end of the vector.
     * @param _data The elements to append.
     * @param _size Number of elements to append.
     * @returns Number of appended elements. Less than _size if vector capacity is reached.
     **/
    uint32_t append(const T * const _data, uint32_t _size) {
        uint32_t num = uns::min(_size, capacity - this->size);
        for (uint32_t i = 0; i < num; ++i) {
            this->operator[](this->size++) = _data[i];
        }
        return num;
    }

    /* @brief Appends elements to the end of the vector.
     * @param vec The elements to append.
     * @returns Number of appended elements. Less than vec.size if vector capacity is reached.
     **/
    template <uint32_t capacity2>
    uint32_t append(const Vec<T, capacity2>& vec) {
        return this->append(vec.data, vec.size);
    }

    /* @brief Appends one element to the end of the vector.
     * @param item The element to append.
     * @returns Number of appended elements. 1 if element has been appended successfully, 0 otherwise.
     **/
    uint32_t append(const T& item) {
        return this->append(&item, 1);
    }

    void remove(uint32_t pos) {
        for (uint32_t i = pos; i < this->size - 1; ++i) {
           this->data[i] = this->data[i + 1];
        }
        this->size--;
    }

    bool remove(iterator item) {
        bool found = item >= this->begin() && item < this->end();
        if (found) {
            for(iterator it = item; it < this->end() - 1; ++it) {
                *it = *(it + 1);
            }
            this->size--;
        }
        return found;
    }

    bool remove(const T& item) {
        bool found = false;
        for(uint32_t i = 0; i < this->size; ++i) {
            if (this->data[i] == item) {
                found = true;
            }
            if (found && i < this->size - 1) {  // shifts elements after removed item
                this->data[i] = this->data[i + 1];
            }
        }
        if (found) {
            this->size--;
        }
        return found;
    }

    const_iterator find(const T& item) const {
        const_iterator it = this->begin();
        for (; it < this->end(); ++it) {
            if (*it == item) {
                break;
            }
        }
        return it;
    }

    /* @brief Clears vector.
     **/
    void clear() {
        this->size = 0;
    }

    iterator begin() {
        return this->data;
    }

    const_iterator begin() const {
        return this->data;
    }

    iterator end() {
        return &this->data[this->size];
    }

    const_iterator end() const {
        return &this->data[this->size];
    }
};
} // namespace uns
