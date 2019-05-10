#pragma once

#include <uns/util/arrays.hpp>
#include <uns/util/numeric.hpp>

namespace uns {

/* @brief Ring buffer implementation.
 * @note This class is not concurrent.
 * @tparam T Type of the stored elements.
 * @tparam capacity The capacity of the buffer. Usable capacity is (capacity - 1), because the plus one element is used to determine if the buffer is empty or full.
 **/
template <typename T, uint32_t capacity>
class RingBuffer {
public:
    T data[capacity];           // The buffer.
    volatile uint32_t head;     // The head - writing starts from this point.
    volatile uint32_t tail;     // The tail - reading starts from this point.

    static Status toStatus(bool result) {
        Status status = result ? Status::OK : Status::BUFFER_FULL;
        return status;
    }

public:
    /* @brief Default constructor - initializes head and tail indexes.
     **/
    RingBuffer()
        : head(0)
        , tail(0) {}

    /* @brief Gets writable buffer pointer.
     * @note Do not write to any elements other than this one!
     * @returns Pointer to the writable buffer.
     **/
    T* getWritableBuffer() {
        return this->data + this->head;
    }

    /* @brief Gets readable buffer pointer.
     * @returns Pointer to the readable buffer.
     **/
    const T* getReadableBuffer() const {
        return this->data + this->tail;
    }

    /* @brief Updates head position.
     * @param _count The number of elements that have been read.
     **/
    void updateHead(uint32_t _count) {
        this->head = (this->head + _count) % capacity;
    }

    /* @brief Updates tail position.
     * @param _count The number of elements that have been written.
     **/
    void updateTail(uint32_t _count) {
        this->tail = (this->tail + _count) % capacity;
    }

    /* @brief Puts new element into the buffer.
     * @param value The element to add.
     * @returns Status indicating if element has been added.
     **/
    Status put(const T& value);

    /* @brief Puts new elements into the buffer.
     * @note Either all or none of the elements will be added, depending on the buffer's free space.
     * @param values The array of elements to add.
     * @param _count The number of elements to add.
     * @returns Status indicating if element has been added.
     **/
    Status put(const T * const values, uint32_t _count);

    /* @brief Gets oldest element from the buffer.
     * @param pDest Pointer to the object that will store the result.
     * @returns Status indicating if element has been read.
     **/
    Status get(T *pDest);

    Status check(T *pDest) {
        bool _get = this->head != this->tail;
        if (_get) {
            *pDest = this->data[this->tail];
        }
        return toStatus(_get);
    }

    /* @brief Gets oldest elements from the buffer.
     * @note Either all or none of the elements will be read, depending on the given count parameter.
     * @param buffer Buffer that will store the result elements.
     * @param _count The number of elements to get.
     * @returns Status indicating if element has been read.
     **/
    Status get(T *buffer, uint32_t _count);

    /* @brief Gets size of the buffer.
     * @returns The current number of elements stored in the buffer.
     **/
    uint32_t size() const {
        uint32_t _size = this->head >= this->tail ? this->head - this->tail : capacity + this->head - this->tail;
        return _size;
    }

    /* @brief Gets size of the buffer that is safe to read without addressing out of the buffer (may not be equal to the buffer size).
     * @returns The buffer's safe size.
     **/
    uint32_t safeReadSize() const {
        uint32_t safe_size = min(this->size(), capacity - this->tail);
        return safe_size;
    }

    /* @brief Gets index of given value in buffer.
     * @param value The value to search for.
     * @param start The start position (relative to the tail).
     * @returns Index of the value or -1 if not found.
     **/
    int32_t indexOf(const T& value, uint32_t start = 0);
};

template <typename T, uint32_t capacity>
Status RingBuffer<T, capacity>::put(const T& value) {
    bool _put = this->size() + 1 < capacity;
    if (_put) {
        this->data[this->head] = value;
        this->updateHead(1);
    }

    Status result = _put ? Status::OK : Status::BUFFER_FULL;
    return result;
}

template <typename T, uint32_t capacity>
Status RingBuffer<T, capacity>::put(const T *const values, uint32_t _count) {
    bool _put = this->size() + _count < capacity;

    if (_put) {
        uint32_t sizeAfterHead = min(_count, capacity - this->head),
            sizeBeforeTail = _count > sizeAfterHead ? _count - sizeAfterHead : 0;

        uns::copy(values, this->data + this->head, sizeAfterHead);
        if (sizeBeforeTail > 0) {
            uns::copy(values + sizeAfterHead, this->data, sizeBeforeTail);
        }

        this->updateHead(_count);
    }

    Status result = _put ? Status::OK : Status::BUFFER_FULL;
    return result;
}

template <typename T, uint32_t capacity>
Status RingBuffer<T, capacity>::get(T *pDest) {
    bool _get = !!this->size();
    if (_get) {
        *pDest = this->data[tail];
        this->updateTail(1);
    }

    Status result = _get ? Status::OK : Status::NO_NEW_DATA;
    return result;
}

template <typename T, uint32_t capacity>
Status RingBuffer<T, capacity>::get(T * const buffer, uint32_t _count) {
    bool _get = this->size() >= _count;
    if (_get) {
        uint32_t sizeAfterTail = min(_count, capacity - this->tail),
            sizeBeforeHead = _count > sizeAfterTail ? _count - sizeAfterTail : 0;

        uns::copy(this->data + this->tail, buffer, sizeAfterTail);
        if (sizeBeforeHead > 0) {
            uns::copy(this->data, buffer, sizeBeforeHead);
        }

        this->updateTail(_count);
    }

    Status result = _get ? Status::OK : Status::NO_NEW_DATA;
    return result;
}

template<typename T, uint32_t capacity>
int32_t RingBuffer<T, capacity>::indexOf(const T& value, uint32_t start) {
    uint32_t i;
    int32_t idx = -1;

    // searches after tail
    uint32_t sizeAfterTail = this->safeReadSize();
    for (i = start; i < sizeAfterTail; ++i) {
        if (this->data[this->tail + i] == value) {
            idx = i;
            break;
        }
    }

    // if not found, continues search before head
    if (idx == -1) {
        uint32_t sizeBeforeHead = this->size() - sizeAfterTail;
        for (i = 0; i < sizeBeforeHead; ++i) {
            if (this->data[i] == value) {
                idx = sizeAfterTail + i;
                break;
            }
        }
    }

    return idx;
}
} // namespace uns
