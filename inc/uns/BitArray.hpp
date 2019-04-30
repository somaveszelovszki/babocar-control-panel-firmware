#pragma once

#include <uns/BitContainer.hpp>
#include <uns/util/arrays.hpp>
#include <type_traits>

namespace uns {

/* @brief Array for storing binary values memory-efficiently. Values are stored in 32-bit containers to reduce memory need.
 * @tparam N Number of values stored in the array. Should be a multiple of 32 for the best efficiency.
 **/
template <uint32_t N>
class BitArray {

private:
    static constexpr uint32_t C = 32;                   // Container size.
    static constexpr uint32_t NC = (N + (C - 1)) / C;   // Number of containers.

    typedef BitContainer<C> bit_container_type;         // The bit container type.
    typedef typename bit_container_type::type type;     // The underlying data type.
    bit_container_type data[NC];                        // Array of containers storing the binary values.

public:
    /* @brief Casts bit container to an array of an integral type.
     * @returns The container array.
     **/
    template <typename T, class = typename std::enable_if<std::is_integral<T>::value>::type>
    operator T*() {
        return reinterpret_cast<T*>(this->data);
    }

    /* @brief Casts bit container to an array of an integral type.
     * @returns The container array.
     **/
    template <typename T, class = typename std::enable_if<std::is_integral<T>::value>::type>
    operator const T*() const {
        return reinterpret_cast<T*>(this->data);
    }

    /* @brief Gets given bit of the array.
     * @param pos The position.
     * @returns The given bit of the array.
     **/
    uint1_t get(uint32_t pos) const {
        return this->data[pos / C].read(pos % C);
    }

    /* @brief Sets given bit of the array.
     * @param pos The position.
     * @param value The bit value to write.
     **/
    void set(uint32_t pos, bool value) {
        this->data[pos / C].set(pos % C, value);
    }

    /* @brief Copies binary array to byte array.
     * @param The result byte array.
     **/
    void toBytes(uint8_t * const result) {
        uns::copy<NC>(static_cast<type*>(this->data), reinterpret_cast<type*>(result));
    }

    /* @brief Resets all bits.
     **/
    void reset() {
        for (uint32_t i = 0; i < NC; ++i) {
            this->data[i].reset();
        }
    }
};
} // namespace uns
