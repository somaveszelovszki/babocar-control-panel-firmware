#pragma once

#include <uns/util/types.hpp>

namespace uns {

/* @brief Selects mask for given container size.
 * @tparam B Container bit size.
 **/
template <uint32_t B>
struct ContainerSelector;

/* @brief Selects mask array for bit length of 8 (uint8_t container).
 **/
template <>
struct ContainerSelector<8> {
    typedef uint8_t type;
    static const type masks[8];     // Masks used for reading/writing elements.
};

/* @brief Selects mask array for bit length of 32 (uint32_t container).
 **/
template <>
struct ContainerSelector<32> {
    typedef uint32_t type;
    static const type masks[32];    // Masks used for reading/writing elements.
};

template <uint32_t B>
class BitContainer {
public:
    typedef typename ContainerSelector<B>::type type;   // The underlying data type.

private:
    type value;     // The container value.
    static const type * const masks;    // Masks used for reading/writing elements.

public:
    /* @brief Casts bit container to its container type.
     * @returns The container value.
     **/
    operator type() const {
        return this->value;
    }

    /* @brief Gets given bit of the container.
     * @param pos The position.
     * @returns The bit value at the given position.
     **/
    bool get(uint32_t pos) const {
        return static_cast<bool>(this->value & ContainerSelector<B>::masks[pos]);
    }

    /* @brief Sets given bit of the container.
     * @param pos The position.
     * @param value The bit value to write.
     **/
    void set(uint32_t pos, bool value) {
        if (value) {
            this->value |= ContainerSelector<B>::masks[pos];
        } else {
            this->value &= ~ContainerSelector<B>::masks[pos];
        }
    }

    /* @brief Resets all bits.
     **/
    void reset() {
        this->value = static_cast<type>(0);
    }
};

template <uint32_t B>
const typename BitContainer<B>::type * const BitContainer<B>::masks = ContainerSelector<B>::masks;

} // namespace uns
