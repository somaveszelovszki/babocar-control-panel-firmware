#pragma once

#include <uns/BitArray.hpp>

namespace uns {

/* @brief Grid (2-dimensional array) for storing binary values memory-efficiently. Values are stored in 32-bit containers to reduce memory need.
 * @tparam X Number of binary values stored in a row. Should be a multiple of 32 for the best efficiency.
 * @tparam Y Number of rows.
 **/
template <uint32_t X, uint32_t Y>
class BitGrid {
private:
    BitArray<X> data[Y];   // Array of arrays storing the binary containers.

public:
    /* @brief Default constructor - does not initialize elements.
     **/
    BitGrid() {}

    /* @brief Sets given bit of the grid.
     * @param x The X coordinate.
     * @param y The Y coordinate.
     **/
    void set(uint32_t x, uint32_t y) {
        this->data[y].set(x);
    }

    /* @brief Resets given bit of the grid.
     * @param x The X coordinate.
     * @param y The Y coordinate.
     **/
    void reset(uint32_t x, uint32_t y) {
        this->data[y].reset(x);
    }

    /* @brief Toggles given bit of the grid.
     * @param x The X coordinate.
     * @param y The Y coordinate.
     **/
    void toggle(uint32_t x, uint32_t y) {
        this->data[y].toggle(x);
    }

    /* @brief Reads given bit of the grid.
     * @param x The X coordinate.
     * @param y The Y coordinate.
     * @returns The given bit of the grid.
     **/
    uint1_t read(uint32_t x, uint32_t y) const {
        return this->data[y].read(x);
    }

    /* @brief Writes given bit of the grid.
     * @param pos The position.
     * @param bit The bit value to write.
     **/
    void write(uint32_t x, uint32_t y, uint1_t bit) {
        this->data[y].write(x, bit);
    }

    /* @brief Converts binary grid's given row to byte array.
     * @param y The row to convert.
     * @param The result byte array.
     **/
    void toBytes(uint32_t y, uint8_t result[]) {
        this->data[y].toBytes(result);
    }

    /* @brief Resets all bits.
     **/
    void reset() {
        for (uint32_t y = 0; y < Y; ++y) {
            this->data[y].reset();
        }
    }
};
} // namespace uns
