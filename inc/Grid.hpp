//#pragma once
//
//#include "Array.hpp"
//#include "config/cfg.hpp"
//
//namespace micro {
//
///* @brief Grid (2-dimensional array) of values requiring less than 8 bits.
//Values are stored in byte arrays to reduce memory need.
//// * @tparam B Bit length of the stored values (1, 2 or 4).
// * @tparam X Number of values stored in the X axis.
// * @tparam Y Number of values stored in the Y axis.
//@note (Y * N) must be a multiple of 8!
// * @tparam VPB Stored values per byte.
//@note Do not set this parameter explicitly!
// **/
//template <uint8_t B, uint8_t X, uint8_t Y>
//class Grid {
//private:
//    /* @brief Type of the arrays that stores the horizontal rows of the grid.
//     **/
//    typedef Array<B, X> array_type;
//
//    /* @brief Array of byte arrays containing the binary values.
//     **/
//    array_type data[Y];
//
//public:
//    typedef typename array_type::type type;
//    static const type dataMin;
//    static const type dataMax;
//
//    /* @brief Constructor - does not initialize elements.
//     **/
//    Grid() {}
//
//    /* @brief Gets value at given X,Y coordinate.
////     * @param x The X coordinate of the value to get.
//     * @param y The Y coordinate of the value to get.
//     * @returns The value of the element.
//     **/
//    type get(uint8_t x, uint8_t y) const {
//        return y < Y ? this->data[y].get(x) : 0;
//    }
//
//    /* @brief Sets element at given X,Y coordinate to the given value.
////     * @param x The X coordinate of the value to set.
//     * @param y The Y coordinate of the value to set.
//     * @param value The value to set.
//     **/
//    void set(uint8_t x, uint8_t y, type value) {
//        if (y < Y)
//            this->data[y].set(x, value);
//    }
//
//    void increment(uint8_t x, uint8_t y) {
//        type value = get(x, y);
//        if (value < dataMax)
//            this->set(x, y, value + 1);
//    }
//
//    void decrement(uint8_t x, uint8_t y) {
//        type value = get(x, y);
//        if (value > dataMin)
//            this->set(x, y, value - 1);
//    }
//
//
//#if DEBUG
//    /* @brief Prints grid elements.
//     **/
//    void print() const {
//        for (uint8_t y = 0; y < Y; ++y) {
//            //wdt_reset();
//            data[y].print(y == Y / 2);
//            //Serial.flush();
//        }
//    }
//#endif // DEBUG
//
//    class StreamWriter {
//    private:
//        uint8_t y = -1;
//        const Grid<B, X, Y> *pGrid;
//        typename array_type::StreamWriter arrayWriter;
//
//    public:
//        /* @brief Constructor - does not set anything.
//         **/
//        StreamWriter() {}
//
//        /* @brief Sets grid reference and resets position.
////         * @param _pGrid Pointer to the grid.
//         **/
//        void setGrid(const Grid<B, X, Y> *_pGrid) {
//            pGrid = _pGrid;
//            y = 0;
//            arrayWriter.setArray(&_pGrid->data[0]);
//        }
//
//        /* @brief Writes next segment to byte array.
////         * @param result The result byte array.
//         * @param maxBytesNum Max number of bytes to write (COMM_MSG_DATA_LENGTH by default).
//         * @param pWrittenX The X coordinate of the first written byte.
//         * @param pWrittenY The Y coordinate of the first written byte.
//         * @returns If whole grid has not been written yet: number of bytes written, else: -1 * (number of bytes written).
//         **/
//        int8_t next(ByteArray<COMM_MSG_DATA_LENGTH>& result, uint8_t maxBytesNum = COMM_MSG_DATA_LENGTH, uint8_t *pWrittenX = NULL, uint8_t *pWrittenY = NULL) {
//            if (pWrittenY)
//                *pWrittenY = y;
//
//            int8_t bytesWritten = arrayWriter.next(result, maxBytesNum, pWrittenX);
//            if (bytesWritten < 0) {     // whole row array has been written
//
//                bool finished = ++y == Y;
//                if (finished) {
//                    // bytesWritten is already negative, no need to change it
//                    y = 0;
//                }
//
//                arrayWriter.setArray(&pGrid->data[y]);
//
//                if (!finished && ((bytesWritten *= -1) < (int8_t)maxBytesNum)) {    // not all data bytes have been filled yet
//                    ByteArray<COMM_MSG_DATA_LENGTH> sub(&result[bytesWritten]);
//                    bytesWritten += arrayWriter.next(sub, maxBytesNum - bytesWritten);
//                }
//            }
//
//            return bytesWritten;
//        }
//    };
//
//    friend class Grid<B, X, Y>::StreamWriter;
//};
//
//template<uint8_t B, uint8_t X, uint8_t Y>
//const typename Grid<B, X, Y>::type Grid<B, X, Y>::dataMin = Array<B, X>::dataMin;
//
//template<uint8_t B, uint8_t X, uint8_t Y>
//const typename Grid<B, X, Y>::type Grid<B, X, Y>::dataMax = Array<B, X>::dataMax;
//} // namespace micro