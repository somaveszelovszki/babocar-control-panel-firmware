#pragma once

#include <uns/util/types.hpp>
#include <type_traits>
#include <cmath>

namespace uns {

/* @brief Gets minimum of the two values.
 * @param a The first value.
 * @param b The second value.
 * @returns The minimum of the two values.
 **/
template <typename T>
inline T min(const T& a, const T& b) {
    return a < b ? a : b;
}

/* @brief Gets maximum of the two values.
 * @param a The first value.
 * @param b The second value.
 * @returns The maximum of the two values.
 **/
template <typename T>
inline T max(const T& a, const T& b) {
    return a > b ? a : b;
}

/* @brief Gets average of the two values.
 * @param a The first value.
 * @param b The second value.
 * @returns The average of the two values.
 **/
template <typename T>
inline T avg(const T& a, const T& b) {
    return T((a + b) / 2.0f);
}

/* @brief Gets sign of the value.
 * @restrict Type must be arithmetic.
 * @param value The value.
 * @returns The sign of the value.
 **/
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline Sign sgn(const T& value) {
    return value >= T(0) ? Sign::POSITIVE : Sign::NEGATIVE;
}

/* @brief Checks if a value is between the given boundaries.
 * @tparam T Type of the elements.
 * @param value The value to check.
 * @param b1 The first boundary.
 * @param b2 The second boundary.
 * @returns Boolean value indicating if the value is between the boundaries.
 **/
template <typename T>
inline bool isBtw(const T& value, const T& b1, const T& b2) {
    return b2 >= b1 ? value >= b1 && value <= b2 : value >= b2 && value <= b1;
}

/* @brief Incarcerates value between the given boundaries.
 * @tparam T Type of the elements.
 * @param value The value to incarcerate.
 * @param b1 The first boundary.
 * @param b2 The second boundary.
 * @returns The incarcerated value.
 **/
template <typename T>
inline T incarcerate(const T& value, const T& b1, const T& b2) {
    return b2 > b1 ? min(max(value, b1), b2) : min(max(value, b2), b1);
}

/* @brief Checks if value is in a given range of the reference value.
 * @tparam T Numeric type of the value and the reference.
 * @param value The value to compare to the reference.
 * @param ref The reference value.
 * @param relErr The permitted relative error.
 **/
template <typename T>
inline bool isInRange(const T& value, const T& ref, float32_t relErr) {
    return uns::isBtw(value, static_cast<T>(ref * (1.0f - relErr)), static_cast<T>(ref * (1.0f + relErr)));
}

/* @brief Maps value from from a given range to another.
 * @tparam S Numeric type of the source value and the source range boundaries.
 * @tparam R Numeric type of the result value and the result range boundaries.
 * @param value The value to map.
 * @param fromLow Lower boundary of the source range.
 * @param fromHigh Higher boundary of the source range.
 * @param toLow Lower boundary of the destination range.
 * @param toHigh Higher boundary of the destination range.
 * @returns The mapped value.
 **/
template <typename S, typename R>
inline R map(const S& value, const S& fromLow, const S& fromHigh, const R& toLow, const R& toHigh) {
    return toLow + static_cast<R>((uns::incarcerate(value, fromLow, fromHigh) - fromLow) * (toHigh - toLow) / (fromHigh - fromLow));
}

#define COMMON_EQ_ABS_EPS   0.00001f    // default absolute epsilon for equality check

/* @brief Checks if given value equals the reference with the default epsilon tolerance.
 * @restrict Type must be arithmetic.
 * @tparam T Numeric type of the value, the reference and the epsilon tolerance.
 * @param value The value to compare to the reference.
 * @param ref The reference.
 **/
template <typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value, bool>::type eq(const T& value, const T& ref) {
    static constexpr T eps(COMMON_EQ_ABS_EPS);
    return (value >= ref - eps) && (value <= ref + eps);
}

/* @brief Checks if given value equals the reference with the given epsilon tolerance.
 * @tparam T Type of the value, the reference and the epsilon tolerance.
 * @param value The value to compare to the reference.
 * @param ref The reference.
 * @param eps The epsilon tolerance.
 **/
template <typename T>
inline bool eq(const T& value, const T& ref, const T& eps) {
    return (value >= ref - eps) && (value <= ref + eps);
}

/* @brief Checks if given value equals the reference with the default epsilon tolerance.
 * @restrict Type must be a dimension class.
 * @tparam T Numeric type of the value, the reference and the epsilon tolerance.
 * @param value The value to compare to the reference.
 * @param ref The reference.
 **/
template <typename dim_class_t>
inline typename std::enable_if<dim_class_t::is_dim_class, bool>::type eq(const dim_class_t& value, const dim_class_t& ref) {
    static constexpr dim_class_t eps(typename dim_class_t::stored_unit_inst_t(), COMMON_EQ_ABS_EPS);
    return eq(value, ref, eps);
}

/* @brief Checks if given value equals zero with the given epsilon tolerance.
 * @restrict Type must be arithmetic.
 * @tparam T Numeric type of the value, the reference and the epsilon tolerance.
 * @param value The value to compare to the reference.
 * @param eps The epsilon tolerance - 0.0001f by default.
 **/
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline bool isZero(const T& value, T eps = static_cast<T>(COMMON_EQ_ABS_EPS)) {
    return uns::eq(value, static_cast<T>(0), eps);
}

/* @brief Calculates the hypotenuse of a triangle using the Pythagorean theory.
 * @restrict Type must be arithmetic.
 * @tparam T The type of the values.
 * @param a The length of the first leg of the triangle.
 * @param b The length of the other leg of the triangle.
 * @returns The length of the hypotenuse of the triangle.
 **/
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline T pythag(const T& a, const T& b) {
    const float32_t _a = static_cast<float32_t>(a), _b = static_cast<float32_t>(b);
    return static_cast<T>(sqrtf(_a * _a + _b * _b));
}

/* @brief Calculates vector length using the Pythagorean theory.
 * @restrict Type must be arithmetic.
 * @tparam T The type of the values.
 * @param a The length of the first coordinate.
 * @param b The length of the second coordinate.
 * @param c The length of the third coordinate.
 * @returns The length of the vector.
 **/
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline T pythag(const T& a, const T& b, const T& c) {
    return static_cast<T>(sqrtf(static_cast<float32_t>(a * a + b * b + c * c)));
}

template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline T powerOf(const T& value, uint32_t _pow) {
    T result(1);
    for (uint32_t i = 0; i < _pow; ++i) {
        result *= value;
    }
    return result;
}
} // namespace uns
