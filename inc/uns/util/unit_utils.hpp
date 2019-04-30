#pragma once

#include <uns/util/numeric.hpp>
#include <uns/util/units.hpp>

namespace uns {
constexpr angle_t PI(radians(), 3.14159265358979323846);        	// Pi
constexpr angle_t PI_2(radians(), 3.14159265358979323846 / 2.0); // Pi / 2
constexpr angle_t PI_4(radians(), 3.14159265358979323846 / 4.0); // Pi / 4

/* @brief Calculates sine of given angle.
 * @restrict Type of the result value must be arithmetic.
 * @tparam T Type of the result value - 32-bit floating point number by default.
 * @param value The angle.
 * @returns The sine of the angle.
 **/
template <typename T = float32_t, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline T sin(const angle_t& value) {
    return static_cast<T>(sinf(value.get<radians>())) ;
}

/* @brief Calculates arc-sine of given value.
 * @restrict Type of the value must be arithmetic.
 * @tparam T Type of the value.
 * @param value The value.+
 * @returns The arc-sine of the value.
 **/
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline angle_t asin(const T& value) {
    return angle_t::from<radians>(asinf(static_cast<float32_t>(value))) ;
}

/* @brief Calculates cosine of given angle.
 * @restrict Type of the result value must be arithmetic.
 * @tparam T Type of the result value - 32-bit floating point number by default.
 * @param value The angle.
 * @returns The cosine of the angle.
 **/
template <typename T = float32_t, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline T cos(const angle_t& value) {
    return static_cast<T>(cosf(value.get<radians>())) ;
}

/* @brief Calculates arc-cosine of given value.
 * @restrict Type of the value must be arithmetic.
 * @tparam T Type of the value.
 * @param value The value.
 * @returns The arc-cosine of the value.
 **/
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline angle_t acos(const T& value) {
    return angle_t::from<radians>(acosf(static_cast<float32_t>(value))) ;
}

/* @brief Calculates tangent of given angle.
 * @restrict Type of the result value must be arithmetic.
 * @tparam T Type of the result value - 32-bit floating point number by default.
 * @param value The angle.
 * @returns The tangent of the angle.
 **/
template <typename T = float32_t, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline T tan(const angle_t& value) {
    return static_cast<T>(tanf(value.get<radians>()));
}

/* @brief Calculates arc-tangent of given value.
 * @restrict Type of the value must be arithmetic.
 * @tparam T Type of the value.
 * @param value The value.
 * @returns The arc-tangent of the value.
 **/
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline angle_t atan(const T& value) {
    return angle_t::from<radians>(atanf(static_cast<float32_t>(value)));
}

inline angle_t normalize360(angle_t value) {
    static constexpr angle_t DEG_360 = 2 * uns::PI;
    while(value >= DEG_360) {
        value -= DEG_360;
    }
    while(value < angle_t::ZERO()) {
        value += DEG_360;
    }
    return value;
}

inline angle_t normalize180(angle_t value) {
    while(value >= PI) {
        value -= PI;
    }
    while(value < angle_t::ZERO()) {
        value += PI;
    }
    return value;
}

inline bool eqWithOverflow360(angle_t value, angle_t ref, angle_t eps) {
    static constexpr angle_t DEG_360 = 2 * uns::PI;
    return eq(value, ref, eps) || eq(value + DEG_360, ref, eps) || eq(value - DEG_360, ref, eps);
}

inline bool eqWithOverflow180(angle_t value, angle_t ref, angle_t eps) {
    return eq(value, ref, eps) || eq(value + PI, ref, eps) || eq(value - PI, ref, eps);
}

inline angle_t round45(angle_t value) {
    static constexpr angle_t EPS = uns::PI_4 / 2;
    angle_t result;

    if (uns::eqWithOverflow360(value, uns::PI_4, EPS)) {
        result = uns::PI_4;
    } else if (uns::eqWithOverflow360(value, uns::PI_2, EPS)) {
        result = uns::PI_2;
    } else if (uns::eqWithOverflow360(value, 3 * uns::PI_4, EPS)) {
        result = 3 * uns::PI_4;
    } else if (uns::eqWithOverflow360(value, uns::PI, EPS)) {
        result = uns::PI;
    } else if (uns::eqWithOverflow360(value, 5 * uns::PI_4, EPS)) {
        result = 5 * uns::PI_4;
    } else if (uns::eqWithOverflow360(value, 3 * uns::PI_2, EPS)) {
        result = 3 * uns::PI_2;
    } else if (uns::eqWithOverflow360(value, 7 * uns::PI_4, EPS)) {
        result = 7 * uns::PI_4;
    } else {
        result = angle_t::ZERO();
    }

    return value;
}


/* @brief Calculates vector length using the Pythagorean theory.
 * @param a The length of the first leg of the triangle.
 * @param b The length of the other leg of the triangle.
 * @returns The length of the hypotenuse of the triangle.
 **/
inline distance_t pythag(distance_t a, distance_t b) {
    typedef distance_t::stored_unit_inst_t unit;
    float32_t _a = a.get<unit>(), _b = b.get<unit>();

    return distance_t::from<unit>(uns::pythag(_a, _b));
}

/* @brief Calculates vector length using the Pythagorean theory.
 * @param a The length of the first coordinate.
 * @param b The length of the second coordinate.
 * @param c The length of the third coordinate.
 * @returns The length of the vector.
 **/
inline distance_t pythag(distance_t a, distance_t b, distance_t c) {
    typedef distance_t::stored_unit_inst_t unit;
    float32_t _a = a.get<unit>(), _b = b.get<unit>(), _c = c.get<unit>();

    return distance_t::from<unit>(uns::pythag(_a, _b, _c));
}

inline angle_t straighten(angle_t angle, angle_t eps) {
    if (uns::eq(angle, uns::PI_2, eps)) {
        angle = uns::PI_2;
    } else if (uns::eq(angle, uns::PI, eps)) {
        angle = uns::PI;
    } else if (uns::eq(angle, 3 * uns::PI_2, eps)) {
        angle = 3 * uns::PI_2;
    } else if (angle.isZero(eps) || uns::eq(angle, 2 * uns::PI, eps)) {
        angle = angle_t::ZERO();
    }

    return angle;
}
} // namespace uns
