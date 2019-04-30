#pragma once

#include <uns/util/types.hpp>
#include <type_traits>

namespace uns {

/* @brief Rational number structure with operators.
 **/
class rational {
private:
    /* @brief Simplifies two rational number.
     * @note Simplifies only if numerator is a multiple of the denominator or vica versa.
     * @returns The simplified rational number.
     **/
    static constexpr rational simplify(const rational& r) {
        return r.num == r.den ? (rational){ 1, 1 }
        : r.num % r.den == 0 ? (rational){ r.num / r.den, 1 }
        : r.den % r.num == 0 ? (rational){ 1, r.den / r.num }
        : (rational){ r.num, r.den };
    }

public:
    int64_t num;    // The numerator.
    int64_t den;    // The denominator.

    /* @brief Multiplies two rational numbers.
     * @note Does not simplify result.
     * @param other The other rational number.
     * @returns The unsimplified result of the multiplication.
     **/
    constexpr rational operator*(const rational& other) const {
        return rational::simplify({ this->num * other.num, this->den * other.den });
    }

    /* @brief Divides two rational numbers.
     * @note Does not simplify result.
     * @param other The other rational number.
     * @returns The unsimplified result of the division.
     **/
    constexpr rational operator/(const rational& other) const {
        return rational::simplify({ this->num * other.den, this->den * other.num });
    }

    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    constexpr rational operator*(const T& c) const {
        return { this->num * c, this->den };
    }

    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    constexpr rational operator/(const T& c) const {
        return { this->num / c, this->den };
    }

    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    constexpr friend rational operator*(const T& c, const rational& r) {
        return r * c;
    }

    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    constexpr friend rational operator/(const T& c, const rational& r) {
        return { c * r.den, r.num };
    }

    /* @brief Calculates reciprocal of the rational number.
     * @returns The reciprocal of the rational number.
     **/
    constexpr rational reciprocal() const {
        return { this->den, this->num };
    }
};
} // namespace uns