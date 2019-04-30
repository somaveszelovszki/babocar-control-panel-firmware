#pragma once

#include <uns/util/rational.hpp>

namespace uns {
namespace detail {

/* @brief Helper structure for getting type with larger memory-usage. Default structure.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
template <typename T1, typename T2, class = void> struct larger
{ typedef T1 type; /* By default, the first type is selected. */ };

/* @brief Helper structure for getting type with larger memory-usage. Selected when size of the second type is larger than the first one.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
template <typename T1, typename T2> struct larger<T1, T2, typename std::enable_if<sizeof(T1) < sizeof(T2)>::type>
{ typedef T2 type; /* The second type's size is larger. */ };

/* @brief Helper structure for getting calculation types.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
template <typename T1, typename T2, class = void> struct calc_type;

/* @brief Helper structure for getting calculation types. Selected when both types are integers or both are floating point types.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
template <typename T1, typename T2> struct calc_type<T1, T2, typename std::enable_if<
    std::is_integral<T1>::value == std::is_integral<T2>::value
    && std::is_floating_point<T1>::value == std::is_floating_point<T2>::value>::type>
{ typedef typename larger<T1, T2>::type type; /* The larger type is more accurate. */ };

/* @brief Helper structure for getting calculation types. Selected when the first type is an integer type and the second is a floating point type.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
template <typename T1, typename T2> struct calc_type<T1, T2, typename std::enable_if<std::is_integral<T1>::value && std::is_floating_point<T2>::value>::type>
{ typedef T2 type; /* The floating point type is selected. */ };

/* @brief Helper structure for getting calculation types. Selected when the first type is a floating point type and the second is an integer type.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
template <typename T1, typename T2> struct calc_type<T1, T2, typename std::enable_if<std::is_floating_point<T1>::value && std::is_integral<T2>::value>::type>
{ typedef T1 type; /* The floating point type is selected. */ };

/* @brief Helper structure for getting calculation types. Selected when the first type is an integer type and the second is a rational.
 * @param T1 The first type.
 **/
template <typename T1> struct calc_type<T1, rational, typename std::enable_if<std::is_integral<T1>::value>::type>
{ typedef rational type; /* The rational is selected. */ };

/* @brief Helper structure for getting calculation types. Selected when the first type is a rational and the second is an integer type.
 * @param T2 The second type.
 **/
template <typename T2> struct calc_type<rational, T2, typename std::enable_if<std::is_integral<T2>::value>::type>
{ typedef rational type; /* The rational is selected. */ };

/* @brief Helper structure for getting calculation types. Selected when the first type is a floating point type and the second is a rational.
 * @param T1 The first type.
 **/
template <typename T1> struct calc_type<T1, rational, typename std::enable_if<std::is_floating_point<T1>::value>::type>
{ typedef T1 type; /* The floating point type is selected. */ };

/* @brief Helper structure for getting calculation types. Selected when the first type is a rational and the second is a floating point type.
 * @param T2 The second type.
 **/
template <typename T2> struct calc_type<rational, T2, typename std::enable_if<std::is_floating_point<T2>::value>::type>
{ typedef T2 type; /* The floating point type is selected. */ };

/* @brief Defines a calculation structure. Contains functions for multiplication, division and rescale.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
template <typename T1, typename T2> struct calc;

/* @brief Defines a calculation structure's body.
 * @param T1 The first type.
 * @param T2 The second type.
 * @param mul_expr Multiplication expression.
 * @param div_expr Division expression.
 * @param rescale_expr Rescale expression.
 **/
#define __uns_calc_struct_body(T1, T2, mul_expr, div_expr, rescale_expr)                                \
typedef typename calc_type<T1, T2>::type result_type;                                                   \
                                                                                                        \
static result_type mul(const T1& t1, const T2& t2) {return mul_expr; }                                  \
static constexpr result_type const_mul(const T1& t1, const T2& t2) {return mul_expr; }                  \
                                                                                                        \
static result_type div(const T1& t1, const T2& t2) {return div_expr; }                                  \
static constexpr result_type const_div(const T1& t1, const T2& t2) {return div_expr; }                  \
                                                                                                        \
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>              \
static T rescale(const T& value, const T1& t1, const T2& t2) { return rescale_expr; }                   \
                                                                                                        \
template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>              \
static constexpr T const_rescale(const T& value, const T1& t1, const T2& t2) { return rescale_expr; }

/* @brief Defines a calculation structure's declaration.
 * @param T1 The first type.
 * @param T2 The second type.
 **/
#define __uns_calc_struct_decl(T1, T2) template <> struct calc<T1, T2>

/* @brief Defines a calculation structure's declaration with a template first type.
 * @param T2 The second type.
 **/
#define __uns_calc_struct_template_first_decl(T2) template <typename T1> struct calc<T1, T2>

/* @brief Defines a calculation structure's declaration with a template second type.
 * @param T1 The first type.
 **/
#define __uns_calc_struct_template_second_decl(T1) template <typename T2> struct calc<T1, T2>

/* @brief Defines a calculation structure's declaration with two template types.
 **/
#define __uns_calc_struct_template_both_decl() template <typename T1, typename T2> struct calc

/* @brief Defines a calculation structure.
 * @param T1 The first type.
 * @param T2 The second type.
 * @param mul_expr Multiplication expression.
 * @param div_expr Division expression.
 * @param rescale_expr Rescale expression.
 **/
#define uns_calc_struct(T1, T2, mul_expr, div_expr, rescale_expr) __uns_calc_struct_decl(T1, T2) { __uns_calc_struct_body(T1, T2, mul_expr, div_expr, rescale_expr) }

/* @brief Defines a calculation structure with a template first type.
 * @param T2 The second type.
 * @param mul_expr Multiplication expression.
 * @param div_expr Division expression.
 * @param rescale_expr Rescale expression.
 **/
#define uns_calc_struct_template_first(T2, mul_expr, div_expr, rescale_expr) __uns_calc_struct_template_first_decl(T2) { __uns_calc_struct_body(T1, T2, mul_expr, div_expr, rescale_expr) }

/* @brief Defines a calculation structure with a template second type.
 * @param T1 The first type.
 * @param mul_expr Multiplication expression.
 * @param div_expr Division expression.
 * @param rescale_expr Rescale expression.
 **/
#define uns_calc_struct_template_second(T1, mul_expr, div_expr, rescale_expr) __uns_calc_struct_template_second_decl(T1) { __uns_calc_struct_body(T1, T2, mul_expr, div_expr, rescale_expr) }

/* @brief Defines a calculation structure with two template types.
 * @param mul_expr Multiplication expression.
 * @param div_expr Division expression.
 * @param rescale_expr Rescale expression.
 **/
#define uns_calc_struct_template_both(mul_expr, div_expr, rescale_expr) __uns_calc_struct_template_both_decl() { __uns_calc_struct_body(T1, T2, mul_expr, div_expr, rescale_expr) }

/* @brief Calculation type for two template types.
 **/
uns_calc_struct_template_both(
    t1 * t2,
    t1 / t2,
    value * t1 / t2);

/* @brief Calculation type for a template type and a rational number.
 **/
uns_calc_struct_template_first(rational,
    t1 * t2,
    t1 / t2,
    value * t1 * t2.den / t2.num);

/* @brief Calculation type for rational number and a template type.
 **/
uns_calc_struct_template_second(rational,
    t1 * t2,
    t1 / t2,
    value * t1.num / (t1.den * t2));

/* @brief Calculation type rational numbers.
 **/
uns_calc_struct(rational, rational,
    t1 * t2,
    t1 / t2,
    value * t1.num * t2.den / (t1.den * t2.num));

/* @brief Calculation type for rational and 32-bit floating point number.
 **/
uns_calc_struct(rational, float32_t,
    t1.num * t2 / t1.den,
    t1.num / (t2 * t1.den),
    value * t1.num / (t1.den * t2));

/* @brief Calculation type for 32-bit floating point number and rational.
 **/
uns_calc_struct(float32_t, rational,
    t1 * t2.num / t2.den,
    t1 * t2.den / t2.den,
    value * t1 * t2.den / t2.num);

} // namespace detail
} // namespace uns
