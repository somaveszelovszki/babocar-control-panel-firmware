#pragma once

#include <uns/util/types.hpp>

#include <type_traits>

namespace uns {

#define dimension_name(name) name, name ## 2, name ## 3

/* @brief Unit dimensions.
 **/
enum class Dimension : uint8_t {
    dimension_name(time),                   // [seconds]
    dimension_name(distance),               // [meters]
    dimension_name(weight),                 // [grams]
    dimension_name(angle),                  // [radians, degrees]
    dimension_name(voltage),                // [Volts]
    dimension_name(current),                // [Ampers]
    dimension_name(resistance),             // [Ohms]
    dimension_name(speed),                  // [km/h, m/sec, mm/sec]
    dimension_name(acceleration),           // [m/sec^2, mm/sec^2]
    dimension_name(angular_velocity),       // [rad/sec]
    dimension_name(temperature),            // [°C]

    dimension_name(magnetic_flux),          // [Maxwell]
    dimension_name(magnetic_flux_density)   // [Gauss]
};

/* @brief Units for multiplications.
 **/
enum class Unit : uint8_t {
    giga,           // 10^9
    mega,           // 10^6
    kilo,           // 10^3
    hecto,          // 100
    deca,           // 10
    one,            // 1
    deci,           // 0.1
    centi,          // 0.01
    milli,          // 10^-3
    micro,          // 10^-6
    nano,           // 10^-9

    // non-decimal units
    _60,            // 60 (hours to minutes, minutes to seconds)
    _3600,          // 3600 (hours to seconds)
    deg_to_rad      // 0.0174532925 (degrees to radians)
};

namespace detail {

constexpr float32_t RAD_TO_DEG = 57.2957795f;       // Converts form radians to degrees.
constexpr float32_t DEG_TO_RAD = 0.0174532925f;     // Converts from degrees to radians.

/* @brief Multiplication dimension mapper. Maps result dimension of the two given dimension's multiplication (e.g. capacity for current and time).
 * @tparam _first_dim The first dimension.
 * @tparam _second_dim The second dimension.
 **/
template <Dimension _first_dim, Dimension _second_dim> struct mul_dim;

/* @brief Division dimension mapper. Maps result dimension of the two given dimension's division (e.g. speed for distance and time).
 * @tparam _first_dim The first dimension.
 * @tparam _second_dim The second dimension.
 **/
template <Dimension _num_dim, Dimension _den_dim> struct div_dim;

/* @brief Defines dimension connection (dim1 * dim2 = dim3).
 * @param dim1 The first dimension.
 * @param dim2 The second dimension.
 * @param dim3 The third dimension.
 **/
#define dimension_connections(dim1, dim2, dim3)                                         \
namespace detail {                                                                      \
template <> struct mul_dim<dim1, dim2>  { static constexpr Dimension value = dim3;  };  \
template <> struct mul_dim<dim2, dim1>  { static constexpr Dimension value = dim3;  };  \
template <> struct div_dim<dim3, dim1>  { static constexpr Dimension value = dim2;  };  \
template <> struct div_dim<dim3, dim2>  { static constexpr Dimension value = dim1;  };  \
} // namespace detail

#define square_dimension_connections(dim, sq_dim)                                           \
namespace detail {                                                                          \
template <> struct mul_dim<dim, dim>    { static constexpr Dimension value = sq_dim;    };  \
template <> struct div_dim<sq_dim, dim> { static constexpr Dimension value = dim;       };  \
} // namespace detail

/* @brief Unit multipliers.
 * @tparam unit The unit.
 **/
template <Unit unit> struct unit_multiplier;

template <> struct unit_multiplier<Unit::giga>         { static constexpr float32_t value = 1000000000.0f;          };  // Unit multiplier for giga.
template <> struct unit_multiplier<Unit::mega>         { static constexpr float32_t value = 1000000.0f;             };  // Unit multiplier for mega.
template <> struct unit_multiplier<Unit::kilo>         { static constexpr float32_t value = 1000.0f;                };  // Unit multiplier for kilo.
template <> struct unit_multiplier<Unit::hecto>        { static constexpr float32_t value = 100.0f;                 };  // Unit multiplier for hecto.
template <> struct unit_multiplier<Unit::deca>         { static constexpr float32_t value = 10.0f;                  };  // Unit multiplier for deca.
template <> struct unit_multiplier<Unit::one>          { static constexpr float32_t value = 1.0f;                   };  // Unit multiplier for one.
template <> struct unit_multiplier<Unit::deci>         { static constexpr float32_t value = 1 / 10.0f;              };  // Unit multiplier for deci.
template <> struct unit_multiplier<Unit::centi>        { static constexpr float32_t value = 1 / 100.0f;             };  // Unit multiplier for centi.
template <> struct unit_multiplier<Unit::milli>        { static constexpr float32_t value = 1 / 1000.0f;            };  // Unit multiplier for milli.
template <> struct unit_multiplier<Unit::micro>        { static constexpr float32_t value = 1 / 1000000.0f;         };  // Unit multiplier for micro.
template <> struct unit_multiplier<Unit::nano>         { static constexpr float32_t value = 1 / 1000000000.0f;      };  // Unit multiplier for nano.

// custom unit multipliers
template <> struct unit_multiplier<Unit::_60>          { static constexpr float32_t value = 60.0f;                  };  // Unit multiplier for _60.
template <> struct unit_multiplier<Unit::_3600>        { static constexpr float32_t value = 3600.0f;                };  // Unit multiplier for _3600.
template <> struct unit_multiplier<Unit::deg_to_rad>   { static constexpr float32_t value = DEG_TO_RAD;             };  // Unit multiplier for rad_to_deg.

template <Dimension _dim>
struct base_unit_instance {
    static constexpr Dimension dim = _dim;  // The dimension.
};

template <typename T>
struct is_unit_instance {
    enum { value = std::is_base_of<base_unit_instance<T::dim>, T>::value };
};

/* @brief Unit instance class template. Used for basic units like milliseconds.
 * @tparam _dim The dimension (e.g. time).
 * @tparam _unit The unit (e.g. milli).
 **/
template <Dimension _dim, Unit _unit>
struct unit_instance : public base_unit_instance<_dim> {
    static constexpr float32_t mul = unit_multiplier<_unit>::value; // The unit multiplier.
};

/* @brief Multiplication unit instance class template. Used for multiplication units like Ah.
 * @restrict Types must be unit instance types.
 * @tparam _unit_inst_t1 The unit instance type of the first member (e.g. ampers).
 * @tparam _unit_inst_t2 The dimension of the second member (e.g. hours).
 **/
template <typename T1, typename T2, class = typename std::enable_if<is_unit_instance<T1>::value && is_unit_instance<T2>::value>::type>
struct mul_unit_instance : public base_unit_instance<mul_dim<T1::dim, T2::dim>::value> {
    static constexpr float32_t mul = T1::mul * T2::mul;   // The unit multiplier.
};

/* @brief Division unit instance class template. Used for division units like km/h.
 * @restrict Types must be unit instance types.
 * @tparam _unit_inst_t1 The unit instance type of the numerator (e.g. kilometers).
 * @tparam _unit_inst_t2 The dimension of the denominator (e.g. hours).
 **/
template <typename T1, typename T2, class = typename std::enable_if<is_unit_instance<T1>::value && is_unit_instance<T2>::value>::type>
struct div_unit_instance : public base_unit_instance<div_dim<T1::dim, T2::dim>::value> {
    static constexpr float32_t mul = T1::mul / T2::mul;   // The unit multiplier.
};

template <typename _unit_inst_t>
using square_unit_instance = mul_unit_instance<_unit_inst_t, _unit_inst_t>;

/* @brief Rescales value to another unit.
 * @restrict Type of the value must be arithmetic.
 * @tparam from The source unit
 * @tparam to The result unit.
 * @tparam T Type of the value.
 * @param value The value to rescale.
 * @returns The rescaled value.
 **/
template <typename from_unit_inst_t, typename to_unit_inst_t, typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
inline constexpr typename std::enable_if<from_unit_inst_t::dim == to_unit_inst_t::dim, T>::type rescale_unit(const T& value) {
    return value * from_unit_inst_t::mul / to_unit_inst_t::mul;
}

/* @brief Dimension class template. Used for basic types like time, distance, etc.
 * @tparam _dim The dimension.
 **/
template <Dimension _dim, typename unit_inst_t_ = unit_instance<_dim, Unit::one>, bool explicit_unit = false>
class dim_class {
public:
    enum { is_dim_class = true };
    static constexpr Dimension dim = _dim;   // The dimension.
    typedef unit_inst_t_ unit_inst_t;
    static_assert(_dim == unit_inst_t::dim, "Dimensions do not match!");  // Checks if the dimensions match.

private:
    template <Dimension dim2, typename unit_inst_t2, bool explicit_unit2> friend class dim_class;
    template <typename unit_inst_t2, bool explicit_unit2> using same_dim_class = dim_class<dim, unit_inst_t2, explicit_unit2>;
    template <Dimension dim2, typename unit_inst_t2, bool explicit_unit2> using other_dim_class = dim_class<dim2, unit_inst_t2, explicit_unit2>;

    float32_t value;   // The stored value.

public:
    /* @brief Constructor - sets value.
     * @tparam T Numeric type of the parameter value.
     * param _value The value given in the unit instance.
     **/
    constexpr explicit dim_class(float32_t _value, void*) : value(_value) {}

    /* @brief Default constructor - sets value to 0.
     **/
    constexpr dim_class() : value(0.0f) {}

    /* @brief Constructor - sets value.
     * @tparam T Numeric type of the parameter value.
     * param _value The value given in the unit instance.
     **/
    template <bool enable = explicit_unit, class = typename std::enable_if<enable>::type>
    constexpr explicit dim_class(float32_t _value) : value(_value) {}

    static constexpr dim_class ZERO() { return dim_class(0.0f, nullptr); }

    /* @brief Constructor - sets value.
     * @tparam unit_inst_t Unit instance type.
     * @param [unnamed] The unit instance.
     * param _value The value given in the unit instance.
     **/
    template <typename unit_inst2, bool explicit_unit2>
    constexpr dim_class(const dim_class<dim, unit_inst2, explicit_unit2>& other)
        : value(rescale_unit<unit_inst2, unit_inst_t>(other.value)) {}
    
    /* @brief Gets value in given unit.
     * @restrict Type must be unit instance type.
     * @restrict Dimension of the result instance type must be the same as the dimension of this type.
     * @tparam unit_inst_t The result unit instance type (e.g. milliseconds).
     * @returns The value in given unit.
     **/
    template <bool enable = explicit_unit, class = typename std::enable_if<enable>::type>
    constexpr float32_t get() const {
        return this->value;
    }

    template <bool enable = explicit_unit, class = typename std::enable_if<enable>::type>
    void set(float32_t _value) {
        this->value = _value;
    }
};

/* @brief Adds two dimension class instances.
 * @param other The other dimension class instance.
 * @returns The result of the addition.
 **/
template <typename T1, typename T2>
inline constexpr typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, T1>::type operator+(const T1& d1, const T2& d2) {
    return T1(d1.template get<true>() + static_cast<T1>(d2).template get<true>(), nullptr);
}

/* @brief Subtracts two dimension class instances.
 * @param other The other dimension class instance.
 * @returns The result of the subtraction.
 **/
template <typename T1, typename T2>
inline constexpr typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, T1>::type operator-(const T1& d1, const T2& d2) {
    return T1(d1.template get<true>() - static_cast<T1>(d2).template get<true>(), nullptr);
}

/* @brief Adds two dimension class instances and stores result in this instance.
 * @param other The other dimension class instance.
 * @returns This dimension class instance.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, void>::type operator+=(T1& d1, const T2& d2) {
    d1.template set<true>(d1.template get<true>() + static_cast<T1>(d2).template get<true>());
}

/* @brief Subtracts two dimension class instances and stores result in this instance.
 * @param other The other dimension class instance.
 * @returns This dimension class instance.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, void>::type operator-=(T1& d1, const T2& d2) {
    d1.template set<true>(d1.template get<true>() - static_cast<T1>(d2).template get<true>());
}

 /* @brief Compares two dimension class instances.
 * @param other The other dimension class instance.
 * @returns Boolean value indicating if the two dimension class instances are equal.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, bool>::type operator==(const T1& d1, const T2& d2) {
    return d1.template get<true>() == static_cast<T1>(d2).template get<true>();
}

/* @brief Compares two dimension class instances.
 * @param other The other dimension class instance.
 * @returns Boolean value indicating if the two dimension class instances are not equal.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, bool>::type operator!=(const T1& d1, const T2& d2) {
    return d1.template get<true>() != static_cast<T1>(d2).template get<true>();
}

/* @brief Compares two dimension class instances.
 * @param other The other dimension class instance.
 * @returns Boolean value indicating if this dimension class instances is greater than the other.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, bool>::type operator>(const T1& d1, const T2& d2) {
    return d1.template get<true>() > static_cast<T1>(d2).template get<true>();
}

/* @brief Compares two dimension class instances.
 * @param other The other dimension class instance.
 * @returns Boolean value indicating if this dimension class instances is smaller than the other.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, bool>::type operator<(const T1& d1, const T2& d2) {
    return d1.template get<true>() < static_cast<T1>(d2).template get<true>();
}

/* @brief Compares two dimension class instances.
 * @param other The other dimension class instance.
 * @returns Boolean value indicating if this dimension class instances is greater-or-equal than the other.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, bool>::type operator>=(const T1& d1, const T2& d2) {
    return d1.template get<true>() >= static_cast<T1>(d2).template get<true>();
}

/* @brief Compares two dimension class instances.
 * @param other The other dimension class instance.
 * @returns Boolean value indicating if this dimension class instances is smaller-or-equal than the other.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, bool>::type operator<=(const T1& d1, const T2& d2) {
    return d1.template get<true>() <= static_cast<T1>(d2).template get<true>();
}

/* @brief Gets ratio of two dimension class instances.
 * @param other The other dimension class instance.
 * @returns The ratio of the dimension class instances.
 **/
template <typename T1, typename T2>
inline constexpr typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim == T2::dim, float32_t>::type operator/(const T1& d1, const T2& d2) {
    return d1.template get<true>() / static_cast<T1>(d2).template get<true>();
}

/* @brief Multiplies this value with another dimension class instance.
 * @param other The other dimension class instance.
 * @returns The result dimension class instance.
 **/
template <typename T1, typename T2, typename R = dim_class<mul_dim<T1::dim, T2::dim>::value, mul_unit_instance<typename T1::unit_inst_t, typename T2::unit_inst_t>>>
inline constexpr typename std::enable_if<T1::is_dim_class && T2::is_dim_class, R>::type operator*(const T1& d1, const T2& d2) {
    return R(d1.template get<true>() * d2.template get<true>(), nullptr);
}

/* @brief Divides this value by another dimension class instance.
 * @param other The other dimension class instance.
 * @returns The result dimension class instance.
 **/
template <typename T1, typename T2, typename R = dim_class<div_dim<T1::dim, T2::dim>::value, div_unit_instance<typename T1::unit_inst_t, typename T2::unit_inst_t>>>
inline constexpr typename std::enable_if<T1::is_dim_class && T2::is_dim_class && T1::dim != T2::dim, R>::type operator/(const T1& d1, const T2& d2) {
    return R(d1.template get<true>() / d2.template get<true>(), nullptr);
}

/* @brief Multiplies this value with a constant.
 * @restrict Type of the constant must be arithmetic.
 * @tparam T Type of the constant.
 * @param c The constant.
 * @returns The result dimension class instance.
 **/
template <typename T1, typename T2>
inline constexpr typename std::enable_if<T1::is_dim_class && std::is_arithmetic<T2>::value, T1>::type operator*(const T1& d, const T2& c) {
    return T1(d.template get<true>() * c, nullptr);
}

/* @brief Multiplies dimension class instance with a constant.
 * @restrict Type of the constant must be arithmetic.
 * @param c The constant.
 * @param _dim_class_inst The dimension class instance.
 * @returns The result dimension class instance.
 **/
template <typename T1, typename T2>
inline constexpr typename std::enable_if<T1::is_dim_class && std::is_arithmetic<T2>::value, T1>::type operator*(const T2& c, const T1& d) {
    return d * c;
}

/* @brief Divides this value by a constant.
 * @restrict Type of the constant must be arithmetic.
 * @param c The constant.
 * @returns The result dimension class instance.
 **/
template <typename T1, typename T2>
inline constexpr typename std::enable_if<T1::is_dim_class && std::is_arithmetic<T2>::value, T1>::type operator/(const T1& d, const T2& c) {
    return T1(d.template get<true>() / c, nullptr);
}

/* @brief Multiplies this value with -1.
 * @returns The result dimension class instance.
 **/
template <typename T1>
inline constexpr typename std::enable_if<T1::is_dim_class, T1>::type operator-(const T1& d) {
    return T1(-d.template get<true>(), nullptr);
}

/* @brief Multiplies this value with a constant and stores result in this instance.
 * @restrict Type of the constant must be arithmetic.
 * @param c The constant.
 * @returns This dimension class instance.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && std::is_arithmetic<T2>::value, T1&>::type operator*=(T1& d, const T2& c) {
    d.template set<true>(d.template get<true>() * c);
    return d;
}

/* @brief Divides this value by a constant and stores result in this instance.
 * @restrict Type of the constant must be arithmetic.
 * @param c The constant.
 * @returns This dimension class instance.
 **/
template <typename T1, typename T2>
inline typename std::enable_if<T1::is_dim_class && std::is_arithmetic<T2>::value, T1&>::type operator/=(T1& d, const T2& c) {
    d.template set<true>(d.template get<true>() / c);
    return d;
}

} // namespace detail

dimension_connections(Dimension::speed, Dimension::time, Dimension::distance)           // Dimension connections for speed, time and distance (speed * time = distance).
dimension_connections(Dimension::acceleration, Dimension::time, Dimension::speed)       // Dimension connections for acceleration, time and speed (acceleration * time = speed).
dimension_connections(Dimension::angular_velocity, Dimension::time, Dimension::angle)   // Dimension connections for angular velocity, time and angle (angular velocity * time = angle).

namespace detail {
template <Dimension dim> struct mul_dim<Dimension::angle, dim>  { static constexpr Dimension value = dim; };
template <Dimension dim> struct mul_dim<dim, Dimension::angle>  { static constexpr Dimension value = dim; };
template <Dimension dim> struct div_dim<dim, Dimension::angle>  { static constexpr Dimension value = dim; };
} // namespace detail

#define create_unit_instance_with_unit_prefix(dim, mul, unit)                                                                                       \
typedef detail::dim_class<Dimension::dim, detail::unit_instance<Dimension::dim, Unit::mul>, true> mul ## unit ## _t;                                \
typedef detail::dim_class<Dimension::dim, detail::square_unit_instance<detail::unit_instance<Dimension::dim, Unit::mul>>, true> mul ## unit ## 2_t; \
DEFINE_TYPEINFO(mul ## unit ## _t);                                                                                                                 \
DEFINE_TYPEINFO(mul ## unit ## 2_t)

#define create_unit_instance_without_unit_prefix(dim, mul, unit)                                                                                    \
typedef detail::dim_class<Dimension::dim, detail::unit_instance<Dimension::dim, Unit::mul>, true> unit ## _t;                                       \
typedef detail::dim_class<Dimension::dim, detail::square_unit_instance<detail::unit_instance<Dimension::dim, Unit::mul>>, true> unit ## 2_t;        \
DEFINE_TYPEINFO(unit ## _t);                                                                                                                        \
DEFINE_TYPEINFO(unit ## 2_t)

#define create_unit_instances(dim, unit)                            \
square_dimension_connections(Dimension::dim, Dimension::dim ## 2)   \
create_unit_instance_with_unit_prefix(dim, giga, unit);             \
create_unit_instance_with_unit_prefix(dim, mega, unit);             \
create_unit_instance_with_unit_prefix(dim, kilo, unit);             \
create_unit_instance_with_unit_prefix(dim, hecto, unit);            \
create_unit_instance_with_unit_prefix(dim, deca, unit);             \
create_unit_instance_without_unit_prefix(dim, one, unit);           \
create_unit_instance_with_unit_prefix(dim, deci, unit);             \
create_unit_instance_with_unit_prefix(dim, centi, unit);            \
create_unit_instance_with_unit_prefix(dim, milli, unit);            \
create_unit_instance_with_unit_prefix(dim, micro, unit);            \
create_unit_instance_with_unit_prefix(dim, nano, unit);             \
typedef detail::dim_class<Dimension::dim> dim ## _t;

#define create_mul_unit_instance(unit1, unit2, unit) \
typedef detail::dim_class<detail::mul_unit_instance<unit1 ## _t::unit_inst_t, unit2 ## _t::unit_inst_t>::dim,   \
        detail::mul_unit_instance<unit1 ## _t::unit_inst_t, unit2 ## _t::unit_inst_t>, true> unit ## _t;        \
DEFINE_TYPEINFO(unit ## _t)

#define create_div_unit_instance(unit1, unit2, unit) \
typedef detail::dim_class<detail::div_unit_instance<unit1 ## _t::unit_inst_t, unit2 ## _t::unit_inst_t>::dim,   \
        detail::div_unit_instance<unit1 ## _t::unit_inst_t, unit2 ## _t::unit_inst_t>, true> unit ## _t;        \
DEFINE_TYPEINFO(unit ## _t)

create_unit_instances(time, second);
create_unit_instance_without_unit_prefix(time, _3600, hour);
create_unit_instance_without_unit_prefix(time, _60, minute);

create_unit_instances(distance, meter);
create_unit_instances(weight, gram);
create_unit_instances(voltage, volt);
create_unit_instances(current, ampere);
create_unit_instances(resistance, ohm);

create_unit_instances(angle, radian);
create_unit_instance_without_unit_prefix(angle, deg_to_rad, degree);

create_unit_instances(temperature, celsius);
create_unit_instances(magnetic_flux, maxwell);

typedef detail::dim_class<Dimension::speed> speed_t;
create_div_unit_instance(kilometer, hour, km_per_hour);
create_div_unit_instance(meter, second, m_per_sec);
create_div_unit_instance(centimeter, second, cm_per_sec);
create_div_unit_instance(millimeter, second, mm_per_sec);

create_div_unit_instance(m_per_sec, second, m_per_sec2);
create_div_unit_instance(cm_per_sec, second, cm_per_sec2);
create_div_unit_instance(mm_per_sec, second, mm_per_sec2);

typedef detail::dim_class<Dimension::angular_velocity> angular_velocity_t;
create_div_unit_instance(radian, second, rad_per_sec);
create_div_unit_instance(degree, second, deg_per_sec);

} // namespace uns

