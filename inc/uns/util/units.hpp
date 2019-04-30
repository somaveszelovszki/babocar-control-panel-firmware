#pragma once

#include <math.h>
#include <uns/util/calc.hpp>
#include <uns/util/numeric.hpp>

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
    one_per_10e4,   // 10^-4
    one_per_10e5,   // 10^-5
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
template <> struct mul_dim<dim1, dim2>  { static constexpr Dimension value = dim3;  };  \
template <> struct mul_dim<dim2, dim1>  { static constexpr Dimension value = dim3;  };  \
template <> struct div_dim<dim3, dim1>  { static constexpr Dimension value = dim2;  };  \
template <> struct div_dim<dim3, dim2>  { static constexpr Dimension value = dim1;  }

#define square_dimension_connections(dim, sq_dim)                                                   \
template <> struct detail::mul_dim<dim, dim>    { static constexpr Dimension value = sq_dim;    };  \
template <> struct detail::div_dim<sq_dim, dim> { static constexpr Dimension value = dim;       }

dimension_connections(Dimension::speed, Dimension::time, Dimension::distance);          // Dimension connections for speed, time and distance (speed * time = distance).
dimension_connections(Dimension::acceleration, Dimension::time, Dimension::speed);      // Dimension connections for acceleration, time and speed (acceleration * time = speed).
dimension_connections(Dimension::angular_velocity, Dimension::time, Dimension::angle);  // Dimension connections for angular velocity, time and angle (angular velocity * time = angle).

template <Dimension dim> struct mul_dim<Dimension::angle, dim>  { static constexpr Dimension value = dim; };
template <Dimension dim> struct mul_dim<dim, Dimension::angle>  { static constexpr Dimension value = dim; };
template <Dimension dim> struct div_dim<Dimension::angle, dim>  { static constexpr Dimension value = dim; };
template <Dimension dim> struct div_dim<dim, Dimension::angle>  { static constexpr Dimension value = dim; };

/* @brief Unit multipliers.
 * @tparam unit The unit.
 **/
template <Unit unit> struct unit_multiplier;

template <> struct unit_multiplier<Unit::giga>         { static constexpr float32_t value = 1000000000.0f;            };  // Unit multiplier for giga.
template <> struct unit_multiplier<Unit::mega>         { static constexpr float32_t value = 1000000.0f;               };  // Unit multiplier for mega.
template <> struct unit_multiplier<Unit::kilo>         { static constexpr float32_t value = 1000.0f;                  };  // Unit multiplier for kilo.
template <> struct unit_multiplier<Unit::hecto>        { static constexpr float32_t value = 100.0f;                   };  // Unit multiplier for hecto.
template <> struct unit_multiplier<Unit::deca>         { static constexpr float32_t value = 10.0f;                    };  // Unit multiplier for deca.
template <> struct unit_multiplier<Unit::one>          { static constexpr float32_t value = 1.0f;                  };  // Unit multiplier for one.
template <> struct unit_multiplier<Unit::deci>         { static constexpr float32_t value = 1 / 10.0f;                 };  // Unit multiplier for deci.
template <> struct unit_multiplier<Unit::centi>        { static constexpr float32_t value = 1 / 100.0f;                };  // Unit multiplier for centi.
template <> struct unit_multiplier<Unit::milli>        { static constexpr float32_t value = 1 / 1000.0f;               };  // Unit multiplier for milli.
template <> struct unit_multiplier<Unit::one_per_10e4> { static constexpr float32_t value = 1 / 10000.0f;              };  // Unit multiplier for one_per_10e4.
template <> struct unit_multiplier<Unit::one_per_10e5> { static constexpr float32_t value = 1 / 100000.0f;             };  // Unit multiplier for one_per_10e5.
template <> struct unit_multiplier<Unit::micro>        { static constexpr float32_t value = 1 / 1000000.0f;            };  // Unit multiplier for micro.
template <> struct unit_multiplier<Unit::nano>         { static constexpr float32_t value = 1 / 1000000000.0f;         };  // Unit multiplier for nano.

// custom unit multipliers
template <> struct unit_multiplier<Unit::_60>          { static constexpr float32_t value = 60.0f;                    };  // Unit multiplier for _60.
template <> struct unit_multiplier<Unit::_3600>        { static constexpr float32_t value = 3600.0f;                  };  // Unit multiplier for _3600.
template <> struct unit_multiplier<Unit::deg_to_rad>   { static constexpr float32_t value = DEG_TO_RAD;              };  // Unit multiplier for rad_to_deg.

static constexpr Unit min_unit = Unit::nano;  // Minimum unit - used for calculations.
static constexpr Unit max_unit = Unit::giga;  // Maximum unit - used for calculations.

/* @brief Unit instance class template. Used for basic units like milliseconds.
 * @tparam _dim The dimension (e.g. time).
 * @tparam _unit The unit (e.g. milli).
 **/
template <Dimension _dim, Unit _unit>
struct unit_instance {
    static constexpr bool is_unit_instance = true;                  // Indicates that class is an uns unit instance class.
    static constexpr Dimension dim = _dim;                          // The dimension.
    static constexpr float32_t mul = unit_multiplier<_unit>::value;  // The unit multiplier.
};

/* @brief Multiplication unit instance class template. Used for multiplication units like Ah.
 * @restrict Types must be unit instance types.
 * @tparam _unit_inst_t1 The unit instance type of the first member (e.g. ampers).
 * @tparam _unit_inst_t2 The dimension of the second member (e.g. hours).
 **/
template <typename _unit_inst_t1, typename _unit_inst_t2, class = typename std::enable_if<_unit_inst_t1::is_unit_instance && _unit_inst_t2::is_unit_instance>::type>
struct mul_unit_instance {
    static constexpr bool is_unit_instance = true;                                                                  // Indicates that class is an uns unit instance class.
    static constexpr Dimension dim = mul_dim<_unit_inst_t1::dim, _unit_inst_t2::dim>::value;                        // The dimension.
    static constexpr float32_t mul = _unit_inst_t1::mul * _unit_inst_t2::mul;                  // The unit multiplier.
};

/* @brief Division unit instance class template. Used for division units like km/h.
 * @restrict Types must be unit instance types.
 * @tparam _unit_inst_t1 The unit instance type of the numerator (e.g. kilometers).
 * @tparam _unit_inst_t2 The dimension of the denominator (e.g. hours).
 **/
template <typename _unit_inst_t1, typename _unit_inst_t2, class = typename std::enable_if<_unit_inst_t1::is_unit_instance && _unit_inst_t2::is_unit_instance>::type>
struct div_unit_instance {
    static constexpr bool is_unit_instance = true;                                                                  // Indicates that class is an uns unit instance class.
    static constexpr Dimension dim = div_dim<_unit_inst_t1::dim, _unit_inst_t2::dim>::value;                        // The dimension.
    static constexpr float32_t mul = _unit_inst_t1::mul / _unit_inst_t2::mul;                   // The unit multiplier.
};

template <typename _unit_inst_t>
using square_unit_instance = mul_unit_instance<_unit_inst_t, _unit_inst_t>;

#define create_unit_instance(dim, mul, name)            \
typedef detail::unit_instance<dim, mul> name;           \
typedef detail::square_unit_instance<name> name ## 2;

} // namespace detail

square_dimension_connections(Dimension::time, Dimension::time2);                // Square dimension connections for time.
square_dimension_connections(Dimension::distance, Dimension::distance2);        // Square dimension connections for distance.
square_dimension_connections(Dimension::speed, Dimension::speed2);              // Square dimension connections for speed.

create_unit_instance(Dimension::time, Unit::_3600, hours);                      // Unit instance class for hours.
create_unit_instance(Dimension::time, Unit::_60, minutes);                      // Unit instance class for minutes.
create_unit_instance(Dimension::time, Unit::one, seconds);                      // Unit instance class for seconds.
create_unit_instance(Dimension::time, Unit::milli, milliseconds);               // Unit instance class for milliseconds.
create_unit_instance(Dimension::time, Unit::one_per_10e5, _10em5_seconds);      // Unit instance class for 10^-5 seconds.
create_unit_instance(Dimension::time, Unit::micro, microseconds);               // Unit instance class for microseconds.

create_unit_instance(Dimension::distance, Unit::kilo, kilometers);              // Unit instance class for kilometers.
create_unit_instance(Dimension::distance, Unit::one, meters);                   // Unit instance class for meters.
create_unit_instance(Dimension::distance, Unit::centi, centimeters);            // Unit instance class for centimeters.
create_unit_instance(Dimension::distance, Unit::milli, millimeters);            // Unit instance class for millimeters.
create_unit_instance(Dimension::distance, Unit::one_per_10e4, _10em4_meters);   // Unit instance class for 10^-4 meters.
create_unit_instance(Dimension::distance, Unit::micro, micrometers);            // Unit instance class for micrometers.

create_unit_instance(Dimension::weight, Unit::kilo, kilograms);                 // Unit instance class for kilograms.
create_unit_instance(Dimension::weight, Unit::deca, decagrams);                 // Unit instance class for decagrams.
create_unit_instance(Dimension::weight, Unit::one, grams);                      // Unit instance class for grams.
create_unit_instance(Dimension::weight, Unit::milli, milligrams);               // Unit instance class for milligrams.

create_unit_instance(Dimension::voltage, Unit::kilo, kilovolts);                // Unit instance class for kilovolts.
create_unit_instance(Dimension::voltage, Unit::one, volts);                     // Unit instance class for volts.
create_unit_instance(Dimension::voltage, Unit::milli, millivolts);              // Unit instance class for millivolts.

create_unit_instance(Dimension::current, Unit::kilo, kiloampers);               // Unit instance class for kiloampers.
create_unit_instance(Dimension::current, Unit::one, ampers);                    // Unit instance class for ampers.
create_unit_instance(Dimension::current, Unit::milli, milliampers);             // Unit instance class for milliampers.
create_unit_instance(Dimension::current, Unit::micro, microampers);             // Unit instance class for microampers.

create_unit_instance(Dimension::resistance, Unit::mega, megaohms);              // Unit instance class for megaohms.
create_unit_instance(Dimension::resistance, Unit::kilo, kiloohms);              // Unit instance class for kiloohms.
create_unit_instance(Dimension::resistance, Unit::one, ohms);                   // Unit instance class for ohms.

create_unit_instance(Dimension::angle, Unit::one, radians);                     // Unit instance class for radians.
create_unit_instance(Dimension::angle, Unit::deg_to_rad, degrees);              // Unit instance class for degrees.

create_unit_instance(Dimension::temperature, Unit::one, celsius);               // Unit instance class for Celsius degrees.

create_unit_instance(Dimension::magnetic_flux, Unit::one, maxwell);             // Unit instance class for Maxwells.
create_unit_instance(Dimension::magnetic_flux, Unit::milli, millimaxwell);      // Unit instance class for milliMaxwells.

typedef detail::div_unit_instance<kilometers, hours>            km_per_hour;    // Unit instance class for kilometers/hour.
typedef detail::div_unit_instance<meters, seconds>              m_per_sec;      // Unit instance class for meters/second.
typedef detail::div_unit_instance<millimeters, seconds>         mm_per_sec;     // Unit instance class for millimeters/second.

typedef detail::div_unit_instance<m_per_sec, seconds>           m_per_sec2;     // Unit instance class for meters/second^2.
typedef detail::div_unit_instance<mm_per_sec, seconds>          mm_per_sec2;    // Unit instance class for millimeters/second^2.

typedef detail::div_unit_instance<radians, seconds>             rad_per_sec;    // Unit instance class for radians/second.
typedef detail::div_unit_instance<degrees, seconds>             deg_per_sec;    // Unit instance class for degrees/second.

typedef detail::div_unit_instance<maxwell, centimeters2>        gauss;          // Unit instance class for Gausses.
typedef detail::div_unit_instance<millimaxwell, centimeters2>   milligauss;     // Unit instance class for milliGausses.

namespace detail {

/* @brief Stored parameters for given dimension.
 * @tparam dim The dimension.
 **/
template <Dimension dim> struct stored_params;

template <> struct stored_params<Dimension::time>                   { typedef microseconds unit_inst_t;    }; // Stored parameters for time.
template <> struct stored_params<Dimension::distance>               { typedef micrometers unit_inst_t;     }; // Stored parameters for distance.
template <> struct stored_params<Dimension::weight>                 { typedef milligrams unit_inst_t;      }; // Stored parameters for weight.
template <> struct stored_params<Dimension::angle>                  { typedef radians unit_inst_t;         }; // Stored parameters for angle.
template <> struct stored_params<Dimension::voltage>                { typedef millivolts unit_inst_t;      }; // Stored parameters for voltage.
template <> struct stored_params<Dimension::current>                { typedef microampers unit_inst_t;     }; // Stored parameters for current.
template <> struct stored_params<Dimension::resistance>             { typedef ohms unit_inst_t;            }; // Stored parameters for resistance.
template <> struct stored_params<Dimension::speed>                  { typedef mm_per_sec unit_inst_t;      }; // Stored parameters for speed.
template <> struct stored_params<Dimension::acceleration>           { typedef mm_per_sec2 unit_inst_t;     }; // Stored parameters for acceleration.
template <> struct stored_params<Dimension::angular_velocity>       { typedef rad_per_sec unit_inst_t;     }; // Stored parameters for angular velocity.
template <> struct stored_params<Dimension::temperature>            { typedef celsius unit_inst_t;         }; // Stored parameters for temperature.
template <> struct stored_params<Dimension::magnetic_flux>          { typedef maxwell unit_inst_t;         }; // Stored parameters for magnetic flux.
template <> struct stored_params<Dimension::magnetic_flux_density>  { typedef gauss unit_inst_t;           }; // Stored parameters for magnetic flux density.

/* @brief Rescales value to another unit.
 * @restrict Type of the value must be arithmetic.
 * @tparam from The source unit
 * @tparam to The result unit.
 * @tparam T Type of the value.
 * @param value The value to rescale.
 * @returns The rescaled value.
 **/
template <typename from_unit_inst_t, typename to_unit_inst_t, typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
constexpr typename std::enable_if<from_unit_inst_t::dim == to_unit_inst_t::dim, T>::type rescale_unit(const T& value) {
    return value * from_unit_inst_t::mul / to_unit_inst_t::mul;
}

/* @brief Dimension class template. Used for basic types like time, distance, etc.
 * @tparam _dim The dimension.
 **/
template <Dimension _dim>
class dim_class {
public:
    static constexpr bool is_dim_class = true;       // Indicates that class is an uns dimension class.
    static constexpr Dimension dim = _dim;   // The dimension.
    typedef typename stored_params<_dim>::unit_inst_t stored_unit_inst_t;        // Stores instance type.
    static_assert(_dim == stored_unit_inst_t::dim, "Dimensions do not match!");  // Checks if the dimensions match.

private:
    float32_t value;   // The stored value.

    /* @brief Constructor - sets value.
     * @tparam T Numeric type of the parameter value.
     * param _value The value given in the unit instance.
     **/
    constexpr dim_class(float32_t _value) : value(_value) {}

public:
    static constexpr dim_class ZERO() { return dim_class(0.0f); }

    /* @brief Gets value in given unit.
     * @restrict Type must be unit instance type.
     * @restrict Dimension of the result instance type must be the same as the dimension of this type.
     * @tparam unit_inst_t The result unit instance type (e.g. milliseconds).
     * @returns The value in given unit.
     **/
    template <typename unit_inst_t, class = typename std::enable_if<unit_inst_t::is_unit_instance && unit_inst_t::dim == dim>::type>
    float32_t get() const {
        return rescale_unit<stored_unit_inst_t, unit_inst_t>(this->value);
    }

    /* @brief Sets value in given unit.
     * @restrict Type must be unit instance type.
     * @restrict Dimension of the source instance type must be the same as the dimension of this type.
     * @tparam unit_inst_t The source unit instance type (e.g. milliseconds).
     * @param _value The value in given unit.
     **/
    template <typename unit_inst_t, class = typename std::enable_if<unit_inst_t::is_unit_instance && unit_inst_t::dim == dim>::type>
    void set(float32_t _value) {
        this->value = rescale_unit<unit_inst_t, stored_unit_inst_t>(_value);
    }

    /* @brief Default constructor - sets value to 0.
     **/
    constexpr dim_class() : value(0.0f) {}

    /* @brief Constructor - sets value.
     * @tparam unit_inst_t Unit instance type.
     * @param [unnamed] The unit instance.
     * param _value The value given in the unit instance.
     **/
    template <typename unit_inst_t, class = typename std::enable_if<unit_inst_t::is_unit_instance && unit_inst_t::dim == dim>::type>
    constexpr dim_class(const unit_inst_t&, float32_t _value)
        : value(rescale_unit<unit_inst_t, stored_unit_inst_t>(_value)) {}

    /* @brief Creates dimension class instance from a value in given unit.
     * @restrict Type must be unit instance type.
     * @restrict Dimension of the source instance type must be the same as the dimension of this type.
     * @param _value The value in given unit.
     * @returns The result dimension class instance.
     **/
    template <typename unit_inst_t, class = typename std::enable_if<unit_inst_t::is_unit_instance && unit_inst_t::dim == dim>::type>
    static dim_class<dim> from(float32_t _value) {
        dim_class<dim> instance;
        instance.template set<unit_inst_t>(_value);
        return instance;
    }

    /* @brief Adds two dimension class instances.
     * @param other The other dimension class instance.
     * @returns The result of the addition.
     **/
    constexpr dim_class<dim> operator+(const dim_class<dim>& other) const {
        return dim_class<dim>(this->value + other.value);
    }

    /* @brief Subtracts two dimension class instances.
     * @param other The other dimension class instance.
     * @returns The result of the subtraction.
     **/
    constexpr dim_class<dim> operator-(const dim_class<dim>& other) const {
        return dim_class<dim>(this->value - other.value);
    }

    /* @brief Adds two dimension class instances and stores result in this instance.
     * @param other The other dimension class instance.
     * @returns This dimension class instance.
     **/
    dim_class<dim>& operator+=(const dim_class<dim>& other) {
        this->value += other.value;
        return *this;
    }

    /* @brief Subtracts two dimension class instances and stores result in this instance.
     * @param other The other dimension class instance.
     * @returns This dimension class instance.
     **/
    dim_class<dim>& operator-=(const dim_class<dim>& other) {
        this->value -= other.value;
        return *this;
    }

    /* @brief Compares two dimension class instances.
     * @param other The other dimension class instance.
     * @returns Boolean value indicating if the two dimension class instances are equal.
     **/
    bool operator==(const dim_class<dim>& other) const {
        return this->value == other.value;
    }

    /* @brief Compares two dimension class instances.
     * @param other The other dimension class instance.
     * @returns Boolean value indicating if the two dimension class instances are not equal.
     **/
    bool operator!=(const dim_class<dim>& other) const {
        return this->value != other.value;
    }

    /* @brief Compares two dimension class instances.
     * @param other The other dimension class instance.
     * @returns Boolean value indicating if this dimension class instances is greater than the other.
     **/
    bool operator>(const dim_class<dim>& other) const {
        return this->value > other.value;
    }

    /* @brief Compares two dimension class instances.
     * @param other The other dimension class instance.
     * @returns Boolean value indicating if this dimension class instances is smaller than the other.
     **/
    bool operator<(const dim_class<dim>& other) const {
        return this->value < other.value;
    }

    /* @brief Compares two dimension class instances.
     * @param other The other dimension class instance.
     * @returns Boolean value indicating if this dimension class instances is greater-or-equal than the other.
     **/
    bool operator>=(const dim_class<dim>& other) const {
        return this->value >= other.value;
    }

    /* @brief Compares two dimension class instances.
     * @param other The other dimension class instance.
     * @returns Boolean value indicating if this dimension class instances is smaller-or-equal than the other.
     **/
    bool operator<=(const dim_class<dim>& other) const {
        return this->value <= other.value;
    }

    /* @brief Gets ratio of two dimension class instances.
     * @param other The other dimension class instance.
     * @returns The ratio of the dimension class instances.
     **/
    constexpr float32_t operator/(const dim_class<dim>& other) const {
        return this->value / other.value;
    }

    /* @brief Multiplies this value with a constant.
     * @restrict Type of the constant must be arithmetic.
     * @tparam T Type of the constant.
     * @param c The constant.
     * @returns The result dimension class instance.
     **/
    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    constexpr dim_class<dim> operator*(const T& c) const {
        return dim_class<dim>(c * this->value);
    }

    /* @brief Multiplies dimension class instance with a constant.
     * @restrict Type of the constant must be arithmetic.
     * @param c The constant.
     * @param _dim_class_inst The dimension class instance.
     * @returns The result dimension class instance.
     **/
    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    friend constexpr dim_class<dim> operator*(const T& c, const dim_class<dim>& _dim_class_inst) {
        return _dim_class_inst * c;
    }

    /* @brief Divides this value by a constant.
     * @restrict Type of the constant must be arithmetic.
     * @param c The constant.
     * @returns The result dimension class instance.
     **/
    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    constexpr dim_class<dim> operator/(const T& c) const {
        return dim_class<dim>(this->value / c);
    }

    /* @brief Multiplies this value with -1.
     * @returns The result dimension class instance.
     **/
    constexpr dim_class<dim> operator-() const {
        return (*this) * (-1);
    }

    /* @brief Multiplies this value with a constant and stores result in this instance.
     * @restrict Type of the constant must be arithmetic.
     * @param c The constant.
     * @returns This dimension class instance.
     **/
    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    dim_class<dim>& operator*=(const T& c) {
        this->value *= c;
        return *this;
    }

    /* @brief Divides this value by a constant and stores result in this instance.
     * @restrict Type of the constant must be arithmetic.
     * @param c The constant.
     * @returns This dimension class instance.
     **/
    template <typename T, class = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    dim_class<dim>& operator/=(const T& c) {
        this->value /= c;
        return *this;
    }

    /* @brief Multiplies this value with another dimension class instance.
     * @param other The other dimension class instance.
     * @returns The result dimension class instance.
     **/
    template <typename _dim_class2, typename _res_dim_class = dim_class<mul_dim<dim, _dim_class2::dim>::value>>
    _res_dim_class operator*(const _dim_class2& other) const {
        return _res_dim_class::template from<mul_unit_instance<stored_unit_inst_t, typename _dim_class2::stored_unit_inst_t>>(this->value * other.template get<typename _dim_class2::stored_unit_inst_t>());
    }

    /* @brief Divides this value by another dimension class instance.
     * @param other The other dimension class instance.
     * @returns The result dimension class instance.
     **/
    template <typename _dim_class2, typename _res_dim_class = dim_class<div_dim<dim, _dim_class2::dim>::value>>
    _res_dim_class operator/(const _dim_class2& other) const {
        // converts this instance's value to the minimum unit to prevent rounding errors
        // e.g. dividing 1 millimeter (stored as 1 millimeter) by 1 second (stored as 1000000 microseconds)
        // without this conversion the result would be speed(millimeters_per_microseconds(), 1 / 1000000) -> 0 mm/sec
        // using this conversion the result will be speed(nanometers_per_microseconds(), 1000000 / 1000000) -> 1 mm/sec
        typedef unit_instance<dim, min_unit> min_unit_inst_t;
        float32_t ratio = this->get<min_unit_inst_t>() / other.template get<typename _dim_class2::stored_unit_inst_t>();
        return _res_dim_class::template from<div_unit_instance<min_unit_inst_t, typename _dim_class2::stored_unit_inst_t>>(ratio);
    }

    friend dim_class abs(const dim_class& inst) {
        return dim_class(abs(inst.value));
    }

    bool isZero(dim_class eps = dim_class(0.00001f)) const {
        return uns::isZero(this->value, eps.value);
    }
};
} // namespace detail

typedef detail::dim_class<Dimension::time>                  time_t;                     // Dimension class for time.
typedef detail::dim_class<Dimension::distance>              distance_t;                 // Dimension class for distance.
typedef detail::dim_class<Dimension::weight>                weight_t;                   // Dimension class for weight.
typedef detail::dim_class<Dimension::angle>                 angle_t;                    // Dimension class for angle.
typedef detail::dim_class<Dimension::voltage>               voltage_t;                  // Dimension class for voltage.
typedef detail::dim_class<Dimension::current>               current_t;                  // Dimension class for current.
typedef detail::dim_class<Dimension::resistance>            resistance_t;               // Dimension class for resistance.
typedef detail::dim_class<Dimension::speed>                 speed_t;                    // Dimension class for speed.
typedef detail::dim_class<Dimension::acceleration>          acceleration_t;             // Dimension class for acceleration.
typedef detail::dim_class<Dimension::angular_velocity>      angular_velocity_t;         // Dimension class for angular velocity.
typedef detail::dim_class<Dimension::temperature>           temperature_t;              // Dimension class for temperature.
typedef detail::dim_class<Dimension::magnetic_flux>         magnetic_flux_t;            // Dimension class for magnetic flux.
typedef detail::dim_class<Dimension::magnetic_flux_density> magnetic_flux_density_t;    // Dimension class for magnetic flux density.

} // namespace uns
