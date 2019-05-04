#pragma once

#include <uns/util/units.hpp>

namespace uns {

static constexpr float32_t SQRT_2 = std::sqrt(2.0f);        // sqrt(2)
static constexpr float32_t SQRT_3 = std::sqrt(3.0f);        // sqrt(3)

constexpr radian_t PI = radian_t(3.14159265358979323846);   // Pi
constexpr radian_t PI_2 = PI / 2;                           // Pi / 2
constexpr radian_t PI_4 = PI / 4;                           // Pi / 4

const m_per_sec2_t G = m_per_sec2_t(9.81f);  // Gravitational acceleration.

} // namespace uns
