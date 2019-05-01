#pragma once

#include <uns/Line.hpp>

namespace uns {

struct ControlProps {
    m_per_sec_t speed;
    Line line;
};

static_assert(uns_sizeof_ControlProps == sizeof(ControlProps), "Size of ControlProps does not match required value!");

} // namespace uns
