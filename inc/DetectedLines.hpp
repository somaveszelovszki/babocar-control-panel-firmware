#pragma once

#include "LinePattern.hpp"

namespace micro {

struct DetectedLines {
    struct {
        Lines front;
        Lines rear;
    } lines;
    LinePattern pattern;
};

} // namespace micro
