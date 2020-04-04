#pragma once

#include <micro/utils/LinePattern.hpp>

namespace micro {

struct DetectedLines {
    struct {
        Lines lines;
        LinePattern pattern;
    } front, rear;
};

} // namespace micro
