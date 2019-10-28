#pragma once

#include "LinePattern.hpp"

namespace micro {

struct DetectedLines {
    Lines lines;
    Line mainLine;
    LinePattern pattern;
};

} // namespace micro
