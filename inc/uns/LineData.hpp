#pragma once

#include <uns/LinePattern.hpp>
#include <uns/Point2.hpp>
#include <uns/container/Vec.hpp>
#include <uns/config/cfg_track.hpp>

namespace uns {
struct LineData {
    Line centerLine;
    LinePattern pattern;
    Vec<distance_t, cfg::MAX_LINES> coarsePositions;
};

static_assert(sizeof(LineData) == uns_sizeof_DetectedLines, "Size of LineData does not match required size!");
} // namespace uns
