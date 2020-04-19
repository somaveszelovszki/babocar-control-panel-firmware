#pragma once

#include <micro/container/vec.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>

#include <functional>

struct TrackInfo;

struct TrackSegment {
    bool isFast;
    std::function<bool(const TrackInfo&, const micro::LinePattern&)> hasBecomeActive;
    std::function<micro::ControlData(const TrackInfo&, const micro::LinePattern&, const micro::MainLine&)> getControl;
};

typedef micro::vec<TrackSegment, 20> TrackSegments;

struct TrackInfo {
    uint8_t lap = 0;
    TrackSegments::const_iterator seg;
    micro::CarProps segStartCarProps;
};

extern const TrackSegments trackSegments;
