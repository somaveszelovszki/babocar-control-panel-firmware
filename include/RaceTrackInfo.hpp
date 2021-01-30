#pragma once

#include <track.hpp>

struct RaceTrackInfo {
    const TrackSegments& segments;
    TrackSegments::const_iterator seg;
    micro::CarProps segStartCarProps;
    micro::OrientedLine segStartLine;
    uint8_t lap;
    micro::millisecond_t lapStartTime;

    RaceTrackInfo(const TrackSegments& segments);

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo, const micro::MainLine& mainLine);

    TrackSegments::const_iterator nextSegment() const;
};
