#pragma once

#include <micro/math/unit_utils.hpp>
#include <RaceTrackInfo.hpp>
#include <track.hpp>

using namespace micro;

template <typename T>
const T& track_get(const T values[], uint8_t lap) {
    return values[lap - 1];
}

template <typename T>
T track_map_linear(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const T& start, const T& end) {
    return map(car.distance, trackInfo.segStartCarProps.distance, trackInfo.segStartCarProps.distance + trackInfo.seg->length, start, end);
}

bool hasBecomeActive_Fast(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);
bool hasBecomeActive_BrakeSign(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);
bool hasBecomeActive_SingleLine(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);
bool hasBecomeActive_distance(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);

micro::ControlData getControl_CommonFast(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::MainLine& mainLine, const micro::m_per_sec_t targetSpeed);
micro::ControlData getControl_CommonSlow(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::MainLine& mainLine);
