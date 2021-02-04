#pragma once

#include <micro/math/unit_utils.hpp>
#include <RaceTrackInfo.hpp>
#include <track.hpp>

template <typename T>
const T& track_get(const T values[], uint8_t lap) {
    return values[lap - 1];
}

template <typename T>
T track_map_linear(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const T& start, const T& end) {
    return micro::map(car.distance, trackInfo.segStartCarProps.distance, trackInfo.segStartCarProps.distance + trackInfo.seg->length, start, end);
}

template <typename T>
T track_map_pyramid(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const T& start, const T& middle, const T& end) {
    const micro::meter_t startDist  = trackInfo.segStartCarProps.distance;
    const micro::meter_t middleDist = startDist + trackInfo.seg->length / 2;
    const micro::meter_t endDist    = startDist + trackInfo.seg->length;

    return micro::isBtw(car.distance, startDist, middleDist) ?
        micro::map(car.distance, startDist, middleDist, start, middle) :
        micro::map(car.distance, middleDist, endDist, middle, end);
}

micro::millimeter_t track_map_pos_linear(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::millimeter_t& end);
micro::millimeter_t track_map_pos_pyramid(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::millimeter_t& middle, const micro::millimeter_t& end);
micro::radian_t track_map_angle_linear(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::radian_t& end);
micro::radian_t track_map_angle_pyramid(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::radian_t& middle, const micro::radian_t& end);

bool hasBecomeActive_Fast(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);
bool hasBecomeActive_BrakeSign(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);
bool hasBecomeActive_SingleLine(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);
bool hasBecomeActive_distance(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::LinePattern& pattern);

micro::ControlData getControl_CommonFast(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::MainLine& mainLine, const micro::m_per_sec_t targetSpeed);
micro::ControlData getControl_CommonSlow(const micro::CarProps& car, const RaceTrackInfo& trackInfo, const micro::MainLine& mainLine);
