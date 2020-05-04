#pragma once

#include <micro/container/vec.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>

#include <cfg_track.hpp>

#include <functional>

struct TrackSpeeds {
    micro::m_per_sec_t fast;

#if TRACK == RACE_TRACK
    micro::m_per_sec_t slow1_prepare;
    micro::m_per_sec_t slow1_round_begin;
    micro::m_per_sec_t slow1_round_end;
    micro::m_per_sec_t slow2_prepare;
    micro::m_per_sec_t slow2_begin;
    micro::m_per_sec_t slow2_round_begin;
    micro::m_per_sec_t slow2_round_end;
    micro::m_per_sec_t slow3_prepare;
    micro::m_per_sec_t slow3_round_begin;
    micro::m_per_sec_t slow3_round_end;
    micro::m_per_sec_t slow3_end;
    micro::m_per_sec_t slow4_prepare;
    micro::m_per_sec_t slow4_round_begin;
    micro::m_per_sec_t slow4_round_end;
#elif TRACK == TEST_TRACK
    micro::m_per_sec_t slow1_prepare;
    micro::m_per_sec_t slow1_chicane;
    micro::m_per_sec_t slow2_prepare;
    micro::m_per_sec_t slow2_begin_chicane;
    micro::m_per_sec_t slow2_round_begin;
    micro::m_per_sec_t slow2_round_end;
    micro::m_per_sec_t slow2_end_chicane;
    micro::m_per_sec_t slow3_prepare;
    micro::m_per_sec_t slow3_chicane;
    micro::m_per_sec_t slow4_prepare;
    micro::m_per_sec_t slow4_begin_chicane;
    micro::m_per_sec_t slow4_round_begin;
    micro::m_per_sec_t slow4_round_end;
    micro::m_per_sec_t slow4_end_chicane;
#endif
};

struct BrakeOffsets {
    micro::meter_t slow1;
    micro::meter_t slow2;
    micro::meter_t slow3;
    micro::meter_t slow4;
};

struct TrackInfo;

struct TrackSegment {
    bool isFast;
    micro::meter_t length;
    std::function<bool(const TrackInfo&, const micro::LinePattern&)> hasBecomeActive;
    std::function<micro::ControlData(const TrackInfo&, const micro::MainLine&)> getControl;
};

typedef micro::vec<TrackSegment, 20> TrackSegments;

struct TrackInfo {
    uint8_t lap = 0;
    TrackSegments::const_iterator seg;
    micro::CarProps segStartCarProps;
    micro::OrientedLine segStartLine;
};

extern const TrackSegments trackSegments;
