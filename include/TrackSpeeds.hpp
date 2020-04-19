#pragma once

#include <micro/utils/units.hpp>

struct TrackSpeeds {
    micro::m_per_sec_t fast;
    micro::m_per_sec_t slow1_prepare;
    micro::m_per_sec_t slow1_round;
    micro::m_per_sec_t slow2_prepare;
    micro::m_per_sec_t slow2_begin;
    micro::m_per_sec_t slow2_round;
    micro::m_per_sec_t slow3_prepare;
    micro::m_per_sec_t slow3_round;
    micro::m_per_sec_t slow3_end;
    micro::m_per_sec_t slow4_prepare;
    micro::m_per_sec_t slow4_round;
};

namespace micro { DEFINE_TYPEINFO(TrackSpeeds); }
