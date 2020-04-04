#pragma once

#include <micro/utils/units.hpp>

struct TrackSpeeds {
    micro::m_per_sec_t fast;
    micro::m_per_sec_t slow_1;
    micro::m_per_sec_t slow_2_begin;
    micro::m_per_sec_t slow_2;
    micro::m_per_sec_t slow_3;
    micro::m_per_sec_t slow_3_end;
    micro::m_per_sec_t slow_4;
};

namespace micro { DEFINE_TYPEINFO(TrackSpeeds); }
