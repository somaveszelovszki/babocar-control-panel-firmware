#include <cfg_track.hpp>

#if TRACK == TEST_TRACK

#include <RaceTrackController.hpp>

using namespace micro;

namespace {

template <typename T>
struct Data {
    T fast1;
    T slow1_prepare;
    T slow1_chicane1;
    T slow1_chicane2;
    T fast2;
    T slow2_prepare;
    T slow2_begin;
    T slow2_round1;
    T slow2_round2;
    T slow2_end1;
    T slow2_end2;
    T fast3;
    T slow3_prepare;
    T slow3_chicane1;
    T slow3_chicane2;
    T fast4;
    T slow4_prepare;
    T slow4_begin;
    T slow4_round1;
    T slow4_round2;
    T slow4_end1;
    T slow4_end2;
};

constexpr std::array speeds = {
//                   |fast1|        slow1        |fast2|                   slow2                  |fast3|        slow3        |fast4|                  slow4                   |
//                   |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |
    Data<m_per_sec_t>{{1.7}, {1.2}, {1.2}, {1.2}, {1.7}, {1.2}, {1.2}, {1.2}, {1.2}, {1.2}, {1.2}, {1.7}, {1.8}, {1.8}, {1.8}, {3.0}, {1.7}, {1.7}, {2.0}, {2.0}, {1.7}, {1.7}}, // Lap 1
    Data<m_per_sec_t>{{2.0}, {1.6}, {1.6}, {1.6}, {3.0}, {1.8}, {1.8}, {2.0}, {2.0}, {1.8}, {1.8}, {3.0}, {1.6}, {1.8}, {1.8}, {3.0}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}}, // Lap 2
    Data<m_per_sec_t>{{3.0}, {1.5}, {1.5}, {1.5}, {3.0}, {1.2}, {1.2}, {1.2}, {1.2}, {1.2}, {1.2}, {1.7}, {1.6}, {1.6}, {1.6}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}}, // Lap 3
    Data<m_per_sec_t>{{3.0}, {2.0}, {2.0}, {2.0}, {3.0}, {2.0}, {2.0}, {2.2}, {2.2}, {2.0}, {2.0}, {3.0}, {2.2}, {2.2}, {2.2}, {3.0}, {2.0}, {2.0}, {2.2}, {2.2}, {2.0}, {2.0}}, // Lap 4
    Data<m_per_sec_t>{{3.0}, {2.2}, {2.2}, {2.2}, {3.0}, {2.0}, {2.0}, {2.2}, {2.2}, {2.0}, {2.0}, {3.0}, {2.2}, {2.2}, {2.2}, {3.0}, {2.0}, {2.0}, {2.2}, {2.2}, {2.0}, {2.0}}, // Lap 5
    Data<m_per_sec_t>{{3.0}, {2.2}, {2.2}, {2.2}, {3.0}, {2.0}, {2.0}, {2.2}, {2.2}, {2.0}, {2.0}, {3.0}, {2.2}, {2.2}, {2.2}, {3.0}, {2.0}, {2.0}, {2.2}, {2.2}, {2.0}, {2.0}}, // Lap 6
    Data<m_per_sec_t>{{3.0}                                                                                                                                                                        }  // Finish
};

constexpr std::array rampTimes = {
//                     |fast1|        slow1        |fast2|                   slow2                  |fast3|        slow3        |fast4|                  slow4                   |
//                     |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 1
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 2
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 3
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 4
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 5
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 6
    Data<millisecond_t>{{800}                                                                                                                                                                        }  // Finish
};

constexpr std::array endLinePositions = {
//                    |fast1|        slow1        |fast2|                   slow2                  |fast3|        slow3        |fast4|                  slow4                   |
//                    |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |
    Data<centimeter_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 1
    Data<centimeter_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 2
    Data<centimeter_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 3
    Data<centimeter_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 4
    Data<centimeter_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 5
    Data<centimeter_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 6
    Data<centimeter_t>{{ 0 }                                                                                                                                                                        }  // Finish
};

constexpr std::array endLineAngles = {
//                |fast1|        slow1        |fast2|                   slow2                  |fast3|        slow3        |fast4|                  slow4                   |
//                |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |     |prepare chic1  chic2 |     |prepare begin  round1 round2  end1   end2 |
    Data<degree_t>{{ 0 }, {-15}, { 15}, { 0 }, { 0 }, { 15}, {-15}, {-15}, {-15}, { 15}, { 0 }, { 0 }, {-15}, { 15}, { 0 }, { 0 }, { 15}, {-15}, {-15}, {-15}, { 15}, { 0 }}, // Lap 1
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 2
    Data<degree_t>{{ 0 }, {-15}, { 15}, { 0 }, { 0 }, { 15}, {-15}, {-15}, {-15}, { 15}, { 0 }, { 0 }, {-15}, { 15}, { 0 }, { 0 }, { 15}, {-15}, {-15}, {-15}, { 15}, { 0 }}, // Lap 3
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 4
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 5
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 6
    Data<degree_t>{{ 0 }                                                                                                                                                                        }  // Finish
};

LapTrackSections buildLapTrackSections(
        const Data<m_per_sec_t>& speeds,
        const Data<millisecond_t>& rampTimes,
        const Data<centimeter_t>& endLinePositions,
        const Data<degree_t>& endLineAngles,
        const OrientedLine& lineGradientStart) {

#define ADD_TRACK_SECTION(name, isFast, length, transitionCriteria)                          \
sections.push_back(TrackSection{                                                             \
    #name,                                                                                   \
    isFast,                                                                                  \
    length,                                                                                  \
    TrackSection::TransitionCriteria::transitionCriteria,                                    \
    speeds.name,                                                                             \
    rampTimes.name,                                                                          \
    {                                                                                        \
        sections.empty() ? lineGradientStart : sections.back()->control.lineGradient.second, \
        { endLinePositions.name, endLineAngles.name }                                        \
    }                                                                                        \
})

    LapTrackSections sections;

    ADD_TRACK_SECTION(fast1,          true,  meter_t(9.00), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow1_prepare,  false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow1_chicane1, false, meter_t(0.65), distance()                       );
    ADD_TRACK_SECTION(slow1_chicane2, false, meter_t(0.65), acceleration()                   );
    ADD_TRACK_SECTION(fast2,          true,  meter_t(9.70), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow2_prepare,  false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow2_begin,    false, meter_t(1.20), distance()                       );
    ADD_TRACK_SECTION(slow2_round1,   false, meter_t(1.55), distance()                       );
    ADD_TRACK_SECTION(slow2_round2,   false, meter_t(1.55), distance()                       );
    ADD_TRACK_SECTION(slow2_end1,     false, meter_t(0.60), distance()                       );
    ADD_TRACK_SECTION(slow2_end2,     false, meter_t(0.60), acceleration()                   );
    ADD_TRACK_SECTION(fast3,          true,  meter_t(9.70), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow3_prepare,  false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow3_chicane1, false, meter_t(0.65), distance()                       );
    ADD_TRACK_SECTION(slow3_chicane2, false, meter_t(0.65), acceleration()                   );
    ADD_TRACK_SECTION(fast4,          true,  meter_t(9.00), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow4_prepare,  false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow4_begin,    false, meter_t(2.00), distance()                       );
    ADD_TRACK_SECTION(slow4_round1,   false, meter_t(1.55), distance()                       );
    ADD_TRACK_SECTION(slow4_round2,   false, meter_t(1.55), distance()                       );
    ADD_TRACK_SECTION(slow4_end1,     false, meter_t(0.85), distance()                       );
    ADD_TRACK_SECTION(slow4_end2,     false, meter_t(0.85), acceleration()                   );

    return sections;
}

} // namespace

RaceTrackSections buildRaceTrackSections() {
    RaceTrackSections sections;

    static_assert(sections.size() == speeds.size(), "Speeds array size is incorrect!");
    static_assert(sections.size() == rampTimes.size(), "Ramp times array size is incorrect!");
    static_assert(sections.size() == endLinePositions.size(), "End line positions array size is incorrect!");
    static_assert(sections.size() == endLineAngles.size(), "End line angles array size is incorrect!");

    for (uint32_t i = 0u; i < sections.size(); ++i) {
        const OrientedLine lineGradientStart = i == 0 ? OrientedLine() : sections[i - 1].back()->lineGradient.second;
        sections[i] = buildLapTrackSections(speeds[i], rampTimes[i], endLinePositions[i], endLineAngles[i], lineGradientStart);
    }

    return sections;
}

#endif // TRACK == RACE_TRACK

