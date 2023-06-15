#include <cfg_track.hpp>

#if TRACK == RACE_TRACK

#include <RaceTrackController.hpp>

using namespace micro;

namespace {

template <typename T>
struct Data {
    T fast1;
    T slow1_prepare;
    T slow1_round1;
    T slow1_round2;
    T fast2;
    T slow2_prepare;
    T slow2_begin1;
    T slow2_begin2;
    T slow2_round1;
    T slow2_round2;
    T fast3;
    T slow3_prepare;
    T slow3_round1;
    T slow3_round2;
    T slow3_end1;
    T slow3_end2;
    T fast4;
    T slow4_prepare;
    T slow4_round1;
    T slow4_round2;
};

constexpr std::array speeds = {
//                   |fast1|         slow1       |fast2|               slow2               |fast3|               slow3               |fast4|        slow4        |
//                   |     |prepare round1 round2|     |prepare begin1 begin2 round1 round2|     |prepare round1 round2  end1   end2 |     |prepare round1 round2|
    Data<m_per_sec_t>{{1.0}, {1.2}, {1.2}, {1.2}, {1.7}, {1.2}, {1.2}, {1.2}, {1.2}, {1.2}, {1.7}, {1.6}, {1.6}, {1.6}, {1.6}, {1.6}, {3.5}, {2.0}, {2.0}, {2.0}}, // Lap 1
    Data<m_per_sec_t>{{3.0}, {2.3}, {2.3}, {2.3}, {4.5}, {2.0}, {2.0}, {2.0}, {2.4}, {2.4}, {4.5}, {2.4}, {2.4}, {2.4}, {2.0}, {2.0}, {4.5}, {2.3}, {2.3}, {2.3}}, // Lap 2
    Data<m_per_sec_t>{{3.6}, {1.5}, {1.5}, {1.5}, {3.6}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}, {1.7}, {1.8}, {1.8}, {1.8}, {1.6}, {1.6}, {2.2}, {2.0}, {2.0}, {2.0}}, // Lap 3
    Data<m_per_sec_t>{{5.5}, {2.6}, {2.6}, {2.6}, {5.5}, {2.2}, {2.2}, {2.2}, {2.5}, {2.5}, {5.5}, {2.5}, {2.5}, {2.5}, {2.2}, {2.2}, {5.5}, {2.4}, {2.6}, {2.6}}, // Lap 4
    Data<m_per_sec_t>{{6.0}, {2.8}, {3.0}, {3.0}, {6.0}, {2.5}, {2.5}, {2.5}, {2.7}, {2.7}, {6.0}, {2.8}, {3.0}, {2.7}, {2.2}, {2.2}, {6.0}, {2.6}, {2.8}, {2.8}}, // Lap 5
    Data<m_per_sec_t>{{7.3}, {2.8}, {3.0}, {3.0}, {7.3}, {2.5}, {2.5}, {2.5}, {2.7}, {2.7}, {7.3}, {2.8}, {3.0}, {2.7}, {2.2}, {2.2}, {7.3}, {2.4}, {2.7}, {2.7}}, // Lap 6
    Data<m_per_sec_t>{{7.3}                                                                                                                                     }  // Finish
};

constexpr std::array rampTimes = {
//                     |fast1|         slow1       |fast2|               slow2               |fast3|               slow3               |fast4|        slow4        |
//                     |     |prepare round1 round2|     |prepare begin1 begin2 round1 round2|     |prepare round1 round2  end1   end2 |     |prepare round1 round2|
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 1
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 2
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 3
    Data<millisecond_t>{{600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}}, // Lap 4
    Data<millisecond_t>{{600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}}, // Lap 5
    Data<millisecond_t>{{600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}, {600}}, // Lap 6
    Data<millisecond_t>{{600}                                                                                                                                     }  // Finish
};

constexpr std::array endLinePositions = {
//                    |fast1|         slow1       |fast2|               slow2               |fast3|               slow3               |fast4|        slow4        |
//                    |     |prepare round1 round2|     |prepare begin1 begin2 round1 round2|     |prepare round1 round2  end1   end2 |     |prepare round1 round2|
    Data<centimeter_t>{{ 0 }, { 0 }, { 5 }, { 0 }, { 0 }, { 0 }, {-5 }, { 0 }, { 5 }, { 0 }, { 0 }, { 0 }, { 5 }, { 0 }, {-5 }, { 0 }, { 0 }, { 0 }, { 5 }, { 0 }}, // Lap 1
    Data<centimeter_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 2
    Data<centimeter_t>{{ 0 }, { 0 }, { 5 }, { 0 }, { 0 }, { 0 }, {-5 }, { 0 }, { 5 }, { 0 }, { 0 }, { 0 }, { 5 }, { 0 }, {-5 }, { 0 }, { 0 }, { 0 }, { 5 }, { 0 }}, // Lap 3
    Data<centimeter_t>{{ 0 }, {-5 }, { 12}, { 0 }, { 0 }, { 5 }, {-12}, { 0 }, { 12}, { 0 }, { 0 }, { 0 }, { 12}, { 0 }, {-12}, { 0 }, { 0 }, {-5 }, { 12}, { 0 }}, // Lap 4
    Data<centimeter_t>{{ 0 }, {-5 }, { 12}, { 0 }, { 0 }, { 5 }, {-12}, { 0 }, { 12}, { 0 }, { 0 }, { 0 }, { 12}, { 0 }, {-12}, { 0 }, { 0 }, {-5 }, { 12}, { 0 }}, // Lap 5
    Data<centimeter_t>{{ 0 }, {-5 }, { 12}, { 0 }, { 0 }, { 5 }, {-12}, { 0 }, { 12}, { 0 }, { 0 }, { 0 }, { 12}, { 0 }, {-12}, { 0 }, { 0 }, {-5 }, { 12}, { 0 }}, // Lap 6
    Data<centimeter_t>{{ 0 }                                                                                                                                     }  // Finish
};

constexpr std::array endLineAngles = {
//                |fast1|         slow1       |fast2|               slow2               |fast3|               slow3               |fast4|        slow4        |
//                |     |prepare round1 round2|     |prepare begin1 begin2 round1 round2|     |prepare round1 round2  end1   end2 |     |prepare round1 round2|
    Data<degree_t>{{ 0 }, {-15}, {-7 }, { 0 }, { 0 }, { 15}, { 5 }, {-5 }, {-15}, { 0 }, { 0 }, { 0 }, {-15}, { 5 }, { 15}, { 0 }, { 0 }, {-15}, {-7 }, { 0 }}, // Lap 1
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 2
    Data<degree_t>{{ 0 }, {-15}, {-7 }, { 0 }, { 0 }, { 15}, { 5 }, {-5 }, {-15}, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 3
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 4
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 5
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 6
    Data<degree_t>{{ 0 }                                                                                                                                     }  // Finish
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

    ADD_TRACK_SECTION(fast1,         true,  meter_t(5.60), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow1_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow1_round1,  false, meter_t(1.20), distance()                       );
    ADD_TRACK_SECTION(slow1_round2,  false, meter_t(1.20), acceleration()                   );
    ADD_TRACK_SECTION(fast2,         true,  meter_t(7.30), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow2_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow2_begin1,  false, meter_t(1.00), distance()                       );
    ADD_TRACK_SECTION(slow2_begin2,  false, meter_t(1.00), distance()                       );
    ADD_TRACK_SECTION(slow2_round1,  false, meter_t(1.90), distance()                       );
    ADD_TRACK_SECTION(slow2_round2,  false, meter_t(1.40), acceleration()                   );
    ADD_TRACK_SECTION(fast3,         true,  meter_t(7.70), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow3_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow3_round1,  false, meter_t(1.90), distance()                       );
    ADD_TRACK_SECTION(slow3_round2,  false, meter_t(1.70), distance()                       );
    ADD_TRACK_SECTION(slow3_end1,    false, meter_t(0.80), distance()                       );
    ADD_TRACK_SECTION(slow3_end2,    false, meter_t(0.80), acceleration()                   );
    ADD_TRACK_SECTION(fast4,         true,  meter_t(7.30), pattern(LinePattern::BRAKE)      );
    ADD_TRACK_SECTION(slow4_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow4_round1,  false, meter_t(1.10), distance()                       );
    ADD_TRACK_SECTION(slow4_round2,  false, meter_t(1.10), acceleration()                   );

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
        const OrientedLine lineGradientStart = i == 0 ? OrientedLine() : sections[i - 1].back()->control.lineGradient.second;
        sections[i] = buildLapTrackSections(speeds[i], rampTimes[i], endLinePositions[i], endLineAngles[i], lineGradientStart);
    }

    return sections;
}

#endif // TRACK == RACE_TRACK
