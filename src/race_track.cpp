#include <cfg_track.hpp>

#if TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

#include <RaceTrackController.hpp>
#include <track.hpp>

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
    Data<m_per_sec_t>{{3.5}, {1.8}, {1.8}, {1.8}, {3.5}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}, {3.5}, {1.8}, {1.8}, {1.8}, {1.8}, {1.8}, {3.5}, {1.8}, {1.8}, {1.8}}, // Lap 2
    Data<m_per_sec_t>{{3.6}, {1.5}, {1.5}, {1.5}, {3.6}, {1.4}, {1.4}, {1.4}, {1.4}, {1.4}, {1.7}, {1.1}, {1.1}, {1.1}, {1.1}, {1.1}, {1.3}, {1.8}, {1.8}, {1.8}}, // Lap 3
    Data<m_per_sec_t>{{5.5}, {2.0}, {2.0}, {2.0}, {5.5}, {2.0}, {2.0}, {2.0}, {2.0}, {2.0}, {5.5}, {2.0}, {2.0}, {2.0}, {2.0}, {2.0}, {5.5}, {2.2}, {2.2}, {2.2}}, // Lap 4
    Data<m_per_sec_t>{{6.5}, {2.2}, {2.2}, {2.2}, {6.5}, {2.0}, {2.0}, {2.0}, {2.2}, {2.2}, {6.5}, {2.2}, {2.2}, {2.2}, {2.2}, {2.2}, {6.5}, {2.3}, {2.3}, {2.3}}, // Lap 5
    Data<m_per_sec_t>{{7.3}, {2.3}, {2.3}, {2.3}, {6.5}, {2.0}, {2.0}, {2.0}, {2.2}, {2.2}, {6.8}, {2.2}, {2.2}, {2.2}, {2.2}, {2.2}, {6.8}, {2.3}, {2.3}, {2.3}}, // Lap 6
    Data<m_per_sec_t>{{7.3}                                                                                                                                     }  // Finish
};

constexpr std::array rampTimes = {
//                     |fast1|         slow1       |fast2|               slow2               |fast3|               slow3               |fast4|        slow4        |
//                     |     |prepare round1 round2|     |prepare begin1 begin2 round1 round2|     |prepare round1 round2  end1   end2 |     |prepare round1 round2|
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 1
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 2
    Data<millisecond_t>{{800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}, {800}}, // Lap 3
    Data<millisecond_t>{{500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}}, // Lap 4
    Data<millisecond_t>{{500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}}, // Lap 5
    Data<millisecond_t>{{500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}, {500}}, // Lap 6
    Data<millisecond_t>{{500}                                                                                                                                     }  // Finish
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
    Data<degree_t>{{ 0 }, {-15}, {-15}, { 0 }, { 0 }, { 15}, { 10}, {-15}, {-15}, { 0 }, { 0 }, {-15}, {-15}, { 15}, { 10}, { 0 }, { 0 }, { 0 }, {-15}, { 0 }}, // Lap 1
    Data<degree_t>{{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }}, // Lap 2
    Data<degree_t>{{ 0 }, {-15}, {-15}, { 0 }, { 0 }, { 15}, { 10}, {-15}, {-15}, { 0 }, { 0 }, {-15}, {-15}, { 15}, { 10}, { 0 }, { 0 }, { 0 }, {-15}, { 0 }}, // Lap 3
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

#define ADD_TRACK_SECTION(name, isFast, length, transitionCriteria)                             \
sections.push_back(TrackSection{                                                                \
    isFast,                                                                                     \
    length,                                                                                     \
    TrackSection::TransitionCriteria::transitionCriteria,                                       \
    TrackSection::ControlParameters{                                                            \
        speeds.name,                                                                            \
        rampTimes.name,                                                                         \
        {                                                                                       \
            sections.empty() ? lineGradientStart : sections.back().control.lineGradient.second, \
            { endLinePositions.name, endLineAngles.name }                                       \
        }                                                                                       \
    }                                                                                           \
})

    LapTrackSections sections;

    ADD_TRACK_SECTION(fast1,         true,  meter_t(5.60), pattern(LinePattern::BRAKE));
    ADD_TRACK_SECTION(slow1_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow1_round1,  false, meter_t(1.20), distance());
    ADD_TRACK_SECTION(slow1_round2,  false, meter_t(1.20), acceleration());
    ADD_TRACK_SECTION(fast2,         true,  meter_t(7.30), pattern(LinePattern::BRAKE));
    ADD_TRACK_SECTION(slow2_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow2_begin1,  false, meter_t(1.00), distance());
    ADD_TRACK_SECTION(slow2_begin2,  false, meter_t(1.00), distance());
    ADD_TRACK_SECTION(slow2_round1,  false, meter_t(1.90), distance());
    ADD_TRACK_SECTION(slow2_round2,  false, meter_t(1.40), acceleration());
    ADD_TRACK_SECTION(fast3,         true,  meter_t(7.70), pattern(LinePattern::BRAKE));
    ADD_TRACK_SECTION(slow3_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow3_round1,  false, meter_t(1.90), distance());
    ADD_TRACK_SECTION(slow3_round2,  false, meter_t(1.70), distance());
    ADD_TRACK_SECTION(slow3_end1,    false, meter_t(0.80), distance());
    ADD_TRACK_SECTION(slow3_end2,    false, meter_t(0.80), acceleration());
    ADD_TRACK_SECTION(fast4,         true,  meter_t(7.30), pattern(LinePattern::BRAKE));
    ADD_TRACK_SECTION(slow4_prepare, false, meter_t(3.00), pattern(LinePattern::SINGLE_LINE));
    ADD_TRACK_SECTION(slow4_round1,  false, meter_t(1.10), distance());
    ADD_TRACK_SECTION(slow4_round2,  false, meter_t(1.10), acceleration());

    return sections;
}

RaceLapTrackSectionProvider provider;

} // namespace

LapTrackSections RaceLapTrackSectionProvider::operator()(const size_t lap) {
	const auto i = lap - 1;
	const auto sections = buildLapTrackSections(speeds[i], rampTimes[i], endLinePositions[i], endLineAngles[i], lastLineGradient_);
	lastLineGradient_ = sections.back().control.lineGradient.second;
	return sections;
}

OverridableLapTrackSectionProvider raceLapTrackSectionProvider(provider);

#endif // TRACK == RACE_TRACK || COMPILE_ALL_TRACKS
