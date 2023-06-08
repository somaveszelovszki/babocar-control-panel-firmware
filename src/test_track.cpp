#include <cfg_track.hpp>

#if TRACK == TEST_TRACK

#include <RaceTrackInfo.hpp>
#include <track_utils.hpp>

using namespace micro;

namespace {

constexpr millimeter_t ROUND_LINE_OFFSET_SAFETY_CAR = centimeter_t(5);
constexpr millimeter_t ROUND_LINE_OFFSET_RACE       = centimeter_t(12);
constexpr radian_t ROUND_LINE_ANGLE_SAFETY_CAR      = degree_t(15);

constexpr uint32_t FAST1               = 0u;
constexpr uint32_t SLOW1_PREPARE       = 1u;
constexpr uint32_t SLOW1_CHICANE       = 2u;
constexpr uint32_t FAST2               = 3u;
constexpr uint32_t SLOW2_PREPARE       = 4u;
constexpr uint32_t SLOW2_BEGIN_CHICANE = 5u;
constexpr uint32_t SLOW2_ROUND_BEGIN   = 6u;
constexpr uint32_t SLOW2_ROUND_END     = 7u;
constexpr uint32_t SLOW2_END_CHICANE   = 8u;
constexpr uint32_t FAST3               = 9u;
constexpr uint32_t SLOW3_PREPARE       = 10u;
constexpr uint32_t SLOW3_CHICANE       = 11u;
constexpr uint32_t FAST4               = 12u;
constexpr uint32_t SLOW4_PREPARE       = 13u;
constexpr uint32_t SLOW4_BEGIN_CHICANE = 14u;
constexpr uint32_t SLOW4_ROUND_BEGIN   = 15u;
constexpr uint32_t SLOW4_ROUND_END     = 16u;
constexpr uint32_t SLOW4_END_CHICANE   = 17u;

ControlData getControl_Fast1(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap)[FAST1]);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast1;

    return controlData;
}

ControlData getControl_Fast2(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap)[FAST2]);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast2;

    return controlData;
}

ControlData getControl_Fast3(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap)[FAST3]);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast3;

    return controlData;
}

ControlData getControl_Fast4(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap)[FAST4]);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast4;

    return controlData;
}

ControlData getControl_Slow1_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow1 ? speeds[SLOW1_PREPARE] : speeds[FAST1];
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast1;

    return controlData;
}

ControlData getControl_Slow1_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW1_CHICANE];

    return controlData;
}

ControlData getControl_Slow2_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow2 ? speeds[SLOW2_PREPARE] : speeds[FAST2];
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast2;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.angle = track_map_angle_linear(car, trackInfo, ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_begin_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW2_BEGIN_CHICANE];

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, ROUND_LINE_OFFSET_SAFETY_CAR);
        controlData.lineControl.target.angle = track_map_angle_linear(car, trackInfo, -ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW2_ROUND_BEGIN];

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, ROUND_LINE_OFFSET_RACE);
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW2_ROUND_END];

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, centimeter_t(0));
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_end_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW2_END_CHICANE];

    return controlData;
}

ControlData getControl_Slow3_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow3 ? speeds[SLOW3_PREPARE] : speeds[FAST3];
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast3;

    return controlData;
}

ControlData getControl_Slow3_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW3_CHICANE];

    return controlData;
}

ControlData getControl_Slow4_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow4 ? speeds[SLOW4_PREPARE] : speeds[FAST4];
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast4;

    return controlData;
}

ControlData getControl_Slow4_begin_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW4_BEGIN_CHICANE];

    return controlData;
}

ControlData getControl_Slow4_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW4_ROUND_BEGIN];

    controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, ROUND_LINE_OFFSET_RACE);
    controlData.lineControl.target.angle = radian_t(0);

    return controlData;
}

ControlData getControl_Slow4_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW4_ROUND_END];

    controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, centimeter_t(0));
    controlData.lineControl.target.angle = radian_t(0);

    return controlData;
}

ControlData getControl_Slow4_end_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW4_END_CHICANE];

    return controlData;
}

} // namespace

TrackSpeeds trackSpeeds[cfg::NUM_RACE_LAPS + 1] = {
//  ||  fast1  ||        slow1       ||  fast2  ||                       slow2                         ||  fast3  ||        slow3       ||  fast4  ||                        slow4                        ||
//  ||         || prepare    chicane ||         || prepare   begin_chi round_begin round_end   end_chi ||         || prepare    chicane ||         || prepare   begin_chi round_begin round_end   end_chi ||
    { { 1.70f }, { 1.20f }, { 1.20f }, { 1.70f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.70f }, { 1.80f }, { 1.80f }, { 3.00f }, { 1.70f }, { 1.70f }, { 2.00f }, { 2.00f }, { 1.70f } }, // Lap 1
    { { 2.00f }, { 1.60f }, { 1.60f }, { 3.00f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 1.80f }, { 3.00f }, { 1.60f }, { 1.80f }, { 3.00f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 2
    { { 3.00f }, { 1.50f }, { 1.50f }, { 3.00f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.70f }, { 1.60f }, { 1.60f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 3
    { { 3.00f }, { 2.00f }, { 2.00f }, { 3.00f }, { 2.00f }, { 2.00f }, { 2.20f }, { 2.20f }, { 2.00f }, { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.00f }, { 2.00f }, { 2.20f }, { 2.20f }, { 2.00f } }, // Lap 4
    { { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.00f }, { 2.00f }, { 2.20f }, { 2.20f }, { 2.00f }, { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.00f }, { 2.00f }, { 2.20f }, { 2.20f }, { 2.00f } }, // Lap 5
    { { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.00f }, { 2.00f }, { 2.20f }, { 2.20f }, { 2.00f }, { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.00f }, { 2.00f }, { 2.20f }, { 2.20f }, { 2.00f } }, // Lap 6
    { { 3.00f }                                                                                                                                                                                            }  // Finish
};

AccelerationRamps accelerationRamps[cfg::NUM_RACE_LAPS + 1] = {
//  ||      slow1        ||      slow2        ||      slow3        ||      slow4        ||
    { millisecond_t(1000), millisecond_t(1000), millisecond_t(1000), millisecond_t(1000) }, // Lap 1
    { millisecond_t(1000), millisecond_t(1000), millisecond_t(1000), millisecond_t(1000) }, // Lap 2
    { millisecond_t(1000), millisecond_t(1000), millisecond_t(1000), millisecond_t(1000) }, // Lap 3
    { millisecond_t(1000), millisecond_t(1000), millisecond_t(1000), millisecond_t(1000) }, // Lap 4
    { millisecond_t(1000), millisecond_t(1000), millisecond_t(1000), millisecond_t(1000) }, // Lap 5
    { millisecond_t(1000), millisecond_t(1000), millisecond_t(1000), millisecond_t(1000) }, // Lap 6
    { millisecond_t(1000)                                                                }  // Finish
};

BrakeOffsets brakeOffsets[cfg::NUM_RACE_LAPS + 1] = {
//  ||     slow1       ||     slow2       ||     slow3       ||     slow4       ||
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 1
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 2
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 3
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 4
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 5
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 6
    { centimeter_t(0)                                                    }  // Finish
};

const TrackSegments trackSegments = {
    { true,  meter_t(9.00f), hasBecomeActive_Fast,       getControl_Fast1               },
    { false, meter_t(3.00f), hasBecomeActive_BrakeSign,  getControl_Slow1_prepare       },
    { false, meter_t(1.30f), hasBecomeActive_SingleLine, getControl_Slow1_chicane       },
    { true,  meter_t(9.70f), hasBecomeActive_Fast,       getControl_Fast2               },
    { false, meter_t(3.00f), hasBecomeActive_BrakeSign,  getControl_Slow2_prepare       },
    { false, meter_t(1.20f), hasBecomeActive_SingleLine, getControl_Slow2_begin_chicane },
    { false, meter_t(1.55f), hasBecomeActive_distance,   getControl_Slow2_round_begin   },
    { false, meter_t(1.55f), hasBecomeActive_distance,   getControl_Slow2_round_end     },
    { false, meter_t(1.20f), hasBecomeActive_distance,   getControl_Slow2_end_chicane   },
    { true,  meter_t(9.70f), hasBecomeActive_Fast,       getControl_Fast3               },
    { false, meter_t(3.00f), hasBecomeActive_BrakeSign,  getControl_Slow3_prepare       },
    { false, meter_t(1.30f), hasBecomeActive_SingleLine, getControl_Slow3_chicane       },
    { true,  meter_t(9.00f), hasBecomeActive_Fast,       getControl_Fast4               },
    { false, meter_t(3.00f), hasBecomeActive_BrakeSign,  getControl_Slow4_prepare       },
    { false, meter_t(2.00f), hasBecomeActive_SingleLine, getControl_Slow4_begin_chicane },
    { false, meter_t(1.55f), hasBecomeActive_distance,   getControl_Slow4_round_begin   },
    { false, meter_t(1.55f), hasBecomeActive_distance,   getControl_Slow4_round_end     },
    { false, meter_t(1.70f), hasBecomeActive_distance,   getControl_Slow4_end_chicane   }
};

#endif // TRACK == TEST_TRACK
