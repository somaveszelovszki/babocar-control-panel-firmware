#include <cfg_track.hpp>

#if TRACK == RACE_TRACK

#include <RaceTrackInfo.hpp>
#include <track_utils.hpp>

using namespace micro;

namespace {

constexpr millimeter_t ROUND_LINE_OFFSET_SAFETY_CAR = centimeter_t(5);
constexpr millimeter_t ROUND_LINE_OFFSET_RACE       = centimeter_t(12);
constexpr radian_t ROUND_LINE_ANGLE_SAFETY_CAR      = degree_t(15);

constexpr uint32_t FAST1             = 0u;
constexpr uint32_t SLOW1_PREPARE     = 1u;
constexpr uint32_t SLOW1_ROUND       = 2u;
constexpr uint32_t FAST2             = 3u;
constexpr uint32_t SLOW2_PREPARE     = 4u;
constexpr uint32_t SLOW2_BEGIN       = 5u;
constexpr uint32_t SLOW2_ROUND_BEGIN = 6u;
constexpr uint32_t SLOW2_ROUND_END   = 7u;
constexpr uint32_t FAST3             = 8u;
constexpr uint32_t SLOW3_PREPARE     = 9u;
constexpr uint32_t SLOW3_ROUND_BEGIN = 10u;
constexpr uint32_t SLOW3_ROUND_END   = 11u;
constexpr uint32_t SLOW3_END         = 12u;
constexpr uint32_t FAST4             = 13u;
constexpr uint32_t SLOW4_PREPARE     = 14u;
constexpr uint32_t SLOW4_ROUND       = 15u;

const TrackSpeeds trackSpeeds[cfg::NUM_RACE_LAPS + 1] = {
//  ||  fast1  ||        slow1       ||  fast2  ||                   slow2                  ||  fast3  ||                   slow3                  ||  fast4  ||        slow4       ||
//  ||         || prepare     round  ||         || prepare     begin   round_begin round_end||         || prepare  round_begin round_end     end   ||         || prepare     round  ||
    { { 1.00f }, { 1.20f }, { 1.20f }, { 1.70f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.70f }, { 1.60f }, { 1.60f }, { 1.60f }, { 1.60f }, { 3.50f }, { 2.00f }, { 2.00f } }, // Lap 1
    { { 3.00f }, { 2.30f }, { 2.30f }, { 4.50f }, { 2.00f }, { 2.00f }, { 2.40f }, { 2.40f }, { 4.50f }, { 2.40f }, { 2.40f }, { 2.40f }, { 2.00f }, { 4.50f }, { 2.30f }, { 2.30f } }, // Lap 2
    { { 3.60f }, { 1.50f }, { 1.50f }, { 3.60f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.60f }, { 2.20f }, { 2.00f }, { 2.00f } }, // Lap 3
    { { 5.50f }, { 2.60f }, { 2.60f }, { 5.50f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 5.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.20f }, { 5.50f }, { 2.40f }, { 2.60f } }, // Lap 4
    { { 6.00f }, { 2.80f }, { 3.00f }, { 6.00f }, { 2.50f }, { 2.50f }, { 2.70f }, { 2.70f }, { 6.00f }, { 2.80f }, { 3.00f }, { 2.70f }, { 2.20f }, { 6.00f }, { 2.60f }, { 2.80f } }, // Lap 5
    { { 7.30f }, { 2.80f }, { 3.00f }, { 7.30f }, { 2.50f }, { 2.50f }, { 2.70f }, { 2.70f }, { 7.30f }, { 2.80f }, { 3.00f }, { 2.70f }, { 2.20f }, { 7.30f }, { 2.40f }, { 2.70f } }, // Lap 6
    { { 7.30f }                                                                                                                                                                      }  // Finish
};

const AccelerationRamps accelerationRamps[cfg::NUM_RACE_LAPS + 1] = {
//  ||      fast1       ||      fast2       ||      fast3       ||      fast4       ||
    { millisecond_t(800), millisecond_t(800), millisecond_t(800), millisecond_t(800) }, // Lap 1
    { millisecond_t(800), millisecond_t(800), millisecond_t(800), millisecond_t(800) }, // Lap 2
    { millisecond_t(800), millisecond_t(800), millisecond_t(800), millisecond_t(800) }, // Lap 3
    { millisecond_t(600), millisecond_t(600), millisecond_t(600), millisecond_t(600) }, // Lap 4
    { millisecond_t(600), millisecond_t(600), millisecond_t(600), millisecond_t(600) }, // Lap 5
    { millisecond_t(300), millisecond_t(300), millisecond_t(300), millisecond_t(300) }, // Lap 6
    { millisecond_t(300)                                                             }  // Finish
};

const BrakeOffsets brakeOffsets[cfg::NUM_RACE_LAPS + 1] = {
//  ||     slow1     ||     slow2     ||     slow3     ||     slow4     ||
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 1
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 2
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 3
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 4
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 5
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 6
    { centimeter_t(0)                                                    }  // Finish
};

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

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.angle = track_map_angle_linear(car, trackInfo, -ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow1_round(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW1_ROUND];

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = track_map_pos_pyramid(car, trackInfo, ROUND_LINE_OFFSET_SAFETY_CAR, centimeter_t(0));
        controlData.lineControl.target.angle = track_map_angle_linear(car, trackInfo, radian_t(0));
    } else {
        controlData.lineControl.target.pos   = track_map_pos_pyramid(car, trackInfo, ROUND_LINE_OFFSET_RACE, centimeter_t(0));
        controlData.lineControl.target.angle = radian_t(0);
    }

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


ControlData getControl_Slow2_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW2_BEGIN];

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = track_map_pos_pyramid(car, trackInfo, -ROUND_LINE_OFFSET_SAFETY_CAR, centimeter_t(0));
        controlData.lineControl.target.angle = track_map_angle_pyramid(car, trackInfo, ROUND_LINE_ANGLE_SAFETY_CAR, radian_t(0));
    } else {
        controlData.lineControl.target.pos   = track_map_pos_pyramid(car, trackInfo, -ROUND_LINE_OFFSET_RACE, centimeter_t(0));
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW2_ROUND_BEGIN];

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, ROUND_LINE_OFFSET_SAFETY_CAR);
        controlData.lineControl.target.angle = track_map_angle_pyramid(car, trackInfo, -ROUND_LINE_ANGLE_SAFETY_CAR, -ROUND_LINE_ANGLE_SAFETY_CAR);
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
        controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, centimeter_t(0));
        controlData.lineControl.target.angle = track_map_angle_linear(car, trackInfo, radian_t(0));
    } else {
        controlData.lineControl.target.pos   = track_map_pos_linear(car, trackInfo, centimeter_t(0));
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow3_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow3 ? speeds[SLOW3_PREPARE] : speeds[FAST3];
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast3;

    return controlData;
}

ControlData getControl_Slow3_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW3_ROUND_BEGIN];
    controlData.lineControl.target.pos = track_map_pos_linear(car, trackInfo, ROUND_LINE_OFFSET_RACE);

    return controlData;
}

ControlData getControl_Slow3_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW3_ROUND_END];
    controlData.lineControl.target.pos = track_map_pos_linear(car, trackInfo, centimeter_t(0));

    return controlData;
}

ControlData getControl_Slow3_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW3_END];
    controlData.lineControl.target.pos = track_map_pos_pyramid(car, trackInfo, -ROUND_LINE_OFFSET_RACE, centimeter_t(0));

    return controlData;
}

ControlData getControl_Slow4_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow4 ? speeds[SLOW4_PREPARE] : speeds[FAST4];
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast4;

    return controlData;
}

ControlData getControl_Slow4_round(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap)[SLOW4_ROUND];
    controlData.lineControl.target.pos = track_map_pos_pyramid(car, trackInfo, ROUND_LINE_OFFSET_RACE, centimeter_t(0));

    return controlData;
}

const TrackSegments trackSegments = {
    { true,  meter_t(5.6f), hasBecomeActive_Fast,         getControl_Fast1               },
    { false, meter_t(3.0f), hasBecomeActive_BrakeSign,    getControl_Slow1_prepare       },
    { false, meter_t(2.4f), hasBecomeActive_SingleLine,   getControl_Slow1_round         },
    { true,  meter_t(7.3f), hasBecomeActive_Fast,         getControl_Fast2               },
    { false, meter_t(3.0f), hasBecomeActive_BrakeSign,    getControl_Slow2_prepare       },
    { false, meter_t(2.0f), hasBecomeActive_SingleLine,   getControl_Slow2_begin         },
    { false, meter_t(1.9f), hasBecomeActive_distance,     getControl_Slow2_round_begin   },
    { false, meter_t(1.4f), hasBecomeActive_distance,     getControl_Slow2_round_end     },
    { true,  meter_t(7.7f), hasBecomeActive_Fast,         getControl_Fast3               },
    { false, meter_t(3.0f), hasBecomeActive_BrakeSign,    getControl_Slow3_prepare       },
    { false, meter_t(1.9f), hasBecomeActive_SingleLine,   getControl_Slow3_round_begin   },
    { false, meter_t(1.7f), hasBecomeActive_distance,     getControl_Slow3_round_end     },
    { false, meter_t(1.6f), hasBecomeActive_distance,     getControl_Slow3_end           },
    { true,  meter_t(7.3f), hasBecomeActive_Fast,         getControl_Fast4               },
    { false, meter_t(3.0f), hasBecomeActive_BrakeSign,    getControl_Slow4_prepare       },
    { false, meter_t(2.2f), hasBecomeActive_SingleLine,   getControl_Slow4_round         }
};

} // namespace

RaceTrackInfo buildRaceTrackInfo() {
    return RaceTrackInfo(trackSpeeds, accelerationRamps, brakeOffsets, trackSegments);
}

#endif // TRACK == RACE_TRACK
