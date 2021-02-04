#include <RaceTrackInfo.hpp>
#include <track_utils.hpp>

using namespace micro;

namespace {

constexpr millimeter_t ROUND_LINE_OFFSET_SAFETY_CAR = centimeter_t(5);
constexpr millimeter_t ROUND_LINE_OFFSET_RACE       = centimeter_t(12);
constexpr radian_t ROUND_LINE_ANGLE_SAFETY_CAR      = degree_t(15);

struct TrackSpeeds {
    micro::m_per_sec_t fast1;
    micro::m_per_sec_t slow1_prepare;
    micro::m_per_sec_t slow1_chicane;
    micro::m_per_sec_t fast2;
    micro::m_per_sec_t slow2_prepare;
    micro::m_per_sec_t slow2_begin_chicane;
    micro::m_per_sec_t slow2_round_begin;
    micro::m_per_sec_t slow2_round_end;
    micro::m_per_sec_t slow2_end_chicane;
    micro::m_per_sec_t fast3;
    micro::m_per_sec_t slow3_prepare;
    micro::m_per_sec_t slow3_chicane;
    micro::m_per_sec_t fast4;
    micro::m_per_sec_t slow4_prepare;
    micro::m_per_sec_t slow4_begin_chicane;
    micro::m_per_sec_t slow4_round_begin;
    micro::m_per_sec_t slow4_round_end;
    micro::m_per_sec_t slow4_end_chicane;
};

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

BrakeOffsets brakeOffsets[cfg::NUM_RACE_LAPS] = {
//  ||     slow1       ||     slow2       ||     slow3       ||     slow4       ||
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 1
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 2
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 3
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 4
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 5
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }  // Lap 6
};

ControlData getControl_Fast1(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap).fast1);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast1;

    return controlData;
}

ControlData getControl_Fast2(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap).fast2);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast2;

    return controlData;
}

ControlData getControl_Fast3(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap).fast3);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast3;

    return controlData;
}

ControlData getControl_Fast4(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, track_get(trackSpeeds, trackInfo.lap).fast4);

    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast4;

    return controlData;
}

ControlData getControl_Slow1_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow1 ? speeds.slow1_prepare : speeds.fast1;
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast1;

    return controlData;
}

ControlData getControl_Slow1_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow1_chicane;

    return controlData;
}

ControlData getControl_Slow2_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow2 ? speeds.slow2_prepare : speeds.fast2;
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast2;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.angle = track_map_linear<radian_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.angle, ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_begin_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow2_begin_chicane;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = track_map_linear<millimeter_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.pos, sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR);
        controlData.lineControl.target.angle = track_map_linear<radian_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.angle, -ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow2_round_begin;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = track_map_linear<millimeter_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.pos, sgn(car.speed) * ROUND_LINE_OFFSET_RACE);
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow2_round_end;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = track_map_linear<millimeter_t>(car, trackInfo, sgn(car.speed) * ROUND_LINE_OFFSET_RACE, centimeter_t(0));
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_end_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow2_end_chicane;

    return controlData;
}

ControlData getControl_Slow3_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow3 ? speeds.slow3_prepare : speeds.fast3;
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast3;

    return controlData;
}

ControlData getControl_Slow3_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow3_chicane;

    return controlData;
}

ControlData getControl_Slow4_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = track_get(trackSpeeds, trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > track_get(brakeOffsets, trackInfo.lap).slow4 ? speeds.slow4_prepare : speeds.fast4;
    controlData.rampTime = track_get(accelerationRamps, trackInfo.lap).fast4;

    return controlData;
}

ControlData getControl_Slow4_begin_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow4_begin_chicane;

    return controlData;
}

ControlData getControl_Slow4_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow4_round_begin;

    controlData.lineControl.target.pos   = track_map_linear<millimeter_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.pos, sgn(car.speed) * ROUND_LINE_OFFSET_RACE);
    controlData.lineControl.target.angle = radian_t(0);

    return controlData;
}

ControlData getControl_Slow4_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow4_round_end;

    controlData.lineControl.target.pos   = track_map_linear<millimeter_t>(car, trackInfo, sgn(car.speed) * ROUND_LINE_OFFSET_RACE, centimeter_t(0));
    controlData.lineControl.target.angle = radian_t(0);

    return controlData;
}

ControlData getControl_Slow4_end_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = track_get(trackSpeeds, trackInfo.lap).slow4_end_chicane;

    return controlData;
}

} // namespace

const TrackSegments testTrackSegments = {
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
