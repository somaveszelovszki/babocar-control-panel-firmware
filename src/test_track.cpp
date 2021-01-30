#include <RaceTrackInfo.hpp>
#include <track.hpp>

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
    { { 1.70f }, { 1.20f }, { 1.20f }, { 1.70f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.70f }, { 2.00f }, { 2.00f }, { 4.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f } }, // Lap 1
    { { 4.00f }, { 2.00f }, { 2.00f }, { 4.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f }, { 4.00f }, { 2.00f }, { 2.00f }, { 4.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f } }, // Lap 2
    { { 3.00f }, { 1.50f }, { 1.50f }, { 3.00f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.20f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 3
    { { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f }, { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f } }, // Lap 4
    { { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f }, { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f } }, // Lap 5
    { { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f }, { 3.00f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.20f }, { 2.20f }, { 2.50f }, { 2.50f }, { 2.00f } }, // Lap 6
    { { 3.00f }                                                                                                                                                                                            }  // Finish
};

BrakeOffsets brakeOffsets[cfg::NUM_RACE_LAPS] = {
//  ||     slow1       ||     slow2       ||     slow3       ||     slow4       ||
    { centimeter_t(100), centimeter_t(100), centimeter_t(100), centimeter_t(100) }, // Lap 1
    { centimeter_t(100), centimeter_t(100), centimeter_t(100), centimeter_t(100) }, // Lap 2
    { centimeter_t(100), centimeter_t(100), centimeter_t(100), centimeter_t(100) }, // Lap 3
    { centimeter_t(100), centimeter_t(100), centimeter_t(100), centimeter_t(100) }, // Lap 4
    { centimeter_t(100), centimeter_t(100), centimeter_t(100), centimeter_t(100) }, // Lap 5
    { centimeter_t(100), centimeter_t(100), centimeter_t(100), centimeter_t(100) }  // Lap 6
};

const TrackSpeeds& getSpeeds(uint8_t lap) {
    return trackSpeeds[lap - 1];
}

const BrakeOffsets& getBrakeOffsets(uint8_t lap) {
    return brakeOffsets[lap - 1];
}

template <typename T>
T mapByTrackSegDistance(const CarProps& car, const RaceTrackInfo& trackInfo, const T& start, const T& end) {
    return map(car.distance, trackInfo.segStartCarProps.distance, trackInfo.segStartCarProps.distance + trackInfo.seg->length, start, end);
}

bool hasBecomeActive_Fast(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    static bool signDetected = false;
    static meter_t lastSignDist = meter_t(0);

    bool active = false;
    if (LinePattern::ACCELERATE == pattern.type) {
        signDetected = true;
        lastSignDist = car.distance;
    } else if (car.distance - lastSignDist > meter_t(5)) {
        signDetected = false;
    }

    if (signDetected && car.orientedDistance > centimeter_t(25)) {
        signDetected = false;
        active = true;
    }
    return active;
}

bool hasBecomeActive_BrakeSign(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_SingleLine(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::SINGLE_LINE == pattern.type;
}

bool hasBecomeActive_distance(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

ControlData getControl_CommonFast(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine, const m_per_sec_t targetSpeed) {
    static bool fastSpeedEnabled = true;

    if (fastSpeedEnabled && abs(mainLine.centerLine.pos) > centimeter_t(12)) {
        fastSpeedEnabled = false;
    } else if (!fastSpeedEnabled && car.orientedDistance > centimeter_t(50)) {
        fastSpeedEnabled = true;
    }

    ControlData controlData;
    controlData.speed                    = fastSpeedEnabled ? targetSpeed : m_per_sec_t(1.5f);
    controlData.rampTime                 = millisecond_t(500);
    controlData.rearSteerEnabled         = false;
    controlData.lineControl.actual       = mainLine.centerLine;
    controlData.lineControl.target.pos   = centimeter_t(0);
    controlData.lineControl.target.angle = radian_t(0);
    return controlData;
}

ControlData getControl_CommonSlow(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData;
    controlData.rampTime                 = millisecond_t(500);
    controlData.rearSteerEnabled         = true;
    controlData.lineControl.actual       = mainLine.centerLine;
    controlData.lineControl.target.pos   = millimeter_t(0);
    controlData.lineControl.target.angle = radian_t(0);
    return controlData;
}

ControlData getControl_Fast1(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast1);
}

ControlData getControl_Fast2(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast2);
}

ControlData getControl_Fast3(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast3);
}

ControlData getControl_Fast4(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast4);
}

ControlData getControl_Slow1_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow1 ? speeds.slow1_prepare : speeds.fast1;

    return controlData;
}

ControlData getControl_Slow1_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow1_chicane;

    return controlData;
}

ControlData getControl_Slow2_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow2 ? speeds.slow2_prepare : speeds.fast2;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.angle, ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_begin_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_begin_chicane;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.pos, sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR);
        controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.angle, -ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_begin;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.pos, sgn(car.speed) * ROUND_LINE_OFFSET_RACE);
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_end;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, sgn(car.speed) * ROUND_LINE_OFFSET_RACE, centimeter_t(0));
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow2_end_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_end_chicane;

    return controlData;
}

ControlData getControl_Slow3_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow3 ? speeds.slow3_prepare : speeds.fast3;

    return controlData;
}

ControlData getControl_Slow3_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_chicane;

    return controlData;
}

ControlData getControl_Slow4_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow4 ? speeds.slow4_prepare : speeds.fast4;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.angle, ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow4_begin_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_begin_chicane;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.pos, sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR);
        controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.angle, -ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow4_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round_begin;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.pos, sgn(car.speed) * ROUND_LINE_OFFSET_RACE);
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow4_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round_end;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.pos   = sgn(car.speed) * ROUND_LINE_OFFSET_SAFETY_CAR;
        controlData.lineControl.target.angle = -ROUND_LINE_ANGLE_SAFETY_CAR;
    } else {
        controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, sgn(car.speed) * ROUND_LINE_OFFSET_RACE, centimeter_t(0));
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow4_end_chicane(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_end_chicane;

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
