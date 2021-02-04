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
    micro::m_per_sec_t slow1_round;
    micro::m_per_sec_t fast2;
    micro::m_per_sec_t slow2_prepare;
    micro::m_per_sec_t slow2_begin;
    micro::m_per_sec_t slow2_round_begin;
    micro::m_per_sec_t slow2_round_end;
    micro::m_per_sec_t fast3;
    micro::m_per_sec_t slow3_prepare;
    micro::m_per_sec_t slow3_round_begin;
    micro::m_per_sec_t slow3_round_end;
    micro::m_per_sec_t slow3_end;
    micro::m_per_sec_t fast4;
    micro::m_per_sec_t slow4_prepare;
    micro::m_per_sec_t slow4_round;
};

TrackSpeeds trackSpeeds[cfg::NUM_RACE_LAPS + 1] = {
//  ||  fast1  ||        slow1       ||  fast2  ||                   slow2                  ||  fast3  ||                   slow3                  ||  fast4  ||        slow4       ||
//  ||         || prepare     round  ||         || prepare     begin   round_begin round_end||         || prepare  round_begin round_end     end   ||         || prepare     round  ||
    { { 3.50f }, { 1.80f }, { 1.80f }, { 1.70f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 1.70f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 4.00f }, { 1.80f }, { 1.80f } }, // Lap 1
    { { 4.50f }, { 1.80f }, { 1.80f }, { 4.00f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 4.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 4.00f }, { 1.80f }, { 1.80f } }, // Lap 2
    { { 3.50f }, { 1.80f }, { 1.80f }, { 3.00f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 1.70f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 3
    { { 5.00f }, { 2.05f }, { 2.05f }, { 3.00f }, { 1.80f }, { 1.80f }, { 2.20f }, { 2.20f }, { 3.00f }, { 2.10f }, { 2.10f }, { 2.10f }, { 1.70f }, { 3.00f }, { 1.95f }, { 1.95f } }, // Lap 4
    { { 5.80f }, { 2.05f }, { 2.05f }, { 3.00f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 3.00f }, { 2.25f }, { 2.25f }, { 2.25f }, { 1.80f }, { 3.00f }, { 1.95f }, { 1.95f } }, // Lap 5
    { { 6.50f }, { 2.05f }, { 2.05f }, { 3.00f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 3.00f }, { 2.25f }, { 2.25f }, { 2.25f }, { 1.80f }, { 3.00f }, { 1.95f }, { 1.95f } }, // Lap 6
    { { 7.00f }                                                                                                                                                                                            }  // Finish
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
    //     slow1            slow2            slow3            slow4
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 1
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 2
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 3
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 4
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 5
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }  // Lap 6
};

const TrackSpeeds& getSpeeds(uint8_t lap) {
    return trackSpeeds[lap - 1];
}

const AccelerationRamps& getAccelerationRamps(uint8_t lap) {
    return accelerationRamps[lap - 1];
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

bool hasBecomeActive_Slow1_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow1_round(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::SINGLE_LINE == pattern.type;
}

bool hasBecomeActive_Slow2_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow2_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow3_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow3_end(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow4_round(const CarProps& car, const RaceTrackInfo& trackInfo, const LinePattern& pattern) {
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
    controlData.rearSteerEnabled         = false;
    controlData.lineControl.actual       = mainLine.centerLine;
    controlData.lineControl.target.pos   = centimeter_t(0);
    controlData.lineControl.target.angle = radian_t(0);
    return controlData;
}

ControlData getControl_CommonSlow(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData;
    controlData.rampTime                 = millisecond_t(100);
    controlData.rearSteerEnabled         = true;
    controlData.lineControl.actual       = mainLine.centerLine;
    controlData.lineControl.target.pos   = millimeter_t(0);
    controlData.lineControl.target.angle = radian_t(0);
    return controlData;
}

ControlData getControl_Fast1(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast1);

    controlData.rampTime = getAccelerationRamps(trackInfo.lap).fast1;

    return controlData;
}

ControlData getControl_Fast2(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast2);

    controlData.rampTime = getAccelerationRamps(trackInfo.lap).fast2;

    return controlData;
}

ControlData getControl_Fast3(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast3);

    controlData.rampTime = getAccelerationRamps(trackInfo.lap).fast3;

    return controlData;
}

ControlData getControl_Fast4(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonFast(car, trackInfo, mainLine, getSpeeds(trackInfo.lap).fast4);

    controlData.rampTime = getAccelerationRamps(trackInfo.lap).fast4;

    return controlData;
}

ControlData getControl_Slow1_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow1 ? speeds.slow1_prepare : speeds.fast1;
    controlData.rampTime = getAccelerationRamps(trackInfo.lap).fast1;

    if (1 == trackInfo.lap || 3 == trackInfo.lap) {
        controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartControlData.lineControl.target.angle, ROUND_LINE_ANGLE_SAFETY_CAR);
    } else {
        controlData.lineControl.target.angle = radian_t(0);
    }

    return controlData;
}

ControlData getControl_Slow1_round(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow1_round;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(10));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow2_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow2 ? speeds.slow2_prepare : speeds.fast2;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}


ControlData getControl_Slow2_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow2_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(-12));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow2_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(10));

    return controlData;
}

ControlData getControl_Slow3_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow3 ? speeds.slow3_prepare : speeds.fast3;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow3_round_begin(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(-12));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow3_round_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_round_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow3_end(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-10));

    return controlData;
}

ControlData getControl_Slow4_prepare(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow4 ? speeds.slow4_prepare : speeds.fast4;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow4_round(const CarProps& car, const RaceTrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(10));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

} // namespace

const TrackSegments raceTrackSegments = {
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
