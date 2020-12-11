#include <track.hpp>

using namespace micro;

namespace {

TrackSpeeds trackSpeeds[cfg::NUM_RACE_LAPS + 1] = {

#if TRACK == RACE_TRACK
//  ||  fast   ||             slow1             ||                   slow2                  ||                   slow3                  ||             slow4             ||
//  ||  (all)  || prepare  round_begin round_end|| prepare     begin   round_begin round_end|| prepare  round_begin round_end     end   || prepare  round_begin round_end||
    { { 3.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 1
    { { 4.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 2
    { { 3.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 3
    { { 5.00f }, { 2.05f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.20f }, { 2.20f }, { 2.10f }, { 2.10f }, { 2.10f }, { 1.70f }, { 1.95f }, { 1.95f }, { 1.95f } }, // Lap 4
    { { 5.80f }, { 2.05f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.25f }, { 2.25f }, { 2.25f }, { 1.80f }, { 1.95f }, { 1.95f }, { 1.95f } }, // Lap 5
    { { 6.50f }, { 2.05f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.25f }, { 2.25f }, { 2.25f }, { 1.80f }, { 1.95f }, { 1.95f }, { 1.95f } }, // Lap 6
    { { 7.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f } }  // Finish
#elif TRACK == TEST_TRACK
//  ||  fast   ||        slow1       ||                        slow2                        ||        slow3       ||                        slow4                        ||
//  ||  (all)  || prepare    chicane || prepare   begin_chi round_begin round_end   end_chi || prepare    chicane || prepare   begin_chi round_begin round_end   end_chi ||
    { { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f } }, // Lap 1
    { { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f } }, // Lap 2
    { { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f } }, // Lap 3
    { { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f } }, // Lap 4
    { { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f } }, // Lap 5
    { { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f } }, // Lap 6
    { { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f }, { 2.50f } }  // Finish
#endif

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

const BrakeOffsets& getBrakeOffsets(uint8_t lap) {
    return brakeOffsets[lap - 1];
}

template <typename T>
T mapByTrackSegDistance(const CarProps& car, const TrackInfo& trackInfo, const T& start, const T& end) {
    return map(car.distance, trackInfo.segStartCarProps.distance, trackInfo.segStartCarProps.distance + trackInfo.seg->length, start, end);
}

radian_t getFixOrientationLineAngle(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return trackInfo.segStartCarProps.pose.angle - car.pose.angle - mainLine.centerLine.angle;
}

bool hasBecomeActive_Fast(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
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

#if TRACK == RACE_TRACK

bool hasBecomeActive_Slow1_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow1_round_begin(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow1_round_end(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow2_begin(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_round_begin(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_round_end(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow3_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_round_begin(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_round_end(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow3_end(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow4_round_begin(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_round_end(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

#elif TRACK == TEST_TRACK

bool hasBecomeActive_Slow1_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow1_chicane(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow2_begin_chicane(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_round_begin(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_round_end(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_end_chicane(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow3_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_chicane(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_prepare(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow4_begin_chicane(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_round_begin(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_round_end(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_end_chicane(const CarProps& car, const TrackInfo& trackInfo, const LinePattern& pattern) {
    return car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

#endif

ControlData getControl_CommonFast(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    static bool fastSpeedEnabled = true;

    if (fastSpeedEnabled && abs(mainLine.centerLine.pos) > centimeter_t(12)) {
        fastSpeedEnabled = false;
    } else if (!fastSpeedEnabled && car.orientedDistance > centimeter_t(50)) {
        fastSpeedEnabled = true;
    }

    ControlData controlData;
    controlData.speed                    = fastSpeedEnabled ? getSpeeds(trackInfo.lap).fast : m_per_sec_t(2.0f);
    controlData.rampTime                 = millisecond_t(500);
    controlData.controlType              = ControlData::controlType_t::Line;
    controlData.lineControl.actual       = mainLine.centerLine;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, millimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, radian_t(0));
    return controlData;
}

ControlData getControl_CommonSlow(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData;
    controlData.rampTime                 = millisecond_t(500);
    controlData.controlType              = ControlData::controlType_t::Line;
    controlData.lineControl.actual       = mainLine.centerLine;
    controlData.lineControl.target.pos   = millimeter_t(0);
    controlData.lineControl.target.angle = radian_t(0);
    return controlData;
}

#if TRACK == RACE_TRACK

ControlData getControl_Fast1(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Slow1_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow1 ? speeds.slow1_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow1_round_begin(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow1_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(10));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow1_round_end(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow1_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(10));

    return controlData;
}

ControlData getControl_Fast2(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Slow2_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow2 ? speeds.slow2_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}


ControlData getControl_Slow2_begin(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow2_round_begin(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(-12));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow2_round_end(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(10));

    return controlData;
}

ControlData getControl_Fast3(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Slow3_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow3 ? speeds.slow3_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow3_round_begin(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(-12));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow3_round_end(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_round_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow3_end(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-10));

    return controlData;
}

ControlData getControl_Fast4(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Slow4_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow4 ? speeds.slow4_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow4_round_begin(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(10));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow4_round_end(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(10));

    return controlData;
}

#elif TRACK == TEST_TRACK

ControlData getControl_Fast1(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Fast2(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Fast3(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Fast4(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(car, trackInfo, mainLine);
}

ControlData getControl_Slow1_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow1 ? speeds.slow1_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow1_chicane(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow1_chicane;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = getFixOrientationLineAngle(car, trackInfo, mainLine);

    return controlData;
}

ControlData getControl_Slow2_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow2 ? speeds.slow2_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow2_begin_chicane(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_begin_chicane;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow2_round_begin(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(-12));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow2_round_end(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow2_end_chicane(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_end_chicane;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = getFixOrientationLineAngle(car, trackInfo, mainLine);

    return controlData;
}

ControlData getControl_Slow3_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow3 ? speeds.slow3_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow3_chicane(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_chicane;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = getFixOrientationLineAngle(car, trackInfo, mainLine);

    return controlData;
}

ControlData getControl_Slow4_prepare(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow4 ? speeds.slow4_prepare : speeds.fast;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow4_begin_chicane(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_begin_chicane;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow4_round_begin(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round_begin;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(-12));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow4_round_end(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round_end;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = mapByTrackSegDistance<radian_t>(car, trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow4_end_chicane(const CarProps& car, const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(car, trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_end_chicane;
    controlData.lineControl.target.pos   = mapByTrackSegDistance<millimeter_t>(car, trackInfo, trackInfo.segStartLine.pos, sgn(car.speed) * centimeter_t(0));
    controlData.lineControl.target.angle = getFixOrientationLineAngle(car, trackInfo, mainLine);

    return controlData;
}

#endif

} // namespace

const TrackSegments trackSegments = {
#if TRACK == RACE_TRACK
    { true,  meter_t(5.6f), hasBecomeActive_Fast,                getControl_Fast1               },
    { false, meter_t(3.3f), hasBecomeActive_Slow1_prepare,       getControl_Slow1_prepare       },
    { false, meter_t(0.9f), hasBecomeActive_Slow1_round_begin,   getControl_Slow1_round_begin   },
    { false, meter_t(0.8f), hasBecomeActive_Slow1_round_end,     getControl_Slow1_round_end     },
    { true,  meter_t(6.6f), hasBecomeActive_Fast,                getControl_Fast2               },
    { false, meter_t(3.1f), hasBecomeActive_Slow2_prepare,       getControl_Slow2_prepare       },
    { false, meter_t(1.6f), hasBecomeActive_Slow2_begin,         getControl_Slow2_begin         },
    { false, meter_t(1.6f), hasBecomeActive_Slow2_round_begin,   getControl_Slow2_round_begin   },
    { false, meter_t(1.6f), hasBecomeActive_Slow2_round_end,     getControl_Slow2_round_end     },
    { true,  meter_t(6.9f), hasBecomeActive_Fast,                getControl_Fast3               },
    { false, meter_t(3.0f), hasBecomeActive_Slow3_prepare,       getControl_Slow3_prepare       },
    { false, meter_t(1.6f), hasBecomeActive_Slow3_round_begin,   getControl_Slow3_round_begin   },
    { false, meter_t(1.6f), hasBecomeActive_Slow3_round_end,     getControl_Slow3_round_end     },
    { false, meter_t(1.6f), hasBecomeActive_Slow3_end,           getControl_Slow3_end           },
    { true,  meter_t(6.6f), hasBecomeActive_Fast,                getControl_Fast4               },
    { false, meter_t(3.2f), hasBecomeActive_Slow4_prepare,       getControl_Slow4_prepare       },
    { false, meter_t(0.9f), hasBecomeActive_Slow4_round_begin,   getControl_Slow4_round_begin   },
    { false, meter_t(0.8f), hasBecomeActive_Slow4_round_end,     getControl_Slow4_round_end     }
#elif TRACK == TEST_TRACK
    { true,  meter_t(0.0f), hasBecomeActive_Fast,                getControl_Fast1               },
    { false, meter_t(0.0f), hasBecomeActive_Slow1_prepare,       getControl_Slow1_prepare       },
    { false, meter_t(0.0f), hasBecomeActive_Slow1_chicane,       getControl_Slow1_chicane       },
    { true,  meter_t(0.0f), hasBecomeActive_Fast,                getControl_Fast2               },
    { false, meter_t(0.0f), hasBecomeActive_Slow2_prepare,       getControl_Slow2_prepare       },
    { false, meter_t(0.0f), hasBecomeActive_Slow2_begin_chicane, getControl_Slow2_begin_chicane },
    { false, meter_t(0.0f), hasBecomeActive_Slow2_round_begin,   getControl_Slow2_round_begin   },
    { false, meter_t(0.0f), hasBecomeActive_Slow2_round_end,     getControl_Slow2_round_end     },
    { false, meter_t(0.0f), hasBecomeActive_Slow2_end_chicane,   getControl_Slow2_end_chicane   },
    { true,  meter_t(0.0f), hasBecomeActive_Fast,                getControl_Fast3               },
    { false, meter_t(0.0f), hasBecomeActive_Slow3_prepare,       getControl_Slow3_prepare       },
    { false, meter_t(0.0f), hasBecomeActive_Slow3_chicane,       getControl_Slow3_chicane       },
    { true,  meter_t(0.0f), hasBecomeActive_Fast,                getControl_Fast4               },
    { false, meter_t(0.0f), hasBecomeActive_Slow4_prepare,       getControl_Slow4_prepare       },
    { false, meter_t(0.0f), hasBecomeActive_Slow4_begin_chicane, getControl_Slow4_begin_chicane },
    { false, meter_t(0.0f), hasBecomeActive_Slow4_round_begin,   getControl_Slow4_round_begin   },
    { false, meter_t(0.0f), hasBecomeActive_Slow4_round_end,     getControl_Slow4_round_end     },
    { false, meter_t(0.0f), hasBecomeActive_Slow4_end_chicane,   getControl_Slow4_end_chicane   }
#endif

};
