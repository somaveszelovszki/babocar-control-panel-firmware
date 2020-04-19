#include <globals.hpp>
#include <track.hpp>
#include <track.hpp>

using namespace micro;

namespace {

const TrackSpeeds& getSpeeds(uint8_t lap) {
    return globals::trackSpeeds[lap - 1];
}

const BrakeOffsets& getBrakeOffsets(uint8_t lap) {
    return globals::brakeOffsets[lap - 1];
}

template <typename T>
T mapByTrackSegDistance(const TrackInfo& trackInfo, const T& start, const T& end) {
    return map(globals::car.distance.get(), trackInfo.segStartCarProps.distance.get(), (trackInfo.segStartCarProps.distance + trackInfo.seg->length).get(), start, end);
}

bool hasBecomeActive_Fast(const TrackInfo& trackInfo, const LinePattern& pattern) {
    static bool signDetected = false;
    static meter_t lastSignDist = meter_t(0);

    bool active = false;
    if (LinePattern::ACCELERATE == pattern.type) {
        signDetected = true;
        lastSignDist = globals::car.distance;
    } else if (globals::car.distance - lastSignDist > meter_t(5)) {
        signDetected = false;
    }

    if (signDetected && globals::car.orientedDistance > centimeter_t(25)) {
        signDetected = false;
        active = true;
    }
    return active;
}

bool hasBecomeActive_Slow1_prepare(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow1_round(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return globals::car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_prepare(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow2_begin(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return globals::car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow2_round_begin(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return globals::car.distance - trackInfo.segStartCarProps.distance > centimeter_t(120);
}

bool hasBecomeActive_Slow2_round_end(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return globals::car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow3_prepare(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_round_begin(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow3_round_end(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return globals::car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow3_end(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return globals::car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

bool hasBecomeActive_Slow4_prepare(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return LinePattern::BRAKE == pattern.type;
}

bool hasBecomeActive_Slow4_round(const TrackInfo& trackInfo, const LinePattern& pattern) {
    return globals::car.distance - trackInfo.segStartCarProps.distance > trackInfo.seg->length;
}

ControlData getControl_CommonFast(const TrackInfo& trackInfo, const MainLine& mainLine) {
    static bool fastSpeedEnabled = true;

    if (fastSpeedEnabled && abs(mainLine.centerLine.pos) > centimeter_t(12)) {
        fastSpeedEnabled = false;
    } else if (!fastSpeedEnabled && globals::car.orientedDistance > centimeter_t(50)) {
        fastSpeedEnabled = true;
    }

    ControlData controlData;
    controlData.speed                = fastSpeedEnabled ? getSpeeds(trackInfo.lap).fast : m_per_sec_t(2.0f);
    controlData.rampTime             = millisecond_t(500);
    controlData.controlType          = ControlData::controlType_t::Line;
    controlData.lineControl.baseline = mainLine.centerLine;
    controlData.lineControl.offset   = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, millimeter_t(0));
    controlData.lineControl.angle    = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, radian_t(0));
    return controlData;
}

ControlData getControl_CommonSlow(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData;
    controlData.rampTime             = millisecond_t(500);
    controlData.controlType          = ControlData::controlType_t::Line;
    controlData.lineControl.baseline = mainLine.centerLine;
    controlData.lineControl.offset   = millimeter_t(0);
    controlData.lineControl.angle    = radian_t(0);
    return controlData;
}

ControlData getControl_Fast1(const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(trackInfo, mainLine);
}

ControlData getControl_Slow1_prepare(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = globals::car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow1 ? speeds.slow1_prepare : speeds.fast;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow1_round(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow1_round;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(-10));

    return controlData;
}

ControlData getControl_Fast2(const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(trackInfo, mainLine);
}

ControlData getControl_Slow2_prepare(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = globals::car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow2 ? speeds.slow2_prepare : speeds.fast;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}


ControlData getControl_Slow2_begin(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_begin;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow2_round_begin(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_begin;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(-12));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow2_round_end(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow2_round_end;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(-10));

    return controlData;
}

ControlData getControl_Fast3(const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(trackInfo, mainLine);
}

ControlData getControl_Slow3_prepare(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = globals::car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow3 ? speeds.slow3_prepare : speeds.fast;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow3_round_begin(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_round_begin;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(-12));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(0));

    return controlData;
}

ControlData getControl_Slow3_round_end(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_round_end;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(-15));

    return controlData;
}

ControlData getControl_Slow3_end(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData   = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow3_end;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(10));

    return controlData;
}

ControlData getControl_Fast4(const TrackInfo& trackInfo, const MainLine& mainLine) {
    return  getControl_CommonFast(trackInfo, mainLine);
}

ControlData getControl_Slow4_prepare(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(trackInfo, mainLine);
    const TrackSpeeds& speeds = getSpeeds(trackInfo.lap);

    controlData.speed = globals::car.distance - trackInfo.segStartCarProps.distance > getBrakeOffsets(trackInfo.lap).slow4 ? speeds.slow4_prepare : speeds.fast;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(15));

    return controlData;
}

ControlData getControl_Slow4_round(const TrackInfo& trackInfo, const MainLine& mainLine) {
    ControlData controlData = getControl_CommonSlow(trackInfo, mainLine);

    controlData.speed = getSpeeds(trackInfo.lap).slow4_round;
    controlData.lineControl.offset = mapByTrackSegDistance<millimeter_t>(trackInfo, trackInfo.segStartLine.pos, centimeter_t(0));
    controlData.lineControl.angle  = mapByTrackSegDistance<radian_t>(trackInfo, trackInfo.segStartLine.angle, degree_t(-10));

    return controlData;
}

} // namespace

const TrackSegments trackSegments = {
    { true,  meter_t(5.6f), hasBecomeActive_Fast,              getControl_Fast1             },
    { false, meter_t(3.3f), hasBecomeActive_Slow1_prepare,     getControl_Slow1_prepare     },
    { false, meter_t(1.7f), hasBecomeActive_Slow1_round,       getControl_Slow1_round       },
    { true,  meter_t(6.6f), hasBecomeActive_Fast,              getControl_Fast2             },
    { false, meter_t(3.1f), hasBecomeActive_Slow2_prepare,     getControl_Slow2_prepare     },
    { false, meter_t(1.6f), hasBecomeActive_Slow2_begin,       getControl_Slow2_begin       },
    { false, meter_t(1.6f), hasBecomeActive_Slow2_round_begin, getControl_Slow2_round_begin },
    { false, meter_t(1.6f), hasBecomeActive_Slow2_round_end,   getControl_Slow2_round_end   },
    { true,  meter_t(6.9f), hasBecomeActive_Fast,              getControl_Fast3             },
    { false, meter_t(3.0f), hasBecomeActive_Slow3_prepare,     getControl_Slow3_prepare     },
    { false, meter_t(1.6f), hasBecomeActive_Slow3_round_begin, getControl_Slow3_round_begin },
    { false, meter_t(1.6f), hasBecomeActive_Slow3_round_end,   getControl_Slow3_round_end   },
    { false, meter_t(1.6f), hasBecomeActive_Slow3_end,         getControl_Slow3_end         },
    { true,  meter_t(6.6f), hasBecomeActive_Fast,              getControl_Fast4             },
    { false, meter_t(3.2f), hasBecomeActive_Slow4_prepare,     getControl_Slow4_prepare     },
    { false, meter_t(1.7f), hasBecomeActive_Slow4_round,       getControl_Slow4_round       }
};
