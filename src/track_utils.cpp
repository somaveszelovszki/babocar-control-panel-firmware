#include <track_utils.hpp>

using namespace micro;

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
    controlData.speed                    = fastSpeedEnabled ? targetSpeed : m_per_sec_t(2.0f);
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
