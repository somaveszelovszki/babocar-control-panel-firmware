#include <RaceTrackController.hpp>

#include <limits>
#include <algorithm>

#include <micro/log/log.hpp>
#include <micro/math/numeric.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/port/timer.hpp>

using namespace micro;

bool TrackSection::checkTransition(const CarProps& car, const LinePattern& pattern) {
    transitionPatternDetected |= (transitionCriteria.patternType && transitionCriteria.patternType == pattern.type);
    return (transitionPatternDetected
            || car.distance - startCarProps.distance >= length + transitionCriteria.maxDistanceOvershoot)
            && car.orientedDistance >= transitionCriteria.minOrientedDistance;
}

ControlData TrackSection::getControl(const CarProps& car, const MainLine& mainLine) {
    ControlData controlData;

    controlData.lineControl.actual = mainLine.centerLine;
    controlData.lineControl.target = getTargetLine(car);
    controlData.rearSteerEnabled   = !isFast;

    controlData.speed = [this, &car, &controlData](){
        const bool isCloseToLine = abs(controlData.lineControl.actual.pos - controlData.lineControl.target.pos) < centimeter_t(10);

        if (fullSpeedEnabled) {
            fullSpeedEnabled = !isFast || isCloseToLine;
        } else if (isCloseToLine && car.orientedDistance > centimeter_t(50)) {
            fullSpeedEnabled = true;
        }

        return fullSpeedEnabled ? control.speed : control.speed / 2;
    }();

    controlData.rampTime = control.rampTime;
    return controlData;
}

micro::OrientedLine TrackSection::getTargetLine(const micro::CarProps& car) const {
    const auto lerpDistance = [&car, this](const auto& from, const auto& to){
        return micro::map(car.distance, startCarProps.distance, startCarProps.distance + length, from, to);
    };

    return {
        lerpDistance(control.lineGradient.first.pos, control.lineGradient.second.pos),
        lerpDistance(control.lineGradient.first.angle, control.lineGradient.second.angle)
    };
}

RaceTrackController::RaceTrackController(RaceTrackSections sections)
    : sections_(std::move(sections)) {}

ControlData RaceTrackController::update(const CarProps& car, const LineInfo& lineInfo, const MainLine& mainLine) {
    if (section().checkTransition(car, lineInfo.front.pattern)) {
        sectionIdx_ = incr_overflow(sectionIdx_, lapSections().size());

        if (sectionIdx_ == 0) {
            ++lap_;
        }

        section().startCarProps = car;
    }

    return section().getControl(car, mainLine);
}

void RaceTrackController::setSection(const CarProps& car, const size_t lap, const size_t sectionIdx) {
    lap_ = lap;
    sectionIdx_ = sectionIdx;
    section().startCarProps = car;
}

size_t RaceTrackController::getFastSectionIndex(size_t n) const {
    size_t i = 0u;

    for (; i < sections_[0].size(); ++i) {
        if (sections_[0][i].isFast && --n == 0) {
            break;
        }
    }

    return i < sections_[0].size() ? i : std::numeric_limits<size_t>::max();
}

LapControlParameters RaceTrackController::getControlParameters() const {
    LapControlParameters lapControl;
    const auto& sections = lapSections();
    std::transform(sections.begin(), sections.end(), std::inserter(lapControl, lapControl.end()),
        [](const auto& s){ return std::pair{s.name, s.control}; });
    return lapControl;
}

void RaceTrackController::overrideControlParameters(const LapControlParameters& lapControl) {
    if (!sectionsOverride_) {
        sectionsOverride_ = lapSections();
    }

    for (const auto& [name, control] : lapControl) {
        if (const auto it = std::find_if(sectionsOverride_->begin(), sectionsOverride_->end(),
            [&name](const auto& s){ return s.name == name; });
            it != sectionsOverride_->end()) {
            it->control = control;
        }
    }
}

const LapTrackSections& RaceTrackController::lapSections() const {
    return sectionsOverride_ ? *sectionsOverride_ : sections_[lap_ - 1];
}

LapTrackSections& RaceTrackController::lapSections() {
    return sectionsOverride_ ? *sectionsOverride_ : sections_[lap_ - 1];
}

const TrackSection& RaceTrackController::section() const {
    return lapSections()[sectionIdx_];
}

TrackSection& RaceTrackController::section() {
    return lapSections()[sectionIdx_];
}
