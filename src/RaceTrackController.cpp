#include <micro/math/numeric.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/port/timer.hpp>
#include <micro/utils/log.hpp>

#include <RaceTrackController.hpp>

#include <limits>
#include <algorithm>

using namespace micro;

bool TrackSection::checkTransition(const CarProps& car, const LinePattern& pattern) {
    this->transitionPatternDetected |= (!this->transitionCriteria.patternType || this->transitionCriteria.patternType == pattern.type);
    return this->transitionPatternDetected
            && car.distance - this->startCarProps.distance > this->length + this->transitionCriteria.distanceTolerance
            && car.orientedDistance > this->transitionCriteria.minOrientedDistance;
}

ControlData TrackSection::getControl(const CarProps& car, const MainLine& mainLine) {
    ControlData controlData;

    controlData.lineControl.actual = mainLine.centerLine;
    controlData.lineControl.target = getTargetLine(car);
    controlData.rearSteerEnabled   = !this->isFast;

    const bool isCloseToLine = abs(controlData.lineControl.actual.pos - controlData.lineControl.target.pos) < centimeter_t(12);

    if (this->fullSpeedEnabled) {
        this->fullSpeedEnabled = !this->isFast || isCloseToLine;
    } else if (isCloseToLine && car.orientedDistance > centimeter_t(50)) {
        this->fullSpeedEnabled = true;
    }

    controlData.speed    = this->fullSpeedEnabled ? this->control.speed : this->control.speed / 2;
    controlData.rampTime = this->control.rampTime;

    return controlData;
}

micro::OrientedLine TrackSection::getTargetLine(const micro::CarProps& car) const {
    const auto map = [&car, this](const auto& from, const auto& to){
        return micro::map(car.distance, this->startCarProps.distance, this->startCarProps.distance + this->length, from, to);
    };

    return {
        map(this->control.lineGradient.first.pos, this->control.lineGradient.second.pos),
        map(this->control.lineGradient.first.angle, this->control.lineGradient.second.angle)
    };
}

RaceTrackController::RaceTrackController(RaceTrackSections sections)
    : sections_(std::move(sections)) {}

ControlData RaceTrackController::update(const CarProps& car, const LineInfo& lineInfo, const MainLine& mainLine) {
    if (this->section().checkTransition(car, lineInfo.front.pattern)) {
        this->section_ = incr_overflow(this->section_, this->lapSections().size());

        if (this->section_ == 0) {
            ++this->lap_;
        }

        this->section().startCarProps = car;
    }

    return this->section().getControl(car, mainLine);
}

void RaceTrackController::overrideSections(const LapTrackSections& sections) {
    this->sectionsOverride_ = sections;
}

void RaceTrackController::setSection(const CarProps& car, const uint32_t lap, const uint32_t section) {
    this->lap_ = lap;
    this->section_ = section;
    this->section().startCarProps = car;
}

uint32_t RaceTrackController::getFastSectionIndex(uint32_t n) const {
    uint32_t i = 0u;

    for (; i < sections_[0].size(); ++i) {
        if (sections_[0][i].isFast && --n == 0) {
            break;
        }
    }

    return i < sections_[0].size() ? i : std::numeric_limits<uint32_t>::max();
}

LapControlParameters RaceTrackController::getControlParameters() const {
    LapControlParameters lapControl;
    const auto& sections = this->lapSections();
    std::transform(sections.begin(), sections.end(), std::inserter(lapControl, lapControl.end()),
        [](const auto& s){ return std::pair{s.name, s.control}; });
    return lapControl;
}

void RaceTrackController::setControlParameters(const LapControlParameters& lapControl) {
    auto& sections = this->lapSections();
    for (const auto& [name, control] : lapControl) {
        if (const auto it = std::find_if(sections.begin(), sections.end(),
            [&name](const auto& s){ return s.name == name; });
            it != sections.end()) {
            it->control = control;
        }
    }
}

const LapTrackSections& RaceTrackController::lapSections() const {
    return this->sectionsOverride_ ? *this->sectionsOverride_ : this->sections_[this->lap_ - 1];
}

LapTrackSections& RaceTrackController::lapSections() {
    return this->sectionsOverride_ ? *this->sectionsOverride_ : this->sections_[this->lap_ - 1];
}

const TrackSection& RaceTrackController::section() const {
    return this->lapSections()[this->section_];
}

TrackSection& RaceTrackController::section() {
    return this->lapSections()[this->section_];
}
