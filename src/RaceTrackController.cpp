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
        return micro::lerp(car.distance, startCarProps.distance, startCarProps.distance + length, from, to);
    };

    return {
        lerpDistance(control.lineGradient.first.pos, control.lineGradient.second.pos),
        lerpDistance(control.lineGradient.first.angle, control.lineGradient.second.angle)
    };
}

LapTrackSections OverridableLapTrackSectionProvider::operator()(const size_t lap) {
	if (overridenSections_.empty()) {
		lastUntouchedLap_ = lap;
	}

	auto sections = underlyingProvider_(lastUntouchedLap_);

	for (const auto& [index, control] : overridenSections_) {
		sections[index].control = control;
	}

	return sections;
}
void OverridableLapTrackSectionProvider::overrideControlParameters(const size_t index, const TrackSection::ControlParameters& control) {
	overridenSections_[index] = control;
}

RaceTrackController::RaceTrackController(ILapTrackSectionProvider& sectionProvider)
    : sectionProvider_{sectionProvider}
	, sections_{sectionProvider_(1)}
{}

ControlData RaceTrackController::update(const CarProps& car, const LineInfo& lineInfo, const MainLine& mainLine) {
    if (sections_[sectionIdx_].checkTransition(car, lineInfo.front.pattern)) {
        const auto newSectionIdx = incr_overflow(sectionIdx_, sections_.size());
        const auto newLap = newSectionIdx == 0 ? lap_ + 1 : lap_;
        setSection(car, newLap, newSectionIdx);
    }

    return sections_[sectionIdx_].getControl(car, mainLine);
}

void RaceTrackController::setSection(const CarProps& car, const size_t lap, const size_t sectionIdx) {
    if (lap_ != lap) {
        lap_ = lap;
        sections_ = sectionProvider_(lap_);
    }

    sectionIdx_ = sectionIdx;
    sections_[sectionIdx_].startCarProps = car;
    LOG_DEBUG("Track section changed. Lap: {}, section: {}", lap_, sectionIdx_);
}

size_t RaceTrackController::getFastSectionIndex(size_t n) const {
    for (size_t i = 0; i < sections_.size(); ++i) {
        if (sections_[i].isFast && --n == 0) {
            return i;
        }
    }

    return std::numeric_limits<size_t>::max();
}

LapControlParameters RaceTrackController::getControlParameters() const {
    LapControlParameters lapControl;
    std::transform(sections_.begin(), sections_.end(), std::inserter(lapControl, lapControl.end()),
        [](const auto& s){ return s.control; });
    return lapControl;
}

void RaceTrackController::overrideControlParameters(const size_t index, const TrackSection::ControlParameters& control) {
    if (index >= sections_.size()) {
        return;
    }

    sections_[index].control = control;
}
