#pragma once

#include <micro/container/vec.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/LinePattern.hpp>

#include <cfg_track.hpp>

#include <array>
#include <optional>
#include <utility>

#include <etl/string.h>
#include <etl/map.h>

struct TrackSection {
    struct TransitionCriteria {
        std::optional<micro::LinePattern::type_t> patternType;
        micro::meter_t distanceTolerance;
        micro::meter_t minOrientedDistance;

        static TransitionCriteria distance() {
            return { std::nullopt, micro::meter_t(0), micro::meter_t(0) };
        }

        static TransitionCriteria pattern(const micro::LinePattern::type_t patternType) {
            return { patternType, micro::centimeter_t(150), micro::meter_t(0) };
        }

        static TransitionCriteria acceleration() {
            return { std::nullopt, micro::centimeter_t(150), micro::centimeter_t(25) };
        }
    };

    struct ControlParameters {
        micro::m_per_sec_t speed;
        micro::millisecond_t rampTime;
        std::pair<micro::OrientedLine, micro::OrientedLine> lineGradient;
    };

    using Name = etl::string<15>;

    Name name;
    bool isFast = false;
    micro::meter_t length;
    TransitionCriteria transitionCriteria;
    ControlParameters control;
    micro::CarProps startCarProps;
    bool transitionPatternDetected = false;
    bool fullSpeedEnabled = true;

    bool checkTransition(const micro::CarProps& car, const micro::LinePattern& pattern);
    micro::ControlData getControl(const micro::CarProps& car, const micro::MainLine& mainLine);

    micro::OrientedLine getTargetLine(const micro::CarProps& car) const;
};

using LapControlParameters = etl::map<TrackSection::Name, TrackSection::ControlParameters, 30>;
using LapTrackSections = micro::vec<TrackSection, 30>;
using RaceTrackSections = std::array<LapTrackSections, cfg::NUM_RACE_LAPS + 1>;

class RaceTrackController {
public:
    explicit RaceTrackController(RaceTrackSections sections);

    micro::ControlData update(const micro::CarProps& car, const micro::LineInfo& lineInfo, const micro::MainLine& mainLine);

    void overrideSections(const LapTrackSections& sections);

    void setSection(const micro::CarProps& car, const uint32_t lap, const uint32_t section);

    uint32_t getFastSectionIndex(uint32_t n) const;

    bool isCurrentSectionFast() const { return this->section().isFast; }
    micro::meter_t getSectionStartDistance() const { return this->section().startCarProps.distance; }

    uint32_t lap() const { return this->lap_; }
    uint32_t sectionNumber() const { return this->section_; }

    LapControlParameters getControlParameters() const;
    void setControlParameters(const LapControlParameters& lapControl);

private:
    const LapTrackSections& lapSections() const;
    LapTrackSections& lapSections();
    const TrackSection& section() const;
    TrackSection& section();

private:
    RaceTrackSections sections_;
    std::optional<LapTrackSections> sectionsOverride_;

    uint32_t lap_ = 0u;
    uint32_t section_ = 0u;
};
