#pragma once

#include <array>
#include <cfg_track.hpp>
#include <etl/string.h>
#include <limits>
#include <micro/container/map.hpp>
#include <micro/container/vector.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/LinePattern.hpp>
#include <optional>
#include <utility>

struct TrackSection {
    struct TransitionCriteria {
        std::optional<micro::LinePattern::type_t> patternType;
        micro::meter_t maxDistanceOvershoot;
        micro::meter_t minOrientedDistance;

        static TransitionCriteria distance() {
            return {std::nullopt, micro::meter_t(0), micro::meter_t(0)};
        }

        static TransitionCriteria pattern(const micro::LinePattern::type_t patternType) {
            return {patternType, micro::centimeter_t(150), micro::meter_t(0)};
        }

        static TransitionCriteria acceleration() {
            return {micro::LinePattern::ACCELERATE, micro::centimeter_t(150),
                    micro::centimeter_t(25)};
        }
    };

    struct ControlParameters {
        micro::m_per_sec_t speed;
        micro::millisecond_t rampTime;
        std::pair<micro::OrientedLine, micro::OrientedLine> lineGradient;

        bool operator==(const ControlParameters& other) const {
            return speed == other.speed && rampTime == other.rampTime &&
                   lineGradient == other.lineGradient;
        }

        bool operator!=(const ControlParameters& other) const { return !(*this == other); }
    };

    bool isFast = false;
    micro::meter_t length;
    TransitionCriteria transitionCriteria;
    ControlParameters control;
    micro::CarProps startCarProps;
    bool transitionPatternDetected = false;
    bool fullSpeedEnabled          = true;

    bool checkTransition(const micro::CarProps& car, const micro::LinePattern& pattern);
    micro::ControlData getControl(const micro::CarProps& car, const micro::MainLine& mainLine);

    micro::OrientedLine getTargetLine(const micro::CarProps& car) const;
};

using IndexedSectionControlParameters = std::pair<size_t, TrackSection::ControlParameters>;
using LapControlParameters =
    micro::vector<TrackSection::ControlParameters, cfg::MAX_NUM_RACE_SEGMENTS>;
using LapTrackSections  = micro::vector<TrackSection, cfg::MAX_NUM_RACE_SEGMENTS>;
using RaceTrackSections = std::array<LapTrackSections, cfg::NUM_RACE_LAPS + 1>;

class ILapTrackSectionProvider {
  public:
    virtual LapTrackSections operator()(const size_t lap) = 0;
    ~ILapTrackSectionProvider()                           = default;
};

class OverridableLapTrackSectionProvider : public ILapTrackSectionProvider {
  public:
    explicit OverridableLapTrackSectionProvider(ILapTrackSectionProvider& underlyingProvider)
        : underlyingProvider_{underlyingProvider} {}

    ~OverridableLapTrackSectionProvider() = default;

    LapTrackSections operator()(const size_t lap);
    void overrideControlParameters(const size_t index,
                                   const TrackSection::ControlParameters& control);

  private:
    ILapTrackSectionProvider& underlyingProvider_;
    micro::map<size_t, TrackSection::ControlParameters, cfg::MAX_NUM_RACE_SEGMENTS>
        overridenSections_;
    size_t lastUntouchedLap_ = 0;
};

class RaceTrackController {
  public:
    explicit RaceTrackController(ILapTrackSectionProvider& sectionProvider);

    void initialize(const micro::CarProps& car, const size_t lap, const size_t sectionIdx);

    micro::ControlData update(const micro::CarProps& car, const micro::LineInfo& lineInfo,
                              const micro::MainLine& mainLine);

    size_t getFastSectionIndex(const size_t n) const;

    bool isCurrentSectionFast() const { return sections_[sectionIdx_].isFast; }
    micro::meter_t getSectionStartDistance() const {
        return sections_[sectionIdx_].startCarProps.distance;
    }

    size_t lap() const { return lap_; }
    size_t sectionIndex() const { return sectionIdx_; }

    LapControlParameters getControlParameters() const;
    void overrideControlParameters(const size_t index,
                                   const TrackSection::ControlParameters& control);

  private:
    void setSection(const micro::CarProps& car, const size_t lap, const size_t sectionIdx);

  private:
    ILapTrackSectionProvider& sectionProvider_;
    LapTrackSections sections_;

    size_t lap_        = 0;
    size_t sectionIdx_ = 0;
};

#define EXPECT_EQ_TRACK_CONTROL_PARAMETERS(expected, result)                                       \
    EXPECT_NEAR_UNIT_DEFAULT(expected.speed, result.speed);                                        \
    EXPECT_NEAR_UNIT_DEFAULT(expected.rampTime, result.rampTime);                                  \
    EXPECT_EQ_MICRO_ORIENTED_LINE(expected.lineGradient.first, result.lineGradient.first);         \
    EXPECT_EQ_MICRO_ORIENTED_LINE(expected.lineGradient.second, result.lineGradient.second)
