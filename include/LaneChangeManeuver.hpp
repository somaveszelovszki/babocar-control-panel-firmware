#pragma once

#include <micro/control/maneuver.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>

class LaneChangeManeuver : public micro::Maneuver {
public:
    LaneChangeManeuver();

    void initialize(const micro::CarProps& car, const micro::Sign initialSpeedSign, const micro::Sign patternDir, const micro::Direction patternSide,
        const micro::Sign safetyCarFollowSpeedSign, const micro::m_per_sec_t speed, const micro::meter_t laneDistance);

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) override;

private:
    enum class state_t : uint8_t {
        CheckOrientation,
        Stop,
        FollowTrajectory
    };

    void buildTrajectory(const micro::CarProps& car);

    micro::Sign patternDir_;
    micro::Direction patternSide_;
    micro::Sign initialSpeedSign_;
    micro::Sign safetyCarFollowSpeedSign_;
    micro::m_per_sec_t speed_;
    micro::meter_t laneDistance_;

    state_t state_;
    micro::Trajectory trajectory_;
    micro::Timer reverseStopTimer_;
};
