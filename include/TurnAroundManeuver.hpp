#pragma once

#include <micro/control/maneuver.hpp>
#include <micro/utils/trajectory.hpp>

class TurnAroundManeuver : public micro::Maneuver {
  public:
    TurnAroundManeuver();

    void initialize(const micro::CarProps& car, const micro::Sign targetSpeedSign,
                    const micro::m_per_sec_t speed, const micro::meter_t sineArcLength,
                    const micro::meter_t circleRadius);

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo,
                micro::MainLine& mainLine, micro::ControlData& controlData) override;

  public:
    enum class state_t : uint8_t { Stop, FollowTrajectory };

    void buildTrajectory(const micro::CarProps& car);

    micro::m_per_sec_t speed_;
    micro::meter_t sineArcLength_;
    micro::meter_t circleRadius_;

    state_t state_;
    micro::Trajectory trajectory_;
};
