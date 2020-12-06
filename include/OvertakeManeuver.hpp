#pragma once

#include <micro/control/maneuver.hpp>
#include <micro/utils/trajectory.hpp>

class OvertakeManeuver : public micro::Maneuver {
public:
    OvertakeManeuver();

    void initialize(const micro::CarProps& car,
        const micro::m_per_sec_t beginSpeed, const micro::m_per_sec_t straightStartSpeed, const micro::m_per_sec_t straightEndSpeed, const micro::m_per_sec_t endSpeed,
        const micro::meter_t sectionLength, const micro::meter_t prepareDistance, const micro::meter_t beginSineArcLength, const micro::meter_t endSineArcLength,
        const micro::meter_t sideDistance);

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) override;

private:
    enum class state_t : uint8_t {
        Prepare,
        FollowTrajectory
    };

    void buildTrajectory(const micro::CarProps& car);

    micro::CarProps initialCarProps_;

    micro::m_per_sec_t beginSpeed_;
    micro::m_per_sec_t straightStartSpeed_;
    micro::m_per_sec_t straightEndSpeed_;
    micro::m_per_sec_t endSpeed_;

    micro::meter_t sectionLength_;
    micro::meter_t prepareDistance_;
    micro::meter_t beginSineArcLength_;
    micro::meter_t endSineArcLength_;
    micro::meter_t sideDistance_;
    micro::meter_t endSlideDistance_;

    state_t state_;
    micro::Trajectory trajectory_;
};
