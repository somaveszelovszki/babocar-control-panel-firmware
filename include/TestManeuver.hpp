#pragma once

#include <micro/control/maneuver.hpp>
#include <micro/utils/trajectory.hpp>

class TestManeuver : public micro::Maneuver {
public:
    TestManeuver();

    void initialize(const micro::CarProps& car);

    void update(const micro::CarProps& car, const micro::LineInfo& lineInfo, micro::MainLine& mainLine, micro::ControlData& controlData) override;

public:
    void buildTrajectory(const micro::CarProps& car);

    micro::Trajectory trajectory_;
};
