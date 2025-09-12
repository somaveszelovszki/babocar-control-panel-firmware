#include <TestManeuver.hpp>
#include <micro/math/numeric.hpp>

using namespace micro;

TestManeuver::TestManeuver() : Maneuver() {
}

void TestManeuver::initialize(const CarProps& car) {
    Maneuver::initialize();
    this->trajectory_.clear();
}

void TestManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine,
                          ControlData& controlData) {
    if (this->trajectory_.length() == meter_t(0)) {
        this->buildTrajectory(car);
    }

    controlData = this->trajectory_.update(car);

    if (this->trajectory_.finished(car, lineInfo, centimeter_t(80))) {
        this->finish();
    }
}

void TestManeuver::buildTrajectory(const micro::CarProps& car) {
    const m_per_sec_t speed = m_per_sec_t(-0.8f);

    const radian_t forwardAngle = speed >= m_per_sec_t(0) ? car.pose.angle : car.pose.angle + PI;

    this->trajectory_.setStartConfig(Trajectory::config_t{car.pose, speed}, car.distance);

    this->trajectory_.appendSineArc(
        Trajectory::config_t{
            Pose{this->trajectory_.lastConfig().pose.pos +
                     vec2m{centimeter_t(70), centimeter_t(30)}.rotate(forwardAngle),
                 this->trajectory_.lastConfig().pose.angle},
            speed},
        forwardAngle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI);

    this->trajectory_.appendSineArc(
        Trajectory::config_t{
            Pose{this->trajectory_.lastConfig().pose.pos +
                     vec2m{centimeter_t(60), centimeter_t(-30)}.rotate(forwardAngle),
                 this->trajectory_.lastConfig().pose.angle},
            speed},
        forwardAngle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI_2);

    this->trajectory_.appendLine(Trajectory::config_t{
        Pose{this->trajectory_.lastConfig().pose.pos +
                 vec2m{centimeter_t(100), centimeter_t(0)}.rotate(forwardAngle - degree_t(22)),
             this->trajectory_.lastConfig().pose.angle},
        speed});
}
