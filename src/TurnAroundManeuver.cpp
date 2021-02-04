#include <micro/math/numeric.hpp>
#include <TurnAroundManeuver.hpp>

using namespace micro;

TurnAroundManeuver::TurnAroundManeuver()
    : Maneuver()
    , state_(state_t::Stop) {}

void TurnAroundManeuver::initialize(const CarProps& car, const Sign targetSpeedSign, const m_per_sec_t speed, const meter_t sineArcLength, const meter_t circleRadius) {
    Maneuver::initialize();

    this->speed_         = targetSpeedSign * speed;
    this->sineArcLength_ = sineArcLength;
    this->circleRadius_  = circleRadius;
    this->state_         = state_t::Stop;

    this->trajectory_.clear();
}

void TurnAroundManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {
    switch (this->state_) {

    case state_t::Stop:
        controlData.speed    = m_per_sec_t(0);
        controlData.rampTime = millisecond_t(700);

        controlData.rearSteerEnabled    = false;
        controlData.lineControl.actual  = mainLine.centerLine;
        controlData.lineControl.target  = { millimeter_t(0), radian_t(0) };

        if (abs(car.speed) < cm_per_sec_t(2)) {
            this->buildTrajectory(car);
            this->state_ = state_t::FollowTrajectory;
        }
        break;

    case state_t::FollowTrajectory:
        controlData = this->trajectory_.update(car);

        if (this->trajectory_.finished(car, lineInfo, centimeter_t(20))) {
            this->finish();
        }
        break;
    }
}

void TurnAroundManeuver::buildTrajectory(const micro::CarProps& car) {

    const radian_t forwardAngle = this->speed_ > m_per_sec_t(0) ? car.pose.angle : car.pose.angle + PI;

    this->trajectory_.setStartConfig(Trajectory::config_t{
        car.pose,
        this->speed_
    }, car.distance);

    this->trajectory_.appendSineArc(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ sineArcLength_, -circleRadius_ }.rotate(forwardAngle),
            forwardAngle
        },
        this->speed_
    }, forwardAngle, Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

    this->trajectory_.appendCircle(
        this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(0), circleRadius_ }.rotate(forwardAngle),
        PI,
        this->speed_);

    this->trajectory_.appendSineArc(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ sineArcLength_, circleRadius_ - centimeter_t(5) }.rotate(forwardAngle + PI),
            normalize360(forwardAngle + PI)
        },
        this->speed_
    }, normalize360(forwardAngle + PI), Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

    this->trajectory_.appendLine(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(30), centimeter_t(0) }.rotate(forwardAngle + PI),
            normalize360(forwardAngle + PI)
        },
        this->speed_
    });
}
