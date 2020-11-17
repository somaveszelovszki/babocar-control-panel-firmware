#include <micro/math/numeric.hpp>
#include <TestManeuver.hpp>

using namespace micro;

TestManeuver::TestManeuver()
    : Maneuver()
    , state_(state_t::Stop) {}

void TestManeuver::initialize(const CarProps& car, const m_per_sec_t speed, const meter_t sineArcLength, const meter_t circleRadius) {
    Maneuver::initialize();

    this->speed_         = speed;
    this->sineArcLength_ = sineArcLength;
    this->circleRadius_  = circleRadius;
    this->state_         = state_t::Stop;

    this->trajectory_.clear();
}

void TestManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {
    switch (this->state_) {

    case state_t::Stop:
        controlData.speed    = m_per_sec_t(0);
        controlData.rampTime = second_t(1);

        controlData.controlType         = ControlData::controlType_t::Line;
        controlData.lineControl.actual  = mainLine.centerLine;
        controlData.lineControl.target  = { millimeter_t(0), radian_t(0) };

        if (abs(car.speed) < cm_per_sec_t(2)) {
            this->buildTrajectory(car);
            this->state_ = state_t::FollowTrajectory;
        }
        break;

    case state_t::FollowTrajectory:
        controlData = this->trajectory_.update(car);


        if (this->trajectory_.finished(car, lineInfo)) {
            this->finish();
        }
        break;
    }
}

void TestManeuver::buildTrajectory(const micro::CarProps& car) {

    const radian_t forwardAngle = this->speed_ >= m_per_sec_t(0) ? car.pose.angle : car.pose.angle + PI;

    this->trajectory_.setStartConfig(Trajectory::config_t{
        car.pose,
        speed_
    }, car.distance);

    this->trajectory_.appendSineArc(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ sineArcLength_, -circleRadius_ }.rotate(forwardAngle),
            car.pose.angle
        },
        speed_
    }, car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI);

    this->trajectory_.appendCircle(
        this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(0), circleRadius_ }.rotate(forwardAngle),
        PI,
        speed_);

    this->trajectory_.appendSineArc(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ sineArcLength_, circleRadius_ }.rotate(forwardAngle + PI),
            car.pose.angle + PI
        },
        speed_
    }, car.pose.angle + PI, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI);
}
