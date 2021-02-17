#include <micro/math/numeric.hpp>

#include <cfg_car.hpp>
#include <LaneChangeManeuver.hpp>

using namespace micro;

LaneChangeManeuver::LaneChangeManeuver()
    : Maneuver()
    , patternDir_(Sign::NEUTRAL)
    , patternSide_(Direction::CENTER)
    , initialSpeedSign_(Sign::NEUTRAL)
    , safetyCarFollowSpeedSign_(Sign::NEUTRAL)
    , state_(state_t::CheckOrientation) {}

void LaneChangeManeuver::initialize(const micro::CarProps& car, const micro::Sign initialSpeedSign, const micro::Sign patternDir, const micro::Direction patternSide,
    const micro::Sign safetyCarFollowSpeedSign, const micro::m_per_sec_t speed, const micro::meter_t laneDistance) {
    Maneuver::initialize();

    this->patternDir_               = patternDir;
    this->patternSide_              = patternSide;
    this->initialSpeedSign_         = initialSpeedSign;
    this->safetyCarFollowSpeedSign_ = safetyCarFollowSpeedSign;
    this->speed_                    = this->safetyCarFollowSpeedSign_ * speed;
    this->laneDistance_             = laneDistance;
    this->state_                    = state_t::CheckOrientation;

    this->trajectory_.clear();
}

void LaneChangeManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {
    switch (this->state_) {
    case state_t::CheckOrientation:
        controlData.rearSteerEnabled   = true;
        controlData.lineControl.actual = mainLine.centerLine;
        controlData.lineControl.target = { millimeter_t(0), radian_t(0) };

        if (abs(mainLine.centerLine.pos) < centimeter_t(3) && abs(mainLine.centerLine.angle) < degree_t(4)) {
            this->state_ = state_t::Stop;
        }
        break;

    case state_t::Stop:
        controlData.speed    = m_per_sec_t(0);
        controlData.rampTime = millisecond_t(200);

        controlData.rearSteerEnabled   = false;
        controlData.lineControl.actual = mainLine.centerLine;
        controlData.lineControl.target = { millimeter_t(0), radian_t(0) };

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

void LaneChangeManeuver::buildTrajectory(const micro::CarProps& car) {

    this->trajectory_.setStartConfig(Trajectory::config_t{
        car.pose,
        this->speed_
    }, car.distance);

    const radian_t forwardAngle = Sign::POSITIVE == this->safetyCarFollowSpeedSign_ ? car.pose.angle : normalize360(car.pose.angle + PI);

    if (this->initialSpeedSign_ * this->patternDir_ == this->safetyCarFollowSpeedSign_) {

        this->trajectory_.appendSineArc(Trajectory::config_t{
            Pose{
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(110), -this->laneDistance_ + centimeter_t(8) }.rotate(forwardAngle),
                car.pose.angle
            },
            this->speed_,
        }, forwardAngle, Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

    } else {
        meter_t radius = this->laneDistance_ / 2;

        if (radius < cfg::MIN_TURN_RADIUS) {

            const meter_t sineArcWidth = 2 * (cfg::MIN_TURN_RADIUS - radius) + centimeter_t(5);
            radius = cfg::MIN_TURN_RADIUS;

            this->trajectory_.appendSineArc(Trajectory::config_t{
                Pose{
                    this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(60), -sineArcWidth }.rotate(forwardAngle),
                    car.pose.angle
                },
                this->speed_,
            }, car.pose.angle, Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

            this->trajectory_.appendCircle(
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(0), radius }.rotate(forwardAngle),
                PI,
                this->speed_);

        } else {
            this->trajectory_.appendCircle(
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(0), radius }.rotate(forwardAngle),
                PI,
                this->speed_);
        }
    }
}
