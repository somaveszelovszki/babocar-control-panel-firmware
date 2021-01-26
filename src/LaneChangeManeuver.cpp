#include <micro/math/numeric.hpp>

#include <cfg_car.hpp>
#include <LaneChangeManeuver.hpp>

using namespace micro;

LaneChangeManeuver::LaneChangeManeuver()
    : Maneuver()
    , patternDir_(Sign::NEUTRAL)
    , patternSide_(Direction::CENTER)
    , safetyCarFollowSpeedSign_(Sign::NEUTRAL)
    , state_(state_t::FollowTrajectory) {}

void LaneChangeManeuver::initialize(const micro::CarProps& car, const micro::Sign patternDir, const micro::Direction patternSide, const micro::Sign safetyCarFollowSpeedSign,
        const micro::m_per_sec_t speed, const micro::meter_t laneDistance) {
    Maneuver::initialize();

    this->patternDir_               = patternDir;
    this->patternSide_              = patternSide;
    this->safetyCarFollowSpeedSign_ = safetyCarFollowSpeedSign;
    this->speed_                    = speed;
    this->laneDistance_             = laneDistance;
    this->state_                    = state_t::FollowTrajectory;

    this->trajectory_.clear();
    this->buildTrajectory(car);
}

void LaneChangeManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {
    switch (this->state_) {

    case state_t::FollowTrajectory:
        controlData = this->trajectory_.update(car);

        if (this->trajectory_.finished(car, lineInfo, centimeter_t(20))) {
            if (Sign::POSITIVE == this->safetyCarFollowSpeedSign_) {
                this->finish();
            } else {
                this->reverseStopTimer_.start(second_t(3));
                this->state_ = state_t::ReverseWait;
            }
        }
        break;

    case state_t::ReverseWait:
        controlData.speed    = m_per_sec_t(0);
        controlData.rampTime = millisecond_t(500);

        if (this->reverseStopTimer_.checkTimeout()) {
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

    const Sign laneDistSign = Direction::LEFT == this->patternSide_ ? Sign::POSITIVE : Sign::NEGATIVE;

    if (this->safetyCarFollowSpeedSign_ == this->patternDir_) {

        this->trajectory_.appendSineArc(Trajectory::config_t{
            Pose{
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(100), laneDistSign * this->laneDistance_ }.rotate(car.pose.angle),
                car.pose.angle
            },
            this->speed_,
        }, car.pose.angle, Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

    } else {
        meter_t radius = this->laneDistance_ / 2;

        if (radius < cfg::MIN_TURN_RADIUS) {

            const meter_t sineArcWidth = cfg::MIN_TURN_RADIUS - radius;
            radius = cfg::MIN_TURN_RADIUS;

            this->trajectory_.appendSineArc(Trajectory::config_t{
                Pose{
                    this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(30), -laneDistSign * sineArcWidth }.rotate(car.pose.angle),
                    car.pose.angle
                },
                this->speed_,
            }, car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI);

            this->trajectory_.appendCircle(
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(0), laneDistSign * radius }.rotate(car.pose.angle),
                laneDistSign * PI,
                this->speed_);

            this->trajectory_.appendSineArc(Trajectory::config_t{
                Pose{
                    this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(30), laneDistSign * sineArcWidth }.rotate(car.pose.angle + PI),
                    car.pose.angle + PI
                },
                this->speed_,
            }, car.pose.angle + PI, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI);

        } else {
            this->trajectory_.appendCircle(
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(0), laneDistSign * radius }.rotate(car.pose.angle),
                laneDistSign * PI,
                this->speed_);
        }
    }

    this->trajectory_.appendLine(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(40), centimeter_t(0) }.rotate(car.pose.angle + PI),
            car.pose.angle + PI
        },
        this->speed_
    });
}
