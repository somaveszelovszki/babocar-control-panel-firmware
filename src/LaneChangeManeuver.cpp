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
    , state_(state_t::Stop) {}

void LaneChangeManeuver::initialize(const micro::CarProps& car, const micro::Sign patternDir, const micro::Direction patternSide, const micro::Sign safetyCarFollowSpeedSign,
        const micro::m_per_sec_t speed, const micro::meter_t laneDistance) {
    Maneuver::initialize();

    this->patternDir_               = patternDir;
    this->patternSide_              = patternSide;
    this->initialSpeedSign_         = sgn(car.speed);
    this->safetyCarFollowSpeedSign_ = safetyCarFollowSpeedSign;
    this->speed_                    = this->safetyCarFollowSpeedSign_ * speed;
    this->laneDistance_             = laneDistance;
    this->state_                    = state_t::Stop;

    this->trajectory_.clear();
}

void LaneChangeManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {
    switch (this->state_) {
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

    const Sign laneDistSign = Direction::LEFT == this->patternSide_ ? Sign::POSITIVE : Sign::NEGATIVE;

    if (this->safetyCarFollowSpeedSign_ == this->initialSpeedSign_ * this->patternDir_) {

        this->trajectory_.appendSineArc(Trajectory::config_t{
            Pose{
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(110), laneDistSign * this->laneDistance_ }.rotate(car.pose.angle),
                car.pose.angle
            },
            this->speed_,
        }, car.pose.angle, Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

        this->trajectory_.appendLine(Trajectory::config_t{
            Pose{
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(30), centimeter_t(0) }.rotate(car.pose.angle),
                car.pose.angle
            },
            this->speed_
        });

    } else {
        meter_t radius = this->laneDistance_ / 2;

        if (radius < cfg::MIN_TURN_RADIUS) {

            const meter_t sineArcWidth = centimeter_t(5) + cfg::MIN_TURN_RADIUS - radius;
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

        this->trajectory_.appendLine(Trajectory::config_t{
            Pose{
                this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(30), centimeter_t(0) }.rotate(car.pose.angle + PI),
                car.pose.angle + PI
            },
            this->speed_
        });
    }
}
