#include <micro/math/numeric.hpp>
#include <micro/utils/log.hpp>
#include <OvertakeManeuver.hpp>

using namespace micro;

OvertakeManeuver::OvertakeManeuver()
    : Maneuver()
    , targetSpeedSign_(Sign::POSITIVE)
    , state_(state_t::Prepare) {}

void OvertakeManeuver::initialize(const micro::CarProps& car, const micro::Sign targetSpeedSign,
    const micro::m_per_sec_t beginSpeed, const micro::m_per_sec_t straightStartSpeed, const micro::m_per_sec_t straightSpeed, const micro::m_per_sec_t endSpeed,
    const micro::meter_t sectionLength, const micro::meter_t prepareDistance, const micro::meter_t beginSineArcLength, const micro::meter_t endSineArcLength,
    const micro::meter_t sideDistance) {
    Maneuver::initialize();

    this->previousCarPositions_.emplace(car.distance.get(), car.pose.pos);

    this->initialDistance_    = car.distance;
    this->targetSpeedSign_    = targetSpeedSign;
    this->beginSpeed_         = this->targetSpeedSign_ * beginSpeed;
    this->straightStartSpeed_ = this->targetSpeedSign_ * straightStartSpeed;
    this->straightSpeed_      = this->targetSpeedSign_ * straightSpeed;
    this->endSpeed_           = this->targetSpeedSign_ * endSpeed;
    this->sectionLength_      = sectionLength;
    this->prepareDistance_    = prepareDistance;
    this->beginSineArcLength_ = beginSineArcLength;
    this->endSineArcLength_   = endSineArcLength;
    this->sideDistance_       = sideDistance;
    this->endSlideDistance_   = sideDistance * 3;
    this->state_              = state_t::Prepare;

    this->trajectory_.clear();
}

void OvertakeManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {

    if (car.distance - meter_t(this->previousCarPositions_.back()->first) >= centimeter_t(10)) {
        this->previousCarPositions_.emplace(car.distance.get(), car.pose.pos);
    }

    switch (this->state_) {

    case state_t::Prepare:
        // does not change longitudinal control in order to continue following safety car

        controlData.rearSteerEnabled    = true;
        controlData.lineControl.actual  = mainLine.centerLine;
        controlData.lineControl.target  = { millimeter_t(0), radian_t(0) };

        if (car.orientedDistance >= this->prepareDistance_ && car.distance - this->initialDistance_ >= this->prepareDistance_) {
            this->buildTrajectory(car);
            this->state_ = state_t::FollowTrajectory;
        }
        break;

    case state_t::FollowTrajectory:
        controlData = this->trajectory_.update(car);

        if (this->trajectory_.finished(car, lineInfo, this->endSlideDistance_)) {
            this->finish();
        }
        break;
    }
}

void OvertakeManeuver::buildTrajectory(const micro::CarProps& car) {

    static constexpr meter_t STRAIGHT_SPEED_RAMP_DIST = meter_t(2);

    const meter_t distDiff          = car.distance - this->initialDistance_;
    const meter_t fastSectionLength = this->sectionLength_ - distDiff - this->beginSineArcLength_ - this->endSineArcLength_;

    const point2m anglePosDiff = car.pose.pos - this->previousCarPositions_.lerp((car.distance - this->prepareDistance_ / 2).get());
    const radian_t forwardAngle1 = anglePosDiff.getAngle();
    const radian_t forwardAngle2 = Sign::POSITIVE == this->targetSpeedSign_ ? car.pose.angle : normalize360(car.pose.angle + PI);

    const radian_t forwardAngle = avg(forwardAngle1, forwardAngle2);

    LOG_DEBUG("Overtake: start pos: (%f, %f) | forward angle: %fdeg", car.pose.pos.X.get(), car.pose.pos.Y.get(), static_cast<degree_t>(forwardAngle).get());

    this->trajectory_.setStartConfig(Trajectory::config_t{
        car.pose,
        this->beginSpeed_
    }, car.distance);

    this->trajectory_.appendSineArc(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ this->beginSineArcLength_, this->sideDistance_ }.rotate(forwardAngle),
            this->trajectory_.lastConfig().pose.angle
        },
        this->straightStartSpeed_
    }, forwardAngle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI);

    this->trajectory_.appendLine(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ STRAIGHT_SPEED_RAMP_DIST, centimeter_t(0) }.rotate(forwardAngle),
            this->trajectory_.lastConfig().pose.angle
        },
        this->straightSpeed_
    });

    this->trajectory_.appendLine(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ fastSectionLength - STRAIGHT_SPEED_RAMP_DIST, centimeter_t(0) }.rotate(forwardAngle),
            this->trajectory_.lastConfig().pose.angle
        },
        this->straightSpeed_
    });

    this->trajectory_.appendSineArc(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ this->endSineArcLength_, -this->sideDistance_ }.rotate(forwardAngle),
            this->trajectory_.lastConfig().pose.angle
        },
        this->endSpeed_
    }, forwardAngle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI_2);

    this->trajectory_.appendLine(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ this->endSlideDistance_, centimeter_t(0) }.rotate(forwardAngle - degree_t(40)),
            this->trajectory_.lastConfig().pose.angle
        },
        this->endSpeed_
    });
}
