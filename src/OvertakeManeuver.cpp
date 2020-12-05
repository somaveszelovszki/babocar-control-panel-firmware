#include <micro/math/numeric.hpp>
#include <OvertakeManeuver.hpp>

using namespace micro;

OvertakeManeuver::OvertakeManeuver()
    : Maneuver()
    , state_(state_t::Prepare) {}

void OvertakeManeuver::initialize(const micro::CarProps& car,
    const micro::m_per_sec_t beginSpeed, const micro::m_per_sec_t straightStartSpeed, const micro::m_per_sec_t straightEndSpeed, const micro::m_per_sec_t endSpeed,
    const micro::meter_t sectionLength, const micro::meter_t prepareDistance, const micro::meter_t beginSineArcLength, const micro::meter_t endSineArcLength,
    const micro::meter_t sideDistance) {
    Maneuver::initialize();

    const Sign speedSign = sgn(car.speed);

    this->initialCarProps_    = car;
    this->beginSpeed_         = speedSign * beginSpeed;
    this->straightStartSpeed_ = speedSign * straightStartSpeed;
    this->straightEndSpeed_   = speedSign * straightEndSpeed;
    this->endSpeed_           = speedSign * endSpeed;
    this->sectionLength_      = sectionLength;
    this->prepareDistance_    = prepareDistance;
    this->beginSineArcLength_ = beginSineArcLength;
    this->endSineArcLength_   = endSineArcLength;
    this->sideDistance_       = sideDistance;
    this->state_              = state_t::Prepare;

    this->trajectory_.clear();
}

void OvertakeManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine, ControlData& controlData) {
    switch (this->state_) {

    case state_t::Prepare:
        // does not change longitudinal control in order to continue following safety car

        controlData.controlType         = ControlData::controlType_t::Line;
        controlData.lineControl.actual  = mainLine.centerLine;
        controlData.lineControl.target  = { millimeter_t(0), radian_t(0) };

        if (car.orientedDistance >= this->prepareDistance_ && car.distance - this->initialCarProps_.distance >= this->prepareDistance_) {
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

void OvertakeManeuver::buildTrajectory(const micro::CarProps& car) {

    const point2m posDiff           = car.pose.pos - this->initialCarProps_.pose.pos;
    const meter_t fastSectionLength = this->sectionLength_ - posDiff.length() - this->beginSineArcLength_ - this->endSineArcLength_;
    const radian_t forwardAngle     = posDiff.getAngle();

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
            this->trajectory_.lastConfig().pose.pos + vec2m{ fastSectionLength, centimeter_t(0) }.rotate(forwardAngle),
            this->trajectory_.lastConfig().pose.angle
        },
        this->straightEndSpeed_
    });

    this->trajectory_.appendSineArc(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ this->endSineArcLength_, -this->sideDistance_ }.rotate(forwardAngle),
            this->trajectory_.lastConfig().pose.angle
        },
        this->endSpeed_
    }, forwardAngle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, radian_t(0), PI);

    this->trajectory_.appendLine(Trajectory::config_t{
        Pose{
            this->trajectory_.lastConfig().pose.pos + vec2m{ centimeter_t(100), centimeter_t(0) }.rotate(forwardAngle),
            this->trajectory_.lastConfig().pose.angle
        },
        this->endSpeed_
    });
}
