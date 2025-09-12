#include <LaneChangeManeuver.hpp>
#include <cfg_car.hpp>

#include <micro/log/log.hpp>
#include <micro/math/numeric.hpp>

using namespace micro;

LaneChangeManeuver::LaneChangeManeuver()
    : Maneuver(), patternDir_(Sign::NEUTRAL), patternSide_(Direction::CENTER),
      initialSpeedSign_(Sign::NEUTRAL), safetyCarFollowSpeedSign_(Sign::NEUTRAL),
      state_(state_t::CheckOrientation) {
}

void LaneChangeManeuver::initialize(const micro::CarProps& car, const micro::Sign initialSpeedSign,
                                    const micro::Sign patternDir,
                                    const micro::Direction patternSide,
                                    const micro::Sign safetyCarFollowSpeedSign,
                                    const micro::m_per_sec_t speed,
                                    const micro::meter_t laneDistance,
                                    const bool reverseBeforeSine) {
    Maneuver::initialize();

    patternDir_               = patternDir;
    patternSide_              = patternSide;
    initialSpeedSign_         = initialSpeedSign;
    safetyCarFollowSpeedSign_ = safetyCarFollowSpeedSign;
    speed_                    = safetyCarFollowSpeedSign_ * speed;
    laneDistance_             = laneDistance;
    state_                    = state_t::CheckOrientation;
    reverseBeforeSine_        = reverseBeforeSine;

    trajectory_.clear();
}

void LaneChangeManeuver::update(const CarProps& car, const LineInfo& lineInfo, MainLine& mainLine,
                                ControlData& controlData) {
    switch (state_) {
    case state_t::CheckOrientation:
        controlData.rearSteerEnabled   = true;
        controlData.lineControl.actual = mainLine.centerLine;
        controlData.lineControl.target = {millimeter_t(0), radian_t(0)};

        if (abs(mainLine.centerLine.pos) < centimeter_t(3) &&
            abs(mainLine.centerLine.angle) < degree_t(4)) {
            if (initialSpeedSign_ == safetyCarFollowSpeedSign_ && reverseBeforeSine_) {
                state_            = state_t::Reverse;
                reverseStartDist_ = car.distance;
                LOG_DEBUG("Lane change state: REVERSE");
            } else {
                state_ = state_t::Stop;
                LOG_DEBUG("Lane change state: STOP");
            }
        }
        break;

    case state_t::Stop:
        controlData.speed    = m_per_sec_t(0);
        controlData.rampTime = millisecond_t(200);

        controlData.rearSteerEnabled   = false;
        controlData.lineControl.actual = mainLine.centerLine;
        controlData.lineControl.target = {millimeter_t(0), radian_t(0)};

        if (abs(car.speed) < cm_per_sec_t(2)) {
            buildTrajectory(car);
            state_ = state_t::FollowTrajectory;
            LOG_DEBUG("Lane change state: FOLLOW_TRAJECTORY");
        }
        break;

    case state_t::Reverse:
        controlData.speed    = -speed_;
        controlData.rampTime = millisecond_t(350);

        controlData.rearSteerEnabled   = false;
        controlData.lineControl.actual = mainLine.centerLine;
        controlData.lineControl.target = {millimeter_t(0), radian_t(0)};

        if (car.distance > reverseStartDist_ + centimeter_t(30)) {
            state_ = state_t::Stop;
            LOG_DEBUG("Lane change state: STOP");
        }
        break;

    case state_t::FollowTrajectory:
        controlData = trajectory_.update(car);

        if (trajectory_.finished(car, lineInfo, centimeter_t(20))) {
            LOG_DEBUG("Lane change finished");
            finish();
        }
        break;
    }
}

void LaneChangeManeuver::buildTrajectory(const micro::CarProps& car) {
    trajectory_.setStartConfig(Trajectory::config_t{car.pose, speed_}, car.distance);

    const radian_t forwardAngle = Sign::POSITIVE == safetyCarFollowSpeedSign_
                                      ? car.pose.angle
                                      : normalize360(car.pose.angle + PI);
    const auto side =
        initialSpeedSign_ * safetyCarFollowSpeedSign_ * micro::underlying_value(patternSide_);

    if (initialSpeedSign_ * patternDir_ == safetyCarFollowSpeedSign_) {
        trajectory_.appendSineArc(
            Trajectory::config_t{
                Pose{trajectory_.lastConfig().pose.pos +
                         vec2m{centimeter_t(90), side * (laneDistance_ - centimeter_t(5))}.rotate(
                             forwardAngle),
                     car.pose.angle},
                speed_,
            },
            forwardAngle, Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

    } else {
        meter_t radius = laneDistance_ / 2;

        if (radius < cfg::MIN_TURN_RADIUS) {
            const meter_t sineArcWidth = 2 * (cfg::MIN_TURN_RADIUS - radius) + centimeter_t(10);
            radius                     = cfg::MIN_TURN_RADIUS;

            trajectory_.appendSineArc(
                Trajectory::config_t{
                    Pose{trajectory_.lastConfig().pose.pos +
                             vec2m{centimeter_t(60), -side * sineArcWidth}.rotate(forwardAngle),
                         car.pose.angle},
                    speed_,
                },
                car.pose.angle, Trajectory::orientationUpdate_t::PATH_ORIENTATION, radian_t(0), PI);

            trajectory_.appendCircle(trajectory_.lastConfig().pose.pos +
                                         vec2m{centimeter_t(0), side * radius}.rotate(forwardAngle),
                                     PI, speed_);

        } else {
            trajectory_.appendCircle(trajectory_.lastConfig().pose.pos +
                                         vec2m{centimeter_t(0), side * radius}.rotate(forwardAngle),
                                     PI, speed_);
        }
    }
}
