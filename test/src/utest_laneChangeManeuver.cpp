#include <micro/test/utils.hpp>

#include <cfg_car.hpp>

#define private public
#include <LaneChangeManeuver.hpp>
#undef private

using namespace micro;

namespace {

const meter_t TRAJECTORY_LENGTH_SINE   = centimeter_t(105);
const meter_t TRAJECTORY_LENGTH_CIRCLE = centimeter_t(190);

const point2m TRAJECTORY_END_POS_SINE_LEFT    = { centimeter_t(90), centimeter_t(55) };
const point2m TRAJECTORY_END_POS_SINE_RIGHT   = { centimeter_t(90), centimeter_t(-55) };
const point2m TRAJECTORY_END_POS_CIRCLE_LEFT  = { centimeter_t(60), centimeter_t(50) };
const point2m TRAJECTORY_END_POS_CIRCLE_RIGHT = { centimeter_t(60), centimeter_t(-50) };

void test(const micro::Sign& initialSpeedSign, const micro::Sign patternDir, const micro::Direction patternSide, const micro::Sign safetyCarFollowSpeedSign,
    const meter_t expectedLength, const point2m& expectedEndPos) {
    LaneChangeManeuver maneuver;
    CarProps car;
    LineInfo lineInfo;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    ControlData controlData;

    maneuver.initialize(car, initialSpeedSign, patternDir, patternSide, safetyCarFollowSpeedSign,  m_per_sec_t(1), centimeter_t(60));
    
    // updates maneuver twice to go through orientation check and build trajectory
    maneuver.update(car, lineInfo, mainLine, controlData);
    maneuver.update(car, lineInfo, mainLine, controlData);

    EXPECT_NEAR_UNIT(expectedLength, maneuver.trajectory_.length(), centimeter_t(5));

    EXPECT_NEAR_UNIT(expectedEndPos.X, maneuver.trajectory_.lastConfig().pose.pos.X, centimeter_t(5));
    EXPECT_NEAR_UNIT(expectedEndPos.Y, maneuver.trajectory_.lastConfig().pose.pos.Y, centimeter_t(5));
}

// Positive initial speed sign

TEST(LaneChangeManeuver, POSITIVE_POSITIVE_LEFT_POSITIVE) {
    test(Sign::POSITIVE, Sign::POSITIVE, Direction::LEFT, Sign::POSITIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_LEFT);
}

TEST(LaneChangeManeuver, POSITIVE_POSITIVE_LEFT_NEGATIVE) {
    test(Sign::POSITIVE, Sign::POSITIVE, Direction::LEFT, Sign::NEGATIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_RIGHT.rotate180());
}

TEST(LaneChangeManeuver, POSITIVE_POSITIVE_RIGHT_POSITIVE) {
    test(Sign::POSITIVE, Sign::POSITIVE, Direction::RIGHT, Sign::POSITIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_RIGHT);
}

TEST(LaneChangeManeuver, POSITIVE_POSITIVE_RIGHT_NEGATIVE) {
    test(Sign::POSITIVE, Sign::POSITIVE, Direction::RIGHT, Sign::NEGATIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_LEFT.rotate180());
}

TEST(LaneChangeManeuver, POSITIVE_NEGATIVE_LEFT_POSITIVE) {
    test(Sign::POSITIVE, Sign::NEGATIVE, Direction::LEFT, Sign::POSITIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_LEFT);
}

TEST(LaneChangeManeuver, POSITIVE_NEGATIVE_LEFT_NEGATIVE) {
    test(Sign::POSITIVE, Sign::NEGATIVE, Direction::LEFT, Sign::NEGATIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_RIGHT.rotate180());
}

TEST(LaneChangeManeuver, POSITIVE_NEGATIVE_RIGHT_POSITIVE) {
    test(Sign::POSITIVE, Sign::NEGATIVE, Direction::RIGHT, Sign::POSITIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_RIGHT);
}

TEST(LaneChangeManeuver, POSITIVE_NEGATIVE_RIGHT_NEGATIVE) {
    test(Sign::POSITIVE, Sign::NEGATIVE, Direction::RIGHT, Sign::NEGATIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_LEFT.rotate180());
}

// Negative initial speed sign

TEST(LaneChangeManeuver, NEGATIVE_POSITIVE_LEFT_POSITIVE) {
    test(Sign::NEGATIVE, Sign::POSITIVE, Direction::LEFT, Sign::POSITIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_RIGHT);
}

TEST(LaneChangeManeuver, NEGATIVE_POSITIVE_LEFT_NEGATIVE) {
    test(Sign::NEGATIVE, Sign::POSITIVE, Direction::LEFT, Sign::NEGATIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_LEFT.rotate180());
}

TEST(LaneChangeManeuver, NEGATIVE_POSITIVE_RIGHT_POSITIVE) {
    test(Sign::NEGATIVE, Sign::POSITIVE, Direction::RIGHT, Sign::POSITIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_LEFT);
}

TEST(LaneChangeManeuver, NEGATIVE_POSITIVE_RIGHT_NEGATIVE) {
    test(Sign::NEGATIVE, Sign::POSITIVE, Direction::RIGHT, Sign::NEGATIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_RIGHT.rotate180());
}

TEST(LaneChangeManeuver, NEGATIVE_NEGATIVE_LEFT_POSITIVE) {
    test(Sign::NEGATIVE, Sign::NEGATIVE, Direction::LEFT, Sign::POSITIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_RIGHT);
}

TEST(LaneChangeManeuver, NEGATIVE_NEGATIVE_LEFT_NEGATIVE) {
    test(Sign::NEGATIVE, Sign::NEGATIVE, Direction::LEFT, Sign::NEGATIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_LEFT.rotate180());
}

TEST(LaneChangeManeuver, NEGATIVE_NEGATIVE_RIGHT_POSITIVE) {
    test(Sign::NEGATIVE, Sign::NEGATIVE, Direction::RIGHT, Sign::POSITIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE_LEFT);
}

TEST(LaneChangeManeuver, NEGATIVE_NEGATIVE_RIGHT_NEGATIVE) {
    test(Sign::NEGATIVE, Sign::NEGATIVE, Direction::RIGHT, Sign::NEGATIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE_RIGHT.rotate180());
}

} // namespace
