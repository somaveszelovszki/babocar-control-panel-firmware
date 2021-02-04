#include <micro/test/utils.hpp>

#include <cfg_car.hpp>

#define private public
#include <LaneChangeManeuver.hpp>
#undef private

using namespace micro;

namespace {

const meter_t TRAJECTORY_LENGTH_SINE   = centimeter_t(124);
const meter_t TRAJECTORY_LENGTH_CIRCLE = centimeter_t(190);

const point2m TRAJECTORY_END_POS_SINE   = { centimeter_t(110), centimeter_t(-52) };
const point2m TRAJECTORY_END_POS_CIRCLE = { centimeter_t(60), centimeter_t(55) };

void test(const micro::Sign& initialSpeedSign, const micro::Sign patternDir, const micro::Direction patternSide, const micro::Sign safetyCarFollowSpeedSign,
    const meter_t expectedLength, const point2m& expectedEndPos) {
    LaneChangeManeuver maneuver;
    CarProps car;
    LineInfo lineInfo;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    ControlData controlData;

    maneuver.initialize(car, initialSpeedSign, patternDir, patternSide, safetyCarFollowSpeedSign,  m_per_sec_t(1), centimeter_t(60));
    maneuver.update(car, lineInfo, mainLine, controlData);

    EXPECT_NEAR_UNIT(expectedLength, maneuver.trajectory_.length(), centimeter_t(5));

    EXPECT_NEAR_UNIT(expectedEndPos.X, maneuver.trajectory_.lastConfig().pose.pos.X, centimeter_t(5));
    EXPECT_NEAR_UNIT(expectedEndPos.Y, maneuver.trajectory_.lastConfig().pose.pos.Y, centimeter_t(5));
}

} // namespace

TEST(LaneChangeManeuver, POSITIVE_POSITIVE_POSITIVE) {
    test(Sign::POSITIVE, Sign::POSITIVE, Direction::RIGHT, Sign::POSITIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE);
}

TEST(LaneChangeManeuver, POSITIVE_NEGATIVE_POSITIVE) {
    test(Sign::POSITIVE, Sign::NEGATIVE, Direction::LEFT, Sign::POSITIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE);
}

TEST(LaneChangeManeuver, NEGATIVE_POSITIVE_POSITIVE) {
    test(Sign::NEGATIVE, Sign::POSITIVE, Direction::RIGHT, Sign::POSITIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE);
}

TEST(LaneChangeManeuver, NEGATIVE_NEGATIVE_POSITIVE) {
    test(Sign::NEGATIVE, Sign::NEGATIVE, Direction::LEFT, Sign::POSITIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE);
}

TEST(LaneChangeManeuver, POSITIVE_POSITIVE_NEGATIVE) {
    test(Sign::POSITIVE, Sign::POSITIVE, Direction::RIGHT, Sign::NEGATIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE.rotate180());
}

TEST(LaneChangeManeuver, POSITIVE_NEGATIVE_NEGATIVE) {
    test(Sign::POSITIVE, Sign::NEGATIVE, Direction::LEFT, Sign::NEGATIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE.rotate180());
}

TEST(LaneChangeManeuver, NEGATIVE_POSITIVE_NEGATIVE) {
    test(Sign::NEGATIVE, Sign::POSITIVE, Direction::RIGHT, Sign::NEGATIVE, TRAJECTORY_LENGTH_SINE, TRAJECTORY_END_POS_SINE.rotate180());
}

TEST(LaneChangeManeuver, NEGATIVE_NEGATIVE_NEGATIVE) {
    test(Sign::NEGATIVE, Sign::NEGATIVE, Direction::LEFT, Sign::NEGATIVE, TRAJECTORY_LENGTH_CIRCLE, TRAJECTORY_END_POS_CIRCLE.rotate180());
}