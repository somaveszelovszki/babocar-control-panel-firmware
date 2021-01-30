#include <micro/test/utils.hpp>

#include <cfg_car.hpp>

#define private public
#include <OvertakeManeuver.hpp>
#undef private

using namespace micro;

namespace {

m_per_sec_t OVERTAKE_BEGIN_SPEED          = m_per_sec_t(1.0f);
m_per_sec_t OVERTAKE_STRAIGHT_START_SPEED = m_per_sec_t(1.2f);
m_per_sec_t OVERTAKE_STRAIGHT_SPEED       = m_per_sec_t(2.0f);
m_per_sec_t OVERTAKE_END_SPEED            = m_per_sec_t(1.8f);

meter_t OVERTAKE_SECTION_LENGTH           = centimeter_t(800);
meter_t OVERTAKE_PREPARE_DISTANCE         = centimeter_t(70);
meter_t OVERTAKE_BEGIN_SINE_ARC_LENGTH    = centimeter_t(180);
meter_t OVERTAKE_END_SINE_ARC_LENGTH      = centimeter_t(180);
meter_t OVERTAKE_SIDE_DISTANCE            = centimeter_t(50);

const meter_t TRAJECTORY_LENGTH  = centimeter_t(860);
const point2m TRAJECTORY_END_POS = { centimeter_t(915), centimeter_t(-71) };

void test(const Sign targetSpeedSign, const point2m& expectedEndPos) {
    OvertakeManeuver maneuver;
    CarProps car;
    LineInfo lineInfo;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    ControlData controlData;

    maneuver.initialize(car, targetSpeedSign,
        OVERTAKE_BEGIN_SPEED, OVERTAKE_STRAIGHT_START_SPEED, OVERTAKE_STRAIGHT_SPEED, OVERTAKE_END_SPEED,
        OVERTAKE_SECTION_LENGTH, OVERTAKE_PREPARE_DISTANCE, OVERTAKE_BEGIN_SINE_ARC_LENGTH, OVERTAKE_END_SINE_ARC_LENGTH,
        OVERTAKE_SIDE_DISTANCE);

    car.pose.pos         = { targetSpeedSign * meter_t(1), meter_t(0) };
    car.distance         = meter_t(1);
    car.orientedDistance = meter_t(1);
    maneuver.update(car, lineInfo, mainLine, controlData);

    EXPECT_NEAR_UNIT(TRAJECTORY_LENGTH, maneuver.trajectory_.length(), centimeter_t(5));

    EXPECT_NEAR_UNIT(expectedEndPos.X, maneuver.trajectory_.lastConfig().pose.pos.X, centimeter_t(5));
    EXPECT_NEAR_UNIT(expectedEndPos.Y, maneuver.trajectory_.lastConfig().pose.pos.Y, centimeter_t(5));
}

} // namespace

TEST(OvertakeManeuver, POSITIVE) {
    test(Sign::POSITIVE, TRAJECTORY_END_POS);
}

TEST(OvertakeManeuver, NEGATIVE) {
    test(Sign::NEGATIVE, TRAJECTORY_END_POS.rotate180());
}
