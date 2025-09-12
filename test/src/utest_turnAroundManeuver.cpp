#include <cfg_car.hpp>

#include <micro/test/utils.hpp>

#define private public
#include <TurnAroundManeuver.hpp>
#undef private

using namespace micro;

namespace {

constexpr m_per_sec_t TURN_AROUND_SPEED       = m_per_sec_t(1.0f);
constexpr meter_t TURN_AROUND_RADIUS          = centimeter_t(40);
constexpr meter_t TURN_AROUND_SINE_ARC_LENGTH = centimeter_t(60);

} // namespace

TEST(TurnAroundManeuver, simple) {
    TurnAroundManeuver maneuver;
    CarProps car;
    LineInfo lineInfo;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    ControlData controlData;

    maneuver.initialize(car, Sign::POSITIVE, TURN_AROUND_SPEED, TURN_AROUND_SINE_ARC_LENGTH,
                        TURN_AROUND_RADIUS);
    maneuver.update(car, lineInfo, mainLine, controlData);

    EXPECT_NEAR_UNIT(centimeter_t(300), maneuver.trajectory_.length(), centimeter_t(5));
}
