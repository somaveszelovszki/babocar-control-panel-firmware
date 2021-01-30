#include <micro/test/utils.hpp>

#include <cfg_car.hpp>
#include <RaceTrackInfo.hpp>
#include <track.hpp>

using namespace micro;

TEST(raceTrackInfo, test) {
    RaceTrackInfo trackInfo(testTrackSegments);
    trackInfo.lap = 1;
    trackInfo.seg = trackInfo.segments.begin();

    CarProps car;
    LineInfo lineInfo;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    ControlData controlData;
    controlData.speed = m_per_sec_t(1);
    controlData.rampTime = millisecond_t(500);
    car.orientedDistance = centimeter_t(50); // necessary for correct ACCELERATE segment detections

    car.distance = meter_t(0);
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::BRAKE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::ACCELERATE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::BRAKE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::ACCELERATE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::BRAKE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::ACCELERATE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::BRAKE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    trackInfo.update(car, lineInfo, mainLine);

    car.distance += trackInfo.seg->length + millimeter_t(1);
    lineInfo.front.pattern = { LinePattern::ACCELERATE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    trackInfo.update(car, lineInfo, mainLine);

    EXPECT_EQ(2, trackInfo.lap);
}
