#include <micro/container/vec.hpp>
#include <micro/test/utils.hpp>

#include <LabyrinthNavigator.hpp>
#include <cfg_car.hpp>
#include <track.hpp>

using namespace micro;

namespace {

m_per_sec_t LABYRINTH_SPEED      = m_per_sec_t(1.0f);
m_per_sec_t LABYRINTH_FAST_SPEED = m_per_sec_t(1.0f);
m_per_sec_t LANE_CHANGE_SPEED    = m_per_sec_t(0.8f);

#define START_SEGMENT       'W'
#define PREV_SEGMENT        'M'
#define LANE_CHANGE_SEGMENT 'N'

LabyrinthGraph graph = buildTestLabyrinthGraph();
const Segment *startSeg = graph.findSegment(START_SEGMENT);
const Connection *prevConn = graph.findConnection(*graph.findSegment(PREV_SEGMENT), *startSeg);
const Segment *laneChangeSeg = graph.findSegment(LANE_CHANGE_SEGMENT);

} // namespace

TEST(labyrinthNavigator_test_labyrinth, W_O) {
    LabyrinthNavigator navigator(graph, startSeg, prevConn, LABYRINTH_SPEED, LABYRINTH_FAST_SPEED);
    navigator.initialize();

    CarProps car;
    LineInfo lineInfo;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    ControlData controlData;
    controlData.speed = LABYRINTH_FAST_SPEED;
    controlData.rampTime = millisecond_t(500);

    navigator.setTargetSegment(graph.findSegment('O'), false);


    car.pose.pos   = { meter_t(0), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = meter_t(0);

    lineInfo.front.lines   = { { centimeter_t(0), 1 } };
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;





    car.pose.pos   = { centimeter_t(330), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(330);

    lineInfo.front.lines   = { { centimeter_t(0), 1 }, { centimeter_t(3.8f), 2 } };
    lineInfo.front.pattern = { LinePattern::JUNCTION_1, Sign::NEGATIVE, Direction::CENTER, centimeter_t(300) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;





    car.pose.pos   = { centimeter_t(350), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(350);

    lineInfo.front.lines   = { { centimeter_t(0), 1 }, { centimeter_t(10), 2 } };
    lineInfo.front.pattern = { LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT, centimeter_t(340) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 }, { centimeter_t(3.8f), 2 } };
    lineInfo.rear.pattern  = { LinePattern::JUNCTION_1, Sign::NEGATIVE, Direction::CENTER, centimeter_t(340) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;





    car.pose.pos   = { centimeter_t(370), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(370);

    lineInfo.front.lines   = { { centimeter_t(0), 1 } };
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 }, { centimeter_t(10), 2 } };
    lineInfo.rear.pattern  = { LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT, centimeter_t(340) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;




    car.pose.pos   = { centimeter_t(400), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(400);

    lineInfo.front.lines   = { { centimeter_t(0), 1 } };
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;




    navigator.setTargetSegment(graph.findSegment('N'), false);

    car.pose.pos   = { centimeter_t(450), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(450);

    lineInfo.front.lines   = { { centimeter_t(0), 1 } };
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;





    car.pose.pos   = { centimeter_t(370), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(550);

    lineInfo.front.lines   = { { centimeter_t(0), 1 } };
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 }, { centimeter_t(10), 2 } };
    lineInfo.rear.pattern  = { LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT, centimeter_t(340) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;



    car.pose.pos   = { centimeter_t(350), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(570);

    lineInfo.front.lines   = { { centimeter_t(0), 1 }, { centimeter_t(10), 2 } };
    lineInfo.front.pattern = { LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT, centimeter_t(340) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 }, { centimeter_t(3.8f), 2 } };
    lineInfo.rear.pattern  = { LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER, centimeter_t(340) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;



    car.pose.pos   = { centimeter_t(330), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = centimeter_t(590);

    lineInfo.front.lines   = { { centimeter_t(0), 1 }, { centimeter_t(3.8f), 2 } };
    lineInfo.front.pattern = { LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER, centimeter_t(300) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;



    car.pose.pos   = { meter_t(250), meter_t(0) };
    car.pose.angle = radian_t(0);
    car.distance   = meter_t(700);

    lineInfo.front.lines   = { { centimeter_t(0), 1 } };
    lineInfo.front.pattern = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };
    lineInfo.rear.lines    = { { centimeter_t(0), 1 } };
    lineInfo.rear.pattern  = { LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) };

    micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);
    navigator.update(car, lineInfo, mainLine, controlData);
    car.speed = controlData.speed;
}
