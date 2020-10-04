#include <micro/debug/SystemManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/math/numeric.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>
#include <micro/utils/units.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <DetectedLines.hpp>
#include <LabyrinthGraphBuilder.hpp>
#include <LabyrinthNavigator.hpp>

using namespace micro;

#define RANDOM_SEGMENT true

extern queue_t<CarProps, 1> carPropsQueue;
extern queue_t<point2m, 1> carPosUpdateQueue;
extern queue_t<radian_t, 1> carOrientationUpdateQueue;
extern queue_t<linePatternDomain_t, 1> linePatternDomainQueue;
extern queue_t<DetectedLines, 1> detectedLinesQueue;
extern queue_t<ControlData, 1> controlQueue;
extern queue_t<radian_t, 1> carOrientationUpdateQueue;
extern queue_t<char, 1> radioRecvQueue;

extern Sign safetyCarFollowSpeedSign;

namespace {

m_per_sec_t speed_LAB_FWD      = m_per_sec_t(1.1f);
m_per_sec_t speed_LAB_FWD_slow = m_per_sec_t(1.0f);
m_per_sec_t speed_LAB_BWD      = m_per_sec_t(-0.8f);
m_per_sec_t speed_LANE_CHANGE  = m_per_sec_t(0.8f);

const LabyrinthGraph graph = buildLabyrinthGraph();
const LabyrinthGraph::Segments::const_iterator startSeg = graph.findSegment('A');
LabyrinthNavigator navigator(graph, *graph.findFirstConnection(*startSeg, *graph.findSegment('X')), *startSeg);

uint8_t numFoundSegments  = 0;
millisecond_t endTime;

struct {
    Segment *seg = nullptr;
    Trajectory trajectory;
} laneChange;

void onJunctionDetected(const point2m& pos, const meter_t distance, radian_t inOri, radian_t outOri, uint8_t numInSegments, Direction inSegmentDir, uint8_t numOutSegments) {

    LabyrinthGraph::Junctions::const_iterator junc = graph.findJunction(pos, inOri, outOri, numInSegments, numOutSegments);

    if (graph.junctions.end() != junc) {
        LOG_DEBUG("currentSeg: %c, junction: %u (%f, %f)", navigator.currentSegment()->name, static_cast<uint32_t>(junc->id), junc->pos.X.get(), junc->pos.Y.get());

        carPosUpdateQueue.overwrite(junc->pos);

        if (junc != navigator.nextConnection()->junction) {
            // an error has occurred, resets navigator

            const Segment *currentSeg      = junc->getSideSegments(inOri)->second.begin()->second;
            const Connection *junctionConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [junc](const Connection *c) {
                return c->junction == junc;
            });

            const Connection *prevConn = *std::find_if(currentSeg->edges.begin(), currentSeg->edges.end(), [junctionConn, currentSeg](const Connection *c) {
                return LabyrinthRoute::isNewConnectionValid(*junctionConn, *currentSeg, *c);
            });

            navigator.reset(*prevConn, *currentSeg);
        }

        navigator.onJunctionDetected(distance);

        // TODO handle dead-ends
    //    if (!nextSeg->isDeadEnd) {
    //        passedFwdRoute.push_back(*nextConn);
    //    }
    } else {
        // TODO
        LOG_ERROR("Junction not found");
    }
}

void updateCarOrientation(const CarProps& car, const DetectedLines& detectedLines) {
    static meter_t lastUpdatedOrientedDistance;

    if (car.orientedDistance - lastUpdatedOrientedDistance > centimeter_t(40) &&
        car.distance - detectedLines.front.pattern.startDist > centimeter_t(100) &&
        eqWithOverflow360(car.pose.angle, round90(car.pose.angle), degree_t(10))) {

        carOrientationUpdateQueue.overwrite(round90(car.pose.angle));
        lastUpdatedOrientedDistance = car.orientedDistance;
    }
}

LabyrinthGraph::Segments::const_iterator getTargetSegment(bool labyrinthDiscovered) {
    LabyrinthGraph::Segments::const_iterator seg = graph.segments.end();
    if (labyrinthDiscovered) {
        seg = laneChange.seg;
    } else {
        static char prevSegId = '\0';
        char segId = prevSegId;
        radioRecvQueue.peek(segId, millisecond_t(0));
        seg = segId != prevSegId ? graph.findSegment(segId) : navigator.targetSegment();
        prevSegId = segId;
    }
    return seg;
}

void handleTargetSegmentChange(const LabyrinthGraph::Segments::const_iterator targetSeg, ControlData& controlData) {
    if (targetSeg != navigator.targetSegment()) {
        navigator.setTargetSegment(*targetSeg);

        // if car is in a dead-end segment and a new target segment is received, starts going reverse instantly
//        if (navigator.currentSegment()->isDeadEnd) {
//            controlData.speed = speed_LAB_BWD;
//            controlData.rampTime = millisecond_t(400);
//        }
        // TODO handle dead-end

        endTime += second_t(10);
        ++numFoundSegments;
    }
}

void enforceTargetDirection(const Direction targetDir, const DetectedLines& detectedLines, MainLine& mainLine) {
    static constexpr millimeter_t MAX_LINE_JUMP = centimeter_t(3.5f);

    // updates the main line (if an error occurs, keeps previous line)
    if (detectedLines.front.lines.size() && detectedLines.rear.lines.size()) {
        switch (targetDir) {
        case Direction::LEFT:
            mainLine.frontLine = detectedLines.front.lines[0];
            mainLine.rearLine  = *detectedLines.rear.lines.back();
            break;
        case Direction::CENTER:
            mainLine.frontLine = detectedLines.front.lines.size() == 1 ? detectedLines.front.lines[0] : detectedLines.front.lines[1];
            mainLine.rearLine  = detectedLines.rear.lines.size()  == 1 ? detectedLines.rear.lines[0]  : detectedLines.rear.lines[1];
            break;
        case Direction::RIGHT:
            mainLine.frontLine = *detectedLines.front.lines.back();
            mainLine.rearLine  = detectedLines.rear.lines[0];
            break;
        }
    }
}

bool navigateLabyrinth(const CarProps& car, const DetectedLines& prevDetectedLines, const DetectedLines& detectedLines, MainLine& mainLine, ControlData& controlData) {

    static uint8_t numInSegments  = 0;
    static Direction inSegmentDir = Direction::CENTER;

    static const bool runOnce = [&controlData]() {
        endTime = getTime() + millisecond_t(20);
        carOrientationUpdateQueue.overwrite(radian_t(0));
        controlData.speed = speed_LAB_FWD;
        controlData.rampTime = millisecond_t(500);
        return true;
    }();
    UNUSED(runOnce);

    const bool labyrinthDiscovered = numFoundSegments == cfg::NUM_LABYRINTH_SEGMENTS;
    bool finished = false;

    updateCarOrientation(car, detectedLines);

    controlData.controlType = ControlData::controlType_t::Line;

    handleTargetSegmentChange(getTargetSegment(labyrinthDiscovered), controlData);

    if (detectedLines.front.pattern != prevDetectedLines.front.pattern) {

        switch (detectedLines.front.pattern.type) {
        case LinePattern::JUNCTION_1:
        case LinePattern::JUNCTION_2:
        case LinePattern::JUNCTION_3:
        {
            LOG_DEBUG("Junction pattern");
            const uint8_t numSegments = LinePattern::JUNCTION_1 == detectedLines.front.pattern.type ? 1 :
                LinePattern::JUNCTION_2 == detectedLines.front.pattern.type ? 2 : 3;

            if (Sign::NEGATIVE == detectedLines.front.pattern.dir) {
                numInSegments = numSegments;

                // Line pattern direction indicates on which side of the current line the OTHER lines are,
                // so if the current line is the leftmost line of three lines, pattern direction will be RIGHT.
                // Segment direction in a junction indicates which side the car should steer in order to follow a segment,
                // when leaving a junction.
                // Currently the car is entering a junction, therefore if the current line is the leftmost,
                // the car will need to steer to the right when leaving the junction.
                // So the segment direction is the same as the line pattern direction (in this example, RIGHT).
                inSegmentDir = detectedLines.front.pattern.side;

                // if the car is going backwards, mirrored pattern sides are detected
//                if (currentSeg->isDeadEnd) {
//                    inSegmentDir         = -inSegmentDir;
//                    controlData.speed    = speed_LAB_FWD_slow;
//                    controlData.rampTime = millisecond_t(700);
//                }
                // TODO handle dead-ends

                // follows center line in junctions
                if (3 == detectedLines.front.lines.size()) {
                    mainLine.frontLine = detectedLines.front.lines[1];
                }
                if (3 == detectedLines.rear.lines.size()) {
                    mainLine.rearLine = detectedLines.rear.lines[1];
                }

            } else if (Sign::POSITIVE == detectedLines.front.pattern.dir) {

                const radian_t carOri = car.pose.angle;
                const radian_t inOri  = round90(carOri + PI);
                const radian_t outOri = round90(carOri);

//                if (currentSeg->isDeadEnd) {
//                    if (prevConn) {
//                        // when coming back from a dead-end, junction should be handled as if the car
//                        // arrived from the segment before the dead-end segment
//                        currentSeg = prevConn->getOtherSegment(*currentSeg);
//                        numInSegments = prevConn->junction->getSideSegments(inOri)->second.size();
//                        inSegmentDir = prevConn->getManeuver(*currentSeg).direction;
//                        carPosUpdateQueue.overwrite(prevConn->junction->pos);
//                        prevConn = nullptr;
//                    } else {
//                        LOG_ERROR("Current segment is dead-end but there has been no junctions yet");
//                    }
//                }
                // TODO handle dead-ends

                onJunctionDetected(car.pose.pos, car.distance, inOri, outOri, numInSegments, inSegmentDir, numSegments);
            }
            break;
        }

        case LinePattern::DEAD_END:
            controlData.speed = speed_LAB_BWD;
            controlData.rampTime = millisecond_t(400);
            break;

        case LinePattern::LANE_CHANGE:
            if (labyrinthDiscovered) {
                finished = true;
            }
            break;

        default:
            break;
        }
    }

    const Direction targetDir = navigator.update(car.distance);
    enforceTargetDirection(targetDir, detectedLines, mainLine);

    mainLine.updateCenterLine(car.speed >= m_per_sec_t(0));

    controlData.controlType         = ControlData::controlType_t::Line;
    controlData.lineControl.actual  = mainLine.centerLine;
    controlData.lineControl.desired = { millimeter_t(0), radian_t(0) };

    return finished;
}

bool changeLane(const CarProps& car, const DetectedLines& detectedLines, ControlData& controlData) {

    static constexpr meter_t LANE_DISTANCE = centimeter_t(60);

    if (laneChange.trajectory.length() == meter_t(0)) {

        laneChange.trajectory.setStartConfig(Trajectory::config_t{
            car.pose,
            speed_LANE_CHANGE
        }, car.distance);

        if (safetyCarFollowSpeedSign == detectedLines.front.pattern.dir) {

            laneChange.trajectory.appendSineArc(Trajectory::config_t{
                Pose{
                    laneChange.trajectory.lastConfig().pose.pos + vec2m{ centimeter_t(80), -safetyCarFollowSpeedSign * LANE_DISTANCE }.rotate(car.pose.angle),
                    car.pose.angle
                },
                speed_LANE_CHANGE,
            }, car.pose.angle, Trajectory::orientationUpdate_t::FIX_ORIENTATION, 30, radian_t(0), PI);

        } else {
            laneChange.trajectory.appendCircle(
                laneChange.trajectory.lastConfig().pose.pos + vec2m{ centimeter_t(0), safetyCarFollowSpeedSign * LANE_DISTANCE / 2 }.rotate(car.pose.angle),
                PI,
                speed_LANE_CHANGE, 30);
        }

        laneChange.trajectory.appendLine(Trajectory::config_t{
            Pose{
                laneChange.trajectory.lastConfig().pose.pos + vec2m{ centimeter_t(50), centimeter_t(0) }.rotate(laneChange.trajectory.lastConfig().pose.angle),
                laneChange.trajectory.lastConfig().pose.angle
            },
            speed_LANE_CHANGE
        });
    }

    controlData = laneChange.trajectory.update(car);

    const bool finished = laneChange.trajectory.length() - laneChange.trajectory.coveredDistance() < centimeter_t(40) && LinePattern::NONE != detectedLines.front.pattern.type;
    if (finished) {
        laneChange.trajectory.clear();
    }
    return finished;
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {

    SystemManager::instance().registerTask();

    DetectedLines prevDetectedLines, detectedLines;
    ControlData controlData;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    while (true) {
        const cfg::ProgramState programState = static_cast<cfg::ProgramState>(SystemManager::instance().programState());
        if (isBtw(enum_cast(programState), enum_cast(cfg::ProgramState::NavigateLabyrinth), enum_cast(cfg::ProgramState::LaneChange))) {

            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));

            linePatternDomainQueue.overwrite(linePatternDomain_t::Labyrinth);
            detectedLinesQueue.peek(detectedLines, millisecond_t(0));
            micro::updateMainLine(detectedLines.front.lines, detectedLines.rear.lines, mainLine, car.speed >= m_per_sec_t(0));

            switch (programState) {
            case cfg::ProgramState::NavigateLabyrinth:
                if (navigateLabyrinth(car, prevDetectedLines, detectedLines, mainLine, controlData)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::LaneChange));
                }
                break;

            case cfg::ProgramState::LaneChange:
                if (changeLane(car, detectedLines, controlData)) {
                    SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::ReachSafetyCar));
                }
                break;
            default:
                break;
            }

            controlQueue.overwrite(controlData);
            prevDetectedLines = detectedLines;
        }

        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(2));
    }
}
