#include <utility>

#include <micro/container/vector.hpp>
#include <micro/debug/ParamManager.hpp>
#include <micro/debug/TaskMonitor.hpp>
#include <micro/log/log.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/port/timer.hpp>
#include <micro/math/numeric.hpp>
#include <micro/math/random_generator.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>
#include <micro/utils/units.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>
#include <LaneChangeManeuver.hpp>
#include <LabyrinthNavigator.hpp>
#include <track.hpp>

using namespace micro;

namespace {

constexpr auto LABYRINTH_SPEED          = m_per_sec_t(0.85f);
constexpr auto LABYRINTH_DEAD_END_SPEED = m_per_sec_t(0.65f);
constexpr auto LANE_CHANGE_SPEED        = m_per_sec_t(0.65f);
constexpr auto LANE_DISTANCE            = centimeter_t(60);

#if TRACK == RACE_TRACK
#define START_SEGMENT                    "UX"
#define PREV_SEGMENT                     "X_"
#define LANE_CHANGE_SEGMENT              "QV"
#define LAST_JUNCTION_BEFORE_LANE_CHANGE 'Q'
#elif TRACK == TEST_TRACK
#define START_SEGMENT                    "WY"
#define PREV_SEGMENT                     "Y_"
#define LANE_CHANGE_SEGMENT              "NQ"
#define LAST_JUNCTION_BEFORE_LANE_CHANGE 'N'
#endif

class DirectionGenerator : public micro::irandom_generator {
public:
    using Route = micro::vector<micro::Direction, 20>;

    void initialize(const Route& route = {}) {
        true_random_ = micro::random_generator{static_cast<uint16_t>(micro::getExactTime().get())};
        route_ = route;
    }

    float operator()() override {
        if (!route_.empty()) {
            const auto dir = route_.front();
            route_.erase(route_.begin());
            switch (dir) {
            case micro::Direction::LEFT:
                return 0.999999f;

            case micro::Direction::CENTER:
                return 0.5f;

            case micro::Direction::RIGHT:
                return 0.0f;

            default:
                break;
            }
        }

        return true_random_();
    }

private:
    micro::random_generator true_random_;
    Route route_;
};

LabyrinthGraph graph;
DirectionGenerator directionGenerator;
LabyrinthNavigator navigator(graph, directionGenerator);
millisecond_t endTime;

const auto _ = []() {
    buildLabyrinthGraph(graph);

    const auto* prevSeg                      = graph.findSegment(PREV_SEGMENT);
    const auto* startSeg                     = graph.findSegment(START_SEGMENT);
    const auto* laneChangeSeg                = graph.findSegment(LANE_CHANGE_SEGMENT);
    const auto* lastJunctionBeforeLaneChange = graph.findJunction(LAST_JUNCTION_BEFORE_LANE_CHANGE);
    const auto* prevConn                     = graph.findConnection(*prevSeg, *startSeg);

    const auto unvisitedSegments = graph.getVisitableSegments();

    // The following segments are forbidden because they lead to dead-end segments.
    // Randomly navigating into them is not allowed, but they can be used for route creation.
    const SegmentIds forbiddenSegments = {
#if TRACK == TEST_TRACK
        "WY",
        "AC"
#elif TRACK == RACE_TRACK
        "MP",
        "MQ",
        "NS",
        "OU",
        "PR",
        "TU",
        "UX"
#endif
    };

    navigator.initialize(
        unvisitedSegments,
        forbiddenSegments,
        startSeg,
        prevConn,
        laneChangeSeg,
        lastJunctionBeforeLaneChange,
        LABYRINTH_SPEED,
        LABYRINTH_DEAD_END_SPEED);

    return true;
}();

struct JunctionPatternInfo {
    Sign dir            = Sign::NEUTRAL;
    Direction side      = Direction::CENTER;
    uint8_t numSegments = 0;
};

LaneChangeManeuver laneChange;

void handleRadioCommand() {
    char command[cfg::RADIO_COMMAND_MAX_LENGTH];
    if (!radioCommandQueue.receive(command, millisecond_t(0))) {
        return;
    }

    if (etl::strlen(command, cfg::RADIO_COMMAND_MAX_LENGTH) != 6) {
        LOG_WARN("Invalid labyrinth command: {}", command);
        return;
    }

    if (etl::strcmp(command, "FLOOD!") == 0) {
        navigator.navigateToLaneChange();
    } else {
        char junctions[3] = { command[0], command[1], command[2] };

        const auto fixPartitions = [](auto& prev, auto& next) {
            // In the test track, some segments are partitioned into multiple 'fake' segments.
            // Since the application logic cannot handle segments that are not separated by real junctions,
            // these partitions are joined together, forming one segment.
#if TRACK == TEST_TRACK
            if (prev == 'A' || prev == 'Y') {
                prev = '_';
            }

            if (next == 'A' || next == 'Y') {
                next = '_';
            }

            if ((prev == 'C' && next == 'B') || (prev == 'B' && next == 'D') || (prev == 'D' && next == 'F')) {
                prev = 'C';
                next = 'F';
            } else if ((prev == 'F' && next == 'D') || (prev == 'D' && next == 'B') || (prev == 'B' && next == 'C')) {
                prev = 'F';
                next = 'C';
            }

#elif TRACK == RACE_TRACK
            if ((prev == 'A' && next == 'C') || (prev == 'C' && next == 'F')) {
                prev = 'A';
                next = 'F';
            } else if ((prev == 'F' && next == 'C') || (prev == 'C' && next == 'A')) {
                prev = 'F';
                next = 'A';
            }
#endif
        };

        fixPartitions(junctions[0], junctions[1]);
        fixPartitions(junctions[1], junctions[2]);

        const auto current = Segment::makeId(junctions[0], junctions[1]);
        const auto next = Segment::makeId(junctions[1], junctions[2]);
        const auto now = micro::getTime();

        LabyrinthNavigator::ObstaclePositions obstaclePositions{{ current, next, now, false }};

#if TRACK == RACE_TRACK
        // If the obstacle is in cross-roads, the crossing segment needs to be added to the list.
        // Crossing segments:
        //     DI - FG
        //     IN - KL
        //     NS - RT
        //     OW - TU
        //     MQ - PR
        if (current == "DI") {
            obstaclePositions.insert({ "FG", "EG", now, true });
        } else if (current == "FG") {
            obstaclePositions.insert({ "DI", "BD", now, true });
        } else if (current == "IN") {
            obstaclePositions.insert({ "KL", "LO", now, true });
        } else if (current == "KL") {
            obstaclePositions.insert({ "IN", "NS", now, true });
        } else if (current == "NS") {
            obstaclePositions.insert({ "RT", "RP", now, true });
        } else if (current == "RT") {
            obstaclePositions.insert({ "NS", "SW", now, true });
        } else if (current == "OW") {
            obstaclePositions.insert({ "TU", "UX", now, true });
        } else if (current == "TU") {
            obstaclePositions.insert({ "OW", "VW", now, true });
        } else if (current == "MQ") {
            obstaclePositions.insert({ "PR", "P_", now, true });
        } else if (current == "PR") {
            obstaclePositions.insert({ "MQ", "Q_", now, true });
        }
#endif // TRACK == RACE_TRACK

        navigator.setObstaclePositions(obstaclePositions);
    }
}

bool shouldHandle(const ProgramState::Value state) {
    return isBtw(underlying_value(state), underlying_value(ProgramState::LabyrinthRoute), underlying_value(ProgramState::LaneChange));
}

void enforceGraphValidity() {
    if (!graph.valid()) {
        while (true) {
            LOG_ERROR("Labyrinth graph is invalid!");
            taskMonitor.notify(false);
            os_sleep(millisecond_t(100));
        }
    }
}

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {
    taskMonitor.registerInitializedTask();

    LineInfo lineInfo;
    ControlData controlData;
    LineDetectControl lineDetectControlData;
    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

    auto currentProgramState = ProgramState::INVALID;

    while (true) {
        const auto prevProgramState = std::exchange(currentProgramState, programState.get());
        if (shouldHandle(currentProgramState)) {
            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));

            lineInfoQueue.peek(lineInfo, millisecond_t(0));
            micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);

            lineDetectControlData.domain = linePatternDomain_t::Labyrinth;
            lineDetectControlData.isReducedScanRangeEnabled = false;

            switch (currentProgramState) {
            case ProgramState::LabyrinthRoute:
            case ProgramState::LabyrinthRandom:
            {
                if (currentProgramState != prevProgramState) {
                    enforceGraphValidity();
                    carOrientationUpdateQueue.overwrite(radian_t(0));
                    endTime = getTime() + minute_t(5);

                    switch (currentProgramState) {
                    case ProgramState::LabyrinthRoute:
                        directionGenerator.initialize({
                            micro::Direction::LEFT,   // UT
                            micro::Direction::RIGHT,  // NT
                            micro::Direction::CENTER, // IN
                            micro::Direction::LEFT,   // FI
                            micro::Direction::RIGHT,  // AF
                            micro::Direction::LEFT,   // AB
                            micro::Direction::CENTER, // BE
                            micro::Direction::LEFT,   // EJ
                            micro::Direction::CENTER, // JL
                            micro::Direction::CENTER  // KL

                        });
                        break;

                    case ProgramState::LabyrinthRandom:
                        directionGenerator.initialize();
                        break;

                    default:
                        break;
                    }
                }

                handleRadioCommand();
                navigator.update(car, lineInfo, mainLine, controlData);

                const Pose correctedCarPose = navigator.correctedCarPose();
                if (correctedCarPose.angle != car.pose.angle) {
                    carOrientationUpdateQueue.overwrite(correctedCarPose.angle);
                    LOG_DEBUG("Car orientation updated: {} -> {} [deg]",
                        static_cast<degree_t>(car.pose.angle).get(), static_cast<degree_t>(correctedCarPose.angle).get());
                }
                if (correctedCarPose.pos != car.pose.pos) {
                    carPosUpdateQueue.overwrite(correctedCarPose.pos);
                    LOG_DEBUG("Car position updated: [{}, {}] -> [{}, {}] [m]",
                        car.pose.pos.X.get(), car.pose.pos.Y.get(), correctedCarPose.pos.X.get(), correctedCarPose.pos.Y.get());
                }

                if (navigator.finished()) {
                    programState.set(ProgramState::LaneChange);
                }
                break;
            }

            case ProgramState::LaneChange:
                if (currentProgramState != prevProgramState) {
                    const LinePattern& pattern = (LinePattern::LANE_CHANGE == lineInfo.front.pattern.type ? lineInfo.front : lineInfo.rear).pattern;
                    laneChange.initialize(car, sgn(car.speed), pattern.dir, pattern.side, Sign::POSITIVE, LANE_CHANGE_SPEED, LANE_DISTANCE);
                }

                laneChange.update(car, lineInfo, mainLine, controlData);

                if (laneChange.finished()) {
                    programState.set(ProgramState::ReachSafetyCar);
                }
                break;

            default:
                LOG_ERROR("Invalid program state: {}", underlying_value(currentProgramState));
                break;
            }

            controlQueue.overwrite(controlData);
            lineDetectControlQueue.overwrite(lineDetectControlData);
        }

        taskMonitor.notify(true);
        os_sleep(millisecond_t(2));
    }
}
