#include <utility>
#include <variant>

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

constexpr auto LABYRINTH_SPEED          = m_per_sec_t(1.1f);
constexpr auto LABYRINTH_DEAD_END_SPEED = m_per_sec_t(0.85f);
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

class RandomGeneratorWrapper : public micro::irandom_generator {
public:
    template <typename T>
    void setGenerator(const T& generator) {
        generator_ = generator;
    }

    float operator()() override {
        return std::visit([](auto& g){ return g(); }, generator_);
    }

private:
    std::variant<micro::random_generator, micro::fixed_number_generator> generator_;
};

LabyrinthGraph graph;
RandomGeneratorWrapper randomWrapper;
LabyrinthNavigator navigator(graph, randomWrapper);
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
    const auto forbiddenSegments = {
#if TRACK == TEST_TRACK
        "WY",
        "AC"
#elif TRACK == RACE_TRACK
        "OU",
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

        const auto current = graph.findSegment(Segment::makeId(junctions[0], junctions[1]));
        const auto next = graph.findSegment(Segment::makeId(junctions[1], junctions[2]));

        navigator.setObstaclePosition({ current, next, micro::getTime() });
    }
}

bool shouldHandle(const ProgramState::Value state) {
    return isBtw(underlying_value(state), underlying_value(ProgramState::NavigateLabyrinth), underlying_value(ProgramState::LaneChange));
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
            case ProgramState::NavigateLabyrinth:
            case ProgramState::NavigateLabyrinthLeft:
            case ProgramState::NavigateLabyrinthRight:
            {
                if (currentProgramState != prevProgramState) {
                    enforceGraphValidity();
                    carOrientationUpdateQueue.overwrite(radian_t(0));
                    endTime = getTime() + minute_t(5);

                    switch (currentProgramState) {
                    case ProgramState::NavigateLabyrinth:
                        randomWrapper.setGenerator(random_generator{static_cast<uint16_t>(micro::getExactTime().get())});
                        break;

                    case ProgramState::NavigateLabyrinthLeft:
                        randomWrapper.setGenerator(micro::fixed_number_generator{0.999999f});
                        break;

                    case ProgramState::NavigateLabyrinthRight:
                        randomWrapper.setGenerator(micro::fixed_number_generator{0.0});
                        break;

                    default:
                        break;
                    }
                }

                meter_t frontDistance;
                frontDistanceQueue.peek(frontDistance);

                meter_t rearDistance;
                frontDistanceQueue.peek(rearDistance);

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
