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

constexpr auto LABYRINTH_SPEED          = m_per_sec_t(1.0f);
constexpr auto LABYRINTH_FAST_SPEED     = m_per_sec_t(1.2f);
constexpr auto LABYRINTH_DEAD_END_SPEED = m_per_sec_t(0.85f);
constexpr auto LANE_CHANGE_SPEED        = m_per_sec_t(0.65f);
constexpr auto LANE_DISTANCE            = centimeter_t(60);

#if TRACK == RACE_TRACK
#define START_SEGMENT       "UX"
#define PREV_SEGMENT        "U_"
#define LANE_CHANGE_SEGMENT "VW"
#define FLOOD_SEGMENT       "Q_"
#elif TRACK == TEST_TRACK
#define START_SEGMENT       "WY"
#define PREV_SEGMENT        "Y_"
#define LANE_CHANGE_SEGMENT "NQ"
#define FLOOD_SEGMENT       "X_"
#endif

LabyrinthGraph graph;
millisecond_t endTime;
millisecond_t lastFloodCommandTime;

struct JunctionPatternInfo {
    Sign dir            = Sign::NEUTRAL;
    Direction side      = Direction::CENTER;
    uint8_t numSegments = 0;
};

LaneChangeManeuver laneChange;

void handleRadioCommand(LabyrinthNavigator& navigator) {
    etl::string<RADIO_COMMAND_MAX_LENGTH> command;
    if (!radioCommandQueue.receive(command, millisecond_t(0))) {
        return;
    }

    if (command.length() != 6) {
        LOG_WARN("Invalid labyrinth command: {}", command);
        return;
    }

    if (command == "FLOOD!") {
        lastFloodCommandTime = micro::getTime();
    } else {
        char prev = command[0];
        char next = command[1];

#if TRACK == TEST_TRACK
        // In the test track, some segments are partitioned into multiple 'fake' segments.
        // Since the application logic cannot handle segments that are not separated by real junctions,
        // these partitions are joined together, forming one segment.
        if ((prev == 'C' && next == 'B') || (prev == 'B' && next == 'D') || (prev == 'D' && next == 'F')) {
            prev = 'C';
            next = 'F';
        } else if ((prev == 'F' && next == 'D') || (prev == 'D' && next == 'B') || (prev == 'B' && next == 'C')) {
            prev = 'F';
            next = 'C';
        }
#endif // TRACK == TEST_TRACK

        navigator.setForbiddenSegment(graph.findSegment(Segment::makeId(prev, next)));
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

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {
    buildLabyrinthGraph(graph);

    RandomGeneratorWrapper random{};
    LabyrinthNavigator navigator(graph, random);

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
                        random.setGenerator(random_generator{static_cast<uint16_t>(micro::getExactTime().get())});
                        break;

                    case ProgramState::NavigateLabyrinthLeft:
                        random.setGenerator(micro::fixed_number_generator{0.999999f});
                        break;

                    case ProgramState::NavigateLabyrinthRight:
                        random.setGenerator(micro::fixed_number_generator{0.0});
                        break;

                    default:
                        break;
                    }

                    const auto* prevSeg       = graph.findSegment(PREV_SEGMENT);
                    const auto* startSeg      = graph.findSegment(START_SEGMENT);
                    const auto* laneChangeSeg = graph.findSegment(LANE_CHANGE_SEGMENT);
                    const auto* floodSeg      = graph.findSegment(FLOOD_SEGMENT);
                    const auto* prevConn      = graph.findConnection(*prevSeg, *startSeg);

                    navigator.initialize(
                        graph.getVisitableSegments(),
                        startSeg,
                        prevConn,
                        laneChangeSeg,
                        floodSeg,
                        LABYRINTH_SPEED,
                        LABYRINTH_FAST_SPEED,
                        LABYRINTH_DEAD_END_SPEED);
                }

                handleRadioCommand(navigator);
                navigator.setFlood(micro::getTime() - lastFloodCommandTime < millisecond_t(450));
                navigator.update(car, lineInfo, mainLine, controlData);

                const Pose correctedCarPose = navigator.correctedCarPose();
                if (correctedCarPose.angle != car.pose.angle) {
                    carOrientationUpdateQueue.overwrite(correctedCarPose.angle);
                    LOG_DEBUG("Car orientation updated: {} -> {} [deg]", static_cast<degree_t>(car.pose.angle).get(), static_cast<degree_t>(correctedCarPose.angle).get());
                }
                if (correctedCarPose.pos != car.pose.pos) {
                    carPosUpdateQueue.overwrite(correctedCarPose.pos);
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
