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
#include <micro/utils/variant_utils.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>
#include <LaneChangeManeuver.hpp>
#include <LabyrinthNavigator.hpp>
#include <track.hpp>

using namespace micro;

namespace {

m_per_sec_t LABYRINTH_SPEED          = m_per_sec_t(1.0f);
m_per_sec_t LABYRINTH_FAST_SPEED     = m_per_sec_t(1.2f);
m_per_sec_t LABYRINTH_DEAD_END_SPEED = m_per_sec_t(0.85f);
m_per_sec_t LANE_CHANGE_SPEED        = m_per_sec_t(0.65f);

constexpr meter_t LANE_DISTANCE = centimeter_t(60);

#if TRACK == RACE_TRACK
#define START_SEGMENT       'U'
#define PREV_SEGMENT        'N'
#define LANE_CHANGE_SEGMENT 'B'
#define LAST_VALID_SEGMENT  'U'
#elif TRACK == TEST_TRACK
#define START_SEGMENT       'W'
#define PREV_SEGMENT        'M'
#define LANE_CHANGE_SEGMENT 'N'
#define LAST_VALID_SEGMENT  'W'
#endif

LabyrinthGraph graph;
const Segment *startSeg = nullptr;
const Connection *prevConn = nullptr;
const Segment *laneChangeSeg = nullptr;
micro::vector<const Segment*, cfg::NUM_LABYRINTH_GATE_SEGMENTS> foundSegments;
millisecond_t endTime;

struct JunctionPatternInfo {
    Sign dir            = Sign::NEUTRAL;
    Direction side      = Direction::CENTER;
    uint8_t numSegments = 0;
};

LaneChangeManeuver laneChange;

void updateObstacleInfo(LabyrinthNavigator& navigator) {
    char segId = '\0';
    radioRecvQueue.peek(segId, millisecond_t(0));
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

class fixed_number_generator : public micro::irandom_generator {
public:
    explicit fixed_number_generator(const float value) : value_{value} {}
    float operator()() const override { return value_; }

private:
    float value_{}
};

class random_generator_wrapper : public micro::irandom_generator {
public:
    template <typename T>
    void setGenerator(const T& generator) {
        generator_ = generator;
    }

    float operator()() override {
        return std::visit(micro::variant_visitor{
            [](const std::monostate&) { return 0.0f; }
            [](const auto& generator) { return generator(); }
        }, generator_);
    }

private:
    std::variant<std::monostate, micro::random_generator, fixed_number_generator> generator_;
};

} // namespace

extern "C" void runProgLabyrinthTask(void const *argument) {
    buildLabyrinthGraph(graph);
    startSeg = graph.findSegment(START_SEGMENT);
    prevConn = graph.findConnection(*graph.findSegment(PREV_SEGMENT), *startSeg);
    laneChangeSeg = graph.findSegment(LANE_CHANGE_SEGMENT);

    random_generator_wrapper random{};
    LabyrinthNavigator navigator(graph, random, startSeg, prevConn, laneChangeSeg, LABYRINTH_SPEED, LABYRINTH_FAST_SPEED, LABYRINTH_DEAD_END_SPEED);

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
                        random.setGenerator(fixed_number_generator{0.999999f});
                        break;

                    case ProgramState::NavigateLabyrinthRight:
                        random.setGenerator(fixed_number_generator{0.0});
                        break;
                    }

                    navigator.initialize(graph.getVisitableSegments());
                }

                updateObstacleInfo(navigator);
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
