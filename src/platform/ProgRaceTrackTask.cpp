#include <micro/debug/ParamManager.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/log/log.hpp>
#include <micro/math/numeric.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/state.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/trajectory.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>
#include <OvertakeManeuver.hpp>
#include <RaceTrackController.hpp>
#include <TestManeuver.hpp>
#include <track.hpp>
#include <TurnAroundManeuver.hpp>

using namespace micro;

namespace {

constexpr centimeter_t MAX_VALID_SAFETY_CAR_DISTANCE = centimeter_t(120);

m_per_sec_t SAFETY_CAR_SLOW_MAX_SPEED     = m_per_sec_t(1.15f);
m_per_sec_t SAFETY_CAR_FAST_MAX_SPEED     = m_per_sec_t(1.7f);
m_per_sec_t REACH_SAFETY_CAR_SPEED        = m_per_sec_t(0.6f);
m_per_sec_t OVERTAKE_BEGIN_SPEED          = m_per_sec_t(1.0f);
m_per_sec_t OVERTAKE_STRAIGHT_START_SPEED = m_per_sec_t(1.5f);
m_per_sec_t OVERTAKE_STRAIGHT_SPEED       = m_per_sec_t(4.0f);
m_per_sec_t OVERTAKE_END_SPEED            = m_per_sec_t(2.0f);

meter_t OVERTAKE_SECTION_LENGTH           = centimeter_t(820);
meter_t OVERTAKE_PREPARE_DISTANCE         = centimeter_t(100);
meter_t OVERTAKE_BEGIN_SINE_ARC_LENGTH    = centimeter_t(140);
meter_t OVERTAKE_END_SINE_ARC_LENGTH      = centimeter_t(180);
meter_t OVERTAKE_SIDE_DISTANCE            = centimeter_t(50);

RaceTrackController trackController(buildTrackSections());

OvertakeManeuver overtake;
TestManeuver testManeuver;

m_per_sec_t safetyCarFollowSpeed(meter_t distFromSafetyCar, bool isFast) {
    return map(distFromSafetyCar, meter_t(0.3f), meter_t(0.8f), m_per_sec_t(0), isFast ? SAFETY_CAR_FAST_MAX_SPEED : SAFETY_CAR_SLOW_MAX_SPEED);
}

uint32_t getFastSection(const uint32_t n) {
    return trackController.getFastSectionIndex(n);
}

uint32_t getFastSection(const cfg::ProgramState programState) {
    return getFastSection(
        cfg::ProgramState::Race          == programState ? 1 :
        cfg::ProgramState::Race_segFast2 == programState ? 2 :
        cfg::ProgramState::Race_segFast3 == programState ? 3 :
        cfg::ProgramState::Race_segFast4 == programState ? 4 : micro::numeric_limits<uint32_t>::max()
    );
}

bool shouldHandle(const cfg::ProgramState programState) {
    return isBtw(underlying_value(programState), underlying_value(cfg::ProgramState::ReachSafetyCar), underlying_value(cfg::ProgramState::Test));
}

} // namespace

extern "C" void runProgRaceTrackTask(void) {

    SystemManager::instance().registerTask();

    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    const uint32_t overtakeSection = getFastSection(3);
    meter_t lastDistWithSafetyCar;
    uint8_t lastOvertakeLap = 0;

    cfg::ProgramState prevProgramState = cfg::ProgramState::INVALID;

    m_per_sec_t targetSpeed = m_per_sec_t(1);
    // REGISTER_PARAM(globalParams, targetSpeed);

    while (true) {
        const cfg::ProgramState programState = static_cast<cfg::ProgramState>(SystemManager::instance().programState());
        if (shouldHandle(programState)) {
            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));

            LineInfo lineInfo;
            lineInfoQueue.peek(lineInfo, millisecond_t(0));

            micro::meter_t frontDistance;
            frontDistanceQueue.peek(frontDistance, millisecond_t(0));

            IndexedSectionControlParameters sectionControlOverride;
            if (sectionControlOverrideQueue.receive(sectionControlOverride, millisecond_t(0))) {
                const auto& [index, control] = sectionControlOverride;
                trackController.overrideControlParameters(index, control);
            }

            // runs for the first time that this task handles the program state
            if (!shouldHandle(prevProgramState)) {
                const uint32_t currentSection = getFastSection(programState);
                if (currentSection != std::numeric_limits<uint32_t>::max()) { // race
                    trackController.setSection(car, 3, currentSection);
                    SystemManager::instance().setProgramState(underlying_value(cfg::ProgramState::Race));
                } else { // reach safety car
                    trackController.setSection(car, 1, 0);
                }

                lastDistWithSafetyCar = car.distance;
            }

            micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);

            ControlData controlData = trackController.update(car, lineInfo, mainLine);

            LineDetectControl lineDetectControlData;
            lineDetectControlData.domain = linePatternDomain_t::Race;
            lineDetectControlData.isReducedScanRangeEnabled = false;

            if (frontDistance < centimeter_t(100)) {
                lastDistWithSafetyCar = car.distance;
            }

            switch (programState) {
            case cfg::ProgramState::ReachSafetyCar:
                controlData.speed = REACH_SAFETY_CAR_SPEED;
                controlData.rampTime = millisecond_t(500);
                if (frontDistance != centimeter_t(0) && abs(safetyCarFollowSpeed(frontDistance, trackController.isCurrentSectionFast())) < abs(controlData.speed)) {
                    SystemManager::instance().setProgramState(underlying_value(cfg::ProgramState::FollowSafetyCar));
                    LOG_DEBUG("Reached safety car, starts following");
                }
                break;

            case cfg::ProgramState::FollowSafetyCar:
                controlData.speed = safetyCarFollowSpeed(frontDistance, trackController.isCurrentSectionFast());
                controlData.rampTime = millisecond_t(0);

                if (overtakeSection == trackController.sectionIndex() && (1 == trackController.lap() || 3 == trackController.lap()) && trackController.lap() != lastOvertakeLap) {
                    SystemManager::instance().setProgramState(underlying_value(cfg::ProgramState::OvertakeSafetyCar));
                    LOG_DEBUG("Starts overtake");
                } else if (car.distance - lastDistWithSafetyCar > MAX_VALID_SAFETY_CAR_DISTANCE && trackController.isCurrentSectionFast()) {
                    SystemManager::instance().setProgramState(underlying_value(cfg::ProgramState::Race));
                    LOG_DEBUG("Safety car left the track, starts race");
                }
                break;

            case cfg::ProgramState::OvertakeSafetyCar:
                if (programState != prevProgramState) {
                    lastOvertakeLap = trackController.lap();
                    overtake.initialize(car, Sign::POSITIVE,
                        OVERTAKE_BEGIN_SPEED, OVERTAKE_STRAIGHT_START_SPEED, OVERTAKE_STRAIGHT_SPEED, OVERTAKE_END_SPEED,
                        OVERTAKE_SECTION_LENGTH, OVERTAKE_PREPARE_DISTANCE, OVERTAKE_BEGIN_SINE_ARC_LENGTH, OVERTAKE_END_SINE_ARC_LENGTH,
                        OVERTAKE_SIDE_DISTANCE);
                }

                controlData.speed = safetyCarFollowSpeed(frontDistance, trackController.isCurrentSectionFast());
                overtake.update(car, lineInfo, mainLine, controlData);

                if (overtake.finished()) {
                    SystemManager::instance().setProgramState(underlying_value(cfg::ProgramState::Race));
                    LOG_DEBUG("Overtake finished, starts race");
                }
                break;

            case cfg::ProgramState::Race:
                lineDetectControlData.isReducedScanRangeEnabled = trackController.lap() >= 4 &&
                                                                  (car.speed > m_per_sec_t(0) ?
                                                                      lineInfo.front.lines.size() == 1 :
                                                                      lineInfo.rear.lines.size() == 1);

                if (trackController.lap() > cfg::NUM_RACE_LAPS) {
                    SystemManager::instance().setProgramState(underlying_value(cfg::ProgramState::Finish));
                    LOG_DEBUG("Race finished");

                } else if (trackController.lap() <= 3 && frontDistance < (trackController.isCurrentSectionFast() ? MAX_VALID_SAFETY_CAR_DISTANCE - centimeter_t(2) : centimeter_t(80))) {
                    SystemManager::instance().setProgramState(underlying_value(cfg::ProgramState::FollowSafetyCar));
                    LOG_DEBUG("Reached safety car, starts following");

                }
                break;

            case cfg::ProgramState::Finish:
                if (!trackController.isCurrentSectionFast() || car.distance - trackController.getSectionStartDistance() > meter_t(2)) {
                    controlData.speed = m_per_sec_t(0);
                    controlData.rampTime = millisecond_t(1500);
                }
                break;

            case cfg::ProgramState::Test:
                controlData.speed = targetSpeed;
                controlData.rampTime = millisecond_t(500);
                break;

            default:
                LOG_ERROR("Invalid program state counter: [{}]", underlying_value(programState));
                break;
            }

            controlQueue.overwrite(controlData);
            lineDetectControlQueue.overwrite(lineDetectControlData);
            lapControlQueue.overwrite(trackController.getControlParameters());
        }

        prevProgramState = programState;
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(1));
    }
}
