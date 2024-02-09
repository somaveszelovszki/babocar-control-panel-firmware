#include <micro/debug/ParamManager.hpp>
#include <micro/debug/TaskMonitor.hpp>
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

RaceTrackController trackController(lapTrackSectionProvider);

OvertakeManeuver overtake;
TestManeuver testManeuver;

CarProps car;
LineInfo lineInfo;
micro::meter_t frontDistance;
IndexedSectionControlParameters sectionControlOverride;

m_per_sec_t safetyCarFollowSpeed(meter_t distFromSafetyCar, bool isFast) {
    return micro::lerp(distFromSafetyCar, meter_t(0.3f), meter_t(0.8f), m_per_sec_t(0), isFast ? SAFETY_CAR_FAST_MAX_SPEED : SAFETY_CAR_SLOW_MAX_SPEED);
}

uint32_t getFastSection(const uint32_t n) {
    return trackController.getFastSectionIndex(n);
}

uint32_t getFastSection(const ProgramState::Value state) {
    return getFastSection(
        ProgramState::Race          == state ? 1 :
        ProgramState::Race_segFast2 == state ? 2 :
        ProgramState::Race_segFast3 == state ? 3 :
        ProgramState::Race_segFast4 == state ? 4 : micro::numeric_limits<uint32_t>::max()
    );
}

bool shouldHandle(const ProgramState::Value state) {
    return isBtw(underlying_value(state), underlying_value(ProgramState::ReachSafetyCar), underlying_value(ProgramState::Test));
}

class LineDetectScanRangeManager {
public:
    bool checkEnabled(const size_t lap, const CarProps& car, const LineInfo& lineInfo) {
        const auto enable = lap >= 4 && lineInfo.front.lines.size() == 1 && lineInfo.rear.lines.size() == 1;

        if (!enable) {
            lastDisabledDistance_ = car.distance;
            return false;
        } else if (car.distance - lastDisabledDistance_ > meter_t(1)) {
            return true;
        }

        return false;
    }

private:
    meter_t lastDisabledDistance_;
};

} // namespace

extern "C" void runProgRaceTrackTask(void) {
    m_per_sec_t targetSpeed = m_per_sec_t(1);
    REGISTER_PARAM(globalParams, targetSpeed);

    taskMonitor.registerInitializedTask();

    MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
    const uint32_t overtakeSection = getFastSection(3);
    meter_t lastDistWithSafetyCar;
    uint8_t lastOvertakeLap = 0;
    LineDetectScanRangeManager scanRangeManager;

    auto currentProgramState = ProgramState::INVALID;
    auto currentLap = trackController.lap();

    Timer sectionControlOverrideCheckTimer(millisecond_t(50));

    while (true) {
        const auto prevProgramState = std::exchange(currentProgramState, programState.get());
        if (shouldHandle(currentProgramState)) {
			carPropsQueue.peek(car, millisecond_t(0));
			lineInfoQueue.peek(lineInfo, millisecond_t(0));
			frontDistanceQueue.peek(frontDistance, millisecond_t(0));

			if (sectionControlOverrideCheckTimer.checkTimeout() &&
				sectionControlOverrideQueue.receive(sectionControlOverride, millisecond_t(0))) {
				const auto& [index, control] = sectionControlOverride;
				lapTrackSectionProvider.overrideControlParameters(index, control);
				trackController.overrideControlParameters(index, control);
				lapControlQueue.overwrite(trackController.getControlParameters());
			}

			// runs for the first time that this task handles the program state
			if (!shouldHandle(prevProgramState)) {
				const uint32_t currentSection = getFastSection(currentProgramState);
				if (currentSection != std::numeric_limits<uint32_t>::max()) { // race
					trackController.initialize(car, 3, currentSection);
					programState.set(ProgramState::Race);
				} else { // reach safety car
					trackController.initialize(car, 1, 0);
				}

				lastDistWithSafetyCar = car.distance;
			}

			micro::updateMainLine(lineInfo.front.lines, lineInfo.rear.lines, mainLine);

			ControlData controlData = trackController.update(car, lineInfo, mainLine);

			if (std::exchange(currentLap, trackController.lap()) != trackController.lap()) {
				lapControlQueue.overwrite(trackController.getControlParameters());
			}

			LineDetectControl lineDetectControlData;
			lineDetectControlData.domain = linePatternDomain_t::Race;
			lineDetectControlData.isReducedScanRangeEnabled = false;

			if (frontDistance < centimeter_t(100)) {
				lastDistWithSafetyCar = car.distance;
			}

			switch (currentProgramState) {
			case ProgramState::ReachSafetyCar:
				controlData.speed = REACH_SAFETY_CAR_SPEED;
				controlData.rampTime = millisecond_t(500);
				if (frontDistance != centimeter_t(0) && abs(safetyCarFollowSpeed(frontDistance, trackController.isCurrentSectionFast())) < abs(controlData.speed)) {
					programState.set(ProgramState::FollowSafetyCar);
					LOG_DEBUG("Reached safety car, starts following");
				}
				break;

			case ProgramState::FollowSafetyCar:
				controlData.speed = safetyCarFollowSpeed(frontDistance, trackController.isCurrentSectionFast());
				controlData.rampTime = millisecond_t(0);

				if (overtakeSection == trackController.sectionIndex() && (1 == trackController.lap() || 3 == trackController.lap()) && trackController.lap() != lastOvertakeLap) {
					programState.set(ProgramState::OvertakeSafetyCar);
					LOG_DEBUG("Starts overtake");
				} else if (car.distance - lastDistWithSafetyCar > MAX_VALID_SAFETY_CAR_DISTANCE && trackController.isCurrentSectionFast()) {
					programState.set(ProgramState::Race);
					LOG_DEBUG("Safety car left the track, starts race");
				}
				break;

			case ProgramState::OvertakeSafetyCar:
				if (currentProgramState != prevProgramState) {
					lastOvertakeLap = trackController.lap();
					overtake.initialize(car, Sign::POSITIVE,
						OVERTAKE_BEGIN_SPEED, OVERTAKE_STRAIGHT_START_SPEED, OVERTAKE_STRAIGHT_SPEED, OVERTAKE_END_SPEED,
						OVERTAKE_SECTION_LENGTH, OVERTAKE_PREPARE_DISTANCE, OVERTAKE_BEGIN_SINE_ARC_LENGTH, OVERTAKE_END_SINE_ARC_LENGTH,
						OVERTAKE_SIDE_DISTANCE);
				}

				controlData.speed = safetyCarFollowSpeed(frontDistance, trackController.isCurrentSectionFast());
				overtake.update(car, lineInfo, mainLine, controlData);

				if (overtake.finished()) {
					programState.set(ProgramState::Race);
					LOG_DEBUG("Overtake finished, starts race");
				}
				break;

			case ProgramState::Race:
			    lineDetectControlData.isReducedScanRangeEnabled = scanRangeManager.checkEnabled(trackController.lap(), car, lineInfo);

				if (trackController.lap() <= 3
				    && frontDistance < (trackController.isCurrentSectionFast() ? MAX_VALID_SAFETY_CAR_DISTANCE - centimeter_t(2) : centimeter_t(80))) {
					programState.set(ProgramState::FollowSafetyCar);
					LOG_DEBUG("Reached safety car, starts following");
				} else if (trackController.lap() > cfg::NUM_RACE_LAPS) {
                    programState.set(ProgramState::Finish);
                    LOG_DEBUG("Race finished");
                }
				break;

			case ProgramState::Finish:
				if (!trackController.isCurrentSectionFast() || car.distance - trackController.getSectionStartDistance() > meter_t(2)) {
					controlData.speed = m_per_sec_t(0);
					controlData.rampTime = millisecond_t(1500);
				}
				break;

			case ProgramState::Test:
				controlData.speed = targetSpeed;
				controlData.rampTime = millisecond_t(500);
				controlData.lineControl.target.pos = millimeter_t(0);
				controlData.lineControl.target.angle = radian_t(0);
				break;

			default:
				LOG_ERROR("Invalid program state: {}", underlying_value(currentProgramState));
				break;
			}

			controlQueue.overwrite(controlData);
			lineDetectControlQueue.overwrite(lineDetectControlData);
		}

        taskMonitor.notify(true);
        os_sleep(millisecond_t(1));
    }
}
