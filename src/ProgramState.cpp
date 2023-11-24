#include <ProgramState.hpp>

#include <utility>

#include <micro/log/log.hpp>

namespace {

const char* to_string(const ProgramState::Value& value) {
    switch (value) {
        case ProgramState::Value::INVALID:           return "INVALID";
        case ProgramState::Value::WaitStartSignal:   return "WaitStartSignal";
        case ProgramState::Value::NavigateLabyrinth: return "NavigateLabyrinth";
        case ProgramState::Value::LaneChange:        return "LaneChange";
        case ProgramState::Value::ReachSafetyCar:    return "ReachSafetyCar";
        case ProgramState::Value::FollowSafetyCar:   return "FollowSafetyCar";
        case ProgramState::Value::OvertakeSafetyCar: return "OvertakeSafetyCar";
        case ProgramState::Value::Race:              return "Race";
        case ProgramState::Value::Race_segFast2:     return "Race_segFast2";
        case ProgramState::Value::Race_segFast3:     return "Race_segFast3";
        case ProgramState::Value::Race_segFast4:     return "Race_segFast4";
        case ProgramState::Value::Finish:            return "Finish";
        case ProgramState::Value::Test:              return "Test";
        default:                                     return "?";
    }
}

} // namespace

void ProgramState::set(const Value newValue) {
    const auto prevValue = std::exchange(value_, newValue);
    LOG_INFO("Program state changed from {} to {}", to_string(prevValue), to_string(newValue));
}
