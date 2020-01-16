#pragma once

#include <micro/utils/types.hpp>

namespace micro {

enum class ProgramState : uint8_t {
    // Start states
    INVALID           = 0xff,
    WaitStartSignal   = 0,

    // Labyrinth states
    NavigateLabyrinth = 1,
    LaneChange        = 2,

    // RaceTrack states
    ReachSafetyCar    = 3,
    FollowSafetyCar   = 4,
    OvertakeSafetyCar = 5,
    Race              = 6,
};

enum class ProgramTask {
    INVALID,
    Startup,
    Labyrinth,
    RaceTrack
};

inline ProgramTask getActiveTask(const ProgramState programState) {
    ProgramTask task = ProgramTask::INVALID;
    switch (programState) {
    case ProgramState::INVALID:           task = ProgramTask::INVALID;   break;
    case ProgramState::WaitStartSignal:   task = ProgramTask::Startup;   break;
    case ProgramState::NavigateLabyrinth:
    case ProgramState::LaneChange:        task = ProgramTask::Labyrinth; break;
    case ProgramState::ReachSafetyCar:
    case ProgramState::FollowSafetyCar:
    case ProgramState::OvertakeSafetyCar:
    case ProgramState::Race:              task = ProgramTask::RaceTrack; break;
    }
    return task;
}

} // namespace micro
