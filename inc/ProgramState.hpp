#pragma once

#include <micro/utils/types.hpp>

namespace micro {

enum class ProgramState : uint8_t {
    // Start states
    INVALID           = 0,
    WaitStartSignal   = 1,

    // Labyrinth states
    NavigateLabyrinth = 2,
    LaneChange        = 3,

    // RaceTrack states
    ReachSafetyCar    = 4,
    FollowSafetyCar   = 5,
    OvertakeSafetyCar = 6,
    Race              = 7,
    Race_segFast2     = 8,
    Race_segFast3     = 9,
    Race_segFast4     = 10,
    Finish            = 11
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
    case ProgramState::Race:
    case ProgramState::Finish:            task = ProgramTask::RaceTrack; break;
    }
    return task;
}

} // namespace micro
