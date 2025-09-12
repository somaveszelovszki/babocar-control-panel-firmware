#pragma once

#include <cstdint>

class ProgramState {
  public:
    enum Value : uint8_t {
        // Start states
        INVALID               = 0,
        WaitStartSignalRoute  = 1,
        WaitStartSignalRandom = 2,

        // Labyrinth states
        LabyrinthRoute  = 3,
        LabyrinthRandom = 4,
        LaneChange      = 5,

        // RaceTrack states
        ReachSafetyCar    = 6,
        FollowSafetyCar   = 7,
        OvertakeSafetyCar = 8,
        Race              = 9,
        Race_segFast2     = 10,
        Race_segFast3     = 11,
        Race_segFast4     = 12,
        Finish            = 13,
        Test              = 14
    };

    Value get() const { return value_; }
    void set(const Value newValue);

  private:
    Value value_{Value::INVALID};
};
