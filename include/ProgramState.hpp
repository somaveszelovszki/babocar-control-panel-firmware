#pragma once

#include <cstdint>

class ProgramState {
public: 
    enum Value : uint8_t {
        // Start states
        INVALID              = 0,
        WaitStartSignal      = 1,

        // Labyrinth states
        NavigateLabyrinth    = 2,
        LaneChange           = 3,

        // RaceTrack states
        ReachSafetyCar       = 4,
        FollowSafetyCar      = 5,
        OvertakeSafetyCar    = 6,
        Race                 = 7,
        Race_segFast2        = 8,
        Race_segFast3        = 9,
        Race_segFast4        = 10,
        Finish               = 11,
        Test                 = 12
    };

    Value get() const { return value_; }
    void set(const Value newValue);

private:
    Value value_{Value::INVALID};
};
