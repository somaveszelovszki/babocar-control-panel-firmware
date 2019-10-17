#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/debug/params.hpp>

namespace micro {

enum class ProgramState {
    Labyrinth,
    RaceTrack
};

namespace globals {

extern Params debugParams;

extern ProgramState programState;
extern bool useSafetyEnableSignal;
extern bool indicatorLedsEnabled;
extern bool startSignalEnabled;
extern bool lineFollowEnabled;
extern microsecond_t motorController_Ti;
extern float motorController_Kc;
extern float frontLineController_P;
extern float frontLineController_D;
extern float rearLineController_P;
extern float rearLineController_D;
extern m_per_sec_t targetSpeed;
extern CarProps car;

void initializeGlobalParams();

} // namespace globals
} // namespace micro
