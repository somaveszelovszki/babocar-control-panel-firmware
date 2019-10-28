#pragma once

#include <micro/utils/units.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/debug/params.hpp>
#include <ProgramState.hpp>

namespace micro {

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
extern CarProps car;
extern m_per_sec_t targetSpeedOverride;
extern bool targetSpeedOverrideActive;

void initializeGlobalParams();

} // namespace globals
} // namespace micro
