#include <globals.hpp>
#include <cfg_car.hpp>
#include <cfg_track.hpp>

using namespace micro;

namespace globals {

ProgramState programState                = ProgramState::INVALID;
bool useSafetyEnableSignal               = false;
bool indicatorLedsEnabled                = true;
uint8_t reducedLineDetectScanRangeRadius = 10;
CarProps car                             = CarProps();
bool distServoEnabled                    = false;
float distServoTransferRate              = 1.0f;

TrackSpeeds trackSpeeds[cfg::NUM_RACE_LAPS + 1] = {

#if TRACK == RACE_TRACK
//  ||  fast   ||             slow1             ||                   slow2                  ||                   slow3                  ||             slow4             ||
//  ||  (all)  || prepare  round_begin round_end|| prepare     begin   round_begin round_end|| prepare  round_begin round_end     end   || prepare  round_begin round_end||
    { { 3.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 1
    { { 4.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 2
    { { 3.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.70f }, { 1.80f }, { 1.80f }, { 1.80f } }, // Lap 3
    { { 5.00f }, { 2.05f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.20f }, { 2.20f }, { 2.10f }, { 2.10f }, { 2.10f }, { 1.70f }, { 1.95f }, { 1.95f }, { 1.95f } }, // Lap 4
    { { 5.80f }, { 2.05f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.25f }, { 2.25f }, { 2.25f }, { 1.80f }, { 1.95f }, { 1.95f }, { 1.95f } }, // Lap 5
    { { 6.50f }, { 2.05f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.25f }, { 2.25f }, { 2.25f }, { 1.80f }, { 1.95f }, { 1.95f }, { 1.95f } }, // Lap 6
    { { 7.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f } }  // Finish
#elif TRACK == TEST_TRACK
//  ||  fast   ||        slow1       ||                        slow2                        ||        slow3       ||                        slow4                        ||
//  ||  (all)  || prepare    chicane || prepare   begin_chi round_begin round_end   end_chi || prepare    chicane || prepare   begin_chi round_begin round_end   end_chi ||
    { { 3.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f } }, // Lap 1
    { { 4.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f } }, // Lap 2
    { { 3.50f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f }, { 1.80f }, { 1.80f }, { 1.80f }, { 1.80f }, { 2.00f }, { 2.00f }, { 2.00f } }, // Lap 3
    { { 5.00f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.20f }, { 2.20f }, { 2.20f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.20f }, { 2.20f }, { 2.20f } }, // Lap 4
    { { 5.80f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.30f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.30f } }, // Lap 5
    { { 6.50f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.30f }, { 2.05f }, { 2.05f }, { 1.80f }, { 1.80f }, { 2.30f }, { 2.30f }, { 2.30f } }, // Lap 6
    { { 7.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f }, { 0.00f } }  // Finish
#endif

};

BrakeOffsets brakeOffsets[cfg::NUM_RACE_LAPS] = {
    //     slow1            slow2            slow3            slow4
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 1
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 2
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 3
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 4
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }, // Lap 5
    { centimeter_t(0), centimeter_t(0), centimeter_t(0), centimeter_t(0) }  // Lap 6
};

bool isControlTaskOk    = false;
bool isDebugTaskOk      = false;
bool isDistSensorTaskOk = false;
bool isGyroTaskOk       = false;
bool isLineDetectTaskOk = false;
bool isVehicleCanTaskOk = false;

}  // namespace globals
