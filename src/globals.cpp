#include <globals.hpp>
#include <cfg_car.hpp>
#include <micro/debug/params.hpp>

using namespace micro;

namespace globals {

ProgramState programState                       = ProgramState::INVALID;
bool useSafetyEnableSignal                      = false;
bool indicatorLedsEnabled                       = true;
uint8_t reducedLineDetectScanRangeRadius        = 10;
float motorCtrl_P                               = 0.85f;
float motorCtrl_I                               = 0.04f;
float motorCtrl_integral_max                    = 4.0f;
float frontLineCtrl_P_slow                      = 1.5f; // 1.4 m/s
float frontLineCtrl_D_slow                      = 80.0f;
float frontLineCtrl_P_fast                      = 0.5f; // 3 m/s
float frontLineCtrl_D_fast                      = 40.0f;
float frontLineCtrl_P_bwd                       = 4.4f;
float frontLineCtrl_D_bwd                       = 200.0f;
float frontLineCtrl_P_fwd_mul                   = 15.5f;
float frontLineCtrl_D_fwd                       = 40.0f;
CarProps car                                    = CarProps();
bool distSensorEnabled                          = false;
bool distServoEnabled                           = false;
float distServoTransferRate                     = 1.0f;
micro::radian_t frontSteeringServoOffset        = degree_t(90); // TODO
micro::radian_t rearSteeringServoOffset         = degree_t(90); // TODO

TrackSpeeds trackSpeeds[NUM_LAPS + 1] = {
    //     FAST               SLOW_1           SLOW_2_BEGIN          SLOW_2              SLOW_3            SLOW_3_END            SLOW_4
    { m_per_sec_t(3.5f), m_per_sec_t(1.8f),  m_per_sec_t(1.8f), m_per_sec_t(2.0f),  m_per_sec_t(2.0f),  m_per_sec_t(1.7f),  m_per_sec_t(1.8f)  }, // Lap 1
    { m_per_sec_t(4.5f), m_per_sec_t(1.8f),  m_per_sec_t(1.8f), m_per_sec_t(2.0f),  m_per_sec_t(2.0f),  m_per_sec_t(1.7f),  m_per_sec_t(1.8f)  }, // Lap 2
    { m_per_sec_t(3.5f), m_per_sec_t(1.8f),  m_per_sec_t(1.8f), m_per_sec_t(2.0f),  m_per_sec_t(2.0f),  m_per_sec_t(1.7f),  m_per_sec_t(1.8f)  }, // Lap 3
    { m_per_sec_t(5.0f), m_per_sec_t(2.05f), m_per_sec_t(1.8f), m_per_sec_t(2.2f),  m_per_sec_t(2.1f),  m_per_sec_t(1.7f),  m_per_sec_t(1.95f) }, // Lap 4
    { m_per_sec_t(5.8f), m_per_sec_t(2.05f), m_per_sec_t(1.8f), m_per_sec_t(2.3f),  m_per_sec_t(2.25f), m_per_sec_t(1.8f),  m_per_sec_t(1.95f) }, // Lap 4
    { m_per_sec_t(6.5f), m_per_sec_t(2.05f), m_per_sec_t(1.8f), m_per_sec_t(2.3f),  m_per_sec_t(2.25f), m_per_sec_t(1.8f),  m_per_sec_t(1.95f) }, // Lap 4
    { m_per_sec_t(7.0f), m_per_sec_t(0.0f),  m_per_sec_t(0.0f), m_per_sec_t(0.0f),  m_per_sec_t(0.0f),  m_per_sec_t(0.0f),  m_per_sec_t(0.0f)  }  // Finish
};

m_per_sec_t speed_LAB_FWD           = m_per_sec_t(1.1f);
m_per_sec_t speed_LAB_BWD           = m_per_sec_t(-0.8f);
m_per_sec_t speed_LANE_CHANGE       = m_per_sec_t(0.8f);
m_per_sec_t speed_REACH_SAFETY_CAR  = m_per_sec_t(0.8f);
m_per_sec_t speed_TURN_AROUND       = m_per_sec_t(0.6f);
m_per_sec_t speed_OVERTAKE_BEGIN    = m_per_sec_t(1.7f);
m_per_sec_t speed_OVERTAKE_STRAIGHT = m_per_sec_t(5.0f);
m_per_sec_t speed_OVERTAKE_END      = m_per_sec_t(1.2f);
meter_t dist_OVERTAKE_SIDE          = centimeter_t(60);
meter_t dist_BRAKE_OFFSET           = centimeter_t(10);

bool isControlTaskOk    = false;
bool isDebugTaskOk      = false;
bool isDistSensorTaskOk = false;
bool isGyroTaskOk       = false;
bool isLineDetectTaskOk = false;

void registerGlobalParams(micro::Params& params) {

#define REGISTER_GLOBAL(name) params.registerParam(#name, &name)

    REGISTER_GLOBAL(motorCtrl_P);
    REGISTER_GLOBAL(motorCtrl_I);
    REGISTER_GLOBAL(motorCtrl_integral_max);
    REGISTER_GLOBAL(frontLineCtrl_P_fwd_mul);
    REGISTER_GLOBAL(frontLineCtrl_D_fwd);
    REGISTER_GLOBAL(car);

    //params.registerParam("trackSpeeds", &trackSpeeds[3]);

#undef REGISTER_GLOBAL
}

}  // namespace globals
