#include <globals.hpp>
#include <cfg_car.hpp>
#include <micro/debug/params.hpp>

namespace micro {
namespace globals {

ProgramState programState           = ProgramState::INVALID;
bool useSafetyEnableSignal          = true;
bool frontIndicatorLedsEnabled      = false;
bool rearIndicatorLedsEnabled       = true;
bool lineDetectionEnabled           = true;
float motorCtrl_P                   = 0.6f;
float motorCtrl_I                   = 0.04f;
float motorCtrl_integral_max        = 4.0f;
float frontLineCtrl_P_slow          = 1.5f; // 1.4 m/s
float frontLineCtrl_D_slow          = 80.0f;
float frontLineCtrl_P_fast          = 0.5f; // 3 m/s
float frontLineCtrl_D_fast          = 40.0f;
float frontLineCtrl_P_bwd           = 4.4f;
float frontLineCtrl_D_bwd           = 200.0f;
float frontLineCtrl_P_fwd_mul       = 22.0f;
float frontLineCtrl_D_fwd           = 30.0f;
CarProps car                        = CarProps();
bool distSensorEnabled              = false;
bool distServoEnabled               = true;
float distServoTransferRate         = 3.0f;
m_per_sec_t speed_FAST1             = m_per_sec_t(1.0f);
m_per_sec_t speed_FAST2             = m_per_sec_t(1.5f);
m_per_sec_t speed_FAST3             = m_per_sec_t(1.5f);
m_per_sec_t speed_FAST4             = m_per_sec_t(1.5f);
m_per_sec_t speed_FAST5             = m_per_sec_t(1.5f);
m_per_sec_t speed_FAST6             = m_per_sec_t(1.5f);
m_per_sec_t speed_SLOW1             = m_per_sec_t(1.0f);
m_per_sec_t speed_SLOW2             = m_per_sec_t(1.0f);
m_per_sec_t speed_SLOW3             = m_per_sec_t(1.0f);
m_per_sec_t speed_SLOW4             = m_per_sec_t(1.0f);
m_per_sec_t speed_SLOW5             = m_per_sec_t(1.0f);
m_per_sec_t speed_SLOW6             = m_per_sec_t(1.0f);
m_per_sec_t speed_LAB_FWD           = m_per_sec_t(1.0f);
m_per_sec_t speed_LAB_BWD           = m_per_sec_t(-0.8f);
m_per_sec_t speed_LANE_CHANGE       = m_per_sec_t(0.8f);
m_per_sec_t speed_REACH_SAFETY_CAR  = m_per_sec_t(0.8f);
m_per_sec_t speed_TURN_AROUND       = m_per_sec_t(0.6f);
m_per_sec_t speed_OVERTAKE_CURVE    = m_per_sec_t(1.7f);
m_per_sec_t speed_OVERTAKE_STRAIGHT = m_per_sec_t(4.0f);
meter_t dist_OVERTAKE_SIDE          = centimeter_t(55);

bool isControlTaskOk    = false;
bool isDebugTaskOk      = false;
bool isDistSensorTaskOk = false;
bool isGyroTaskOk       = false;
bool isLineDetectTaskOk = false;

void registerGlobalParams(Params& params) {

#define REGISTER_GLOBAL(name) params.registerParam(#name, &name)

    REGISTER_GLOBAL(motorCtrl_P);
    REGISTER_GLOBAL(motorCtrl_I);
    REGISTER_GLOBAL(motorCtrl_integral_max);
    REGISTER_GLOBAL(frontLineCtrl_P_fwd_mul);
    REGISTER_GLOBAL(frontLineCtrl_D_fwd);
    REGISTER_GLOBAL(car);
    REGISTER_GLOBAL(speed_FAST1);
    REGISTER_GLOBAL(speed_FAST2);
    REGISTER_GLOBAL(speed_FAST3);
    REGISTER_GLOBAL(speed_FAST4);
    REGISTER_GLOBAL(speed_FAST5);
    REGISTER_GLOBAL(speed_FAST6);
    REGISTER_GLOBAL(speed_SLOW1);
    REGISTER_GLOBAL(speed_SLOW2);
    REGISTER_GLOBAL(speed_SLOW3);
    REGISTER_GLOBAL(speed_SLOW4);
    REGISTER_GLOBAL(speed_SLOW5);
    REGISTER_GLOBAL(speed_SLOW6);
    REGISTER_GLOBAL(speed_LAB_FWD);
    REGISTER_GLOBAL(speed_LAB_BWD);
    REGISTER_GLOBAL(speed_OVERTAKE_CURVE);
    REGISTER_GLOBAL(speed_OVERTAKE_STRAIGHT);
    REGISTER_GLOBAL(dist_OVERTAKE_SIDE);

#undef REGISTER_GLOBAL
}

}  // namespace globals
}  // namespace micro
