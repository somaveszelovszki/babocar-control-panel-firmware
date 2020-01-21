#include <globals.hpp>
#include <cfg_car.hpp>
#include <micro/debug/params.hpp>

namespace micro {
namespace globals {

ProgramState programState           = ProgramState::INVALID;
bool useSafetyEnableSignal          = true;
bool indicatorLedsEnabled           = true;
bool startSignalEnabled             = false;
bool linePatternCalcEnabled         = false;
bool gyroCalibrationEnabled         = true;
float motorCtrl_P                   = 0.5f;
float motorCtrl_I                   = 0.04f;
float motorCtrl_integral_max        = 4.0f;
float frontLineCtrl_P_slow          = 1.5f;
float frontLineCtrl_D_slow          = 0.0f;
float frontLineCtrl_P_fast          = 0.15f;
float frontLineCtrl_D_fast          = 50.0f;
CarProps car                        = CarProps();
bool distServoEnabled               = true;
float distServoTransferRate         = 3.0f;
m_per_sec_t speed_FAST              = m_per_sec_t(1.5f);
m_per_sec_t speed_SLOW              = m_per_sec_t(1.0f);
meter_t slowSectionStartOffset      = meter_t(1.2);
m_per_sec_t speed_LAB_FWD           = m_per_sec_t(1.0f);
m_per_sec_t speed_LAB_BWD           = m_per_sec_t(0.7f);
m_per_sec_t speed_LANE_CHANGE       = m_per_sec_t(0.8f);
m_per_sec_t speed_REACH_SAFETY_CAR  = m_per_sec_t(0.8f);
m_per_sec_t speed_TURN_AROUND       = m_per_sec_t(0.5f);
m_per_sec_t speed_OVERTAKE_CURVE    = m_per_sec_t(1.4f);
m_per_sec_t speed_OVERTAKE_STRAIGHT = m_per_sec_t(3.0f);

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
    REGISTER_GLOBAL(frontLineCtrl_P_slow);
    REGISTER_GLOBAL(frontLineCtrl_D_slow);
    REGISTER_GLOBAL(frontLineCtrl_P_fast);
    REGISTER_GLOBAL(frontLineCtrl_D_fast);
    REGISTER_GLOBAL(car);
    REGISTER_GLOBAL(distServoEnabled);
    REGISTER_GLOBAL(distServoTransferRate);
    REGISTER_GLOBAL(speed_FAST);
    REGISTER_GLOBAL(speed_SLOW);
    REGISTER_GLOBAL(slowSectionStartOffset);
    REGISTER_GLOBAL(speed_LAB_FWD);
    REGISTER_GLOBAL(speed_LAB_BWD);
    REGISTER_GLOBAL(speed_LANE_CHANGE);

#undef REGISTER_GLOBAL
}

}  // namespace globals
}  // namespace micro
