#include <globals.hpp>

#include <cfg_board.hpp>
#include <cfg_system.hpp>

using namespace micro;

queue_t<radian_t, 1> carOrientationUpdateQueue;
queue_t<point2m, 1> carPosUpdateQueue;
queue_t<CarProps, 1> carPropsQueue;
queue_t<ControlData, 1> controlQueue;
queue_t<meter_t, 1> frontDistanceQueue;
queue_t<LapControlParameters, 1> lapControlQueue;
queue_t<IndexedSectionControlParameters, 8> sectionControlOverrideQueue;
queue_t<ControlData, 1> lastControlQueue;
queue_t<LineDetectControl, 1> lineDetectControlQueue;
queue_t<LineInfo, 1> lineInfoQueue;
queue_t<etl::string<cfg::RADIO_COMMAND_MAX_LENGTH>, 4> radioCommandQueue;
queue_t<meter_t, 1> rearDistancesQueue;

CanManager vehicleCanManager(can_Vehicle);
ParamManager globalParams;
TaskMonitor taskMonitor(cfg::NUM_MONITORED_TASKS);
ProgramState programState;
