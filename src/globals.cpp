#include <globals.hpp>
#include <cfg_board.hpp>

using namespace micro;

queue_t<radian_t, 1> carOrientationUpdateQueue;
queue_t<point2m, 1> carPosUpdateQueue;
queue_t<CarProps, 1> carPropsQueue;
queue_t<ControlData, 1> controlQueue;
queue_t<Distances, 1> distancesQueue;
queue_t<LapControlParameters, 1> lapControlQueue;
queue_t<LapControlParameters, 1> lapControlOverrideQueue;
queue_t<ControlData, 1> lastControlQueue;
queue_t<LineDetectControl, 1> lineDetectControlQueue;
queue_t<LineInfo, 1> lineInfoQueue;
queue_t<char, 1> radioRecvQueue;

CanManager vehicleCanManager(can_Vehicle);

Params globalParams('P');
