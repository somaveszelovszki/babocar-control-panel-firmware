#pragma once

#include <micro/debug/params.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/LinePattern.hpp>

#include "Distances.hpp"
#include "RaceTrackController.hpp"

extern micro::queue_t<micro::radian_t, 1> carOrientationUpdateQueue;
extern micro::queue_t<micro::point2m, 1> carPosUpdateQueue;
extern micro::queue_t<micro::CarProps, 1> carPropsQueue;
extern micro::queue_t<micro::ControlData, 1> controlQueue;
extern micro::queue_t<Distances, 1> distancesQueue;
extern micro::queue_t<LapControlParameters, 1> lapControlQueue;
extern micro::queue_t<LapControlParameters, 1> lapControlOverrideQueue;
extern micro::queue_t<micro::ControlData, 1> lastControlQueue;
extern micro::queue_t<micro::LineDetectControl, 1> lineDetectControlQueue;
extern micro::queue_t<micro::LineInfo, 1> lineInfoQueue;
extern micro::queue_t<char, 1> radioRecvQueue;

extern micro::CanManager vehicleCanManager;

extern micro::Params globalParams;
