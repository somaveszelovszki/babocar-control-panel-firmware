#include <uns/task/common.hpp>
#include <config/cfg_board.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/units.hpp>
#include <uns/bsp/task.hpp>
#include <uns/sensor/DistanceSensor.hpp>
#include <uns/CarProps.hpp>
#include <uns/Line.hpp>

using namespace uns;

extern ProgramTask PROGRAM_TASK;

extern CarProps car;

using namespace uns;

namespace {
constexpr time_t period_DistanceSensor(milliseconds(), 50.0f);
DistanceSensor frontDistSensor(period_DistanceSensor, cfg::gpio_DistSHTD3, cfg::gpio_DistINT2);

bool isSafe(const Line& line) {
    static const distance_t fastSpeedMaxLinePos(millimeters(), 85);
    static const angle_t fastSpeedMaxLineAngle(degrees(), 4.0f);
    static const angle_t fastSpeedMaxServoAngle(degrees(), 6.5f);

    return abs(line.pos) <= fastSpeedMaxLinePos && abs(line.angle) <= fastSpeedMaxLineAngle && abs(car.steeringAngle()) <= fastSpeedMaxServoAngle;
}
} // namespace

extern "C" void runProgTrackTask(void const *argument) {

//    if (!isOk(status = frontDistSensor.initialize())) {
//        //debug::printerr(status, "Front distance sensor init error!");
//    }

    while (!uns::hasErrorHappened()) {
//        if (isOk(status = frontDistSensor.run())) {
//            distance_t dist = frontDistSensor.getMeasured();
//            debug::printf(debug::CONTENT_FLAG_LOG, "Distance: %d cm", (int)dist.get<centimeters>());
//        } else {
//            debug::printerr(status, "Front distance sensor read error!");
//        }
//
//        uns::nonBlockingDelay(time_t::from<milliseconds>(100));
        switch (PROGRAM_TASK) {
            case ProgramTask::SAFETY_CAR_FOLLOW:
                // TODO follow safety car
                uns::nonBlockingDelay(time_t::from<milliseconds>(50));
                break;
            case ProgramTask::OVERTAKE:
                // TODO overtake safety car
                uns::nonBlockingDelay(time_t::from<milliseconds>(50));
                break;
            case ProgramTask::RACE_TRACK:
                // TODO race given number of laps on the track
                uns::nonBlockingDelay(time_t::from<milliseconds>(50));
                break;
            default:
                uns::nonBlockingDelay(time_t::from<milliseconds>(50));
                break;
        }
    }

    uns::deleteCurrentTask();
}

/* Callback for Distance sensor #2 echo.
 */
void uns_Dist2_EchoCallback() {
    frontDistSensor.onEchoReceived();
}
