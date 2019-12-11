#include <cfg_board.h>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/hw/VL53L1X_DistanceSensor.hpp>
#include <micro/task/common.hpp>
#include <micro/sensor/Filter.hpp>

#include <DistancesData.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern QueueHandle_t distancesQueue;

static hw::VL53L1X_DistanceSensor frontDistSensor(i2c_Dist, 0x52);

extern "C" void runDistSensorTask(const void *argument) {

    vTaskDelay(300); // gives time to other tasks to wake up

    frontDistSensor.initialize();

    globals::isDistSensorTaskInitialized = true;
    LOG_DEBUG("Distance sensor task initialized");

    DistancesData distances;

    while (true) {
        if (isOk(frontDistSensor.readDistance(distances.front))) {
            xQueueOverwrite(distancesQueue, &distances);
            vTaskDelay(16);
        } else {
            vTaskDelay(2);
        }
    }

    vTaskDelete(nullptr);
}
