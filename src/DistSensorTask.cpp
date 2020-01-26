#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/hw/VL53L1X_DistanceSensor.hpp>
#include <micro/task/common.hpp>
#include <micro/sensor/Filter.hpp>

#include <cfg_board.h>
#include <globals.hpp>
#include <DistancesData.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#define DISTANCES_QUEUE_LENGTH 1
QueueHandle_t distancesQueue;
static uint8_t distancesQueueStorageBuffer[DISTANCES_QUEUE_LENGTH * sizeof(DistancesData)];
static StaticQueue_t distancesQueueBuffer;

static hw::VL53L1X_DistanceSensor frontDistSensor(i2c_Dist, 0x52);

extern "C" void runDistSensorTask(const void *argument) {

    distancesQueue = xQueueCreateStatic(DISTANCES_QUEUE_LENGTH, sizeof(DistancesData), distancesQueueStorageBuffer, &distancesQueueBuffer);
    HAL_GPIO_WritePin(gpio_DistEn, gpioPin_DistEn, GPIO_PIN_RESET);

    vTaskDelay(300); // gives time to other tasks to wake up

    frontDistSensor.initialize();

    LowPassFilter<meter_t, 3> frontDistFilter;
    DistancesData distances;
    millisecond_t prevReadTime = getTime();

    int i = 0;

    while (true) {
        if (globals::distServoEnabled) {
            if (isOk(frontDistSensor.readDistance(distances.front))) {
                globals::isDistSensorTaskOk = true;

                distances.front = frontDistFilter.update(distances.front);
                if (distances.front > centimeter_t(200)) {
                    distances.front = meter_t::infinity();
                }

                xQueueOverwrite(distancesQueue, &distances);
                prevReadTime = getTime();
                vTaskDelay(19);
            } else if (getTime() - prevReadTime > millisecond_t(200)) {
                distances.front = meter_t(0);
                globals::isDistSensorTaskOk = false;

                HAL_GPIO_WritePin(gpio_DistEn, gpioPin_DistEn, GPIO_PIN_SET);
                vTaskDelay(2);
                HAL_GPIO_WritePin(gpio_DistEn, gpioPin_DistEn, GPIO_PIN_RESET);
                vTaskDelay(30);
                frontDistSensor.initialize();
                prevReadTime = getTime();
            }
        } else {
            globals::isDistSensorTaskOk = true;
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
