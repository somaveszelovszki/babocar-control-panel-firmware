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

static hw::VL53L1X_DistanceSensor frontDistSensor(i2c_X, 0x52);
static Timer distSendTimer;

extern "C" void runDistSensorTask(const void *argument) {

    distancesQueue = xQueueCreateStatic(DISTANCES_QUEUE_LENGTH, sizeof(DistancesData), distancesQueueStorageBuffer, &distancesQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    frontDistSensor.initialize();

    LowPassFilter<meter_t, 3> frontDistFilter;
    DistancesData distances;
    millisecond_t prevReadTime = getTime();
    distSendTimer.start(millisecond_t(500));

    while (true) {
        if (isOk(frontDistSensor.readDistance(distances.front))) {
            globals::isDistSensorTaskOk = true;

            distances.front = frontDistFilter.update(distances.front);
            if (distances.front > centimeter_t(200)) {
                distances.front = meter_t::infinity();
            }

            xQueueOverwrite(distancesQueue, &distances);
            prevReadTime = getTime();
            vTaskDelay(19);
        } else if (getTime() - prevReadTime > millisecond_t(1000)) {
            distances.front = meter_t(0);
            globals::isDistSensorTaskOk = false;

////            SET_BIT(i2c_Dist->Instance->CR1, I2C_CR1_SWRST);
////            vTaskDelay(2);
////            CLEAR_BIT(i2c_Dist->Instance->CR1, I2C_CR1_SWRST);
////            vTaskDelay(5);
////
////            {
////                __HAL_RCC_I2C2_CLK_DISABLE();
////
////                /**I2C2 GPIO Configuration
////                PB10     ------> I2C2_SCL
////                PB3     ------> I2C2_SDA
////                */
////                HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_3);
////
////                /* I2C2 interrupt Deinit */
////                HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
////                HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
////            }
//
//            HAL_GPIO_WritePin(gpio_DistEn, gpioPin_DistEn, GPIO_PIN_SET);
//            vTaskDelay(5);
//            HAL_GPIO_WritePin(gpio_DistEn, gpioPin_DistEn, GPIO_PIN_RESET);
//
////            {
////                GPIO_InitTypeDef GPIO_InitStruct = {0};
////
////                __HAL_RCC_GPIOB_CLK_ENABLE();
////                /**I2C2 GPIO Configuration
////                PB10     ------> I2C2_SCL
////                PB3     ------> I2C2_SDA
////                */
////                GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3;
////                GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
////                GPIO_InitStruct.Pull = GPIO_PULLUP;
////                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
////                GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
////                HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
////
////                /* I2C2 clock enable */
////                __HAL_RCC_I2C2_CLK_ENABLE();
////
////                /* I2C2 interrupt Init */
////                HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
////                HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
////                HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
////                HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
////            }
//
//            vTaskDelay(50);

            //frontDistSensor.initialize();
            prevReadTime = getTime();
        }

        if (distSendTimer.checkTimeout()) {
            //LOG_DEBUG("dist: %fcm", static_cast<centimeter_t>(distances.front).get());
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
