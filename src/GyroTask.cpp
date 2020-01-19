#include <cfg_board.h>
#include <micro/utils/log.hpp>
#include <micro/utils/time.hpp>
#include <micro/hw/MPU9250_Gyroscope.hpp>
#include <micro/task/common.hpp>
#include <micro/sensor/Filter.hpp>

#include <globals.hpp>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

class AngleFilter : public LowPassFilter<radian_t, 10> {
public:
    AngleFilter() : LowPassFilter() {}

    radian_t update(const radian_t& measuredValue) override {
        radian_t meas = measuredValue;
        while (meas < this->filteredValue - PI) {
            meas += 2 * PI;
        }
        while (meas > this->filteredValue + PI) {
            meas -= 2 * PI;
        }
        return normalize360(LowPassFilter::update(meas));
    }
};

static hw::MPU9250 gyro(i2c_Gyro, hw::Ascale::AFS_2G, hw::Gscale::GFS_250DPS, hw::Mscale::MFS_16BITS, MMODE_ODR_100Hz);
static AngleFilter angleFilter;

struct AngleCalc {
    struct MagNormalization {
        gauss_t offset;
        gauss_t amplitude;

        float apply(gauss_t mag) const {
            return (mag - this->offset) / this->amplitude;
        }
    };

    const point2<MagNormalization> norm;

    radian_t getAngle(const point3<gauss_t>& mag) const {
        return micro::atan2(norm.Y.apply(mag.Y), norm.X.apply(mag.X));
    }
};

static const AngleCalc angleCalc = {{
    { gauss_t(-243), gauss_t(217) },
    { gauss_t(494),  gauss_t(199) }
}};

extern "C" void runGyroTask(const void *argument) {

    HAL_GPIO_WritePin(gpio_GyroEn, gpioPin_GyroEn, GPIO_PIN_RESET);

    vTaskDelay(300); // gives time to other tasks to wake up

    gyro.initialize();

    globals::isGyroTaskOk = true;

    millisecond_t prevReadTime = micro::getTime();
    meter_t prevDist = globals::car.distance;

    while (true) {
        const point3<gauss_t> mag = gyro.readMagData();
        if (!isZero(mag.X) || !isZero(mag.Y) || !isZero(mag.Z)) {
            const radian_t newAngle = angleFilter.update(angleCalc.getAngle(mag));

            vTaskSuspendAll();
            const meter_t dist = sgn(globals::car.speed) * (globals::car.distance - prevDist);
            globals::car.pose.angle = avg(newAngle, globals::car.pose.angle);
            globals::car.pose.pos.X += dist * cos(globals::car.pose.angle);
            globals::car.pose.pos.Y += dist * sin(globals::car.pose.angle);
            globals::car.pose.angle = newAngle;
            prevDist = globals::car.distance;
            xTaskResumeAll();

            LOG_DEBUG("%d, %d / %f",
                (int)static_cast<centimeter_t>(globals::car.pose.pos.X).get(),
                (int)static_cast<centimeter_t>(globals::car.pose.pos.Y).get(),
                static_cast<degree_t>(globals::car.pose.angle).get());

            prevReadTime = getTime();
            vTaskDelay(8); // new magnetometer data is available in every ~10ms

        } else if (getTime() - prevReadTime > millisecond_t(30)) {
            HAL_GPIO_WritePin(gpio_GyroEn, gpioPin_GyroEn, GPIO_PIN_SET);
            vTaskDelay(2);
            HAL_GPIO_WritePin(gpio_GyroEn, gpioPin_GyroEn, GPIO_PIN_RESET);
            vTaskDelay(8);
            prevReadTime = getTime();
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
