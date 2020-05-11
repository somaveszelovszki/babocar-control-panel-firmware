#include <cfg_board.h>

#if GYRO_BOARD == GYRO_MPU9250
#include <micro/hw/MPU9250_Gyroscope.hpp>
#elif GYRO_BOARD == GYRO_LSM6DSO
#include <micro/hw/LSM6DSO_Gyroscope.hpp>
#endif

#include <micro/port/task.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_car.hpp>
#include <globals.hpp>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>

#include <micro/sensor/MadgwickAHRS.hpp>

using namespace micro;

queue_t<radian_t, 1> yawUpdateQueue;

namespace {

#if GYRO_BOARD == GYRO_MPU9250
hw::MPU9250_Gyroscope gyro(spi_Gyro, csGpio_Gyro, csGpioPin_Gyro, hw::Ascale::AFS_2G, hw::Gscale::GFS_500DPS, hw::Mscale::MFS_16BITS, MMODE_ODR_100Hz);
#elif GYRO_BOARD == GYRO_LSM6DSO
hw::LSM6DSO_Gyroscope gyro(spi_Gyro, csGpio_Gyro, csGpioPin_Gyro);
#endif

semaphore_t dataReadySemaphore;

void updateCarOrientedDistance(CarProps& car) {
    static meter_t orientedSectionStartDist;
    static radian_t orientation;

    const bool isOriented = eqWithOverflow360(car.pose.angle, orientation, degree_t(3));
    if (!isOriented) {
        orientedSectionStartDist = car.distance;
        orientation = car.pose.angle;
    }

    car.orientedDistance = car.distance - orientedSectionStartDist;
}

void updateCarProps(const rad_per_sec_t yawRate, const radian_t yaw) {

    static radian_t prevYaw = { 0 };
    static meter_t prevDist = { 0 };

    vTaskSuspendAll();
    CarProps car = globals::car;
    xTaskResumeAll();

    const meter_t d_dist      = sgn(car.speed) * (car.distance - prevDist);
    const radian_t d_angle    = normalizePM180(yaw - prevYaw);
    const radian_t speedAngle = car.getSpeedAngle(cfg::CAR_FRONT_REAR_PIVOT_DIST);

    car.pose.angle += d_angle / 2;
    car.pose.pos.X += d_dist * cos(speedAngle);
    car.pose.pos.Y += d_dist * sin(speedAngle);
    car.pose.angle  = normalize360(car.pose.angle + d_angle / 2);
    car.yawRate     = yawRate;

    updateCarOrientedDistance(car);

    vTaskSuspendAll();
    globals::car = car;
    xTaskResumeAll();

    prevDist = car.distance;
    prevYaw  = yaw;
}

} // namespace

extern "C" void runGyroTask(void) {

    gyro.initialize();
    MadgwickAHRS madgwick(gyro.gyroMeanError().Z.get());

    while (true) {
        bool success = false;

        if (dataReadySemaphore.take(millisecond_t(50))) {

            const millisecond_t sampleTime = getExactTime();
            const point3<rad_per_sec_t> gyroData = gyro.readGyroData();
            const point3<m_per_sec2_t> accelData = gyro.readAccelData();

            if (!micro::isinf(gyroData.X) && !micro::isinf(accelData.X)) {

                radian_t yawUpdate;
                if (yawUpdateQueue.receive(yawUpdate, millisecond_t(0))) {
                    madgwick.reset({ madgwick.roll(), madgwick.pitch(), yawUpdate });
                }

                madgwick.update(sampleTime, gyroData, accelData);
                updateCarProps(gyroData.Z, madgwick.yaw());
                globals::isGyroTaskOk = true;
                success = true;
            }
        }

        if (!success) {
            globals::isGyroTaskOk = false;
            LOG_ERROR("Gyro timed out");
            gyro.initialize();
        }
    }
}

void micro_Gyro_CommCpltCallback() {
    gyro.onCommFinished();
}

void micro_Gyro_DataReadyCallback() {
    dataReadySemaphore.give();
}
