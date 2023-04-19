#include <cfg_board.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_car.hpp>

#if GYRO_BOARD == GYRO_MPU9250
#include <micro/hw/MPU9250_Gyroscope.hpp>
#elif GYRO_BOARD == GYRO_LSM6DSO
#include <micro/hw/LSM6DSO_Gyroscope.hpp>
#endif

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>

using namespace micro;

extern CanManager vehicleCanManager;

queue_t<CarProps, 1> carPropsQueue;
queue_t<point2m, 1> carPosUpdateQueue;
queue_t<radian_t, 1> carOrientationUpdateQueue;

namespace {

CarProps car;
bool isRemoteControlled = false;

#if GYRO_BOARD == GYRO_MPU9250
hw::MPU9250_Gyroscope gyro(spi_Gyro, csGpio_Gyro, hw::Ascale::AFS_2G, hw::Gscale::GFS_500DPS, hw::Mscale::MFS_16BITS, MMODE_ODR_100Hz);
#elif GYRO_BOARD == GYRO_LSM6DSO
hw::LSM6DSO_Gyroscope gyro(spi_Gyro, csGpio_Gyro);
#endif

semaphore_t dataReadySemaphore;

canFrame_t rxCanFrame;
CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::id_t vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

void updateCarOrientedDistance(CarProps& car) {
    static meter_t orientedSectionStartDist;
    static radian_t orientation;

    const bool isOriented = eqWithOverflow360(car.pose.angle, orientation, degree_t(5));
    if (!isOriented) {
        orientedSectionStartDist = car.distance;
        orientation = car.pose.angle;
    }

    car.orientedDistance = car.distance - orientedSectionStartDist;
}

void updateCarPose() {
    static meter_t prevDist = { 0 };
    static microsecond_t prevTime = getExactTime();

    const microsecond_t now = getExactTime();

    const meter_t d_dist      = car.distance - prevDist;
    const radian_t d_angle    = car.yawRate * (now - prevTime);
    const radian_t speedAngle = car.getSpeedAngle(cfg::CAR_FRONT_REAR_PIVOT_DIST) + d_angle / 2;

    car.pose.pos.X += d_dist * cos(speedAngle);
    car.pose.pos.Y += d_dist * sin(speedAngle);
    car.pose.angle  = normalize360(car.pose.angle + d_angle);

    updateCarOrientedDistance(car);
    prevDist = car.distance;
    prevTime = now;
}

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::LateralState::id(), [] (const uint8_t * const data) {
        radian_t frontDistSensorServoAngle;
        reinterpret_cast<const can::LateralState*>(data)->acquire(car.frontWheelAngle, car.rearWheelAngle, frontDistSensorServoAngle);
    });

    vehicleCanFrameHandler.registerHandler(can::LongitudinalState::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(car.speed, isRemoteControlled, car.distance);
    });

    const CanFrameIds rxFilter = vehicleCanFrameHandler.identifiers();
    const CanFrameIds txFilter = {};
    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runVehicleStateTask(void) {

    SystemManager::instance().registerTask();

    initializeVehicleCan();

    gyro.initialize();
    WatchdogTimer gyroDataWd(millisecond_t(15));

    REGISTER_READ_ONLY_PARAM(car);
    REGISTER_READ_ONLY_PARAM(isRemoteControlled);

    while (true) {
        while (vehicleCanManager.read(vehicleCanSubscriberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        point2m pos;
        if (carPosUpdateQueue.receive(pos, millisecond_t(0))) {
            LOG_DEBUG("Car pos updated: (%f, %f) -> (%f, %f) | diff: %f [m]",
                car.pose.pos.X.get(), car.pose.pos.Y.get(), pos.X.get(), pos.Y.get(),
                car.pose.pos.distance(pos).get());
            car.pose.pos = pos;
        }

        radian_t orientation;
        if (carOrientationUpdateQueue.receive(orientation, millisecond_t(0))) {
            car.pose.angle = orientation;
        }

        if (dataReadySemaphore.take(millisecond_t(0))) {

            const point3<rad_per_sec_t> gyroData = gyro.readGyroData();
            if (!micro::isinf(gyroData.X)) {
                car.yawRate = gyroData.Z;
                gyroDataWd.reset();
            }
        }

        updateCarPose();
        carPropsQueue.overwrite(car);

        const bool isGyroOk = !gyroDataWd.hasTimedOut();
        if (!isGyroOk) {
            LOG_ERROR("Gyro timed out");
            gyro.initialize();
            gyroDataWd.reset();
        }

        SystemManager::instance().notify(!vehicleCanManager.hasTimedOut(vehicleCanSubscriberId) && isGyroOk);
        os_sleep(millisecond_t(5));
    }
}

void micro_Gyro_CommCpltCallback() {
    gyro.onCommFinished();
}

void micro_Gyro_DataReadyCallback() {
    dataReadySemaphore.give();
}
