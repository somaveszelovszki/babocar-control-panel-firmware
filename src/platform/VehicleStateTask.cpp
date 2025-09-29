#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <globals.hpp>

#include <micro/debug/ParamManager.hpp>
#include <micro/debug/TaskMonitor.hpp>
#include <micro/log/log.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/timer.hpp>

#if GYRO_BOARD == GYRO_MPU9250
#include <micro/hw/MPU9250_Gyroscope.hpp>
#elif GYRO_BOARD == GYRO_LSM6DSO
#include <micro/hw/LSM6DSO_Gyroscope.hpp>
#endif

using namespace micro;

namespace {

CarProps car;

#if GYRO_BOARD == GYRO_MPU9250
hw::MPU9250_Gyroscope gyro(spi_Gyro, csGpio_Gyro, hw::Ascale::AFS_2G, hw::Gscale::GFS_500DPS,
                           hw::Mscale::MFS_16BITS, MMODE_ODR_100Hz);
#elif GYRO_BOARD == GYRO_LSM6DSO
hw::LSM6DSO_Gyroscope gyro(spi_Gyro, csGpio_Gyro);
#endif

semaphore_t dataReadySemaphore;

CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::Id vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

void updateCarOrientedDistance(CarProps& car) {
    static meter_t orientedSectionStartDist;
    static radian_t orientation;

    const bool isOriented = eqWithOverflow360(car.pose.angle, orientation, degree_t(5));
    if (!isOriented) {
        orientedSectionStartDist = car.distance;
        orientation              = car.pose.angle;
    }

    car.orientedDistance = car.distance - orientedSectionStartDist;
}

void updateCarPose() {
    static meter_t prevDist = {0};
    static microsecond_t prevTime;

    const microsecond_t now = getExactTime();

    const meter_t d_dist = car.distance - prevDist;
    const radian_t d_angle =
        car.yawRate * (prevTime > microsecond_t(0) ? now - prevTime : microsecond_t(0));
    const radian_t speedAngle = car.getSpeedAngle(cfg::CAR_FRONT_REAR_PIVOT_DIST) + d_angle / 2;

    car.pose.pos.X += d_dist * cos(speedAngle);
    car.pose.pos.Y += d_dist * sin(speedAngle);
    car.pose.angle = normalize360(car.pose.angle + d_angle);

    updateCarOrientedDistance(car);
    prevDist = car.distance;
    prevTime = now;
}

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::LateralState::id(), [](const uint8_t* const data) {
        radian_t frontDistSensorServoAngle;
        reinterpret_cast<const can::LateralState*>(data)->acquire(
            car.frontWheelAngle, car.rearWheelAngle, frontDistSensorServoAngle);
    });

    vehicleCanFrameHandler.registerHandler(
        can::LongitudinalState::id(), [](const uint8_t* const data) {
            reinterpret_cast<const can::LongitudinalState*>(data)->acquire(
                car.speed, car.isRemoteControlled, car.distance);
        });

    const CanFrameIds rxFilter = vehicleCanFrameHandler.identifiers();
    const CanFrameIds txFilter = {};
    vehicleCanSubscriberId     = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runVehicleStateTask(void) {
    initializeVehicleCan();
    gyro.initialize();

    taskMonitor.registerInitializedTask();

    WatchdogTimer gyroDataWd(millisecond_t(15));

    while (true) {
        while (const auto frame = vehicleCanManager.read(vehicleCanSubscriberId)) {
            vehicleCanFrameHandler.handleFrame(*frame);
        }

        point2m pos;
        if (carPosUpdateQueue.receive(pos, millisecond_t(0))) {
            LOG_DEBUG("Car pos updated: {}, {}) -> ({}, {}) | diff: {} [m]", car.pose.pos.X.get(),
                      car.pose.pos.Y.get(), pos.X.get(), pos.Y.get(),
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

        const auto ok = !vehicleCanManager.hasTimedOut(vehicleCanSubscriberId) && isGyroOk;
        taskMonitor.notify(ok);
        os_sleep(millisecond_t(5));
    }
}

void micro_Gyro_DataReadyCallback() {
    dataReadySemaphore.give();
}
