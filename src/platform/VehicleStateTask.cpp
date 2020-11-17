#include <cfg_board.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/sensor/Filter.hpp>
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

#include <micro/sensor/MadgwickAHRS.hpp>

using namespace micro;

extern CanManager vehicleCanManager;

queue_t<CarProps, 1> carPropsQueue;
queue_t<point2m, 1> carPosUpdateQueue;
queue_t<radian_t, 1> carOrientationUpdateQueue;

namespace {

CarProps car;

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

    const meter_t d_dist      = car.distance - prevDist;
    const radian_t d_angle    = normalizePM180(yaw - prevYaw);
    const radian_t speedAngle = car.getSpeedAngle(cfg::CAR_FRONT_REAR_PIVOT_DIST);

    car.pose.angle += d_angle / 2;
    car.pose.pos.X += d_dist * cos(speedAngle);
    car.pose.pos.Y += d_dist * sin(speedAngle);
    car.pose.angle  = normalize360(car.pose.angle + d_angle / 2);
    car.yawRate     = yawRate;

    updateCarOrientedDistance(car);
    prevDist = car.distance;
    prevYaw  = yaw;
}

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::LateralState::id(), [] (const uint8_t * const data) {
        radian_t frontDistSensorServoAngle;
        reinterpret_cast<const can::LateralState*>(data)->acquire(car.frontWheelAngle, car.rearWheelAngle, frontDistSensorServoAngle);
    });

    vehicleCanFrameHandler.registerHandler(can::LongitudinalState::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(car.speed, car.distance);
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
    MadgwickAHRS madgwick(gyro.gyroMeanError().Z.get());
    WatchdogTimer gyroDataWd(millisecond_t(15));
    millisecond_t lastValidGyroDataTime = getTime();

    REGISTER_READ_ONLY_PARAM(car);

    while (true) {
        const CarProps prevCar = car;

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
            LOG_DEBUG("Car orientation updated: %f -> %f [deg]", static_cast<degree_t>(car.pose.angle).get(), static_cast<degree_t>(orientation).get());
            car.pose.angle = orientation;
            madgwick.reset({ madgwick.roll(), madgwick.pitch(), orientation });
        }

        if (dataReadySemaphore.take(millisecond_t(0))) {

            const millisecond_t sampleTime = getExactTime();
            const point3<rad_per_sec_t> gyroData = gyro.readGyroData();
            const point3<m_per_sec2_t> accelData = gyro.readAccelData();

            if (!micro::isinf(gyroData.X) && !micro::isinf(accelData.X)) {
                madgwick.update(sampleTime, gyroData, accelData);
                updateCarProps(gyroData.Z, madgwick.yaw());
                gyroDataWd.reset();
                lastValidGyroDataTime = getTime();
            }
        }

        if (car != prevCar) {
            carPropsQueue.overwrite(car);
        }

        if (gyroDataWd.hasTimedOut()) {
            LOG_ERROR("Gyro timed out");
            gyro.initialize();
            gyroDataWd.reset();
        }

        SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut() && getTime() - lastValidGyroDataTime < millisecond_t(50));
        os_sleep(millisecond_t(1));
    }
}

void micro_Gyro_CommCpltCallback() {
    gyro.onCommFinished();
}

void micro_Gyro_DataReadyCallback() {
    dataReadySemaphore.give();
}
