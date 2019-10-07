#include <micro/utils/log.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/LineDetectPanelData.h>
#include <micro/panel/MotorPanel.hpp>
#include <micro/panel/MotorPanelData.h>
#include <micro/task/common.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>

#include <globals.hpp>

#include <micro/hw/MPU9250_Gyroscope.hpp>

using namespace micro;

extern LineDetectPanel frontLineDetectPanel;
extern LineDetectPanel rearLineDetectPanel;

hw::MPU9250 gyro(i2c_Gyro, hw::Ascale::AFS_2G, hw::Gscale::GFS_250DPS, hw::Mscale::MFS_16BITS, MMODE_ODR_100Hz);

extern "C" void runSensorTask(const void *argument) {
    //LOG_DEBUG("SensorTask running...");

    while (true) {
//        const point3<gauss_t> mag = gyro.readMagData();
//        //LOG_DEBUG("%f, %f", mag.X.get(), mag.Y.get());
//        if (!isZero(mag.X) || !isZero(mag.Y) || !isZero(mag.Z)) {
//            LOG_DEBUG("%f", static_cast<degree_t>(normalize360(atan2(mag.Y, mag.X))).get());
//        }

        vTaskDelay(20);
    }

    vTaskDelete(nullptr);
}
