#pragma once

#include <uns/bsp/i2c.hpp>
#include <uns/util/units.hpp>
#include <uns/Point3.hpp>

namespace uns {
namespace hw {

/* @brief Stores gyroscope data.
 **/
typedef Point3<angular_velocity_t> GyroData;

class LSM6DS3_Gyroscope {
private:
    i2c_handle_t * const hi2c;  // The handle for the I2C instance.

public :
    /* @brief Constructor - sets I2C handle.
     * @param _hi2c The handle for the I2C instance.
     **/
    LSM6DS3_Gyroscope(i2c_handle_t * const _hi2c);

    /* @brief Initializes gyroscope.
     **/
    Status initialize();

    /* @brief Read gyroscope data from the sensor.
     * @param pResult Pointer to a GyroData structure that will store the current gyroscope data.
     * @returns Status indicating operation success.
     **/
    Status read(GyroData * const pResult);
};
} // namespace hw
} // namespace uns
