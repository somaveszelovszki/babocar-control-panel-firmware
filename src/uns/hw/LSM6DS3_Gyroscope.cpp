#include <uns/hw/LSM6DS3_Gyroscope.hpp>
#include <uns/bsp/tim.hpp>

using namespace uns;

namespace {
constexpr uint16_t  REG_WHO_AM_I    = 0x0F;     // WHO_AM_I register (r). This register is a read-only register. Its value is fixed at 69h.
constexpr uint16_t  REG_CTRL1_XL    = 0x10;     // Linear acceleration sensor control register 1 (r/w).
constexpr uint16_t  REG_CTRL9_XL    = 0x18;     // Linear acceleration sensor control register 9 (r/w).
constexpr uint16_t  REG_INT1_CTRL   = 0x0D;     // INT1 pad control register (r/w). Each bit in this register enables a signal to be carried through INT1.
constexpr uint16_t  REG_CTRL10_C    = 0x19;     // Control register 10 (r/w).
constexpr uint16_t  REG_CTRL2_G     = 0x11;     // Angular rate sensor control register 2 (r/w).
constexpr uint16_t  REG_STATUS      = 0x1E;     // Status register - should be polled to check when a new set of data is available.

constexpr uint16_t  REG_OUTX_L_G    = 0x22;     // Angular rate sensor pitch axis (X) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement.
constexpr uint16_t  REG_OUTX_H_G    = 0x23;     // Angular rate sensor pitch axis (X) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement.
constexpr uint16_t  REG_OUTY_L_G    = 0x24;     // Angular rate sensor roll axis (Y) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement.
constexpr uint16_t  REG_OUTY_H_G    = 0x25;     // Angular rate sensor roll axis (Y) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement.
constexpr uint16_t  REG_OUTZ_L_G    = 0x26;     // Angular rate sensor yaw axis (Z) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement.
constexpr uint16_t  REG_OUTZ_H_G    = 0x27;     // Angular rate sensor yaw axis (Z) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement.

constexpr uint8_t   WHO_AM_I        = 0x69;     // Device id.
constexpr uint8_t   HIGH_PERF_MODE  = 0x60;     // 416Hz (High-Performance mode).
constexpr uint8_t   FS_SEL_2000     = 0x0C;     // Full-scale selection of 2000.
constexpr uint8_t   XYZ_AXES_EN     = 0x38;     // X, Y, Z axes enabled.
constexpr uint8_t   ACC_INT1        = 0x01;     // Acc Data Ready interrupt on INT1.
constexpr uint8_t   GYRO_INT1       = 0x02;     // Gyro Data Ready interrupt on INT1.

constexpr uint8_t   MASK_XLDA       = 0x01;     // Mask for XLDA bit in status register byte - indicates new accelerometer data.
constexpr uint8_t   MASK_GDA        = 0x02;     // Mask for GDA bit in status register byte - indicates new gyroscope data.

constexpr int32_t   GYRO_FS_SEL     = 2000;     // Full-scale selection.

//constexpr uint16_t  DEVICE_ADDR     = (static_cast<uint16_t>(0xD5) << 1) | static_cast<uint16_t>(0x01);   // 7-bit I2C device address.
constexpr uint16_t  DEVICE_ADDR     = static_cast<uint16_t>(0xD5) ;   // 7-bit I2C device address.

constexpr uns::time_t TIMEOUT_INIT(milliseconds(), 20.0f);
constexpr uns::time_t TIMEOUT(milliseconds(), 2.0f);

/* @brief Converts gyroscope value to angular velocity.
 * @param gyroValBuffer The gyroscope value buffer (read from the sensor).
 * @returns The gyroscope value converted to angular velocity.
 **/
angular_velocity_t gyroToAngVel(const uint8_t * const gyroValBuffer)
{
    // 1.18 -> heuristic constant
    constexpr float32_t ratio = (4.375f * GYRO_FS_SEL / 125) / 1000 * 1.22f;    // ratio of angular velocity [deg/sec] and gyroscope value
    int16_t gyroVal = static_cast<int16_t>(gyroValBuffer[0]) | (static_cast<int16_t>(gyroValBuffer[1]) << 8);
    return angular_velocity_t::from<deg_per_sec>(gyroVal * ratio);
}
} // namespace

hw::LSM6DS3_Gyroscope::LSM6DS3_Gyroscope(i2c_handle_t * const _hi2c)
    : hi2c(_hi2c) {}

Status hw::LSM6DS3_Gyroscope::initialize() {
    Status status;
    uint8_t buffer;

    uns::nonBlockingDelay(time_t::from<milliseconds>(25));    // time for gyroscope to initialize itself after startup

    if(isOk(status = I2C_IsDeviceReady(this->hi2c, DEVICE_ADDR, 10, TIMEOUT_INIT))) {
        if (isOk(status = I2C_Mem_Read(this->hi2c, DEVICE_ADDR, REG_WHO_AM_I, 1, &buffer, 1, TIMEOUT_INIT))) {
            if (buffer == WHO_AM_I) {
                uint8_t ctrl2Val = HIGH_PERF_MODE | FS_SEL_2000;
                if (isOk(status = I2C_Mem_Write(this->hi2c, DEVICE_ADDR, REG_CTRL10_C, 1, &XYZ_AXES_EN, 1, TIMEOUT_INIT))    // enables X, Y and Z axes
                    && isOk(status = I2C_Mem_Write(this->hi2c, DEVICE_ADDR, REG_CTRL2_G, 1, &ctrl2Val, 1, TIMEOUT_INIT))) {  // sets high performance mode (416Hz)
                    status = I2C_Mem_Write(this->hi2c, DEVICE_ADDR, REG_INT1_CTRL, 1, &GYRO_INT1, 1, TIMEOUT_INIT);          // enables Gyro Data Ready interrupt on INT1
                }
            } else {
                status = Status::INVALID_ID;
            }
        }
    }

    return status;
}

Status hw::LSM6DS3_Gyroscope::read(hw::GyroData * const pResult) {
    Status status;
    uint8_t buffer[6];
    if (isOk(status = I2C_Mem_Read(this->hi2c, DEVICE_ADDR, REG_STATUS, 1, buffer, 1, TIMEOUT))) {
        if (buffer[0] & MASK_GDA) {
            if (isOk(status = I2C_Mem_Read(this->hi2c, DEVICE_ADDR, REG_OUTX_L_G, 1, buffer, 6, TIMEOUT))) {
                pResult->X = gyroToAngVel(&buffer[0]);
                pResult->Y = gyroToAngVel(&buffer[2]);
                pResult->Z = gyroToAngVel(&buffer[4]);
            }
        } else {
            status = Status::NO_NEW_DATA;
        }
    }

    return status;
}
