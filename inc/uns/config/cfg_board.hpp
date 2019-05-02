#pragma once

#include <uns/bsp/adc.hpp>
#include <uns/bsp/gpio.hpp>
#include <uns/bsp/i2c.hpp>
#include <uns/bsp/spi.hpp>
#include <uns/bsp/tim.hpp>
#include <uns/bsp/uart.hpp>
#include <uns/bsp/dma.hpp>

#include <uns/bsp/queue.hpp>
#include <uns/bsp/mutex.hpp>

#define BSP_LIB_HAL     // BSP library: HAL
#define OS_FREERTOS     // OS: FreeRTOS.

namespace uns {
namespace cfg {

tim_handle_t   * const  tim_System              = uns::getTimerHandle(2);                               // System timer instance.

tim_handle_t   * const  tim_Servo               = uns::getTimerHandle(5);                               // Servo timer instance.
const tim_channel_t     tim_chnl_Servo1         = uns::getTimerChannel(1);                              // Servo1 timer channel.
const tim_channel_t     tim_chnl_Servo2         = uns::getTimerChannel(2);                              // Servo2 timer channel.

tim_handle_t   * const  tim_DC_Motor            = uns::getTimerHandle(8);                               // DC motor timer instance.
const tim_channel_t     tim_chnl_DC_Fwd         = uns::getTimerChannel(2);                              // DC motor forward timer channel.
const tim_channel_t     tim_chnl_DC_Bwd         = uns::getTimerChannel(3);                              // DC motor backward timer channel.

tim_handle_t   * const  tim_RC_Recv             = uns::getTimerHandle(3);                               // RC receiver timer instance.
const tim_channel_t     tim_chnl_RC_Recv1       = uns::getTimerChannel(3);                              // RC receiver timer channel 1.
const tim_channel_t     tim_chnl_RC_Recv2       = uns::getTimerChannel(4);                              // RC receiver timer channel 2.

tim_handle_t   * const  tim_Rotary              = uns::getTimerHandle(1);                               // Rotary encoder timer instance.
const tim_channel_t     tim_chnl_RotaryA        = uns::getTimerChannel(1);                              // Rotary encoder A timer channel.
const tim_channel_t     tim_chnl_RotaryB        = uns::getTimerChannel(2);                              // Rotary encoder B timer channel.

uart_handle_t  * const  uart_Serial             = uns::getUARTHandle(2);                                // Serial UART instance.

uart_handle_t  * const  uart_Bluetooth          = uns::getUARTHandle(1);                                // Bluetooth UART instance.
const gpio_pin_struct   gpio_BluetoothCONN_STAT = { uns::getGPIO(GPIO::A), uns::getGPIOPin(12)  };      // Bluetooth CONN_STAT pin.
const dma_struct        dma_Bluetooth           = { uns::getDMA(2), uns::getDMA_Handle(DMA::BT) };      // Bluetooth DMA.

uart_handle_t  * const  uart_RadioModule        = uns::getUARTHandle(3);                                // Radio module UART instance.
const dma_struct        dma_RadioModule         = { uns::getDMA(1), uns::getDMA_Handle(DMA::RADIO) };   // Radio module DMA.

i2c_handle_t   * const  i2c_Encoder             = uns::getI2CHandle(1);                                 // Encoder (Nucleo) I2C instance.
const dma_struct        dma_Encoder             = { uns::getDMA(1), uns::getDMA_Handle(DMA::ENCODER) }; // Encoder (Nucleo) DMA.
const gpio_pin_struct   gpio_EncoderReset       = { uns::getGPIO(GPIO::B), uns::getGPIOPin(9)   };      // Encoder (Nucleo) RST pin.

i2c_handle_t   * const  i2c_Dist                = uns::getI2CHandle(1);                                 // Distance sensor I2C instance.
const gpio_pin_struct   gpio_DistINT1           = { uns::getGPIO(GPIO::D), uns::getGPIOPin(2)   };      // Distance sensor interrupt pin 1.
const gpio_pin_struct   gpio_DistINT2           = { uns::getGPIO(GPIO::C), uns::getGPIOPin(12)  };      // Distance sensor interrupt pin 2.
const gpio_pin_struct   gpio_DistINT3           = { uns::getGPIO(GPIO::C), uns::getGPIOPin(11)  };      // Distance sensor interrupt pin 3.
const gpio_pin_struct   gpio_DistSHTD1          = { uns::getGPIO(GPIO::A), uns::getGPIOPin(11)  };      // Distance sensor shutdown pin 1.
const gpio_pin_struct   gpio_DistSHTD2          = { uns::getGPIO(GPIO::H), uns::getGPIOPin(0)   };      // Distance sensor shutdown pin 2.
const gpio_pin_struct   gpio_DistSHTD3          = { uns::getGPIO(GPIO::A), uns::getGPIOPin(15)  };      // Distance sensor shutdown pin 3.

spi_handle_t   * const  spi_LedPanel            = uns::getSPIHandle(1);                                 // LedPanel SPI instance.
const gpio_pin_struct   gpio_LedPanelSel0       = { uns::getGPIO(GPIO::B), uns::getGPIOPin(2)   };      // LedPanel Select 0 GPIO pin.
const gpio_pin_struct   gpio_LedPanelSel1       = { uns::getGPIO(GPIO::B), uns::getGPIOPin(1)   };      // LedPanel Select 1 GPIO pin.
const gpio_pin_struct   gpio_LedPanelSel2       = { uns::getGPIO(GPIO::B), uns::getGPIOPin(15)  };      // LedPanel Select 2 GPIO pin.
const gpio_pin_struct   gpio_LedPanelSel3       = { uns::getGPIO(GPIO::B), uns::getGPIOPin(14)  };      // LedPanel Select 3 GPIO pin.
const gpio_pin_struct   gpio_LedPanelOutEnOpto  = { uns::getGPIO(GPIO::B), uns::getGPIOPin(12)  };      // LedPanel optical output enable GPIO pin.
const gpio_pin_struct   gpio_LedPanelOutEnInd   = { uns::getGPIO(GPIO::B), uns::getGPIOPin(13)  };      // LedPanel indicator LED output enable GPIO pin.

//adc_handle_t   * const  adc_DistSHARP           = uns::getADCHandle(1);                                 // SHARP distance sensor ADC instance.
//const adc_channel_t     adc_chnl_DistSHARP1     = uns::getADCChannel(11);                               // SHARP distance sensor channel 1.
//const adc_channel_t     adc_chnl_DistSHARP2     = uns::getADCChannel(4);                                // SHARP distance sensor channel 2.
//const adc_channel_t     adc_chnl_DistSHARP3     = uns::getADCChannel(8);                                // SHARP distance sensor channel 3.

const gpio_pin_struct   gpio_Switch1            = { uns::getGPIO(GPIO::C), uns::getGPIOPin(13)  };      // Switch1 GPIO pin.
const gpio_pin_struct   gpio_Switch2            = { uns::getGPIO(GPIO::B), uns::getGPIOPin(5)  };       // Switch2 GPIO pin.

const gpio_pin_struct   gpio_Led                = { uns::getGPIO(GPIO::A), uns::getGPIOPin(5)   };      // Nucleo LED GPIO pin.

queue_handle_t * const  queue_Log               = uns::getQueueHandle(QUEUE::LOG);                      // LogQueue handle.
queue_handle_t * const  queue_ControlProps      = uns::getQueueHandle(QUEUE::CONTROL_PROPS);            // ControlPropsQueue handle.

uart_handle_t  * const  uart_Gyro               = uns::getUARTHandle(6);                                // Gyroscope (Arduino) UART instance.
const dma_struct        dma_Gyro                = { uns::getDMA(2), uns::getDMA_Handle(DMA::GYRO) };    // Gyroscope (Arduino) DMA.
const gpio_pin_struct   gpio_GyroReset          = { uns::getGPIO(GPIO::C), uns::getGPIOPin(2)   };      // Gyroscope (Arduino) RST pin.

// EXTRA PINS

const gpio_pin_struct   gpio_Extra1             = { uns::getGPIO(GPIO::C), uns::getGPIOPin(0)   };      // Extra GPIO pin 1.
const gpio_pin_struct   gpio_Extra2             = { uns::getGPIO(GPIO::C), uns::getGPIOPin(3)   };      // Extra GPIO pin 2.
const gpio_pin_struct   gpio_Extra3             = { uns::getGPIO(GPIO::H), uns::getGPIOPin(1)   };      // Extra GPIO pin 3.

} // namespace cfg
} // namespace uns


