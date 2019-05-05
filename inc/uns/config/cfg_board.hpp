#pragma once

#include <uns/bsp/adc.hpp>
#include <uns/bsp/gpio.hpp>
#include <uns/bsp/i2c.hpp>
#include <uns/bsp/spi.hpp>
#include <uns/bsp/tim.hpp>
#include <uns/bsp/uart.hpp>
#include <uns/bsp/dma.hpp>

#define BSP_LIB_HAL     // BSP library: HAL

namespace uns {
namespace cfg {

tim_handle_t   * const  tim_System                  = uns::getTimerHandle(2);                               // System timer instance.

tim_handle_t   * const  tim_Servo                   = uns::getTimerHandle(5);                               // Servo timer instance.
const tim_channel_t     tim_chnl_Servo1             = uns::getTimerChannel(1);                              // Servo1 timer channel.
const tim_channel_t     tim_chnl_Servo2             = uns::getTimerChannel(2);                              // Servo2 timer channel.

uart_handle_t  * const  uart_MotorPanel             = uns::getUARTHandle(1);                                // Motor panel UART instance.
uart_handle_t  * const  uart_Serial                 = uns::getUARTHandle(2);                                // Serial UART instance.
uart_handle_t  * const  uart_RadioModule            = uns::getUARTHandle(3);                                // Radio module UART instance.
uart_handle_t  * const  uart_FrontLineDetectPanel   = uns::getUARTHandle(4);                                // Front line detect panel UART instance.
uart_handle_t  * const  uart_RearLineDetectPanel    = uns::getUARTHandle(5);                                // Rear line detect panel UART instance.
uart_handle_t  * const  uart_Bluetooth              = uns::getUARTHandle(6);                                // Bluetooth UART instance.

const dma_struct        dma_Bluetooth               = { uns::getDMA(2), uns::getDMA_Handle(DMA::BT) };      // Bluetooth DMA.
const dma_struct        dma_RadioModule             = { uns::getDMA(1), uns::getDMA_Handle(DMA::RADIO) };   // Radio module DMA.

} // namespace cfg
} // namespace uns


