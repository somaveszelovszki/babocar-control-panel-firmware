#pragma once

#include <micro/bsp/adc.hpp>
#include <micro/bsp/gpio.hpp>
#include <micro/bsp/i2c.hpp>
#include <micro/bsp/spi.hpp>
#include <micro/bsp/tim.hpp>
#include <micro/bsp/uart.hpp>
#include <micro/bsp/dma.hpp>

namespace cfg {

micro::tim_handle_t   * const  tim_System                  = micro::getTimerHandle(2);  // System timer instance.

micro::tim_handle_t   * const  tim_Servo                   = micro::getTimerHandle(5);  // Servo timer instance.
const micro::tim_channel_t     tim_chnl_Servo1             = micro::getTimerChannel(1); // Servo1 timer channel.
const micro::tim_channel_t     tim_chnl_Servo2             = micro::getTimerChannel(2); // Servo2 timer channel.

micro::uart_handle_t  * const  uart_MotorPanel             = micro::getUARTHandle(1);   // Motor panel UART instance.
micro::uart_handle_t  * const  uart_Serial                 = micro::getUARTHandle(2);   // Serial UART instance.
micro::uart_handle_t  * const  uart_RadioModule            = micro::getUARTHandle(3);   // Radio module UART instance.
micro::uart_handle_t  * const  uart_FrontLineDetectPanel   = micro::getUARTHandle(4);   // Front line detect panel UART instance.
micro::uart_handle_t  * const  uart_RearLineDetectPanel    = micro::getUARTHandle(5);   // Rear line detect panel UART instance.
micro::uart_handle_t  * const  uart_Bluetooth              = micro::getUARTHandle(6);   // Bluetooth UART instance.

const micro::dma_struct        dma_Bluetooth               = { micro::getDMA(2), micro::getDMA_Handle(micro::DMA::BT) };    // Bluetooth DMA.
const micro::dma_struct        dma_RadioModule             = { micro::getDMA(1), micro::getDMA_Handle(micro::DMA::RADIO) }; // Radio module DMA.

} // namespace cfg


