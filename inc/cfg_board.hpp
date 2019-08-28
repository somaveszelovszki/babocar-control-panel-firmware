#pragma once

#include <micro/bsp/adc.hpp>
#include <micro/bsp/gpio.hpp>
#include <micro/bsp/i2c.hpp>
#include <micro/bsp/spi.hpp>
#include <micro/bsp/tim.hpp>
#include <micro/bsp/uart.hpp>
#include <micro/bsp/dma.hpp>

namespace cfg {

extern const micro::tim_handle_t tim_System;

extern const micro::tim_handle_t  tim_SteeringServo;
extern const micro::tim_channel_t tim_chnl_FrontServo;
extern const micro::tim_channel_t tim_chnl_RearServo;

extern const micro::uart_handle_t uart_Command;
extern const micro::uart_handle_t uart_MotorPanel;
extern const micro::uart_handle_t uart_RadioModule;
extern const micro::uart_handle_t uart_FrontLineDetectPanel;
extern const micro::uart_handle_t uart_RearLineDetectPanel;

extern const micro::dma_struct dma_Command;
extern const micro::dma_struct dma_MotorPanel;
extern const micro::dma_struct dma_RadioModule;
extern const micro::dma_struct dma_FrontLineDetectPanel;
extern const micro::dma_struct dma_RearLineDetectPanel;

} // namespace cfg
