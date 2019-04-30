#include <config/cfg_board.hpp>

#ifdef BSP_LIB_HAL

#include <uns/util/types.h>
#include <uns/bsp/gpio.hpp>
#include <uns/bsp/tim.hpp>
#include <uns/bsp/adc.hpp>
#include <uns/bsp/i2c.hpp>
#include <uns/bsp/spi.hpp>
#include <uns/bsp/uart.hpp>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"

using namespace uns;

// TIMER

uns::time_t uns::getTime() {
    return uns::time_t::from<milliseconds>(HAL_GetTick());
}

uns::time_t uns::getExactTime() {
    return uns::getTime() + uns::time_t::from<microseconds>(uns::getTimerCounter(cfg::tim_System));
}

namespace {
/* @brief Converts HAL status to uns::Status.
 * @param status The HAL status.
 * @returns The status as an uns::Status.
 **/
Status toStatus(HAL_StatusTypeDef status) {
    return static_cast<Status>(status);
}
} // namespace

extern "C" TIM_HandleTypeDef  htim1;            // TIM1 handle
extern "C" TIM_HandleTypeDef  htim2;            // TIM2 handle
extern "C" TIM_HandleTypeDef  htim3;            // TIM3 handle
extern "C" TIM_HandleTypeDef  htim5;            // TIM5 handle
extern "C" TIM_HandleTypeDef  htim8;            // TIM8 handle
extern "C" UART_HandleTypeDef huart1;           // UART1 handle
extern "C" DMA_HandleTypeDef  hdma_usart1_rx;   // DMA-USART1 handle.
extern "C" DMA_HandleTypeDef  hdma_usart3_rx;   // DMA-USART3 handle.
extern "C" DMA_HandleTypeDef  hdma_usart6_rx;   // DMA-USART6 handle.
extern "C" DMA_HandleTypeDef hdma_i2c1_rx;      // DMA-I2C1 handle.
extern "C" UART_HandleTypeDef huart2;           // UART2 handle
extern "C" UART_HandleTypeDef huart3;           // UART3 handle
extern "C" UART_HandleTypeDef huart6;           // UART6 handle
//extern "C" ADC_HandleTypeDef  hadc1;            // ADC1 handle
extern "C" SPI_HandleTypeDef  hspi1;            // SPI1 handle
extern "C" I2C_HandleTypeDef  hi2c1;            // I2C1 handle

/* @brief Concatenates values.
 * @param a The first value.
 * @param b The second value.
 **/
#define CONCAT(a,b) a##b

// GPIO

#define GPIO_INSTANCE(id)   CONCAT(GPIO,id)         // Creates GPIO instance name.
#define GPIO_PIN(id)        CONCAT(GPIO_PIN_,id)    // Creates GPIO pin name.

gpio_handle_t* uns::getGPIO(GPIO gpio) {
    GPIO_TypeDef *GPIOx = nullptr;
    switch (gpio) {
    case GPIO::A: GPIOx = GPIO_INSTANCE(A); break;
    case GPIO::B: GPIOx = GPIO_INSTANCE(B); break;
    case GPIO::C: GPIOx = GPIO_INSTANCE(C); break;
    case GPIO::D: GPIOx = GPIO_INSTANCE(D); break;
    case GPIO::E: GPIOx = GPIO_INSTANCE(E); break;
    case GPIO::F: GPIOx = GPIO_INSTANCE(F); break;
    case GPIO::G: GPIOx = GPIO_INSTANCE(G); break;
    case GPIO::H: GPIOx = GPIO_INSTANCE(H); break;
    }
    return GPIOx;
}

gpio_pin_t uns::getGPIOPin(res_id_t pin) {
    gpio_pin_t gpioPin = 0;
    switch (pin) {
    case 0:  gpioPin = GPIO_PIN(0);  break;
    case 1:  gpioPin = GPIO_PIN(1);  break;
    case 2:  gpioPin = GPIO_PIN(2);  break;
    case 3:  gpioPin = GPIO_PIN(3);  break;
    case 4:  gpioPin = GPIO_PIN(4);  break;
    case 5:  gpioPin = GPIO_PIN(5);  break;
    case 6:  gpioPin = GPIO_PIN(6);  break;
    case 7:  gpioPin = GPIO_PIN(7);  break;
    case 8:  gpioPin = GPIO_PIN(8);  break;
    case 9:  gpioPin = GPIO_PIN(9);  break;
    case 10: gpioPin = GPIO_PIN(10); break;
    case 11: gpioPin = GPIO_PIN(11); break;
    case 12: gpioPin = GPIO_PIN(12); break;
    case 13: gpioPin = GPIO_PIN(13); break;
    case 14: gpioPin = GPIO_PIN(14); break;
    case 15: gpioPin = GPIO_PIN(15); break;
    }
    return gpioPin;
}

void uns::GPIO_WritePin(const gpio_pin_struct& gpio, PinState pinState) {
    HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(gpio.handle), static_cast<uint16_t>(gpio.pin), static_cast<GPIO_PinState>(pinState));
}

PinState uns::GPIO_ReadPin(const gpio_pin_struct& gpio) {
    return static_cast<PinState>(HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef*>(gpio.handle), static_cast<uint16_t>(gpio.pin)));
}

void uns::GPIO_TogglePin(const gpio_pin_struct& gpio) {
    HAL_GPIO_TogglePin(static_cast<GPIO_TypeDef*>(gpio.handle), static_cast<uint16_t>(gpio.pin));
}

// TIMER

#define TIMER_HANDLE(id)    CONCAT(htim,id)         // Creates timer handle name.
#define TIMER_CHANNEL(id)   CONCAT(TIM_CHANNEL_,id) // Creates timer channel name.

void uns::blockingDelay(time_t delay) {
    HAL_Delay(delay.get<milliseconds>());
}

tim_handle_t* uns::getTimerHandle(res_id_t id) {
    TIM_HandleTypeDef *htim = nullptr;
    switch (id) {
    case 1: htim = &TIMER_HANDLE(1); break;
    case 2: htim = &TIMER_HANDLE(2); break;
    case 3: htim = &TIMER_HANDLE(3); break;
//    case 4: htim = &TIMER_HANDLE(4); break;
    case 5: htim = &TIMER_HANDLE(5); break;
//    case 6: htim = &TIMER_HANDLE(6); break;
//    case 7: htim = &TIMER_HANDLE(7); break;
    case 8: htim = &TIMER_HANDLE(8); break;
    }
    return htim;
}

tim_channel_t uns::getTimerChannel(res_id_t id) {
    tim_channel_t chnl = 0;
    switch (id) {
    case 1: chnl = TIMER_CHANNEL(1); break;
    case 2: chnl = TIMER_CHANNEL(2); break;
    case 3: chnl = TIMER_CHANNEL(3); break;
    case 4: chnl = TIMER_CHANNEL(4); break;
    }
    return chnl;
}

Status uns::writePWM(tim_handle_t * const htim, tim_channel_t channel, uint32_t duty) {
    __HAL_TIM_SET_COMPARE(static_cast<TIM_HandleTypeDef*>(htim), channel, duty);
    return Status::OK;
}

uint32_t uns::getTimerCounter(tim_handle_t * const htim) {
    return __HAL_TIM_GET_COUNTER(static_cast<TIM_HandleTypeDef*>(htim));
}

void uns::setTimerCounter(tim_handle_t * const htim, uint32_t cntr) {
    __HAL_TIM_SET_COUNTER(static_cast<TIM_HandleTypeDef*>(htim), cntr);
}

uint32_t uns::getTimerCompare(tim_handle_t * const htim, uint32_t channel) {
    return __HAL_TIM_GET_COMPARE(static_cast<TIM_HandleTypeDef*>(htim), channel);
}

// ADC

#define ADC_HANDLE(id)    CONCAT(hadc,id)         // Creates ADC handle name.
#define ADC_CHANNEL(id)   CONCAT(ADC_CHANNEL_,id) // Creates ADC channel name.

adc_handle_t* uns::getADCHandle(res_id_t id) {
    ADC_HandleTypeDef *hadc = nullptr;
    switch (id) {
//    case 1: hadc = &ADC_HANDLE(1); break;
//    case 2: hadc = &TIMER_HANDLE(2); break;
//    case 3: hadc = &TIMER_HANDLE(3); break;
    }
    return hadc;
}

adc_channel_t uns::getADCChannel(res_id_t id) {
    adc_channel_t chnl = 0;
    switch (id) {
    case 1:  chnl = ADC_CHANNEL(1);  break;
    case 2:  chnl = ADC_CHANNEL(2);  break;
    case 3:  chnl = ADC_CHANNEL(3);  break;
    case 4:  chnl = ADC_CHANNEL(4);  break;
    case 5:  chnl = ADC_CHANNEL(5);  break;
    case 6:  chnl = ADC_CHANNEL(6);  break;
    case 7:  chnl = ADC_CHANNEL(7);  break;
    case 8:  chnl = ADC_CHANNEL(8);  break;
    case 9:  chnl = ADC_CHANNEL(9);  break;
    case 10: chnl = ADC_CHANNEL(10); break;
    case 11: chnl = ADC_CHANNEL(11); break;
    case 12: chnl = ADC_CHANNEL(12); break;
    case 13: chnl = ADC_CHANNEL(13); break;
    case 14: chnl = ADC_CHANNEL(14); break;
    case 15: chnl = ADC_CHANNEL(15); break;
    case 16: chnl = ADC_CHANNEL(16); break;
    case 17: chnl = ADC_CHANNEL(17); break;
    case 18: chnl = ADC_CHANNEL(18); break;
    }
    return chnl;
}

Status uns::ADC_SetChannel(adc_handle_t * const hadc, adc_channel_t channel) {
    ADC_ChannelConfTypeDef sConfig = { channel, 1, ADC_SAMPLETIME_3CYCLES, 0 };
    return toStatus(HAL_ADC_ConfigChannel(static_cast<ADC_HandleTypeDef*>(hadc), &sConfig));
}

Status uns::ADC_ReadValue(adc_handle_t * const hadc, uint32_t *pResult) {
    Status status;
    ADC_HandleTypeDef *_hadc = static_cast<ADC_HandleTypeDef*>(hadc);

    if (isOk(status = toStatus(HAL_ADC_Start(_hadc)))
        && isOk(status = toStatus(HAL_ADC_PollForConversion(_hadc, 1)))) {

        *pResult = HAL_ADC_GetValue(_hadc);
        status = toStatus(HAL_ADC_Stop(_hadc));
    }

    return status;
}

// I2C

#define I2C_HANDLE(id)      CONCAT(hi2c,id)     // Creates I2C handle name.

i2c_handle_t* uns::getI2CHandle(res_id_t id) {
    I2C_HandleTypeDef *pI2C = nullptr;
    switch (id) {
    case 1: pI2C = &I2C_HANDLE(1); break;
//    case 2: pI2C = &I2C_HANDLE(2); break;
    }
    return pI2C;
}

Status uns::I2C_IsDeviceReady(i2c_handle_t * const hi2c, uint16_t address, uint32_t trials, uns::time_t timeout) {
    return toStatus(HAL_I2C_IsDeviceReady(static_cast<I2C_HandleTypeDef*>(hi2c), address, trials, timeout.get<milliseconds>()));
}

Status uns::I2C_Master_Transmit(i2c_handle_t * const hi2c, uint16_t address, const uint8_t * const txBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_I2C_Master_Transmit(static_cast<I2C_HandleTypeDef*>(hi2c), address, const_cast<uint8_t*>(txBuffer), size, timeout.get<milliseconds>()));
}

Status uns::I2C_Master_Transmit_IT(i2c_handle_t * const hi2c, uint16_t address, const uint8_t * const txBuffer, uint32_t size) {
    return toStatus(HAL_I2C_Master_Transmit_IT(static_cast<I2C_HandleTypeDef*>(hi2c), address, const_cast<uint8_t*>(txBuffer), size));
}

Status uns::I2C_Mem_Write(i2c_handle_t * const hi2c, uint16_t address, uint16_t memAddress, uint16_t memAddressSize, const uint8_t * const txBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_I2C_Mem_Write(static_cast<I2C_HandleTypeDef*>(hi2c), address, memAddress, memAddressSize, const_cast<uint8_t*>(txBuffer), size, timeout.get<milliseconds>()));
}

Status uns::I2C_Master_Receive(i2c_handle_t * const hi2c, uint16_t address, uint8_t * const rxBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_I2C_Master_Receive(static_cast<I2C_HandleTypeDef*>(hi2c), address, rxBuffer, size, timeout.get<milliseconds>()));
}

Status uns::I2C_Master_Receive_IT(i2c_handle_t * const hi2c, uint16_t address, uint8_t * const rxBuffer, uint32_t size) {
    return toStatus(HAL_I2C_Master_Receive_IT(static_cast<I2C_HandleTypeDef*>(hi2c), address, rxBuffer, size));
}

Status uns::I2C_Mem_Read(i2c_handle_t * const hi2c, uint16_t address, uint16_t memAddress, uint16_t memAddressSize, uint8_t * const rxBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_I2C_Mem_Read(static_cast<I2C_HandleTypeDef*>(hi2c), address, memAddress, memAddressSize, rxBuffer, size, timeout.get<milliseconds>()));
}

Status uns::I2C_Slave_Receive_DMA(i2c_handle_t * const hi2c, uint8_t * const rxBuffer, uint32_t size) {
    return toStatus(HAL_I2C_Slave_Receive_DMA(static_cast<I2C_HandleTypeDef*>(hi2c), rxBuffer, size));
}

Status uns::I2C_Slave_Receive(i2c_handle_t * const hi2c, uint8_t * const rxBuffer, uint32_t size, time_t timeout) {
    return toStatus(HAL_I2C_Slave_Receive(static_cast<I2C_HandleTypeDef*>(hi2c), rxBuffer, size, timeout.get<milliseconds>()));
}

// SPI

namespace {
/* @brief Converts HAL status to uns::Status.
 * @param status The HAL status.
 * @returns The status as an uns::Status.
 **/
Status toStatus(HAL_SPI_StateTypeDef status) {
    Status result;
    switch(status) {
    case HAL_SPI_STATE_RESET:   result = Status::OK;    break;
    case HAL_SPI_STATE_READY:   result = Status::OK;    break;
    case HAL_SPI_STATE_ERROR:   result = Status::ERROR; break;
    default:                    result = Status::BUSY;  break;
    }
    return result;
}
} // namespace

#define SPI_HANDLE(id)      CONCAT(hspi,id)     // Creates SPI handle name.

spi_handle_t* uns::getSPIHandle(res_id_t id) {
    SPI_HandleTypeDef *pSPI = nullptr;
    switch (id) {
    case 1: pSPI = &SPI_HANDLE(1); break;
//    case 2: pSPI = &SPI_HANDLE(2); break;
//    case 3: pSPI = &SPI_HANDLE(3); break;
    }
    return pSPI;
}

Status uns::SPI_Receive(spi_handle_t * const hspi, uint8_t * const rxBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_SPI_Receive(static_cast<SPI_HandleTypeDef*>(hspi), rxBuffer, size, timeout.get<milliseconds>()));
}

Status uns::SPI_Transmit(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint32_t size, uns::time_t timeout) {
    Status status = toStatus(HAL_SPI_Transmit(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), size, timeout.get<milliseconds>()));
    if (isOk(status)) {
        while(HAL_SPI_GetState(static_cast<SPI_HandleTypeDef*>(hspi)) != HAL_SPI_STATE_READY) {}
    }
    return status;
}

Status uns::SPI_TransmitReceive(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint8_t * const rxBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_SPI_TransmitReceive(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), rxBuffer, size, timeout.get<milliseconds>()));
}

Status uns::SPI_Receive_IT(spi_handle_t * const hspi, uint8_t * const rxBuffer, uint32_t size) {
    return toStatus(HAL_SPI_Receive_IT(static_cast<SPI_HandleTypeDef*>(hspi), rxBuffer, size));
}

Status uns::SPI_Transmit_IT(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint32_t size) {
    return toStatus(HAL_SPI_Transmit_IT(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), size));
}

Status uns::SPI_TransmitReceive_IT(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint8_t * const rxBuffer, uint32_t size) {
    return toStatus(HAL_SPI_TransmitReceive_IT(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), rxBuffer, size));
}

Status uns::SPI_GetState(spi_handle_t * const hspi) {
    return toStatus(HAL_SPI_GetState(static_cast<SPI_HandleTypeDef*>(hspi)));
}

void uns::SPI_SetReady(spi_handle_t * const hspi) {
    static_cast<SPI_HandleTypeDef*>(hspi)->State = HAL_SPI_STATE_READY;
}

// UART

#define UART_HANDLE(id)      CONCAT(huart,id)   // Creates UART handle name.

uart_handle_t* uns::getUARTHandle(res_id_t id) {
    UART_HandleTypeDef *pUART = nullptr;
    switch (id) {
    case 1: pUART = &UART_HANDLE(1); break;
    case 2: pUART = &UART_HANDLE(2); break;
    case 3: pUART = &UART_HANDLE(3); break;
//    case 4: pUART = &UART_HANDLE(4); break;
//    case 5: pUART = &UART_HANDLE(5); break;
    case 6: pUART = &UART_HANDLE(6); break;
    }
    return pUART;
}

Status uns::UART_Receive(uart_handle_t * const huart, uint8_t * const rxBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_UART_Receive(static_cast<UART_HandleTypeDef*>(huart), rxBuffer, size, timeout.get<milliseconds>()));
}

Status uns::UART_Receive_IT(uart_handle_t * const huart, uint8_t * const rxBuffer, uint32_t size) {
    return toStatus(HAL_UART_Receive_IT(static_cast<UART_HandleTypeDef*>(huart), rxBuffer, size));
}

Status uns::UART_Receive_DMA(uart_handle_t * const huart, uint8_t * const rxBuffer, uint32_t size) {
    return toStatus(HAL_UART_Receive_DMA(static_cast<UART_HandleTypeDef*>(huart), rxBuffer, size));
}

Status uns::UART_Transmit(uart_handle_t * const huart, const uint8_t * const txBuffer, uint32_t size, uns::time_t timeout) {
    return toStatus(HAL_UART_Transmit(static_cast<UART_HandleTypeDef*>(huart), const_cast<uint8_t*>(txBuffer), size, timeout.get<milliseconds>()));
}

Status uns::UART_Transmit_IT(uart_handle_t * const huart, const uint8_t * const txBuffer, uint32_t size) {
    return toStatus(HAL_UART_Transmit_IT(static_cast<UART_HandleTypeDef*>(huart), const_cast<uint8_t*>(txBuffer), size));
}

// DMA

#define DMA_INSTANCE(id)      CONCAT(DMA,id)   // Creates DMA instance name.

dma_t* uns::getDMA(res_id_t dma) {
    DMA_TypeDef *pDMA = nullptr;
    switch (dma) {
    case 1: pDMA = DMA_INSTANCE(1); break;
    case 2: pDMA = DMA_INSTANCE(2); break;
    }
    return pDMA;
}

dma_handle_t* uns::getDMA_Handle(uns::DMA dma) {
    DMA_HandleTypeDef *pDMA = nullptr;
    switch(dma) {
    case DMA::BT:       pDMA = &hdma_usart1_rx; break;
    case DMA::RADIO:    pDMA = &hdma_usart3_rx; break;
    case DMA::ENCODER:  pDMA = &hdma_i2c1_rx; break;
    }
    return pDMA;
}

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void uns_Serial_Uart_TxCpltCallback();       // Callback for Serial UART TxCplt - called when transfer finishes.
extern void uns_RadioModule_Uart_RxCpltCallback();  // Callback for RadioModule UART RxCplt - called when receive finishes.
extern void uns_Gyro_Uart_RxCpltCallback();         // Callback for Gyro (Arduino) UART RxCplt - called when receive finishes.
extern void uns_Bluetooth_Uart_RxCpltCallback(uint32_t len);    // Callback for Bluetooth UART RxCplt - called when receive finishes.
extern void uns_Bluetooth_Uart_TxCpltCallback();    // Callback for Bluetooth UART TxCplt - called when transfer finishes.extern void uns_LedPanel_Spi_TxRxCpltCallback();    // Callback for LedPanel SPI TxRxCplt - called when exchange finishes.
extern void uns_Encoder_I2C_RxCpltCallback();       // Callback for Encoder (Nucleo) I2C RxCplt - called when receive finishes.
extern void uns_LedPanel_Spi_TxRxCpltCallback();    // Callback for LedPanel SPI TxRxCplt - called when exchange finishes.
extern void uns_RcRecv_Tim_IC_CaptureCallback();    // Callback for RcRecv Timer Capture - called when PWM value is captured.
extern void uns_Dist2_EchoCallback();               // Callback for Distance sensor #2 echo.

/* @brief Internal callback - called when SPI reception finishes.
 * @param hspi Pointer to the SPI handle.
 **/
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

}

/* @brief Internal callback - called when SPI transmission finishes.
 * @param hspi Pointer to the SPI handle.
 **/
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {

}

/* @brief Internal callback - called when SPI exchange finishes.
 * @param hspi Pointer to the SPI handle.
 **/
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == cfg::spi_LedPanel) {
        uns_LedPanel_Spi_TxRxCpltCallback();
    }
}

/* @brief Internal callback - called when I2C reception finishes.
 * @param hi2c Pointer to the I2C handle.
 **/
extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == cfg::i2c_Encoder) {
        uns_Encoder_I2C_RxCpltCallback();
    }
}


/* @brief Internal callback - called when UAR receive finishes.
 * @param huart Pointer to the UART handle.
 **/
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == cfg::uart_Gyro) {
        uns_Gyro_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_Bluetooth) {
        uint32_t bytes = MAX_RX_BUFFER_SIZE - ((DMA_HandleTypeDef*)cfg::dma_Bluetooth.handle)->Instance->NDTR;
        uns_Bluetooth_Uart_RxCpltCallback(bytes);
        ((DMA_TypeDef*)cfg::dma_Bluetooth.instance)->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;    // clears DMA flags before next transfer
        ((DMA_HandleTypeDef*)cfg::dma_Bluetooth.handle)->Instance->NDTR = MAX_RX_BUFFER_SIZE; // sets number of bytes to receive
        ((DMA_HandleTypeDef*)cfg::dma_Bluetooth.handle)->Instance->CR |= DMA_SxCR_EN;         // starts DMA transfer

    } else if (huart == cfg::uart_RadioModule) {
        uns_RadioModule_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_Serial) {
        // does not handle Serial RxCplt
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == cfg::uart_Bluetooth) {
        uns_Bluetooth_Uart_TxCpltCallback();
    } else if (huart == cfg::uart_Serial) {
        uns_Serial_Uart_TxCpltCallback();
    }
}

/* @brief Callback function for timer period elapsed event.
 * @param htim Pointer to the timer handle.
 **/
extern "C" void uns_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == cfg::tim_IT_100US) {
//        uns_Rotary_PeriodElapsedCallback();
//    }
}

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 if (htim == cfg::tim_RC_Recv)
  {
     uns_RcRecv_Tim_IC_CaptureCallback();
     uns::setTimerCounter(cfg::tim_RC_Recv, 0); //resets counter after input capture interrupt occurs
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (static_cast<gpio_pin_t>(GPIO_Pin) == cfg::gpio_DistINT2.pin) {
        uns_Dist2_EchoCallback();
    }
}

#endif // BSP_LIB_HAL
