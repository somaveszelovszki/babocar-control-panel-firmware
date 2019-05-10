#include <uns/config/cfg_board.hpp>

#ifdef BSP_LIB_HAL

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

namespace {

/* @brief Converts HAL status to Status.
 * @param status The HAL status.
 * @returns The status as an Status.
 **/
Status toStatus(HAL_StatusTypeDef status) {
    return static_cast<Status>(status);
}

/* @brief Converts HAL status to Status.
 * @param status The HAL status.
 * @returns The status as an Status.
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

namespace uns {

// TIMER

millisecond_t getTime() {
    return millisecond_t(HAL_GetTick());
}

microsecond_t getExactTime() {
    return getTime() + microsecond_t(getTimerCounter(cfg::tim_System));
}

extern "C" TIM_HandleTypeDef  htim1;
extern "C" TIM_HandleTypeDef  htim5;
extern "C" UART_HandleTypeDef huart4;
extern "C" UART_HandleTypeDef huart5;
extern "C" UART_HandleTypeDef huart1;
extern "C" UART_HandleTypeDef huart2;
extern "C" UART_HandleTypeDef huart3;
extern "C" UART_HandleTypeDef huart6;
extern "C" DMA_HandleTypeDef hdma_uart4_rx;
extern "C" DMA_HandleTypeDef hdma_uart5_rx;
extern "C" DMA_HandleTypeDef hdma_usart1_rx;
extern "C" DMA_HandleTypeDef hdma_usart1_tx;
extern "C" DMA_HandleTypeDef hdma_usart3_rx;
extern "C" DMA_HandleTypeDef hdma_usart6_rx;
extern "C" DMA_HandleTypeDef hdma_usart6_tx;
extern "C" I2C_HandleTypeDef hi2c1;

/* @brief Concatenates values.
 * @param a The first value.
 * @param b The second value.
 **/
#define CONCAT(a,b) a##b

// GPIO

#define GPIO_INSTANCE(id)   CONCAT(GPIO,id)         // Creates GPIO instance name.
#define GPIO_PIN(id)        CONCAT(GPIO_PIN_,id)    // Creates GPIO pin name.

gpio_handle_t* getGPIO(GPIO gpio) {
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

gpio_pin_t getGPIOPin(res_id_t pin) {
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

void GPIO_WritePin(const gpio_pin_struct& gpio, PinState pinState) {
    HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(gpio.handle), static_cast<uint16_t>(gpio.pin), static_cast<GPIO_PinState>(pinState));
}

PinState GPIO_ReadPin(const gpio_pin_struct& gpio) {
    return static_cast<PinState>(HAL_GPIO_ReadPin(static_cast<GPIO_TypeDef*>(gpio.handle), static_cast<uint16_t>(gpio.pin)));
}

void GPIO_TogglePin(const gpio_pin_struct& gpio) {
    HAL_GPIO_TogglePin(static_cast<GPIO_TypeDef*>(gpio.handle), static_cast<uint16_t>(gpio.pin));
}

// TIMER

#define TIMER_HANDLE(id)    CONCAT(htim,id)         // Creates timer handle name.
#define TIMER_CHANNEL(id)   CONCAT(TIM_CHANNEL_,id) // Creates timer channel name.

void blockingDelay(millisecond_t delay) {
    HAL_Delay(delay.get());
}

tim_handle_t* getTimerHandle(res_id_t id) {
    TIM_HandleTypeDef *htim = nullptr;
    switch (id) {
    case 1: htim = &TIMER_HANDLE(1); break;
//    case 2: htim = &TIMER_HANDLE(2); break;
//    case 3: htim = &TIMER_HANDLE(3); break;
//    case 4: htim = &TIMER_HANDLE(4); break;
    case 5: htim = &TIMER_HANDLE(5); break;
//    case 6: htim = &TIMER_HANDLE(6); break;
//    case 7: htim = &TIMER_HANDLE(7); break;
//    case 8: htim = &TIMER_HANDLE(8); break;
    }
    return htim;
}

tim_channel_t getTimerChannel(res_id_t id) {
    tim_channel_t chnl = 0;
    switch (id) {
    case 1: chnl = TIMER_CHANNEL(1); break;
    case 2: chnl = TIMER_CHANNEL(2); break;
    case 3: chnl = TIMER_CHANNEL(3); break;
    case 4: chnl = TIMER_CHANNEL(4); break;
    }
    return chnl;
}

Status writePWM(tim_handle_t *htim, tim_channel_t channel, uint32_t duty) {
    __HAL_TIM_SET_COMPARE(static_cast<TIM_HandleTypeDef*>(htim), channel, duty);
    return Status::OK;
}

uint32_t getTimerCounter(tim_handle_t *htim) {
    return __HAL_TIM_GET_COUNTER(static_cast<TIM_HandleTypeDef*>(htim));
}

void setTimerCounter(tim_handle_t *htim, uint32_t cntr) {
    __HAL_TIM_SET_COUNTER(static_cast<TIM_HandleTypeDef*>(htim), cntr);
}

uint32_t getTimerCompare(tim_handle_t *htim, uint32_t channel) {
    return __HAL_TIM_GET_COMPARE(static_cast<TIM_HandleTypeDef*>(htim), channel);
}

// ADC

#define ADC_HANDLE(id)    CONCAT(hadc,id)         // Creates ADC handle name.
#define ADC_CHANNEL(id)   CONCAT(ADC_CHANNEL_,id) // Creates ADC channel name.

adc_handle_t* getADCHandle(res_id_t id) {
    ADC_HandleTypeDef *hadc = nullptr;
    switch (id) {
//    case 1: hadc = &ADC_HANDLE(1); break;
//    case 2: hadc = &TIMER_HANDLE(2); break;
//    case 3: hadc = &TIMER_HANDLE(3); break;
    }
    return hadc;
}

adc_channel_t getADCChannel(res_id_t id) {
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

Status ADC_SetChannel(adc_handle_t *hadc, adc_channel_t channel) {
    ADC_ChannelConfTypeDef sConfig = { channel, 1, ADC_SAMPLETIME_3CYCLES, 0 };
    return toStatus(HAL_ADC_ConfigChannel(static_cast<ADC_HandleTypeDef*>(hadc), &sConfig));
}

Status ADC_ReadValue(adc_handle_t *hadc, uint32_t *pResult) {
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

i2c_handle_t* getI2CHandle(res_id_t id) {
    I2C_HandleTypeDef *pI2C = nullptr;
    switch (id) {
    case 1: pI2C = &I2C_HANDLE(1); break;
//    case 2: pI2C = &I2C_HANDLE(2); break;
    }
    return pI2C;
}

Status I2C_IsDeviceReady(i2c_handle_t *hi2c, uint16_t address, uint32_t trials, millisecond_t timeout) {
    return toStatus(HAL_I2C_IsDeviceReady(static_cast<I2C_HandleTypeDef*>(hi2c), address, trials, timeout.get()));
}

Status I2C_Master_Transmit(i2c_handle_t *hi2c, uint16_t address, const uint8_t *txBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_I2C_Master_Transmit(static_cast<I2C_HandleTypeDef*>(hi2c), address, const_cast<uint8_t*>(txBuffer), size, timeout.get()));
}

Status I2C_Master_Transmit_IT(i2c_handle_t *hi2c, uint16_t address, const uint8_t *txBuffer, uint32_t size) {
    return toStatus(HAL_I2C_Master_Transmit_IT(static_cast<I2C_HandleTypeDef*>(hi2c), address, const_cast<uint8_t*>(txBuffer), size));
}

Status I2C_Mem_Write(i2c_handle_t *hi2c, uint16_t address, uint16_t memAddress, uint16_t memAddressSize, const uint8_t *txBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_I2C_Mem_Write(static_cast<I2C_HandleTypeDef*>(hi2c), address, memAddress, memAddressSize, const_cast<uint8_t*>(txBuffer), size, timeout.get()));
}

Status I2C_Master_Receive(i2c_handle_t *hi2c, uint16_t address, uint8_t *rxBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_I2C_Master_Receive(static_cast<I2C_HandleTypeDef*>(hi2c), address, rxBuffer, size, timeout.get()));
}

Status I2C_Master_Receive_IT(i2c_handle_t *hi2c, uint16_t address, uint8_t *rxBuffer, uint32_t size) {
    return toStatus(HAL_I2C_Master_Receive_IT(static_cast<I2C_HandleTypeDef*>(hi2c), address, rxBuffer, size));
}

Status I2C_Mem_Read(i2c_handle_t *hi2c, uint16_t address, uint16_t memAddress, uint16_t memAddressSize, uint8_t *rxBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_I2C_Mem_Read(static_cast<I2C_HandleTypeDef*>(hi2c), address, memAddress, memAddressSize, rxBuffer, size, timeout.get()));
}

Status I2C_Slave_Receive_DMA(i2c_handle_t *hi2c, uint8_t *rxBuffer, uint32_t size) {
    return toStatus(HAL_I2C_Slave_Receive_DMA(static_cast<I2C_HandleTypeDef*>(hi2c), rxBuffer, size));
}

Status I2C_Slave_Receive(i2c_handle_t *hi2c, uint8_t *rxBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_I2C_Slave_Receive(static_cast<I2C_HandleTypeDef*>(hi2c), rxBuffer, size, timeout.get()));
}

// SPI

#define SPI_HANDLE(id)      CONCAT(hspi,id)     // Creates SPI handle name.

spi_handle_t* getSPIHandle(res_id_t id) {
    SPI_HandleTypeDef *pSPI = nullptr;
    switch (id) {
//    case 1: pSPI = &SPI_HANDLE(1); break;
//    case 2: pSPI = &SPI_HANDLE(2); break;
//    case 3: pSPI = &SPI_HANDLE(3); break;
    }
    return pSPI;
}

Status SPI_Receive(spi_handle_t *hspi, uint8_t *rxBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_SPI_Receive(static_cast<SPI_HandleTypeDef*>(hspi), rxBuffer, size, timeout.get()));
}

Status SPI_Transmit(spi_handle_t *hspi, const uint8_t *txBuffer, uint32_t size, millisecond_t timeout) {
    Status status = toStatus(HAL_SPI_Transmit(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), size, timeout.get()));
    if (isOk(status)) {
        while(HAL_SPI_GetState(static_cast<SPI_HandleTypeDef*>(hspi)) != HAL_SPI_STATE_READY) {}
    }
    return status;
}

Status SPI_TransmitReceive(spi_handle_t *hspi, const uint8_t *txBuffer, uint8_t *rxBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_SPI_TransmitReceive(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), rxBuffer, size, timeout.get()));
}

Status SPI_Receive_IT(spi_handle_t *hspi, uint8_t *rxBuffer, uint32_t size) {
    return toStatus(HAL_SPI_Receive_IT(static_cast<SPI_HandleTypeDef*>(hspi), rxBuffer, size));
}

Status SPI_Transmit_IT(spi_handle_t *hspi, const uint8_t *txBuffer, uint32_t size) {
    return toStatus(HAL_SPI_Transmit_IT(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), size));
}

Status SPI_TransmitReceive_IT(spi_handle_t *hspi, const uint8_t *txBuffer, uint8_t *rxBuffer, uint32_t size) {
    return toStatus(HAL_SPI_TransmitReceive_IT(static_cast<SPI_HandleTypeDef*>(hspi), const_cast<uint8_t*>(txBuffer), rxBuffer, size));
}

Status SPI_GetState(spi_handle_t *hspi) {
    return toStatus(HAL_SPI_GetState(static_cast<SPI_HandleTypeDef*>(hspi)));
}

void SPI_SetReady(spi_handle_t *hspi) {
    static_cast<SPI_HandleTypeDef*>(hspi)->State = HAL_SPI_STATE_READY;
}

// UART

#define UART_HANDLE(id)      CONCAT(huart,id)   // Creates UART handle name.

uart_handle_t* getUARTHandle(res_id_t id) {
    UART_HandleTypeDef *pUART = nullptr;
    switch (id) {
    case 1: pUART = &UART_HANDLE(1); break;
    case 2: pUART = &UART_HANDLE(2); break;
    case 3: pUART = &UART_HANDLE(3); break;
    case 4: pUART = &UART_HANDLE(4); break;
    case 5: pUART = &UART_HANDLE(5); break;
    case 6: pUART = &UART_HANDLE(6); break;
    }
    return pUART;
}

Status UART_Receive(uart_handle_t *huart, uint8_t *rxBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_UART_Receive(static_cast<UART_HandleTypeDef*>(huart), rxBuffer, size, timeout.get()));
}

Status UART_Receive_IT(uart_handle_t *huart, uint8_t *rxBuffer, uint32_t size) {
    return toStatus(HAL_UART_Receive_IT(static_cast<UART_HandleTypeDef*>(huart), rxBuffer, size));
}

Status UART_Receive_DMA(uart_handle_t *huart, uint8_t *rxBuffer, uint32_t size) {
    return toStatus(HAL_UART_Receive_DMA(static_cast<UART_HandleTypeDef*>(huart), rxBuffer, size));
}

Status UART_Transmit(uart_handle_t *huart, const uint8_t *txBuffer, uint32_t size, millisecond_t timeout) {
    return toStatus(HAL_UART_Transmit(static_cast<UART_HandleTypeDef*>(huart), const_cast<uint8_t*>(txBuffer), size, timeout.get()));
}

Status UART_Transmit_IT(uart_handle_t *huart, const uint8_t *txBuffer, uint32_t size) {
    return toStatus(HAL_UART_Transmit_IT(static_cast<UART_HandleTypeDef*>(huart), const_cast<uint8_t*>(txBuffer), size));
}

Status UART_Transmit_DMA(uart_handle_t *huart, const uint8_t *txBuffer, uint32_t size) {
    return toStatus(HAL_UART_Transmit_DMA(static_cast<UART_HandleTypeDef*>(huart), const_cast<uint8_t*>(txBuffer), size));
}

Status UART_Stop_DMA(uart_handle_t *huart) {
    return toStatus(HAL_UART_DMAStop(static_cast<UART_HandleTypeDef*>(huart)));
}

// DMA

#define DMA_INSTANCE(id)      CONCAT(DMA,id)   // Creates DMA instance name.

dma_t* getDMA(res_id_t dma) {
    DMA_TypeDef *pDMA = nullptr;
    switch (dma) {
//    case 1: pDMA = DMA_INSTANCE(1); break;
//    case 2: pDMA = DMA_INSTANCE(2); break;
    }
    return pDMA;
}

dma_handle_t* getDMA_Handle(DMA dma) {
    DMA_HandleTypeDef *pDMA = nullptr;
    switch(dma) {
//    case DMA::BT:       pDMA = &hdma_usart1_rx; break;
//    case DMA::RADIO:    pDMA = &hdma_usart3_rx; break;
//    case DMA::ENCODER:  pDMA = &hdma_i2c1_rx; break;
    }
    return pDMA;
}

} // namespace uns

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void uns_MotorPanel_Uart_RxCpltCallback();               // Callback for motor panel UART RxCplt - called when receive finishes.
extern void uns_Serial_Uart_RxCpltCallback();                   // Callback for Serial UART RxCplt - called when receive finishes.
extern void uns_RadioModule_Uart_RxCpltCallback();              // Callback for radio module UART RxCplt - called when receive finishes.
extern void uns_FrontLineDetectPanel_Uart_RxCpltCallback();     // Callback for front line detect panel UART RxCplt - called when receive finishes.
extern void uns_RearLineDetectPanel_Uart_RxCpltCallback();      // Callback for rear line detect panel UART RxCplt - called when receive finishes.
extern void uns_Bluetooth_Uart_RxCpltCallback();                // Callback for Bluetooth UART RxCplt - called when receive finishes.

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

}

/* @brief Internal callback - called when I2C reception finishes.
 * @param hi2c Pointer to the I2C handle.
 **/
extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {

}


/* @brief Internal callback - called when UAR receive finishes.
 * @param huart Pointer to the UART handle.
 **/
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == cfg::uart_MotorPanel) {
        uns_MotorPanel_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_Serial) {
        uns_Serial_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_RadioModule) {
        uns_RadioModule_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_FrontLineDetectPanel) {
        uns_FrontLineDetectPanel_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_RearLineDetectPanel) {
        uns_RearLineDetectPanel_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_Bluetooth) {
        //uint32_t bytes = MAX_RX_BUFFER_SIZE - ((DMA_HandleTypeDef*)cfg::dma_Bluetooth.handle)->Instance->NDTR;
        uns_Bluetooth_Uart_RxCpltCallback();
        ((DMA_TypeDef*)cfg::dma_Bluetooth.instance)->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;    // clears DMA flags before next transfer
        ((DMA_HandleTypeDef*)cfg::dma_Bluetooth.handle)->Instance->NDTR = MAX_RX_BUFFER_SIZE; // sets number of bytes to receive
        ((DMA_HandleTypeDef*)cfg::dma_Bluetooth.handle)->Instance->CR |= DMA_SxCR_EN;         // starts DMA transfer
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}

/* @brief Callback function for timer period elapsed event.
 * @param htim Pointer to the timer handle.
 **/
extern "C" void uns_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

}

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}

#endif // BSP_LIB_HAL
