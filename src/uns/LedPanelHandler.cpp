#include <uns/LedPanelHandler.hpp>
#include <uns/util/arrays.hpp>
#include <uns/bsp/it.hpp>
#include <uns/bsp/tim.hpp>
#include <uns/bsp/gpio.hpp>
#include <uns/bsp/spi.hpp>
#include <uns/config/cfg_board.hpp>
#include <uns/util/debug.hpp>

using namespace uns;

LedPanelHandler::LedPanelHandler(
        spi_handle_t * const _hspi,
        const gpio_pin_struct& _sel0,
        const gpio_pin_struct& _sel1,
        const gpio_pin_struct& _sel2,
        const gpio_pin_struct& _sel3,
        const gpio_pin_struct& _outEnOpto,
        const gpio_pin_struct& _outEnInd)
    : hspi(_hspi)
    , sel0(_sel0)
    , sel1(_sel1)
    , sel2(_sel2)
    , sel3(_sel3)
    , outEnOpto(_outEnOpto)
    , outEnInd(_outEnInd)
    , optoValues(optoValues1, optoValues2)
    , indicatorValues(&indicatorValues1, &indicatorValues2)
    , state(IC_Select::OPTO)
    , optoIdx(0)
    , cycleState(CycleState::RESET) {

    // disables all ICs on the SPI bus
    uns::GPIO_WritePin(this->sel0, PinState::SET);
    uns::GPIO_WritePin(this->sel1, PinState::SET);
    uns::GPIO_WritePin(this->sel2, PinState::SET);
    uns::GPIO_WritePin(this->sel3, PinState::SET);
}

void LedPanelHandler::getMeasured(uint8_t result[]) {
    this->optoValues.swap();
    uns::copy<cfg::NUM_OPTO>(static_cast<const uint8_t*>(this->optoValues.get()), result);
}

void LedPanelHandler::writeIndicatorLeds(const BitArray<cfg::NUM_OPTO>& values) {
    *static_cast<BitArray<cfg::NUM_OPTO>*>(this->indicatorValues.set()) = values;

    // needs to swap odd and even bytes due to LED driver select value format and LED layout on the board
    uint8_t * const leds = static_cast<uint8_t * const>(this->indicatorValues.set());
    for (uint32_t i = 0; i < cfg::NUM_OPTO / 8; i += 2) {
        std::swap(leds[i], leds[i + 1]);
    }

    this->indicatorValues.swap();
}

void LedPanelHandler::selectNextState() {
    // These are the states of the machine:
    //
    //                    S1
    //                  _______
    //      |----|----|----|----|
    //      | A0 | A1 | A3 | A2 |
    //      |----|----|----|----|
    //      | A4 | A5 |    |    | |
    //      |----|----|----|----| | S2
    //    | |    |    |    |    | |
    // S3 | |----|----|----|----|
    //    | | L0 | L1 | OP | L2 |
    //      |----|----|----|----|
    //             _______
    //             S0 (LSB)
    //
    // A0-A5:   ADC_0_7-ADC_40_47
    // OP:      OPTO
    // L0-L2:   LED_0_15-LED_32_47
    //
    // The states must be selected in the following order: ADCs, then OPTO, then LEDs
    // The states must follow each other in an order where no third state gets selected during state change (neighbor rule).
    // The empty states may be used as transitional states.
    //
    // The following 'switch' statement indicates the selected state machine:
    switch(this->state) {
        case IC_Select::ADC_0_7:    // 0000
            uns::GPIO_WritePin(this->sel0, PinState::SET);    // 0001 (ADC_8_15)
            this->state = IC_Select::ADC_8_15;
            break;
        case IC_Select::ADC_8_15:   // 0001
            uns::GPIO_WritePin(this->sel1, PinState::SET);    // 0011 (ADC_24_31)
            this->state = IC_Select::ADC_24_31;
            break;
        case IC_Select::ADC_24_31:  // 0011
            uns::GPIO_WritePin(this->sel0, PinState::RESET);  // 0010 (ADC_16_23)
            this->state = IC_Select::ADC_16_23;
            break;
        case IC_Select::ADC_16_23:  // 0010
            uns::GPIO_WritePin(this->sel2, PinState::SET);    // 0110 (unused state)
            uns::GPIO_WritePin(this->sel0, PinState::SET);    // 0111 (unused state)
            uns::GPIO_WritePin(this->sel1, PinState::RESET);  // 0101 (ADC_40_47)
            this->state = IC_Select::ADC_40_47;
            break;
        case IC_Select::ADC_40_47:  // 0101
            uns::GPIO_WritePin(this->sel0, PinState::RESET);  // 0100 (ADC_32_39)
            this->state = IC_Select::ADC_32_39;
            break;
        case IC_Select::ADC_32_39:  // 0100
            uns::GPIO_WritePin(this->sel3, PinState::SET);    // 1100 (unused state)
            uns::GPIO_WritePin(this->sel0, PinState::SET);    // 1101 (unused state)
            uns::GPIO_WritePin(this->sel1, PinState::SET);    // 1111 (unused state)
            this->state = IC_Select::OPTO;
            break;
        case IC_Select::OPTO:       // 1011
            uns::GPIO_WritePin(this->sel2, PinState::RESET);  // 1011 (OPTO)
            uns::GPIO_WritePin(this->sel2, PinState::SET);    // 1111 (unused state)
            this->state = IC_Select::LED_32_47;
            break;
        case IC_Select::LED_32_47:  // 1010
            uns::GPIO_WritePin(this->sel0, PinState::RESET);  // 1110 (unused state)
            uns::GPIO_WritePin(this->sel2, PinState::RESET);  // 1010 (LED_32_47)
            uns::GPIO_WritePin(this->sel2, PinState::SET);    // 1110 (unused state)
            uns::GPIO_WritePin(this->sel0, PinState::SET);    // 1111 (unused state)
            uns::GPIO_WritePin(this->sel1, PinState::RESET);  // 1101 (unused state)
            this->state = IC_Select::LED_16_31;
            break;
        case IC_Select::LED_16_31:  // 1001
            uns::GPIO_WritePin(this->sel2, PinState::RESET);  // 1001 (LED_16_31)
            uns::GPIO_WritePin(this->sel2, PinState::SET);    // 1101 (unused state)
            uns::GPIO_WritePin(this->sel0, PinState::RESET);  // 1100 (unused state)
            this->state = IC_Select::LED_0_15;
            break;
        case IC_Select::LED_0_15:   // 1000
            uns::GPIO_WritePin(this->sel2, PinState::RESET);  // 1000 (LED_0_15)
            uns::GPIO_WritePin(this->sel3, PinState::RESET);  // 0000 (ADC_0_7)
            this->state = IC_Select::ADC_0_7;
            break;
    }
}

void LedPanelHandler::start() {
    this->state = IC_Select::OPTO;
    this->optoIdx = 0;
    this->cycleState = CycleState::RUNNING;

    // disables optical sensor and LED outputs
    GPIO_WritePin(this->outEnOpto, PinState::SET);
    GPIO_WritePin(this->outEnInd, PinState::SET);

    // selects OPTO
    uns::GPIO_WritePin(this->sel0, PinState::SET);
    uns::GPIO_WritePin(this->sel1, PinState::SET);
    uns::GPIO_WritePin(this->sel2, PinState::RESET);
    uns::GPIO_WritePin(this->sel3, PinState::SET);

    uns::SPI_SetReady(this->hspi);
    this->sendCommand();
}

void LedPanelHandler::onExchangeFinished() {
    switch(this->state) {
        case IC_Select::ADC_0_7:
            this->state = IC_Select::ADC_0_7;
            /* no break */
        case IC_Select::ADC_8_15:
        case IC_Select::ADC_24_31:
        case IC_Select::ADC_16_23:
        case IC_Select::ADC_40_47:
        case IC_Select::ADC_32_39:
        {
            uint8_t ADC_value = (this->buffer[1] << 2) | (this->buffer[2] >> 6);    // ADC value format: 00000000 00XXXXXX XX000000
            uint4_t idx = (static_cast<uint4_t>(this->state) - static_cast<uint4_t>(IC_Select::ADC_0_7)) * 8 + optoIdx;
            static_cast<uint8_t * const>(this->optoValues.set())[idx] = ADC_value;    // saves measured value

            if (this->state == IC_Select::ADC_32_39) {  // increases optical sensor index when all ADCs have been read
                this->optoIdx = (this->optoIdx + 1) % 8;
            }
            break;
        }

        case IC_Select::OPTO:
            GPIO_WritePin(this->outEnOpto, PinState::RESET);    // re-enables optical LED sensor outputs after their registers have been updated
            GPIO_WritePin(this->outEnInd, PinState::SET);       // disables indicator LED outputs before updating their registers
            break;
        case IC_Select::LED_32_47:
        case IC_Select::LED_16_31:
            break;
        case IC_Select::LED_0_15:
            GPIO_WritePin(this->outEnInd, PinState::RESET);     // re-enables indicator LED outputs after their registers have been updated
            break;
    }
    this->selectNextState();

    if (this->state == IC_Select::OPTO && this->optoIdx == 0) {
        this->cycleState = CycleState::FINISHED;
    } else if (this->cycleState == CycleState::RUNNING) {
        this->sendCommand();
    }
}

void LedPanelHandler::sendCommand() {
    switch(this->state) {
        case IC_Select::ADC_0_7:
        case IC_Select::ADC_8_15:
        case IC_Select::ADC_24_31:
        case IC_Select::ADC_16_23:
        case IC_Select::ADC_40_47:
        case IC_Select::ADC_32_39:
        {
            // Control byte: | START | SEL2 | SEL1 | SEL0 | UNI/BIP | SGL/DIF | PD1 | PD0 |
            // Select bits (according to the datasheet):
            //      SEL2    -   optoIdx's 1st bit (LSB)
            //      SEL1    -   optoIdx's 3rd bit
            //      SEL0    -   optoIdx's 2nd bit
            //
            // @see MAX1110CAP+ datasheet for details
            this->buffer[0] =  0b10001111 | ((this->optoIdx & 0b00000001) << 6) | ((this->optoIdx & 0b00000010) << 3) | ((this->optoIdx & 0b00000100) << 3);
            this->buffer[1] = this->buffer[2] = 0x00;
            SPI_TransmitReceive_IT(this->hspi, this->buffer, this->buffer, 3);
            break;
        }
        case IC_Select::OPTO:
        {
            GPIO_WritePin(this->outEnOpto, PinState::SET);          // disables optical LED sensor outputs before updating their registers
            this->buffer[0] = this->buffer[1] = (0x01 << optoIdx);  // selects every 8th optical sensor
            SPI_TransmitReceive_IT(this->hspi, this->buffer, this->buffer, 2);
            break;
        }
        case IC_Select::LED_32_47:
        case IC_Select::LED_16_31:
        case IC_Select::LED_0_15:
        {
            // calculates led index
            // (*2) is needed because buffer is uint8_t type, but LED driver has 16 bits of data
            uint4_t ledIdx = (static_cast<uint4_t>(this->state) - static_cast<uint4_t>(IC_Select::LED_0_15)) * 2;
            const uint8_t * const leds = &(static_cast<const uint8_t * const>(this->indicatorValues.get())[ledIdx]);
            uns::copy<2>(leds, this->buffer);
            SPI_TransmitReceive_IT(this->hspi, this->buffer, this->buffer, 2);  // sets indicator LED values
            break;
        }
    }
}
