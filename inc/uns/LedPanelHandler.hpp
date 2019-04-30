#pragma once

#include <uns/BitArray.hpp>
#include <uns/config/cfg_car.hpp>
#include <uns/bsp/spi.hpp>
#include <uns/bsp/gpio.hpp>
#include <uns/util/algorithm.hpp>

namespace uns {

/* @brief Handles front and back LED panels, responsible for reading line sensor LEDs and controlling line indicator LEDs.
 **/
class LedPanelHandler {
public:
    /* @brief Constructor - initializes SPI and GPIO pins.
     * @param _hspi Pointer to the SPI handler.
     * @param _sel0 Select pin #0.
     * @param _sel1 Select pin #1.
     * @param _sel2 Select pin #2.
     * @param _sel3 Select pin #3.
     * @param _outEnOpto Output enable pin for the optical sensors.
     * @param _outEnInd Output enable pin for the indicator LEDs.
     **/
    LedPanelHandler(
        spi_handle_t * const _hspi,
        const gpio_pin_struct& _sel0,
        const gpio_pin_struct& _sel1,
        const gpio_pin_struct& _sel2,
        const gpio_pin_struct& _sel3,
        const gpio_pin_struct& _outEnOpto,
        const gpio_pin_struct& _outEnInd);

    /* @brief Gets measured optical values.
     * @param result The array that will store the result values.
     **/
    void getMeasured(uint8_t result[]);

    /* @brief Sets indicator LED values.
     * @note The written values will be forwarded to the LED driver in the next measurement cycle (i.e. this method will NOT result in the LEDs changing values instantly).
     * @param values The indicator LED values.
     **/
    void writeIndicatorLeds(const BitArray<cfg::NUM_OPTO>& values);

    /* @brief Starts new measurement cycle.
     * @note One iteration is ~6ms long.
     **/
    void start();

    /* @brief Called when SPI exchange finishes. Execution depends on the state-machine.
     **/
    void onExchangeFinished();

    /* @brief Checks if the current measurement cycle has finished.
     * @returns Boolean value indicating if the current measurement cycle has finished.
     **/
    bool hasCycleFinished() const {
        return this->cycleState == CycleState::FINISHED;
    }

    /* @brief Suspends current measurement cycle.
     * @note Values of the optical sensors and indicator LEDs that would have been queried/set after the suspension will not be updated.
     **/
    void suspend() {
        this->cycleState = CycleState::SUSPENDED;
    }

private:
    /* @brief Defines measurement cycle state.
     **/
    enum class CycleState : uint8_t {
        RESET,      // Initial state.
        RUNNING,    // Reading optical values, setting indicator LEDs.
        FINISHED,   // Finished cycle - call getMeasured() to obtain line data, call start() to start new measurement cycle.
        SUSPENDED   // Measurement cycle has been suspended - call getMeasured() to obtain line (INCOMPLETE!) data, call start() to start new measurement cycle.
    };

    /* @brief Selects next state by updating the selection pins.
     **/
    void selectNextState();

    /* @brief Sends command to the selected SPI slave.
     **/
    void sendCommand();

    /* @brief Defines SPI slave select value.
     **/
    enum class IC_Select : uint4_t {
        ADC_0_7     = 0x00, // Selector value for ADC 0-7.
        ADC_8_15    = 0x01, // Selector value for ADC 8-15.
        ADC_16_23   = 0x02, // Selector value for ADC 16-23.
        ADC_24_31   = 0x03, // Selector value for ADC 24-31.
        ADC_32_39   = 0x04, // Selector value for ADC 32-39.
        ADC_40_47   = 0x05, // Selector value for ADC 40-47.
        LED_0_15    = 0x08, // Selector value for LED 0-15.
        LED_16_31   = 0x09, // Selector value for LED 16-31.
        LED_32_47   = 0x0A, // Selector value for LED 32-47.
        OPTO        = 0x0B  // Selector value for the optical sensors (0-47).
    };

    spi_handle_t * const hspi;          // Pointer to the SPI handle.
    const gpio_pin_struct& sel0;        // Select pin #0.
    const gpio_pin_struct& sel1;        // Select pin #1.
    const gpio_pin_struct& sel2;        // Select pin #2.
    const gpio_pin_struct& sel3;        // Select pin #3.
    const gpio_pin_struct& outEnOpto;   // Output enable pin for the optical sensors.
    const gpio_pin_struct& outEnInd;    // Output enable pin for the indicator LEDs.

    uint8_t optoValues1[cfg::NUM_OPTO], optoValues2[cfg::NUM_OPTO]; // Background container arrays for the optical sensors values.
    SwapExchange optoValues;            // Swap exchange helper for getting and setting the optical sensor values.

    BitArray<cfg::NUM_OPTO> indicatorValues1, indicatorValues2;     // Background container arrays for the indicator LED values.
    SwapExchange indicatorValues;       // Swap exchange helper for getting and setting the indicator LED values.

    IC_Select state;        // Current SPI slave select state.
    uint3_t optoIdx;        // Current optical sensor index.
    uint8_t buffer[3];      // The SPI exchange buffer.
    CycleState cycleState;  // The current measurement cycle state.
};

} // namespace uns
