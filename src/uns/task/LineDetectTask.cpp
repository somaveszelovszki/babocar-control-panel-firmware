#include <uns/task/common.hpp>
#include <uns/config/cfg_board.hpp>
#include <uns/util/debug.hpp>
#include <uns/bsp/gpio.hpp>
#include <uns/bsp/tim.hpp>
#include <uns/config/cfg_car.hpp>
#include <uns/config/cfg_track.hpp>
#include <uns/util/units.hpp>
#include <uns/BitArray.hpp>
#include <uns/bsp/queue.hpp>
#include <uns/bsp/task.hpp>
#include <uns/LedPanelHandler.hpp>
#include <uns/LedDataAnalyser.hpp>
#include <uns/util/unit_utils.hpp>
#include <uns/LineData.hpp>
#include <uns/CarProps.hpp>

using namespace uns;

extern CarProps car;

namespace {

FrontBackByteContainer measurements;	    // Stores measured optical sensor data.
FrontBackBitContainer indicatorLeds;    // Stores lines matched to the optical sensor positions.

LineData lines;
static_assert(sizeof(lines) == sizeof(DetectedLinesQueueItem_t), "Size of Line vector does not match required value!");

LedPanelHandler ledPanel(cfg::spi_LedPanel, cfg::gpio_LedPanelSel0, cfg::gpio_LedPanelSel1, cfg::gpio_LedPanelSel2, cfg::gpio_LedPanelSel3, cfg::gpio_LedPanelOutEnOpto, cfg::gpio_LedPanelOutEnInd);

LedDataAnalyser ledDataAnalyser(cfg::NUM_OPTO_FRONT, cfg::NUM_OPTO_BACK, cfg::MAX_POS_OPTO_FRONT, cfg::MAX_POS_OPTO_BACK, cfg::DIST_BTW_OPTO_ROWS);

} // namespace

extern "C" void runLineDetectTask(void const *argument) {

    //ledPanel.test();
    ledPanel.start();

    uns::time_t startTime = uns::getTime();

    indicatorLeds.all.reset();
    ledPanel.writeIndicatorLeds(indicatorLeds.all);

    while (!uns::hasErrorHappened()) {
        bool timeout = uns::getTime() - startTime > uns::time_t::from<milliseconds>(10);
        if (ledPanel.hasCycleFinished() || timeout) {
            if (timeout) {
                ledPanel.suspend();
            }

            ledPanel.getMeasured(measurements.all);
            ledDataAnalyser.detectLines(car.pos(), car.orientation(), &measurements, &indicatorLeds, &lines);

            if (cfg::INDICATOR_LEDS_ENABLED) {
                ledPanel.writeIndicatorLeds(indicatorLeds.all);
            }

//            if (uns::getTime() >= prevSendTime + time::from<milliseconds>(500)) {
//                prevSendTime = uns::getTime();
//                debug::printf(debug::CONTENT_FLAG_OPTO_BITS, "%d|%d", *static_cast<const int32_t*>(indicatorLeds.front), *static_cast<const int16_t*>(indicatorLeds.back));
//                printLines();
//            }

            Status status = uns::queueSend(cfg::queue_DetectedLines, &lines);
            if (!isOk(status)) {
                //debug::printerr(status, "Error while sending detected lines!");
            }

            if(!isOk(SPI_GetState(cfg::spi_LedPanel))) {
                uns::nonBlockingDelay(time_t::from<milliseconds>(1));
            }
            ledPanel.start();
            startTime = uns::getTime();
            uns::nonBlockingDelay(time_t::from<milliseconds>(5));
        } else {
            uns::nonBlockingDelay(time_t::from<milliseconds>(1));
        }
    }

    uns::deleteCurrentTask();
}

/* @brief Called when SPI exchange finishes.
 **/
void uns_LedPanel_Spi_TxRxCpltCallback() {
    ledPanel.onExchangeFinished();
}
