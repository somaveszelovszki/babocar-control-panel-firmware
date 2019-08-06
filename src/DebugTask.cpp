#include <micro/task/common.hpp>
#include <micro/container/vec.hpp>
#include <micro/bsp/tim.hpp>
#include <micro/bsp/uart.hpp>
#include <micro/bsp/queue.hpp>
#include <micro/bsp/task.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/container/span.hpp>
#include <micro/utils/debug.hpp>
#include <micro/utils/convert.hpp>
#include <micro/utils/arrays.hpp>
#include <micro/debug/params.hpp>

#include <cfg_board.hpp>
#include <cfg_os.hpp>

#include <globals.hpp>

#include <cstring>

using namespace micro;

namespace micro {

ring_buffer<uint8_t[MAX_RX_BUFFER_SIZE], 3> rxBuffer;
vec<uint8_t, MAX_TX_BUFFER_SIZE> txBuffer;

/* @brief Gets debug message from the buffer.
 * @param rxMsg The destination debug message.
 * @returns Status indicating operation success. Status::OK means a whole message has been received, otherwise the result is Status::NO_NEW_DATA.
 */
Status getRxMsg(debug::LogMessage& rxMsg) {
    Status result = Status::NO_NEW_DATA;
//    int32_t startIdx, endIdx, secondStartIdx;
//
//    if ((startIdx = indexOf(rxBuffer.indexOf(debug::MSG_START)) != -1) {
//        rxBuffer.updateTail(static_cast<uint32_t>(startIdx));      // removes unnecessary characters from the beginning of the RX buffer
//
//        // removes all characters before the second start character if there was no end character in that sequence
//        do {
//            secondStartIdx = rxBuffer.indexOf(debug::MSG_START, 1);
//            endIdx = rxBuffer.indexOf(debug::MSG_END);
//            if (secondStartIdx > 0 && (secondStartIdx < endIdx || endIdx == -1)) {
//                rxBuffer.updateTail(static_cast<uint32_t>(secondStartIdx));
//            } else {
//                break;
//            }
//        } while(true);
//        endIdx = rxBuffer.indexOf(debug::MSG_END);
//        if (endIdx != -1) {
//            // gets characters from the buffer
//            uint32_t size = min(rxBuffer.size(), LOG_MSG_MAX_SIZE - rxMsg.text.size());
//            if (isOk(result = rxBuffer.get(&rxMsg.text[rxMsg.text.size()], size))) {
//                rxMsg.text.size() += size;
//                atoi(&rxMsg.text[sizeof(debug::MSG_START)], reinterpret_cast<int32_t*>(&rxMsg.content));
//            }
//        }
//    } else {
//        rxBuffer.updateTail(rxBuffer.size());   // removes unnecessary characters from the beginning of the RX buffer (all characters are unnecessary, because no START character has been found)
//    }

    return result;
}

/* @brief Handles received debug message.
 * @param rxMsg The received debug message.
 */
void handleRxMsg(debug::LogMessage& rxMsg) {
//    uint32_t dataIdx = indexOf(debug::MSG_SEP, static_cast<const char * const>(rxMsg.text), LOG_MSG_MAX_SIZE) + sizeof(debug::MSG_SEP);
//    const char * const data = &rxMsg.data[dataIdx];
//    if (rxMsg.content | debug::CONTENT_FLAG_SET_CONTENT) {
//        atoi(data, reinterpret_cast<int32_t*>(&contentFlags));
//    }
//
//    uint32_t strIdx = 0;
//
//    if (rxMsg.content & debug::CONTENT_FLAG_CAR_PROPS) {
//        float32_t posX, posY, v, ori;
//        if (isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &posX)) &&
//            isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &posY)) &&
//            isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &v)) &&
//            isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &ori))) {
//
//            // TODO store speed
//            debug::printf(debug::CONTENT_FLAG_LOG, "Speed updated: %f mm/sec", v);
//            //debugMotorPWM = v;
//        }
//    } else if (rxMsg.content & debug::CONTENT_FLAG_CONTROLLER) {
//        float32_t Kc;
////        if (isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &Kc)) &&
////            isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &Ki)) &&
////            isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &Kd)) &&
////            isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &maxIntegralRate))) {
//
//        if (isOk(readFloat(data, rxMsg.text.size() - dataIdx, &strIdx, &Kc))) {
//            // TODO update PI controller
//            debug::printf(debug::CONTENT_FLAG_LOG, "Speed controller Kc updated: %f", Kc);
//        }
//    }
//
//    rxMsg.text.size() = 0;    // resets RX message object
}

} // namespace

extern "C" void runDebugTask(const void *argument) {
    debug::LogMessage txLog;

    UART_Receive_DMA(cfg::uart_Command, *rxBuffer.getWritableBuffer(), MAX_RX_BUFFER_SIZE);

    while (!task::hasErrorHappened()) {
        // handle incoming control messages from the monitoring app
//        if (isOk(getRxMsg(rxMsg))) {
//            handleRxMsg(rxMsg);
//        }

        // receives all available messages coming from the tasks and adds them to the buffer vector
        while(txBuffer.size() + LOG_MSG_MAX_SIZE <= MAX_TX_BUFFER_SIZE && isOk(queueReceive(cfg::queue_Log, &txLog))) {
            txBuffer.append(static_cast<uint8_t>(DebugCode::Log));
            txBuffer.append(txLog.begin(), txLog.end());
        }

        if (txBuffer.size() > 0) {
            while (!isOk(UART_Transmit_IT(cfg::uart_Command, txBuffer.data(), txBuffer.size()))) {  // sends messages once UART is free
                nonBlockingDelay(millisecond_t(1));
            }

            txBuffer.clear();
        }

        nonBlockingDelay(millisecond_t(1));
    }

    taskDeleteCurrent();
}

/* @brief Callback for Serial UART RxCplt - called when receive finishes.
 */
void micro_Command_Uart_RxCpltCallback() {
    // does not handle Serial messages
}
