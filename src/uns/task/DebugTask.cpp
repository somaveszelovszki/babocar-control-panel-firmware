#include <config/cfg_board.hpp>
#include <config/cfg_os.hpp>
#include <string.h>
#include <uns/bsp/tim.hpp>
#include <uns/bsp/uart.hpp>
#include <uns/bsp/task.hpp>
#include <uns/container/RingBuffer.hpp>
#include <uns/container/Vec.hpp>
#include <uns/util/debug.hpp>
#include <uns/bsp/queue.hpp>
#include <uns/util/convert.hpp>
#include <uns/util/arrays.hpp>
#include <uns/CarProps.hpp>
#include <uns/control/PID_Controller.hpp>

#include <string.h>
#include <task/Config.hpp>

using namespace uns;

extern float32_t debugMotorPWM;

namespace {

static uart_handle_t * const uart_Debug = cfg::uart_Bluetooth;

char uartBluetooth_Buffer[MAX_RX_BUFFER_SIZE];
RingBuffer<char, MAX_RX_BUFFER_SIZE * 8> rxBuffer;
Vec<char, MAX_TX_BUFFER_SIZE> txBuffer;
uint32_t contentFlags = 0xffff;     // enables all contents by default

Status readInt(const char *str, uint32_t size, uint32_t *pIdx, int32_t *pResult) {
    Status status;
    if ((*pIdx += uns::atoi(&str[*pIdx], pResult, size - *pIdx))) {
        *pIdx += sizeof('|');
        status = Status::OK;
    } else {
        status = Status::INVALID_DATA;
    }
    return status;
}

Status readFloat(const char *str, uint32_t size, uint32_t *pIdx, float32_t *pResult) {
    Status status;
    if ((*pIdx += uns::atof(&str[*pIdx], pResult, size - *pIdx))) {
        *pIdx += sizeof('|');
        status = Status::OK;
    } else {
        status = Status::INVALID_DATA;
    }
    return status;
}

/* @brief Gets debug message from the buffer.
 * @param rxMsg The destination debug message.
 * @returns Status indicating operation success. Status::OK means a whole message has been received, otherwise the result is Status::NO_NEW_DATA.
 */
Status getRxMsg(debug::Msg& rxMsg) {
    Status result = Status::NO_NEW_DATA;
    int32_t startIdx, endIdx, secondStartIdx;

    if ((startIdx = rxBuffer.indexOf(debug::MSG_START)) != -1) {
        rxBuffer.updateTail(static_cast<uint32_t>(startIdx));      // removes unnecessary characters from the beginning of the RX buffer

        // removes all characters before the second start character if there was no end character in that sequence
        do {
            secondStartIdx = rxBuffer.indexOf(debug::MSG_START, 1);
            endIdx = rxBuffer.indexOf(debug::MSG_END);
            if (secondStartIdx > 0 && (secondStartIdx < endIdx || endIdx == -1)) {
                rxBuffer.updateTail(static_cast<uint32_t>(secondStartIdx));
            } else {
                break;
            }
        } while(true);
        endIdx = rxBuffer.indexOf(debug::MSG_END);
        if (endIdx != -1) {
            // gets characters from the buffer
            uint32_t size = uns::min(rxBuffer.size(), LOG_MSG_MAX_SIZE - rxMsg.text.size);
            if (isOk(result = rxBuffer.get(&rxMsg.text[rxMsg.text.size], size))) {
                rxMsg.text.size += size;
                uns::atoi(&rxMsg.text[sizeof(debug::MSG_START)], reinterpret_cast<int32_t*>(&rxMsg.content));
            }
        }
    } else {
        rxBuffer.updateTail(rxBuffer.size());   // removes unnecessary characters from the beginning of the RX buffer (all characters are unnecessary, because no START character has been found)
    }

    return result;
}

/* @brief Handles received debug message.
 * @param rxMsg The received debug message.
 */
void handleRxMsg(debug::Msg& rxMsg) {
    uint32_t dataIdx = uns::indexOf(debug::MSG_SEP, static_cast<const char * const>(rxMsg.text), LOG_MSG_MAX_SIZE) + sizeof(debug::MSG_SEP);
    const char * const data = &rxMsg.text[dataIdx];
    if (rxMsg.content | debug::CONTENT_FLAG_SET_CONTENT) {
        uns::atoi(data, reinterpret_cast<int32_t*>(&contentFlags));
    }

    uint32_t strIdx = 0;

    if (rxMsg.content & debug::CONTENT_FLAG_CAR_PROPS) {
        float32_t posX, posY, v, ori;
        if (isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &posX)) &&
            isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &posY)) &&
            isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &v)) &&
            isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &ori))) {

            // TODO store speed
            debug::printf(debug::CONTENT_FLAG_LOG, "Speed updated: %f mm/sec", v);
            //debugMotorPWM = v;
        }
    } else if (rxMsg.content & debug::CONTENT_FLAG_CONTROLLER) {
        float32_t Kc;
//        if (isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &Kc)) &&
//            isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &Ki)) &&
//            isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &Kd)) &&
//            isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &maxIntegralRate))) {

        if (isOk(readFloat(data, rxMsg.text.size - dataIdx, &strIdx, &Kc))) {
            // TODO update PI controller
            debug::printf(debug::CONTENT_FLAG_LOG, "Speed controller Kc updated: %f", Kc);
        }
    }

    rxMsg.text.size = 0;    // resets RX message object
}

} // namespace

extern "C" void runDebugTask(const void *argument) {
    debug::Msg rxMsg, txMsg;
    char numBuff[STR_MAX_LEN_INT + 1];

    millisecond_t rxStartTime = uns::getTime();

    uns::UART_Receive_DMA(cfg::uart_Bluetooth, reinterpret_cast<uint8_t*>(uartBluetooth_Buffer), MAX_RX_BUFFER_SIZE);

    while (true) {
        // handle incoming control messages from the monitoring app
        if (isOk(getRxMsg(rxMsg))) {
            handleRxMsg(rxMsg);
        } else if (uns::getTime() - rxStartTime > millisecond_t(100)) {

        }

        // receives all available messages coming from the tasks and adds them to the buffer vector
        while(txBuffer.size + LOG_MSG_MAX_SIZE <= MAX_TX_BUFFER_SIZE && isOk(uns::queueReceive(cfg::queue_Log, &txMsg))) {

            uint32_t contentLen;

            // appends message start character, content flags and separator
            if (!txBuffer.append(debug::MSG_START) ||
                !(contentLen = uns::itoa(txMsg.content, numBuff, STR_MAX_LEN_INT)) ||
                !txBuffer.append(numBuff, contentLen) ||
                !txBuffer.append(debug::MSG_SEP) ||
                !txBuffer.append(static_cast<const char * const>(txMsg.text), txMsg.text.size) ||
                !txBuffer.append(debug::MSG_END)) {
                txBuffer.size = 0;
            }
        }

        if (txBuffer.size > 0) {
            while (!isOk(UART_Transmit_IT(uart_Debug, reinterpret_cast<const uint8_t * const>(static_cast<const char * const>(txBuffer)), txBuffer.size))) {  // sends messages once UART is free
                uns::nonBlockingDelay(millisecond_t(1));
            }

            txBuffer.clear();
        }

        uns::nonBlockingDelay(millisecond_t(1));
    }

    uns::taskDeleteCurrent();
}

/* @brief Callback for Bluetooth UART RxCplt - called when receive finishes.
 */
void uns_Bluetooth_Uart_RxCpltCallback(uint32_t len) {
    rxBuffer.put(uartBluetooth_Buffer, len);
    uns::UART_Receive_DMA(cfg::uart_Bluetooth, reinterpret_cast<uint8_t*>(uartBluetooth_Buffer), MAX_RX_BUFFER_SIZE);
}

/* @brief Callback for Serial UART RxCplt - called when receive finishes.
 */
void uns_Serial_Uart_RxCpltCallback() {
    // does not handle Serial messages
}
