#pragma once

#include <uns/util/types.hpp>

namespace uns {

typedef void dma_handle_t;      // GPIO type - bsp library-dependent.
typedef void dma_t;             // GPIO type - bsp library-dependent.

/* @brief Structure storing DMA handle and instance.
 **/
struct dma_struct {
    dma_t *instance;            // Pointer to the DMA instance.
    dma_handle_t *handle;       // Pointer to the DMA handle.
};

/* @brief GPIO instance ids.
 **/
enum class DMA : uint8_t {
    BT,     // DMA for Bluetooth
    RADIO,  // DMA for Radio module
    GYRO,   // DMA for Gyroscope (Arduino) module
    ENCODER // DMA for Encoder (Nucleo) module
};

/* @brief Gets DMA instance by id.
 * @param dma The DMA id.
 * @returns Pointer to the correspondent DMA instance.
 **/
dma_t* getDMA(res_id_t dma);

/* @brief Gets DMA handle.
 * @param dma The DMA to get.
 * @returns The correspondent DMA handle.
 **/
dma_handle_t* getDMA_Handle(DMA dma);

void DMA_retrigger();
} // namespace uns
