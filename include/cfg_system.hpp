#pragma once

#include <cstdint>

namespace cfg {

constexpr bool    USE_SAFETY_ENABLE_SIGNAL        = true;
constexpr bool    INDICATOR_LEDS_ENABLED          = true;
constexpr uint8_t REDUCED_LINE_DETECT_SCAN_RADIUS = 12;
constexpr uint8_t NUM_MONITORED_TASKS             = 8;
constexpr size_t  RADIO_COMMAND_MAX_LENGTH        = 7;

} // namespace cfg
