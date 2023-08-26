#pragma once

#include <micro/debug/ParamManager.hpp>
#include <micro/utils/log.hpp>

#include <cfg_track.hpp>
#include <RaceTrackController.hpp>

namespace micro {
struct CarProps;
struct ControlData;
} // namespace micro

#if RACE_TRACK == TRACK
#define TRACK_CONTROL_PREFIX 'R'
#else
#define TRACK_CONTROL_PREFIX 'T'
#endif

inline constexpr uint32_t MAX_BUFFER_SIZE = 1024;

enum class DebugMessageType : char {
    LogDebug = 'D',
    LogInfo = 'I',
    LogWarn = 'W',
    LogError = 'E',
    Car = 'C',
    Params = 'P',
    TrackControl = TRACK_CONTROL_PREFIX
};

struct DebugMessageSeparator {
    static constexpr auto value = micro::Log::SEPARATOR;
};

size_t format(char* output, const size_t size, const DebugMessageType type);
size_t format(char* output, const size_t size, const DebugMessageSeparator&);
size_t format(char* output, const size_t size, const micro::CarProps& car, const micro::ControlData& controlData);
size_t format(char* output, const size_t size, const LapControlParameters& lapControl);
size_t format(char* output, const size_t size, const micro::ParamManager::Values& params);
