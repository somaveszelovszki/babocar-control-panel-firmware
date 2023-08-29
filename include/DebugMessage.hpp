#pragma once

#include <tuple>

#include <ArduinoJson.h>

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

class DebugMessage {
private:
    DebugMessage() {}

public:
    enum class Type : char {
        Unknown = '?',
        Car = 'C',
        Params = 'P',
        TrackControl = TRACK_CONTROL_PREFIX
    };

    using value_type = std::variant<
        std::tuple<micro::CarProps, micro::ControlData>,
        micro::ParamManager::Values,
        LapControlParameters>;

    using reference_type = std::variant<
        std::reference_wrapper<const std::tuple<micro::CarProps, micro::ControlData>>,
        std::reference_wrapper<const micro::ParamManager::Values>,
        std::reference_wrapper<const LapControlParameters>>;

    static size_t format(char * const output, const size_t size, const reference_type& data);
    static value_type parse(const char * const input);

private:
    static Type getType(const reference_type& value);
    static value_type getValue(const Type type);

    static size_t formatType(char* output, const size_t size, const Type type);
    static size_t parseType(const char * const input, Type& type);

    static size_t formatSeparator(char* output, const size_t size);
    static size_t parseSeparator(const char * const input);

    void store(const std::tuple<micro::CarProps, micro::ControlData>& data);
    void load(std::tuple<micro::CarProps, micro::ControlData>& data);

    void store(const micro::ParamManager::Values& params);
    void load(micro::ParamManager::Values& params);

    void store(const LapControlParameters& lapControl);
    void load(LapControlParameters& lapControl);

private:
    StaticJsonDocument<1024> jsonDoc_;
};
