#pragma once

#include <RaceTrackController.hpp>
#include <cfg_system.hpp>
#include <cfg_track.hpp>
#include <optional>

#include <micro/debug/ParamManager.hpp>

namespace micro {
struct CarProps;
struct ControlData;
} // namespace micro

class DebugMessage {
  public:
    struct CarData {
        micro::CarProps props;
        micro::ControlData control;
    };

    struct RadioCommand {
        char text[cfg::RADIO_COMMAND_MAX_LENGTH];
    };

    static size_t format(char* const output, const size_t size, const CarData& car);
    static size_t format(char* const output, const size_t size,
                         const micro::ParamManager::NamedParam& param);
    static size_t format(char* const output, const size_t size,
                         const IndexedSectionControlParameters& sectionControl);
    static size_t format(char* const output, const size_t size, const RadioCommand& command);

    static bool parse(char* const input, CarData& OUT car);
    static bool parse(char* const input,
                      std::optional<micro::ParamManager::NamedParam>& OUT namedParam);
    static bool parse(char* const input,
                      std::optional<IndexedSectionControlParameters>& OUT sectionControl);
    static bool parse(char* const input, RadioCommand& OUT command);
};
