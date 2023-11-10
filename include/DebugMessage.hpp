#pragma once

#include <optional>
#include <tuple>
#include <variant>

#include <micro/debug/ParamManager.hpp>

#include <cfg_track.hpp>
#include <RaceTrackController.hpp>

namespace micro {
struct CarProps;
struct ControlData;
} // namespace micro

class DebugMessage {
public:
    using value_type = std::variant<
        std::tuple<micro::CarProps, micro::ControlData>,
        micro::ParamManager::NamedParam,
        IndexedSectionControlParameters>;

    using reference_type = std::variant<
        std::reference_wrapper<const std::tuple<micro::CarProps, micro::ControlData>>,
        std::reference_wrapper<const micro::ParamManager::NamedParam>,
        std::reference_wrapper<const IndexedSectionControlParameters>>;

    static size_t format(char * const output, const size_t size, const reference_type& data);
    static std::optional<value_type> parse(char * const input);
};
