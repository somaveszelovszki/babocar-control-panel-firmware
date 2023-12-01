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
	struct CarData {
		micro::CarProps props;
		micro::ControlData control;
	};

    using ParseResult = std::variant<
        std::tuple<micro::CarProps, micro::ControlData>,
        std::optional<micro::ParamManager::NamedParam>,
        std::optional<IndexedSectionControlParameters>>;

    static size_t format(char * const output, const size_t size, const CarData& car);
    static size_t format(char * const output, const size_t size, const micro::ParamManager::NamedParam& param);
    static size_t format(char * const output, const size_t size, const IndexedSectionControlParameters& sectionControl);
    static std::optional<ParseResult> parse(char * const input);
};
