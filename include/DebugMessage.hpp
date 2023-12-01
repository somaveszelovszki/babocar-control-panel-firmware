#pragma once

#include <optional>

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

    static size_t format(char * const output, const size_t size, const CarData& car);
    static size_t format(char * const output, const size_t size, const micro::ParamManager::NamedParam& param);
    static size_t format(char * const output, const size_t size, const IndexedSectionControlParameters& sectionControl);

    static bool parse(char * const input, CarData& car);
    static bool parse(char * const input, std::optional<micro::ParamManager::NamedParam>& namedParam);
    static bool parse(char * const input, std::optional<IndexedSectionControlParameters>& sectionControl);
};
