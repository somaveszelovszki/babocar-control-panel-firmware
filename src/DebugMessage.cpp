#include <cmath>
#include <cstdlib>

#include <etl/string.h>

#include <micro/debug/ParamManager.hpp>
#include <micro/format/format.hpp>
#include <micro/json/json.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/types.hpp>
#include <micro/utils/str_utils.hpp>

#include <DebugMessage.hpp>

using namespace micro;

namespace {

enum class Type : char {
    Unknown = '?',
    Car = 'C',
    Param = 'P',
    RaceTrackControl = 'R',
    TestTrackControl = 'T'
};

std::optional<ParamManager::value_type> parseParamValue(const micro::JSONValue& json) {
   if (const auto v = json.as<bool>()) {
       return v;
   } else if (const auto v = json.as<int32_t>()) {
       return v;
   } else if (const auto v = json.as<float>()) {
       return v;
   }

   return std::nullopt;
}

std::optional<DebugMessage::ParseResult> getParseValue(const Type type) {
    switch (type) {
    case Type::Car:
    	return std::tuple<CarProps, ControlData>{};

    case Type::Param:
    	return ParamManager::NamedParam{};

    case Type::RaceTrackControl:
    case Type::TestTrackControl:
    	return IndexedSectionControlParameters{};

    default:
    	return std::nullopt;
    }
}

size_t formatType(char* output, const size_t size, const Type type) {
    return micro::format_to_n(output, size, "{}:", micro::underlying_value(type));
}

size_t parseType(const char * const input, Type& type) {
    if (strnlen(input, 2) < 2 || input[1] != ':') {
        return 0;
    }

    type = static_cast<Type>(input[0]);
    return 2;
}

size_t formatSeparator(char* output, const size_t size) {
    return micro::format_to_n(output, size, "{}", Log::SEPARATOR);
}

size_t store(char * const output, const size_t size, const DebugMessage::CarData& car) {
    return micro::format_to_n(output, size,
        "[{},{},{:.2f},{:.2f},{:.2f},{:.2f},{},{:.2f},{},{:.2f},{}]",
        static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.props.pose.pos.X).get())),
        static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.props.pose.pos.Y).get())),
        car.props.pose.angle.get(),
        car.props.speed.get(),
        car.props.frontWheelAngle.get(),
        car.props.rearWheelAngle.get(),
        static_cast<int32_t>(std::lround(car.control.lineControl.actual.pos.get())),
        car.control.lineControl.actual.angle.get(),
        static_cast<int32_t>(std::lround(car.control.lineControl.target.pos.get())),
		car.control.lineControl.target.angle.get(),
        car.props.isRemoteControlled ? 1 : 0);
}

void load(char * const input, std::tuple<CarProps, ControlData>& data) {
    const auto json = micro::JSONParser(input).root();
    auto& [car, controlData] = data;

    car.pose.pos.X      = millimeter_t(static_cast<float>(*json[0].as<int32_t>()));
    car.pose.pos.Y      = millimeter_t(static_cast<float>(*json[1].as<int32_t>()));
    car.pose.angle      = radian_t(*json[2].as<float>());
    car.speed           = m_per_sec_t(*json[3].as<float>());
    car.frontWheelAngle = radian_t(*json[4].as<float>());
    car.rearWheelAngle  = radian_t(*json[5].as<float>());
    controlData.lineControl.actual.pos   = millimeter_t(static_cast<float>(*json[6].as<int32_t>()));
    controlData.lineControl.actual.angle = radian_t(*json[7].as<float>());
    controlData.lineControl.target.pos   = millimeter_t(static_cast<float>(*json[8].as<int32_t>()));
    controlData.lineControl.target.angle = radian_t(*json[9].as<float>());
    car.isRemoteControlled               = !!(*json[10].as<int32_t>());
}

size_t store(char * const output, const size_t size, const ParamManager::NamedParam& namedParam) {
    const auto& [name, value] = namedParam;

    size_t n = micro::format_to_n(output, size, "{{");

    n += std::visit(
        [output, size, n, &name](const auto& v){
            if constexpr (std::is_floating_point_v<std::decay_t<decltype(v)>>) {
                return micro::format_to_n(&output[n], size - n, "\"{}\":{:.2f}", name.c_str(), v);
            }
            return micro::format_to_n(&output[n], size - n, "\"{}\":{}", name.c_str(), v);
        }, value);

    return n + micro::format_to_n(&output[n], size - n, "}}");
}

void load(char * const input, std::optional<ParamManager::NamedParam>& namedParam) {
    const auto json = micro::JSONParser(input).root();
    const auto param = *json.begin();

    if (!param.exists()) {
        namedParam = std::nullopt;
        return;
    }

    namedParam = ParamManager::NamedParam{param.key(), *parseParamValue(param)};
}

size_t store(char * const output, const size_t size, const IndexedSectionControlParameters& sectionControl) {
    const auto& [index, control] = sectionControl;

    return micro::format_to_n(output, size,
        "[{},{:.2f},{},{},{:.2f},{},{:.2f}]",
        index,
        control.speed.get(),
        static_cast<uint32_t>(std::lround(control.rampTime.get())),
        static_cast<int32_t>(std::lround(control.lineGradient.first.pos.get())),
        control.lineGradient.first.angle.get(),
        static_cast<int32_t>(std::lround(control.lineGradient.second.pos.get())),
        control.lineGradient.second.angle.get());
}

void load(char * const input, std::optional<IndexedSectionControlParameters>& sectionControl) {
    const auto json = micro::JSONParser(input).root();
    
    if (json.empty()) {
    	sectionControl = std::nullopt;
    	return;
    }

    sectionControl = IndexedSectionControlParameters{};
    auto& [index, control] = *sectionControl;

    index = *json[0].as<size_t>();
    control.speed = m_per_sec_t(*json[1].as<float>());
    control.rampTime = millisecond_t(*json[2].as<float>());
    control.lineGradient.first.pos = millimeter_t(*json[3].as<float>());
    control.lineGradient.first.angle = radian_t(*json[4].as<float>());
    control.lineGradient.second.pos = millimeter_t(*json[5].as<float>());
    control.lineGradient.second.angle = radian_t(*json[6].as<float>());
}

template <typename T>
size_t formatInput(char * const output, const size_t size, const Type type, const T& input) {
    auto n = formatType(output, size, type);
    n += store(&output[n], size - n, input);
    n += formatSeparator(&output[n], size - n);
    return n;
}

} // namespace



size_t DebugMessage::format(char * const output, const size_t size, const CarData& car) {
	return formatInput(output, size, Type::Car, car);
}

size_t DebugMessage::format(char * const output, const size_t size, const micro::ParamManager::NamedParam& param) {
	return formatInput(output, size, Type::Param, param);
}
size_t DebugMessage::format(char * const output, const size_t size, const IndexedSectionControlParameters& sectionControl) {
#if RACE_TRACK == TRACK
    const auto type = Type::RaceTrackControl;
#else
    const auto type = Type::TestTrackControl;
#endif

	return formatInput(output, size, type, sectionControl);
}

auto DebugMessage::parse(char * const input) -> std::optional<ParseResult> {
    Type type = Type::Unknown;
    const auto n = parseType(input, type);

    auto value = getParseValue(type);
    if (!value) {
        return std::nullopt;
    }

    std::visit(
        [input, n](auto& v){ load(&input[n], v); },
        *value);

    return value;
}
