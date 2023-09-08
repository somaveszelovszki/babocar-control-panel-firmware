#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <variant>

#include <ArduinoJson.h>
#include <etl/string.h>

#include <micro/debug/ParamManager.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/types.hpp>
#include <micro/utils/str_utils.hpp>
#include <micro/utils/units.hpp>

#include <DebugMessage.hpp>

using namespace micro;

namespace {

constexpr auto SEPARATOR_SIZE = std::char_traits<char>::length(micro::Log::SEPARATOR);

auto formatFloat(float value) {
    value = std::llround(value * 100) / 100.0f + micro::sgn(value) * 0.000001f;
    char str[12] = "";
    micro::ftoa(value, str, sizeof(str), 2);
    return serialized(etl::string<sizeof(str)>(str));
}

ParamManager::value_type parseParamValue(const JsonVariant& value) {
    if (value.is<bool>()) {
        return value.as<bool>();
    } else if (value.is<uint8_t>()) {
        return value.as<uint8_t>();
    } else if (value.is<uint16_t>()) {
        return value.as<uint16_t>();
    } else if (value.is<uint32_t>()) {
        return value.as<uint32_t>();
    } else if (value.is<int8_t>()) {
        return value.as<int8_t>();
    } else if (value.is<int16_t>()) {
        return value.as<int16_t>();
    } else if (value.is<int32_t>()) {
        return value.as<int32_t>();
    } else if (value.is<float>()) {
        return value.as<float>();
    }

    throw std::runtime_error{"Unknown param value type"};
}

} // namespace

size_t DebugMessage::format(char * const output, const size_t size, const reference_type& value) {
    auto n = formatType(output, size, getType(value));
    std::visit(
        [output, size, &n](const auto& v){
            DebugMessage msg;
            msg.store(v);
            n += serializeJson(msg.jsonDoc_, &output[n], size - n);
        }, value);

    n += formatSeparator(&output[n], size - n);
    return n;
}

auto DebugMessage::parse(const char * const input) -> value_type {
    Type type = Type::Unknown;
    auto n = parseType(input, type);

    auto value = getValue(type);

    std::visit(
        [input, n](auto& v){
            DebugMessage msg;
            deserializeJson(msg.jsonDoc_, &input[n]);
            msg.load(v);
        }, value);

    return value;
}

auto DebugMessage::getType(const reference_type& value) -> Type {
    switch (value.index()) {
    case 0:  return Type::Car;
    case 1:  return Type::Params;
    case 2:  return Type::TrackControl;
    default: return Type::Unknown;
    }
}

auto DebugMessage::getValue(const Type type) -> value_type {
    switch (type) {
    case Type::Car:          return std::tuple<CarProps, ControlData>{};
    case Type::Params:       return ParamManager::Values{};
    case Type::TrackControl: return LapControlParameters{};
    default:                 throw std::runtime_error{"Invalid DebugMessage type"};
    }
}

size_t DebugMessage::formatType(char* output, const size_t size, const Type type) {
    if (size < 2) {
        return 0;
    }

    output[0] = micro::underlying_value(type);
    output[1] = ':';
    return 2;
}

size_t DebugMessage::parseType(const char * const input, Type& type) {
    if (strnlen(input, 2) < 2 || input[1] != ':') {
        return 0;
    }

    type = static_cast<Type>(input[0]);
    return 2;
}

size_t DebugMessage::formatSeparator(char* output, const size_t size) {
    if (size < SEPARATOR_SIZE) {
        return 0;
    }

    strcpy(output, Log::SEPARATOR);
    return SEPARATOR_SIZE;
}

size_t DebugMessage::parseSeparator(const char * const input) {
    if (strnlen(input, SEPARATOR_SIZE) < SEPARATOR_SIZE
        || !!strncmp(input, Log::SEPARATOR, SEPARATOR_SIZE)) {
        return 0;
    }

    return SEPARATOR_SIZE;
}

void DebugMessage::store(const std::tuple<CarProps, ControlData>& data) {
    const auto& [car, controlData] = data;
    auto items = jsonDoc_.to<JsonArray>();

    items[0]  = static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.pose.pos.X).get()));
    items[1]  = static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.pose.pos.Y).get()));
    items[2]  = formatFloat(car.pose.angle.get());
    items[3]  = formatFloat(car.speed.get());
    items[4]  = formatFloat(car.frontWheelAngle.get());
    items[5]  = formatFloat(car.rearWheelAngle.get());
    items[6]  = static_cast<int32_t>(std::lround(controlData.lineControl.actual.pos.get()));
    items[7]  = formatFloat(controlData.lineControl.actual.angle.get());
    items[8]  = static_cast<int32_t>(std::lround(controlData.lineControl.target.pos.get()));
    items[9]  = formatFloat(controlData.lineControl.target.angle.get());
    items[10] = car.isRemoteControlled ? 1 : 0;
}

void DebugMessage::load(std::tuple<CarProps, ControlData>& data) {
    auto& [car, controlData] = data;
    auto items = jsonDoc_.as<JsonArray>();

    car.pose.pos.X      = millimeter_t(static_cast<float>(items[0].as<int32_t>()));
    car.pose.pos.Y      = millimeter_t(static_cast<float>(items[1].as<int32_t>()));
    car.pose.angle      = radian_t(items[2].as<float>());
    car.speed           = m_per_sec_t(items[3].as<float>());
    car.frontWheelAngle = radian_t(items[4].as<float>());
    car.rearWheelAngle  = radian_t(items[5].as<float>());
    controlData.lineControl.actual.pos   = millimeter_t(static_cast<float>(items[6].as<int32_t>()));
    controlData.lineControl.actual.angle = radian_t(items[7].as<float>());
    controlData.lineControl.target.pos   = millimeter_t(static_cast<float>(items[8].as<int32_t>()));
    controlData.lineControl.target.angle = radian_t(items[9].as<float>());
    car.isRemoteControlled               = !!items[10].as<int32_t>();
}

void DebugMessage::store(const ParamManager::Values& params) {
    for (const auto& [name, param] : params) {
        std::visit([this, &name](const auto& v){
            if constexpr (std::is_same_v<std::decay_t<decltype(v)>, float>) {
                jsonDoc_[name.c_str()] = formatFloat(v);
            } else {
                jsonDoc_[name.c_str()] = v;
            }
        }, param);
    }
}

void DebugMessage::load(ParamManager::Values& params) {
    for (auto entry : jsonDoc_.as<JsonObject>()) {
        params.insert(std::make_pair(ParamManager::Name{
            entry.key().c_str(), entry.key().size()}, parseParamValue(entry.value())));
    }
}

void DebugMessage::store(const LapControlParameters& lapControl) {

    size_t i = 0;
    for (const auto& [name, control] : lapControl) {
        auto section = jsonDoc_[name.c_str()];
        section[0] = formatFloat(control.speed.get());
        section[1] = static_cast<uint32_t>(std::lround(control.rampTime.get()));
        section[2] = static_cast<int32_t>(std::lround(control.lineGradient.first.pos.get()));
        section[3] = formatFloat(control.lineGradient.first.angle.get());
        section[4] = static_cast<int32_t>(std::lround(control.lineGradient.second.pos.get()));
        section[5] = formatFloat(control.lineGradient.second.angle.get());
        i++;
    }
}

void DebugMessage::load(LapControlParameters& lapControl) {
    for (auto entry : jsonDoc_.as<JsonObject>()) {
        auto items = entry.value().as<JsonArray>();

        TrackSection::ControlParameters control;
        control.speed = m_per_sec_t(items[0].as<float>());
        control.rampTime = millisecond_t(items[1].as<float>());
        control.lineGradient.first.pos = millimeter_t(items[2].as<float>());
        control.lineGradient.first.angle = radian_t(items[3].as<float>());
        control.lineGradient.second.pos = millimeter_t(items[4].as<float>());
        control.lineGradient.second.angle = radian_t(items[5].as<float>());

        lapControl.insert(std::make_pair(ParamManager::Name{
            entry.key().c_str(), entry.key().size()}, control));
    }
}
