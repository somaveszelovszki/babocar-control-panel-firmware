#include <cmath>
#include <cstring>
#include <string>
#include <variant>

#include <ArduinoJson.h>
#include <etl/string.h>

#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/types.hpp>
#include <micro/utils/str_utils.hpp>

#include <DebugMessage.hpp>

using namespace micro;

namespace {

constexpr auto SEPARATOR_SIZE = std::char_traits<char>::length(micro::Log::SEPARATOR);

auto serializeFloat(float value) {
    value = std::llround(value * 100) / 100.0f + micro::sgn(value) * 0.000001f;
    char str[12] = "";
    micro::ftoa(value, str, sizeof(str), 2);
    return serialized(etl::string<sizeof(str)>(str));
}
} // namespace

size_t DebugMessage::format(char * const output, const size_t size, const reference_type& data) {
    return std::visit(
        [output, size, &data](const auto& v){
            DebugMessage msg;
            msg.store(v);

            auto n = msg.formatType(output, size, getType(data));
            n += serializeJson(msg.jsonDoc_, &output[n], size - n);
            n += msg.formatSeparator(&output[n], size - n);
            return n;
        }, data);
}

auto DebugMessage::parse(const char * const input) -> value_type {
    // TODO
    return std::monostate{};
}

auto DebugMessage::getType(const reference_type& data) -> Type {
    switch (data.index()) {
    case 0:  return Type::Car;
    case 1:  return Type::Params;
    case 2:  return Type::TrackControl;
    default: return Type::Unknown;
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
    items[2]  = serializeFloat(car.pose.angle.get());
    items[3]  = serializeFloat(car.speed.get());
    items[4]  = serializeFloat(car.frontWheelAngle.get());
    items[5]  = serializeFloat(car.rearWheelAngle.get());
    items[6]  = static_cast<int32_t>(std::lround(controlData.lineControl.actual.pos.get()));
    items[7]  = serializeFloat(controlData.lineControl.actual.angle.get());
    items[8]  = static_cast<int32_t>(std::lround(controlData.lineControl.target.pos.get()));
    items[9]  = serializeFloat(controlData.lineControl.target.angle.get());
    items[10] = car.isRemoteControlled ? 1 : 0;
}

void DebugMessage::load(std::tuple<CarProps, ControlData>& data) {
    auto& [car, controlData] = data;
    auto items = jsonDoc_.to<JsonArray>();

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
                jsonDoc_[name.c_str()] = serializeFloat(v);
            } else {
                jsonDoc_[name.c_str()] = v;
            }
        }, param);
    }
}

void DebugMessage::load(ParamManager::Values& params) {
    // TODO
}

void DebugMessage::store(const LapControlParameters& lapControl) {
    auto sections = jsonDoc_.to<JsonArray>();

    size_t i = 0;
    for (const auto& [name, control] : lapControl) {
        sections[i][0] = name.c_str();
        sections[i][1] = serializeFloat(control.speed.get());
        sections[i][2] = static_cast<uint32_t>(std::lround(control.rampTime.get()));
        sections[i][3] = static_cast<int32_t>(std::lround(control.lineGradient.first.pos.get()));
        sections[i][4] = serializeFloat(control.lineGradient.first.angle.get());
        sections[i][5] = static_cast<int32_t>(std::lround(control.lineGradient.second.pos.get()));
        sections[i][6] = serializeFloat(control.lineGradient.second.angle.get());
        i++;
    }
}

void DebugMessage::load(LapControlParameters& lapControl) {
    // TODO
}
