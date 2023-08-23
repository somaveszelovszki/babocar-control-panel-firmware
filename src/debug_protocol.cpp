#include <debug_protocol.hpp>

#include <cmath>
#include <cstring>
#include <variant>

#include <ArduinoJson.h>

#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/types.hpp>

using namespace micro;

namespace {

size_t format(char* output, const size_t size, const JsonDocument& doc) {
    return serializeJson(doc, output, size);
}

} // namespace

size_t format(char* output, const size_t size, const DebugMessageType type) {
    if (size < 2) {
        return 0;
    }

    output[0] = micro::underlying_value(type);
    output[1] = ':';
    return 2;
}

size_t format(char* output, const size_t size, const DebugMessageSeparator&) {
    strncpy(output, DebugMessageSeparator::value, size);
    return std::min(size, strlen(DebugMessageSeparator::value));
}

size_t format(char* output, const size_t size, const CarProps& car, const ControlData& controlData) {
    StaticJsonDocument<100> doc;
    JsonArray items = doc.to<JsonArray>();

    items[0] = static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.pose.pos.X).get()));
    items[1] = static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.pose.pos.Y).get()));
    items[2] = car.pose.angle.get();
    items[3] = car.speed.get();
    items[4] = car.frontWheelAngle.get();
    items[5] = car.rearWheelAngle.get();
    items[6] = static_cast<int32_t>(std::lround(controlData.lineControl.actual.pos.get()));
    items[7] = controlData.lineControl.actual.angle.get();
    items[8] = static_cast<int32_t>(std::lround(controlData.lineControl.target.pos.get()));
    items[9] = controlData.lineControl.target.angle.get();
    items[10] = car.isRemoteControlled ? 1 : 0;

    return format(output, size, doc);
}

size_t format(char* output, const size_t size, const LapControlParameters& lapControl) {
    StaticJsonDocument<1000> doc;
    JsonArray sections = doc.to<JsonArray>();

    size_t i = 0;
    for (const auto& [name, control] : lapControl) {
        sections[i][0] = name.c_str();
        sections[i][1] = control.speed.get();
        sections[i][2] = static_cast<uint32_t>(std::lround(control.rampTime.get()));
        sections[i][3] = static_cast<int32_t>(std::lround(control.lineGradient.first.pos.get()));
        sections[i][4] = control.lineGradient.first.angle.get();
        sections[i][5] = static_cast<int32_t>(std::lround(control.lineGradient.second.pos.get()));
        sections[i][6] = control.lineGradient.second.angle.get();
        i++;
    }

    return format(output, size, doc);
}

size_t format(char* output, const size_t size, const micro::ParamManager::Values& params) {
    StaticJsonDocument<MAX_BUFFER_SIZE> doc;

    for (const auto& [name, param] : params) {
        std::visit([&doc, &name](const auto& v){ doc[name.c_str()] = v; }, param);
    }

    return format(output, size, doc);
}
