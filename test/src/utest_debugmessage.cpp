#include <algorithm>
#include <string>

#include <etl/string.h>

#include <micro/debug/ParamManager.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/test/utils.hpp>

#include <DebugMessage.hpp>
#include <RaceTrackController.hpp>

using namespace micro;

namespace {

#if RACE_TRACK == TRACK
#define TRACK_CONTROL_PREFIX_STR "R"
#else
#define TRACK_CONTROL_PREFIX_STR "T"
#endif

void expectEqual(const std::tuple<CarProps, ControlData>& expected,
                 const std::tuple<CarProps, ControlData>& result) {
    EXPECT_EQ_MICRO_CAR_PROPS((std::get<0>(expected)), (std::get<0>(result)));
    EXPECT_EQ_MICRO_CONTROL_DATA((std::get<1>(expected)), (std::get<1>(result)));
}

void expectEqual(const std::optional<ParamManager::NamedParam>& expected,
                 const std::optional<ParamManager::NamedParam>& result) {
    ASSERT_EQ(expected.has_value(), result.has_value());
    if (!expected) {
        return;
    }

    EXPECT_EQ(expected->first, result->first);
    ASSERT_EQ(expected->second.index(), result->second.index());

    std::visit(
        [&result](const auto& exp){ EXPECT_EQ(exp, std::get<std::decay_t<decltype(exp)>>(result->second)); },
        expected->second);
}

void expectEqual(const IndexedSectionControlParameters& expected,
                 const IndexedSectionControlParameters& result) {
    EXPECT_EQ(expected.first, result.first);
    EXPECT_EQ_TRACK_CONTROL_PARAMETERS(expected.second, result.second);
}

template <typename T>
void testFormat(const T& data, const char * const expected) {
    char msg[100];
    const auto size = DebugMessage::format(msg, sizeof(msg), data);
    EXPECT_EQ(etl::strlen(expected), size);
    EXPECT_STREQ(expected, msg);
}

void testParse(const DebugMessage::ParseResult& expected, etl::string<512> json) {
    const auto result = DebugMessage::parse(const_cast<char*>(json.c_str()));
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(expected.index(), result->index());

    std::visit(
        [&result](const auto& exp){ expectEqual(exp, std::get<std::decay_t<decltype(exp)>>(*result)); },
        expected);
}

} // namespace

TEST(DebugMessage, formatCarProps) {
    std::tuple<CarProps, ControlData> data;
    auto& [car, controlData] = data;

    car.pose.pos.X = millimeter_t(1);
    car.pose.pos.Y = millimeter_t(2);
    car.pose.angle = radian_t(3.016);
    car.speed = m_per_sec_t(4);
    car.frontWheelAngle = radian_t(0.122);
    car.rearWheelAngle  = radian_t(-0.218);
    car.isRemoteControlled = false;

    controlData.lineControl.actual.pos = millimeter_t(5);
    controlData.lineControl.actual.angle = radian_t(-0.324);
    controlData.lineControl.target.pos = millimeter_t(6);
    controlData.lineControl.target.angle = radian_t(0.4);

    testFormat(data, "C:[1,2,3.02,4.00,0.12,-0.22,5,-0.32,6,0.40,0]\r\n");
}

TEST(DebugMessage, parseCarProps) {
    std::tuple<CarProps, ControlData> expected;
    auto& [car, controlData] = expected;

    car.pose.pos.X = millimeter_t(1);
    car.pose.pos.Y = millimeter_t(2);
    car.pose.angle = radian_t(3.02);
    car.speed = m_per_sec_t(4);
    car.frontWheelAngle = radian_t(0.12);
    car.rearWheelAngle  = radian_t(-0.22);
    car.isRemoteControlled = false;

    controlData.lineControl.actual.pos = millimeter_t(5);
    controlData.lineControl.actual.angle = radian_t(-0.32);
    controlData.lineControl.target.pos = millimeter_t(6);
    controlData.lineControl.target.angle = radian_t(0.4);

    testParse(expected, "C:[1,2,3.02,4.00,0.12,-0.22,5,-0.32,6,0.40,0]\r\n");
}

TEST(DebugMessage, formatParam) {
    testFormat(ParamManager::NamedParam{"b", false}, R"(P:{"b":false})" "\r\n");
    testFormat(ParamManager::NamedParam{"i8", static_cast<int8_t>(8)}, R"(P:{"i8":8})" "\r\n");
    testFormat(ParamManager::NamedParam{"i16", static_cast<int16_t>(16)}, R"(P:{"i16":16})" "\r\n");
    testFormat(ParamManager::NamedParam{"i32", static_cast<int32_t>(32)}, R"(P:{"i32":32})" "\r\n");
    testFormat(ParamManager::NamedParam{"u8", static_cast<uint8_t>(80)}, R"(P:{"u8":80})" "\r\n");
    testFormat(ParamManager::NamedParam{"u16", static_cast<uint16_t>(160)}, R"(P:{"u16":160})" "\r\n");
    testFormat(ParamManager::NamedParam{"u32", static_cast<uint32_t>(320)}, R"(P:{"u32":320})" "\r\n");
    testFormat(ParamManager::NamedParam{"f", 1.2f}, R"(P:{"f":1.20})" "\r\n");
}

TEST(DebugMessage, parseParams) {
    testParse(std::make_optional(ParamManager::NamedParam{"b", false}), R"(P:{"b":false})" "\r\n");
    testParse(std::make_optional(ParamManager::NamedParam{"i8", 8}), R"(P:{"i8":8})" "\r\n");
    testParse(std::make_optional(ParamManager::NamedParam{"i16", 16}), R"(P:{"i16":16})" "\r\n");
    testParse(std::make_optional(ParamManager::NamedParam{"i32", 32}), R"(P:{"i32":32})" "\r\n");
    testParse(std::make_optional(ParamManager::NamedParam{"u8", 80}), R"(P:{"u8":80})" "\r\n");
    testParse(std::make_optional(ParamManager::NamedParam{"u16", 160}), R"(P:{"u16":160})" "\r\n");
    testParse(std::make_optional(ParamManager::NamedParam{"u32", 320}), R"(P:{"u32":320})" "\r\n");
    testParse(std::make_optional(ParamManager::NamedParam{"f", 1.2f}), R"(P:{"f":1.20})" "\r\n");
    testParse(std::optional<ParamManager::NamedParam>(), R"(P:{})" "\r\n");
}

TEST(DebugMessage, formatTrackControl) {
    const IndexedSectionControlParameters sectionControl{
        1, TrackSection::ControlParameters{
            m_per_sec_t{1.5f},
            millisecond_t{300},
            std::make_pair(OrientedLine{millimeter_t{5}, radian_t(0.2)},OrientedLine{millimeter_t{-5}, radian_t(-0.2)})
        }
    };

    testFormat(sectionControl, TRACK_CONTROL_PREFIX_STR R"(:{"1":[1.50,300,5,0.20,-5,-0.20]})" "\r\n");
}

TEST(DebugMessage, parseTrackControl) {
    const IndexedSectionControlParameters expected{
        1, TrackSection::ControlParameters{
            m_per_sec_t{1.5f},
            millisecond_t{300},
            std::make_pair(OrientedLine{millimeter_t{5}, radian_t(0.2)},OrientedLine{millimeter_t{-5}, radian_t(-0.2)})
        }
    };

    testParse(expected, TRACK_CONTROL_PREFIX_STR R"(:{"1":[1.50,300,5,0.20,-5,-0.20]})" "\r\n");
}
