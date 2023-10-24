#include <algorithm>
#include <string>

#include <etl/string.h>

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

void expectEqual(const ParamManager::Values& expected, const ParamManager::Values& result) {
    ASSERT_EQ(expected.size(), result.size());
    for (const auto& [name, param] : result) {
        // TODO
    }
}

void expectEqual(const LapControlParameters& expected, const LapControlParameters& result) {
    ASSERT_EQ(expected.size(), result.size());
    for (const auto& [name, control] : expected) {
        EXPECT_EQ_TRACK_CONTROL_PARAMETERS(control, result.at(name));
    }
}

void testFormat(const DebugMessage::reference_type& data, const char * const expected) {
    char msg[100];
    const auto size = DebugMessage::format(msg, sizeof(msg), data);
    EXPECT_EQ(etl::strlen(expected), size);
    EXPECT_STREQ(expected, msg);
}

void testParse(const DebugMessage::value_type& expected, etl::string<512> json) {
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

TEST(DebugMessage, formatParams) {
    const ParamManager::Values params{
        std::make_pair(ParamManager::Name{"b"}, ParamManager::value_type{false}),
        std::make_pair(ParamManager::Name{"i8"}, ParamManager::value_type{static_cast<int8_t>(8)}),
        std::make_pair(ParamManager::Name{"i16"}, ParamManager::value_type{static_cast<int8_t>(16)}),
        std::make_pair(ParamManager::Name{"i32"}, ParamManager::value_type{static_cast<int8_t>(32)}),
        std::make_pair(ParamManager::Name{"u8"}, ParamManager::value_type{static_cast<uint8_t>(80)}),
        std::make_pair(ParamManager::Name{"u16"}, ParamManager::value_type{static_cast<uint16_t>(160)}),
        std::make_pair(ParamManager::Name{"u32"}, ParamManager::value_type{static_cast<uint32_t>(320)}),
        std::make_pair(ParamManager::Name{"f"}, ParamManager::value_type{1.2f})
    };

    testFormat(params, R"(P:{"b":false,"f":1.20,"i16":16,"i32":32,"i8":8,"u16":160,"u32":320,"u8":80})" "\r\n");
}

TEST(DebugMessage, parseParams) {
     const ParamManager::Values expected{
        std::make_pair(ParamManager::Name{"b"}, ParamManager::value_type{false}),
        std::make_pair(ParamManager::Name{"i8"}, ParamManager::value_type{static_cast<int8_t>(8)}),
        std::make_pair(ParamManager::Name{"i16"}, ParamManager::value_type{static_cast<int8_t>(16)}),
        std::make_pair(ParamManager::Name{"i32"}, ParamManager::value_type{static_cast<int8_t>(32)}),
        std::make_pair(ParamManager::Name{"u8"}, ParamManager::value_type{static_cast<uint8_t>(80)}),
        std::make_pair(ParamManager::Name{"u16"}, ParamManager::value_type{static_cast<uint16_t>(160)}),
        std::make_pair(ParamManager::Name{"u32"}, ParamManager::value_type{static_cast<uint32_t>(320)}),
        std::make_pair(ParamManager::Name{"f"}, ParamManager::value_type{1.2f})
    };

    testParse(expected, R"(P:{"b":false,"f":1.20,"i16":16,"i32":32,"i8":8,"u16":160,"u32":320,"u8":80})" "\r\n");
}

TEST(DebugMessage, formatTrackControl) {
    const LapControlParameters lapControl{
        std::make_pair(TrackSection::Name{"fast1"}, TrackSection::ControlParameters{
            m_per_sec_t{1.5f},
            millisecond_t{300},
            std::make_pair(OrientedLine{millimeter_t{5}, radian_t(0.2)},OrientedLine{millimeter_t{-5}, radian_t(-0.2)})
        })
    };

    testFormat(lapControl, TRACK_CONTROL_PREFIX_STR R"(:{"fast1":[1.50,300,5,0.20,-5,-0.20]})" "\r\n");
}

TEST(DebugMessage, parseTrackControl) {
    const LapControlParameters expected{
        std::make_pair(TrackSection::Name{"fast1"}, TrackSection::ControlParameters{
            m_per_sec_t{1.5f},
            millisecond_t{300},
            std::make_pair(OrientedLine{millimeter_t{5}, radian_t(0.2)},OrientedLine{millimeter_t{-5}, radian_t(-0.2)})
        })
    };

    testParse(expected, TRACK_CONTROL_PREFIX_STR R"(:{"fast1":[1.50,300,5,0.20,-5,-0.20]})" "\r\n");
}
