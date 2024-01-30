#include <algorithm>
#include <gtest/gtest.h>
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

void expectEqual(const DebugMessage::CarData& expected,
                 const DebugMessage::CarData& result) {
    EXPECT_EQ_MICRO_CAR_PROPS(expected.props, result.props);
    EXPECT_EQ_MICRO_CONTROL_DATA(expected.control, result.control);
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

void expectEqual(const std::optional<IndexedSectionControlParameters>& expected,
                 const std::optional<IndexedSectionControlParameters>& result) {
    ASSERT_EQ(expected.has_value(), result.has_value());
    if (!expected) {
        return;
    }

    EXPECT_EQ(expected->first, result->first);
    EXPECT_EQ_TRACK_CONTROL_PARAMETERS(expected->second, result->second);
}

void expectEqual(const DebugMessage::RadioCommand& expected,
                 const DebugMessage::RadioCommand& result) {
    EXPECT_STREQ(expected.text, result.text);
}

template <typename T>
void testFormat(const T& data, const char * const expected) {
    char msg[100];
    const auto size = DebugMessage::format(msg, sizeof(msg), data);
    EXPECT_EQ(etl::strlen(expected), size);
    EXPECT_STREQ(expected, msg);
}

template <typename T>
void testParse(const T& expected, etl::string<512> json) {
	T result{};
    const auto success = DebugMessage::parse(const_cast<char*>(json.c_str()), result);
    EXPECT_TRUE(success);
    expectEqual(expected, result);
}

template <typename T>
void testParseFailure(etl::string<512> json) {
	T result{};
    const auto success = DebugMessage::parse(const_cast<char*>(json.c_str()), result);
    EXPECT_FALSE(success);
}

} // namespace

TEST(DebugMessage, formatCarProps) {
	DebugMessage::CarData car;

    car.props.pose.pos.X = millimeter_t(1);
    car.props.pose.pos.Y = millimeter_t(2);
    car.props.pose.angle = radian_t(3.016);
    car.props.speed = m_per_sec_t(4);
    car.props.frontWheelAngle = radian_t(0.122);
    car.props.rearWheelAngle  = radian_t(-0.218);
    car.props.isRemoteControlled = false;

    car.control.lineControl.actual.pos = millimeter_t(5);
    car.control.lineControl.actual.angle = radian_t(-0.324);
    car.control.lineControl.target.pos = millimeter_t(6);
    car.control.lineControl.target.angle = radian_t(0.4);

    testFormat(car, "C:[1,2,3.02,4.00,0.12,-0.22,5,-0.32,6,0.40,0]\n");
}

TEST(DebugMessage, parseFailure) {
    testParseFailure<DebugMessage::CarData>("P:{}\n");
}

TEST(DebugMessage, parseCarProps) {
	DebugMessage::CarData car;

    car.props.pose.pos.X = millimeter_t(1);
    car.props.pose.pos.Y = millimeter_t(2);
    car.props.pose.angle = radian_t(3.02);
    car.props.speed = m_per_sec_t(4);
    car.props.frontWheelAngle = radian_t(0.12);
    car.props.rearWheelAngle  = radian_t(-0.22);
    car.props.isRemoteControlled = false;

    car.control.lineControl.actual.pos = millimeter_t(5);
    car.control.lineControl.actual.angle = radian_t(-0.32);
    car.control.lineControl.target.pos = millimeter_t(6);
    car.control.lineControl.target.angle = radian_t(0.4);

    testParse(car, "C:[1,2,3.02,4.00,0.12,-0.22,5,-0.32,6,0.40,0]\n");
}

TEST(DebugMessage, formatParam) {
    testFormat(ParamManager::NamedParam{"b", false}, R"(P:{"b":false})" "\n");
    testFormat(ParamManager::NamedParam{"i8", static_cast<int8_t>(8)}, R"(P:{"i8":8})" "\n");
    testFormat(ParamManager::NamedParam{"i16", static_cast<int16_t>(16)}, R"(P:{"i16":16})" "\n");
    testFormat(ParamManager::NamedParam{"i32", static_cast<int32_t>(32)}, R"(P:{"i32":32})" "\n");
    testFormat(ParamManager::NamedParam{"u8", static_cast<uint8_t>(80)}, R"(P:{"u8":80})" "\n");
    testFormat(ParamManager::NamedParam{"u16", static_cast<uint16_t>(160)}, R"(P:{"u16":160})" "\n");
    testFormat(ParamManager::NamedParam{"u32", static_cast<uint32_t>(320)}, R"(P:{"u32":320})" "\n");
    testFormat(ParamManager::NamedParam{"f", 1.2f}, R"(P:{"f":1.20})" "\n");
}

TEST(DebugMessage, parseParam) {
    testParse(std::make_optional(ParamManager::NamedParam{"b", false}), R"(P:{"b":false})" "\n");
    testParse(std::make_optional(ParamManager::NamedParam{"i8", 8}), R"(P:{"i8":8})" "\n");
    testParse(std::make_optional(ParamManager::NamedParam{"i16", 16}), R"(P:{"i16":16})" "\n");
    testParse(std::make_optional(ParamManager::NamedParam{"i32", 32}), R"(P:{"i32":32})" "\n");
    testParse(std::make_optional(ParamManager::NamedParam{"u8", 80}), R"(P:{"u8":80})" "\n");
    testParse(std::make_optional(ParamManager::NamedParam{"u16", 160}), R"(P:{"u16":160})" "\n");
    testParse(std::make_optional(ParamManager::NamedParam{"u32", 320}), R"(P:{"u32":320})" "\n");
    testParse(std::make_optional(ParamManager::NamedParam{"f", 1.2f}), R"(P:{"f":1.20})" "\n");
    testParse(std::optional<ParamManager::NamedParam>(), "P:{}\n");
}

TEST(DebugMessage, formatTrackControl) {
    const IndexedSectionControlParameters sectionControl{
        1, TrackSection::ControlParameters{
            m_per_sec_t{1.5f},
            millisecond_t{300},
            std::make_pair(OrientedLine{millimeter_t{5}, radian_t(0.2)},OrientedLine{millimeter_t{-5}, radian_t(-0.2)})
        }
    };

    testFormat(sectionControl, TRACK_CONTROL_PREFIX_STR ":[1,1.50,300,5,0.2000,-5,-0.2000]\n");
}

TEST(DebugMessage, parseTrackControl) {
    const IndexedSectionControlParameters expected{
        1, TrackSection::ControlParameters{
            m_per_sec_t{1.5f},
            millisecond_t{300},
            std::make_pair(OrientedLine{millimeter_t{5}, radian_t(0.2)},OrientedLine{millimeter_t{-5}, radian_t(-0.2)})
        }
    };

    testParse(std::make_optional(expected), TRACK_CONTROL_PREFIX_STR ":[1,1.50,300,5,0.2000,-5,-0.2000]\n");
    testParse(std::optional<IndexedSectionControlParameters>(), TRACK_CONTROL_PREFIX_STR ":[]\n");
}

TEST(DebugMessage, formatRadioCommand) {
    const DebugMessage::RadioCommand command{"FLOOD!"};
    testFormat(command, "O:\"FLOOD!\"\n");
}

TEST(DebugMessage, parseRadioCommand) {
    const DebugMessage::RadioCommand expected{"FLOOD!"};
    testParse(expected, "O:\"FLOOD!\"\n");
}
