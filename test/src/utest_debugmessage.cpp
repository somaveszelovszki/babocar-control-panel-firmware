#include <algorithm>
#include <string>

#include <etl/string.h>

#include <micro/math/unit_utils.hpp>
#include <micro/test/utils.hpp>

#include <DebugMessage.hpp>

using namespace micro;

namespace {

void testFormat(const DebugMessage::reference_type& data, const etl::string<sizeof(Log::message_t)>& expected) {
    Log::message_t msg;
    const auto size = DebugMessage::format(msg, sizeof(msg), data);
    EXPECT_EQ(expected.size(), size);
    EXPECT_STREQ(expected.c_str(), msg);
}

} // namespace

TEST(DebugMessage, formatCarProps) {
    std::tuple<CarProps, ControlData> data;
    auto& [car, controlData] = data;

    car.pose.pos.X = millimeter_t(1);
    car.pose.pos.Y = millimeter_t(2);
    car.pose.angle = radian_t(3);
    car.speed = m_per_sec_t(4);
    car.frontWheelAngle = radian_t(0.1);
    car.rearWheelAngle  = radian_t(0.2);
    car.isRemoteControlled = false;

    controlData.lineControl.actual.pos = millimeter_t(5);
    controlData.lineControl.actual.angle = radian_t(0.3);
    controlData.lineControl.target.pos = millimeter_t(6);
    controlData.lineControl.target.angle = radian_t(0.4);

    testFormat(data, "C:[1,2,3.00,4.00,0.10,0.20,5,0.30,6,0.40,0]\r\n");
}
