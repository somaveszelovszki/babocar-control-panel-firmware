#include "micro/utils/ControlData.hpp"
#include <micro/utils/Line.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/units.hpp>

#include <micro/test/utils.hpp>

#include <cfg_car.hpp>
#include <RaceTrackController.hpp>

using namespace micro;
using namespace testing;

namespace {

constexpr auto LINE_OFFSET = centimeter_t(10);
constexpr auto LINE_ANGLE = degree_t(15);

class LapTrackSectionProvider : public ILapTrackSectionProvider {
public:
    LapTrackSectionProvider(const bool enableLineOffset, const bool enableLineAngle)
        : enableLineOffset_{enableLineOffset}
        , enableLineAngle_{enableLineAngle} {}

    LapTrackSections operator()(const size_t) override {
        const auto offset = enableLineOffset_ ? LINE_OFFSET : centimeter_t(0);
        const auto angle = enableLineAngle_ ? LINE_ANGLE : degree_t(0);

        return LapTrackSections{
            {
                true,
                meter_t{5},
                TrackSection::TransitionCriteria::pattern(LinePattern::BRAKE),
                TrackSection::ControlParameters{
                m_per_sec_t(3),
                millisecond_t(500),
                {
                    OrientedLine{centimeter_t(0), degree_t(0)},
                    OrientedLine{offset, angle}
                }
                }
            },
            {
                false,
                meter_t{1},
                TrackSection::TransitionCriteria::distance(),
                TrackSection::ControlParameters{
                m_per_sec_t(1),
                millisecond_t(500),
                {
                    OrientedLine{offset, angle},
                    OrientedLine{-offset, -angle}
                }
                }
            },
            {
                false,
                meter_t{2},
                TrackSection::TransitionCriteria::acceleration(),
                TrackSection::ControlParameters{
                m_per_sec_t(2),
                millisecond_t(500),
                {
                    OrientedLine{-offset, -angle},
                    OrientedLine{centimeter_t(0), degree_t(0)}
                }
                }
            }
        };
    }

    ~LapTrackSectionProvider() = default;

private:
    bool enableLineOffset_;
    bool enableLineAngle_;
};

class RaceTrackControllerTestBase : public Test {
public:
   RaceTrackControllerTestBase(const bool enableLineOffset, const bool enableLineAngle)
      : lapTrackSectionProvider_{enableLineOffset, enableLineAngle}
      , trackController_{lapTrackSectionProvider_} {
      setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), degree_t(0)});
   }

protected:
   void setLine(const LinePattern::type_t pattern, const OrientedLine& centerLine) {
      lineInfo_.front.pattern = { pattern, Sign::NEUTRAL, Direction::CENTER, car_.distance };
      mainLine_.centerLine = centerLine;
   }

   void checkUpdate(const size_t lap, const size_t sectionIdx, const ControlData& expectedControl) {
      const auto result = trackController_.update(car_, lineInfo_, mainLine_);
      EXPECT_EQ(lap, trackController_.lap());
      EXPECT_EQ(sectionIdx, trackController_.sectionIndex());
      EXPECT_EQ_MICRO_CONTROL_DATA(expectedControl, result);
   }

protected:
   LapTrackSectionProvider lapTrackSectionProvider_;
   RaceTrackController trackController_;
   CarProps car_;
   LineInfo lineInfo_;
   MainLine mainLine_{cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST};
};

class RaceTrackControllerCenterLineTest : public RaceTrackControllerTestBase {
public:
   RaceTrackControllerCenterLineTest() : RaceTrackControllerTestBase(false, false) {}
};

class RaceTrackControllerOffsetTest : public RaceTrackControllerTestBase {
public:
   RaceTrackControllerOffsetTest() : RaceTrackControllerTestBase(true, false) {}
};

class RaceTrackControllerAngleTest : public RaceTrackControllerTestBase {
public:
   RaceTrackControllerAngleTest() : RaceTrackControllerTestBase(false, true) {}
};

TEST_F(RaceTrackControllerCenterLineTest, sameSection) {
   trackController_.initialize(car_, 1, 0);

   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});

   car_.distance = meter_t(3);
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
}

TEST_F(RaceTrackControllerCenterLineTest, transitionPatternBeforeSectionDistance) {
   trackController_.initialize(car_, 1, 0);
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(3);
   setLine(LinePattern::BRAKE, {centimeter_t(0), degree_t(0)});
   checkUpdate(1, 1,{m_per_sec_t(1), millisecond_t(500), true, {}});
}

TEST_F(RaceTrackControllerCenterLineTest, transitionPatternAfterSectionDistance) {
   trackController_.initialize(car_, 1, 0);
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(5.5f);
   setLine(LinePattern::BRAKE, {centimeter_t(0), degree_t(0)});
   checkUpdate(1, 1,{m_per_sec_t(1), millisecond_t(500), true, {}});
}

TEST_F(RaceTrackControllerCenterLineTest, transitionDistanceToleranceExceeded) {
   trackController_.initialize(car_, 1, 0);
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(7);
   checkUpdate(1, 1,{m_per_sec_t(1), millisecond_t(500), true, {}});
}

TEST_F(RaceTrackControllerCenterLineTest, transitionDistance) {
   car_.distance = meter_t(3);
   trackController_.initialize(car_, 1, 1);
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {}});
   
   car_.distance = meter_t(3.5f);
   checkUpdate(1, 1,{m_per_sec_t(1), millisecond_t(500), true, {}});
   
   car_.distance = meter_t(4.05f);
   checkUpdate(1, 2,{m_per_sec_t(2), millisecond_t(500), true, {}});
}

TEST_F(RaceTrackControllerCenterLineTest, transitionAcceleration) {
   car_.distance = meter_t(3);
   trackController_.initialize(car_, 1, 2);
   checkUpdate(1, 2, {m_per_sec_t(2), millisecond_t(500), true, {}});
   
   car_.distance = meter_t(3.5f);
   car_.orientedDistance = centimeter_t(50);
   setLine(LinePattern::ACCELERATE, {centimeter_t(0), degree_t(0)});
   checkUpdate(2, 0,{m_per_sec_t(3), millisecond_t(500), false, {}});
}

TEST_F(RaceTrackControllerCenterLineTest, limitFastSpeed) {
   trackController_.initialize(car_, 1, 0);
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(1);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), degree_t(0)});
   checkUpdate(1, 0,{m_per_sec_t(3), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(1.5f);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(12), degree_t(0)});
   checkUpdate(1, 0,{m_per_sec_t(1.5f), millisecond_t(500), false, {
      OrientedLine{centimeter_t(12), degree_t(0)},
      OrientedLine{centimeter_t(0), degree_t(0)}
   }});
   
   car_.distance = meter_t(2);
   car_.orientedDistance = centimeter_t(10);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), degree_t(0)});
   checkUpdate(1, 0,{m_per_sec_t(1.5f), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(2.5f);
   car_.orientedDistance = centimeter_t(60);
   checkUpdate(1, 0,{m_per_sec_t(3), millisecond_t(500), false, {}});
}

TEST_F(RaceTrackControllerOffsetTest, interpolateUntilPattern) {
   trackController_.initialize(car_, 1, 0);
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(2.5f);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), degree_t(0)});
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {
      OrientedLine{centimeter_t(0), degree_t(0)},
      OrientedLine{LINE_OFFSET / 2, degree_t(0)}
   }});
   
   car_.distance = meter_t(5);
   setLine(LinePattern::SINGLE_LINE, {LINE_OFFSET, degree_t(0)});
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {
      OrientedLine{LINE_OFFSET, degree_t(0)},
      OrientedLine{LINE_OFFSET, degree_t(0)}
   }});

   car_.distance = meter_t(5.01f);
   setLine(LinePattern::BRAKE, {LINE_OFFSET, degree_t(0)});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{LINE_OFFSET, degree_t(0)},
      OrientedLine{LINE_OFFSET, degree_t(0)}
   }});
}

TEST_F(RaceTrackControllerOffsetTest, interpolateUntilDistance) {
   trackController_.initialize(car_, 1, 1);
   setLine(LinePattern::SINGLE_LINE, {LINE_OFFSET, degree_t(0)});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{LINE_OFFSET, degree_t(0)},
      OrientedLine{LINE_OFFSET, degree_t(0)}
   }});
   
   car_.distance = meter_t(0.5f);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(1), degree_t(0)});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{centimeter_t(1), degree_t(0)},
      OrientedLine{centimeter_t(0), degree_t(0)}
   }});
   
   car_.distance = meter_t(0.75);
   setLine(LinePattern::SINGLE_LINE, {-LINE_OFFSET / 2, degree_t(0)});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{-LINE_OFFSET / 2, degree_t(0)},
      OrientedLine{-LINE_OFFSET / 2, degree_t(0)}
   }});
   
   car_.distance = meter_t(1.1);
   setLine(LinePattern::SINGLE_LINE, {-LINE_OFFSET, degree_t(0)});
   checkUpdate(1, 2, {m_per_sec_t(2), millisecond_t(500), true, {
      OrientedLine{-LINE_OFFSET, degree_t(0)},
      OrientedLine{-LINE_OFFSET, degree_t(0)}
   }});
}

TEST_F(RaceTrackControllerAngleTest, interpolateUntilPattern) {
   trackController_.initialize(car_, 1, 0);
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {}});
   
   car_.distance = meter_t(2.5f);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), degree_t(0)});
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {
      OrientedLine{centimeter_t(0), degree_t(0)},
      OrientedLine{centimeter_t(0), LINE_ANGLE / 2}
   }});
   
   car_.distance = meter_t(5);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), LINE_ANGLE});
   checkUpdate(1, 0, {m_per_sec_t(3), millisecond_t(500), false, {
      OrientedLine{centimeter_t(0), LINE_ANGLE},
      OrientedLine{centimeter_t(0), LINE_ANGLE}
   }});

   car_.distance = meter_t(5.01f);
   setLine(LinePattern::BRAKE, {centimeter_t(0), LINE_ANGLE});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{centimeter_t(0), LINE_ANGLE},
      OrientedLine{centimeter_t(0), LINE_ANGLE}
   }});
}

TEST_F(RaceTrackControllerAngleTest, interpolateUntilDistance) {
   trackController_.initialize(car_, 1, 1);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), LINE_ANGLE});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{centimeter_t(0), LINE_ANGLE},
      OrientedLine{centimeter_t(0), LINE_ANGLE}
   }});
   
   car_.distance = meter_t(0.5f);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), degree_t(1)});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{centimeter_t(0), degree_t(1)},
      OrientedLine{centimeter_t(0), degree_t(0)}
   }});
   
   car_.distance = meter_t(0.75);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), -LINE_ANGLE / 2});
   checkUpdate(1, 1, {m_per_sec_t(1), millisecond_t(500), true, {
      OrientedLine{centimeter_t(0), -LINE_ANGLE / 2},
      OrientedLine{centimeter_t(0), -LINE_ANGLE / 2}
   }});
   
   car_.distance = meter_t(1.1);
   setLine(LinePattern::SINGLE_LINE, {centimeter_t(0), -LINE_ANGLE});
   checkUpdate(1, 2, {m_per_sec_t(2), millisecond_t(500), true, {
      OrientedLine{centimeter_t(0), -LINE_ANGLE},
      OrientedLine{centimeter_t(0), -LINE_ANGLE}
   }});
}

} // namespace
