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

RaceTrackSections buildSections() {
   LapTrackSections sections{
      {
         "fast",
         true,
         meter_t{5},
         TrackSection::TransitionCriteria::pattern(LinePattern::BRAKE),
         TrackSection::ControlParameters{
            m_per_sec_t(3),
            millisecond_t(500),
            {
               OrientedLine{centimeter_t(0), degree_t(0)},
               OrientedLine{centimeter_t(0), degree_t(0)}
            }
         }
      },
      {
         "slow1",
         false,
         meter_t{1},
         TrackSection::TransitionCriteria::distance(),
         TrackSection::ControlParameters{
            m_per_sec_t(1),
            millisecond_t(500),
            {
               OrientedLine{centimeter_t(0), degree_t(0)},
               OrientedLine{centimeter_t(0), degree_t(0)}
            }
         }
      },
      {
         "slow2",
         false,
         meter_t{2},
         TrackSection::TransitionCriteria::acceleration(),
         TrackSection::ControlParameters{
            m_per_sec_t(2),
            millisecond_t(500),
            {
               OrientedLine{centimeter_t(0), degree_t(0)},
               OrientedLine{centimeter_t(0), degree_t(0)}
            }
         }
      }
   };

   return {sections, sections}; // same track section parameters for 2 laps
}

class RaceTrackControllerTest : public Test {
public:
   RaceTrackControllerTest() {
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
   RaceTrackController trackController_{buildSections()};
   CarProps car_;
   LineInfo lineInfo_;
   MainLine mainLine_{cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST};
};

TEST_F(RaceTrackControllerTest, sameSection) {
   trackController_.setSection(car_, 1, 0);

   checkUpdate(1, 0, {
      m_per_sec_t(3), millisecond_t(500), false, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
   
   checkUpdate(1, 0, {
      m_per_sec_t(3), millisecond_t(500), false, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});

   car_.distance = meter_t(3);

   checkUpdate(1, 0, {
      m_per_sec_t(3), millisecond_t(500), false, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
}

TEST_F(RaceTrackControllerTest, transitionPatternBeforeSectionDistance) {
   trackController_.setSection(car_, 1, 0);

   checkUpdate(1, 0, {
      m_per_sec_t(3), millisecond_t(500), false, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
   
   car_.distance = meter_t(3);
   setLine(LinePattern::BRAKE, {centimeter_t(0), degree_t(0)});
   
   checkUpdate(1, 1,{
      m_per_sec_t(1), millisecond_t(500), true, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
}

TEST_F(RaceTrackControllerTest, transitionPatternAfterSectionDistance) {
   trackController_.setSection(car_, 1, 0);

   checkUpdate(1, 0, {
      m_per_sec_t(3), millisecond_t(500), false, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
   
   car_.distance = meter_t(5.5f);
   setLine(LinePattern::BRAKE, {centimeter_t(0), degree_t(0)});
   
   checkUpdate(1, 1,{
      m_per_sec_t(1), millisecond_t(500), true, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
}

TEST_F(RaceTrackControllerTest, transitionDistanceToleranceExceeded) {
   trackController_.setSection(car_, 1, 0);

   checkUpdate(1, 0, {
      m_per_sec_t(3), millisecond_t(500), false, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
   
   car_.distance = meter_t(7);
   
   checkUpdate(1, 1,{
      m_per_sec_t(1), millisecond_t(500), true, {
         OrientedLine{centimeter_t(0), radian_t(0)},
         OrientedLine{centimeter_t(0), radian_t(0)}
      }});
}

} // namespace
