#include <micro/test/utils.hpp>

#include <cfg_car.hpp>
#include <RaceTrackController.hpp>

using namespace micro;

namespace {

RaceTrackController makeRaceTrackController() {
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
               OrientedLine{centimeter_t(0), radian_t(0)},
               OrientedLine{centimeter_t(5), degree_t(10)}
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
               OrientedLine{centimeter_t(5), radian_t(10)},
               OrientedLine{centimeter_t(-5), degree_t(-10)}
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
               OrientedLine{centimeter_t(-5), degree_t(-10)},
               OrientedLine{centimeter_t(0), radian_t(0)}
            }
         }
      }
   };

   return RaceTrackController({sections, sections}); // same track section parameters for 2 laps
}

} // namespace

TEST(RaceTrackController, test) {
   auto trackController = makeRaceTrackController();

   CarProps car;
   LineInfo lineInfo;
   MainLine mainLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

   trackController.setSection(car, 1, 0);

   car.orientedDistance = centimeter_t(50);
}
