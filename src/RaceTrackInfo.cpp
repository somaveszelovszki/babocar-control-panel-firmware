#include <micro/port/timer.hpp>
#include <micro/utils/log.hpp>
#include <RaceTrackInfo.hpp>

using namespace micro;

RaceTrackInfo::RaceTrackInfo(const TrackSegments& segments)
    : segments(segments)
    , seg(segments.end())
    , lap(0) {}

void RaceTrackInfo::update(const CarProps& car, const LineInfo& lineInfo, const MainLine& mainLine, const micro::ControlData& controlData) {
    TrackSegments::const_iterator nextSeg = this->nextSegment();
    if (nextSeg->hasBecomeActive(car, *this, (car.speed >= m_per_sec_t(0) ? lineInfo.front : lineInfo.rear).pattern) ||
        (this->lap >= 4 && car.distance - this->segStartCarProps.distance > this->seg->length + meter_t(1.5f))) {
        this->seg                 = nextSeg;
        this->segStartCarProps    = car;
        this->segStartControlData = controlData;
        this->segStartLine        = mainLine.centerLine;

        if (this->segments.begin() == this->seg) {
            LOG_INFO("Lap %u finished (time: %f seconds)", static_cast<uint32_t>(this->lap), static_cast<second_t>(getTime() - this->lapStartTime).get());
            ++this->lap;
            this->lapStartTime = getTime();
        }
        LOG_INFO("Segment %u became active (lap: %u)", static_cast<uint32_t>(std::distance(this->segments.begin(), this->seg)), static_cast<uint32_t>(this->lap));
    }
}

TrackSegments::const_iterator RaceTrackInfo::nextSegment() const {
    return this->segments.back() == this->seg ? this->segments.begin() : std::next(this->seg);
}
