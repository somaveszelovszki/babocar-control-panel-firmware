#include <LinePattern.hpp>
#include <globals.hpp>

namespace micro {

static constexpr meter_t ACCELERATE_SECTION_LENGTH   = centimeter_t(8);
static constexpr meter_t ACCELERATE_SPACE_LENGTH     = centimeter_t(8);
static constexpr meter_t ACCELERATE_VALIDITY_LENGTH  = centimeter_t(16);
static constexpr meter_t ACCELERATE_PATTERN_LENGTH   = centimeter_t(72);

static constexpr meter_t VALIDITY_LENGTH_SINGLE_LINE = centimeter_t(20);

static constexpr meter_t BRAKE_VALIDITY_LENGTH       = centimeter_t(20);
static constexpr meter_t BRAKE_PATTERN_LENGTH        = centimeter_t(300);

void LinePatternCalculator::update(meter_t currentDist, const LinePositions& front, const LinePositions& rear, const Lines& lines) {
    this->prevMeas[currentMeasIdx] = { currentDist, front, rear, lines };

    switch(this->currentPattern.type) {

    case LinePattern::SINGLE_LINE:
    {
        switch(globals::programState.activeModule()) {

        case ProgramState::ActiveModule::Labyrinth:
        {
            break;
        }

        case ProgramState::ActiveModule::RaceTrack:
        {
            if (front.size() == 3) {
                this->isPatternChangeCheckNeeded = true;
            }

            if (this->isPatternChangeCheckNeeded) {
                if (this->isPatternValid(LinePattern::ACCELERATE, front, rear, lines, currentDist)) {
                    this->changePattern({ LinePattern::ACCELERATE, Sign::POSITIVE, Direction::CENTER, currentDist });
                } else if (this->isPatternValid(LinePattern::BRAKE, front, rear, lines, currentDist)) {
                    this->changePattern({ LinePattern::BRAKE, Sign::POSITIVE, Direction::CENTER, currentDist });
                }
            }
            break;
        }

        default:
            break;
        }

        break;
    }

    case LinePattern::ACCELERATE:
    {
        if (this->isCurrentPatternValidated) {
            if (abs(currentDist - this->currentPattern.startDist) >= ACCELERATE_PATTERN_LENGTH + centimeter_t(5)) {
                this->changePattern({ LinePattern::SINGLE_LINE, Sign::POSITIVE, Direction::CENTER, currentDist });
            }
        } else if (this->isPatternValid(LinePattern::ACCELERATE, front, rear, lines, currentDist)) {
            if (abs(currentDist - this->currentPattern.startDist) >= ACCELERATE_VALIDITY_LENGTH) {
                this->isCurrentPatternValidated = true;
            }
        } else {
            this->changePattern({ LinePattern::SINGLE_LINE, Sign::POSITIVE, Direction::CENTER, currentDist });
        }
        break;
    }

    case LinePattern::BRAKE:
    {
        if (this->isCurrentPatternValidated) {
            if (abs(currentDist - this->currentPattern.startDist) >= BRAKE_PATTERN_LENGTH + centimeter_t(5)) {
                this->changePattern({ LinePattern::SINGLE_LINE, Sign::POSITIVE, Direction::CENTER, currentDist });
            }
        } else if (!this->isPatternValid(LinePattern::BRAKE, front, rear, lines, currentDist)) {
            this->changePattern({ LinePattern::SINGLE_LINE, Sign::POSITIVE, Direction::CENTER, currentDist });
        } else if (abs(currentDist - this->currentPattern.startDist) >= BRAKE_VALIDITY_LENGTH) {
            this->isCurrentPatternValidated = true;
        }
        break;
    }

    case LinePattern::LANE_CHANGE:
    {
        break;
    }

    case LinePattern::JUNCTION:
    {
        break;
    }

    case LinePattern::DEAD_END:
    {
        break;
    }

    default:
        break;
    }

    currentMeasIdx = (currentMeasIdx + 1) % PREV_MEAS_SIZE;
}

void LinePatternCalculator::changePattern(LinePattern newPattern) {
    if (this->isCurrentPatternValidated) {
        this->prevPrevPattern = this->prevPattern;
        this->prevPattern = this->currentPattern;
    }
    this->currentPattern = newPattern;
    this->isCurrentPatternValidated = false;
    this->isPatternChangeCheckNeeded = false;
}

meter_t LinePatternCalculator::distanceSinceNumLinesIs(uint8_t numLines, meter_t currentDist) const {
    meter_t dist = meter_t::zero();
    uint8_t i = this->currentMeasIdx;

    do {
        const StampedLines& l = this->prevMeas[i];
        if (l.front.size() != numLines) {
            break;
        }
        dist = currentDist - l.distance;

    } while ((i = i > 0 ? i - 1 : PREV_MEAS_SIZE - 1) != this->currentMeasIdx);

    return dist;
}

bool LinePatternCalculator::isPatternValid(LinePattern::Type patternType, const LinePositions& front, const LinePositions& rear, const Lines& lines, meter_t currentDist) {
    bool valid = false;

    switch (patternType) {
    case LinePattern::SINGLE_LINE:
        valid = 1 == front.size();
        break;

    case LinePattern::ACCELERATE:
        valid = (3 == front.size() && this->distanceSinceNumLinesIs(3, currentDist) < ACCELERATE_SECTION_LENGTH + centimeter_t(4)) ||
                (1 == front.size() && this->distanceSinceNumLinesIs(1, currentDist) < ACCELERATE_SPACE_LENGTH + centimeter_t(4));
        break;

    case LinePattern::BRAKE:
        valid = 3 == front.size();
        break;

    case LinePattern::LANE_CHANGE:
        valid = false;
        break;

    case LinePattern::JUNCTION:
        valid = false;
        break;

    case LinePattern::DEAD_END:
        valid = false;
        break;

    default:
        valid = false;
        break;
    }

    return valid;
}


} // namespace micro
