#include <LinePattern.hpp>
#include <globals.hpp>
#include <micro/utils/log.hpp>

namespace micro {

static constexpr meter_t ACCELERATE_SECTION_LENGTH   = centimeter_t(8);
static constexpr meter_t ACCELERATE_SPACE_LENGTH     = centimeter_t(8);
static constexpr meter_t ACCELERATE_VALIDITY_LENGTH  = centimeter_t(16);
static constexpr meter_t ACCELERATE_PATTERN_LENGTH   = centimeter_t(72);

static constexpr meter_t SINGLE_LINE_VALIDITY_LENGTH = centimeter_t(10);

static constexpr meter_t BRAKE_VALIDITY_LENGTH       = centimeter_t(20);
static constexpr meter_t BRAKE_PATTERN_LENGTH        = centimeter_t(300);

static constexpr meter_t LANE_CHANGE_VALIDITY_LENGTH = centimeter_t(20);
static constexpr meter_t JUNCTION_VALIDITY_LENGTH    = centimeter_t(20);
static constexpr meter_t DEAD_END_VALIDITY_LENGTH    = centimeter_t(20);

static const meter_t VALIDITY_LENGTHS[] = {
    SINGLE_LINE_VALIDITY_LENGTH,
    ACCELERATE_VALIDITY_LENGTH,
    BRAKE_VALIDITY_LENGTH,
    LANE_CHANGE_VALIDITY_LENGTH,
    JUNCTION_VALIDITY_LENGTH,
    DEAD_END_VALIDITY_LENGTH
};

void LinePatternCalculator::update(const LinePositions& front, const LinePositions& rear, const Lines& lines, meter_t currentDist) {
    this->prevMeas[this->currentMeasIdx = (this->currentMeasIdx + 1) % PREV_MEAS_SIZE] = { currentDist, front, rear, lines };

    if (this->isPatternChangeCheckActive) {
        for (vec<LinePattern, MAX_NUM_POSSIBLE_PATTERNS>::iterator it = this->possiblePatterns.begin(); it != this->possiblePatterns.end();) {

            if (this->isPatternValid(*it, front, rear, lines, currentDist)) {
                if (abs(currentDist - it->startDist) >= VALIDITY_LENGTHS[static_cast<uint8_t>(it->type)]) {
                    this->changePattern(*it);
                }
                ++it;
            } else {
                it = this->possiblePatterns.erase(it);

                if (!this->possiblePatterns.size()) {
                    this->isPatternChangeCheckActive = false;
                    LOG_DEBUG("All possible patterns are invalid, steps back to previous pattern");
                }
            }
        }
    } else {
        LinePattern& current = this->currentPattern();

        switch(current.type) {

        case LinePattern::SINGLE_LINE:
        {
            switch(globals::programState.activeModule()) {

            case ProgramState::ActiveModule::Labyrinth:
            {
                break;
            }

            case ProgramState::ActiveModule::RaceTrack:
            {
                if (3 == front.size()) {
                    this->startPatternChangeCheck({
                        { LinePattern::ACCELERATE, Sign::POSITIVE, Direction::CENTER, currentDist },
                        { LinePattern::BRAKE, Sign::POSITIVE, Direction::CENTER, currentDist }
                    });
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
            if (!this->isPatternValid(current, front, rear, lines, currentDist)) {
                this->startPatternChangeCheck({
                    { LinePattern::SINGLE_LINE, Sign::POSITIVE, Direction::CENTER, currentDist }
                });
            }
            break;
        }

        case LinePattern::BRAKE:
        {
            if (!this->isPatternValid(current, front, rear, lines, currentDist)) {
                this->startPatternChangeCheck({
                    { LinePattern::SINGLE_LINE, Sign::POSITIVE, Direction::CENTER, currentDist }
                });
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
    }
}

void LinePatternCalculator::changePattern(const LinePattern& newPattern) {
    this->prevPatterns[this->currentPatternIdx = (this->currentPatternIdx + 1) % PREV_PATTERNS_SIZE] = newPattern;
    this->isPatternChangeCheckActive = false;
    this->possiblePatterns.clear();

    LOG_DEBUG("Pattern changed from %d to %d",
            static_cast<int32_t>(this->prevPattern(1).type),
            static_cast<int32_t>(this->currentPattern().type));
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

bool LinePatternCalculator::isPatternValid(const LinePattern& pattern, const LinePositions& front, const LinePositions& rear, const Lines& lines, meter_t currentDist) {

    (void)rear;

    bool valid = false;

    switch (pattern.type) {
    case LinePattern::SINGLE_LINE:
        valid = 1 == front.size();
        break;

    case LinePattern::ACCELERATE:
        switch(front.size()) {
        case 1:
            valid = currentDist - pattern.startDist >= ACCELERATE_SECTION_LENGTH - centimeter_t(3) &&
                this->distanceSinceNumLinesIs(1, currentDist) < ACCELERATE_SPACE_LENGTH + centimeter_t(3);
            break;
        case 2:
            valid = true; // entering or leaving a triple-line (don't care)
            break;
        case 3:
            valid = this->distanceSinceNumLinesIs(3, currentDist) < ACCELERATE_SECTION_LENGTH + centimeter_t(3);
            break;
        }
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

void LinePatternCalculator::startPatternChangeCheck(const std::initializer_list<LinePattern>& possiblePatterns) {
    this->possiblePatterns = possiblePatterns;
    this->isPatternChangeCheckActive = true;
}

} // namespace micro
