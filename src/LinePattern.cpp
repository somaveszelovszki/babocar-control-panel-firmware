#include <LinePattern.hpp>
#include <micro/utils/log.hpp>
#include <micro/container/map.hpp>

namespace micro {

static const LinePatternCalculator::LinePatternInfo PATTERN_INFO[] = {
    { // NONE
        centimeter_t(10),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const LinePositions& front, const LinePositions&, meter_t) {
            return 0 == front.size();
        },
        [] (const LinePattern&, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });

            } else if (ProgramState::ActiveModule::RaceTrack == programState.activeModule()) {
                validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                validPatterns.append({ LinePattern::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER });
                validPatterns.append({ LinePattern::BRAKE,       Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // SINGLE_LINE
        centimeter_t(10),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const LinePositions& front, const LinePositions&, meter_t) {
            return 1 == front.size();
        },
        [] (const LinePattern&, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                validPatterns.append({ LinePattern::NONE,       Sign::NEUTRAL,  Direction::CENTER });
                validPatterns.append({ LinePattern::JUNCTION_1, Sign::NEGATIVE, Direction::CENTER });
                validPatterns.append({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::LEFT   });
                validPatterns.append({ LinePattern::JUNCTION_2, Sign::NEGATIVE, Direction::RIGHT  });
                validPatterns.append({ LinePattern::JUNCTION_3, Sign::NEGATIVE, Direction::LEFT   });
                validPatterns.append({ LinePattern::JUNCTION_3, Sign::NEGATIVE, Direction::CENTER });
                validPatterns.append({ LinePattern::JUNCTION_3, Sign::NEGATIVE, Direction::RIGHT  });

            } else if (ProgramState::ActiveModule::RaceTrack == programState.activeModule()) {
                validPatterns.append({ LinePattern::NONE,       Sign::NEUTRAL, Direction::CENTER });
                validPatterns.append({ LinePattern::ACCELERATE, Sign::NEUTRAL, Direction::CENTER });
                validPatterns.append({ LinePattern::BRAKE,      Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // ACCELERATE
        centimeter_t(16),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const LinePositions& front, const LinePositions&, meter_t currentDist) {

            static constexpr meter_t SECTION_LENGTH = centimeter_t(8);
            static constexpr meter_t SPACE_LENGTH   = centimeter_t(8);

            bool valid = false;
            switch(front.size()) {
            case 1:
                valid = currentDist - pattern.startDist >= SECTION_LENGTH - centimeter_t(3) &&
                    LinePatternCalculator::distanceSinceNumLines(prevMeas, 1, currentDist) < SPACE_LENGTH + centimeter_t(3);
                break;
            case 2:
                valid = true; // entering or leaving a triple-line (don't care)
                break;
            case 3:
                valid = LinePatternCalculator::distanceSinceNumLines(prevMeas, 3, currentDist) < SECTION_LENGTH + centimeter_t(3);
                break;
            }
            return valid;
        },
        [] (const LinePattern&, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::RaceTrack == programState.activeModule()) {
                validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // BRAKE
        centimeter_t(20),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const LinePositions& front, const LinePositions&, meter_t) {
            return 3 == front.size();
        },
        [] (const LinePattern&, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::RaceTrack == programState.activeModule()) {
                validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // LANE_CHANGE
        centimeter_t(16),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const LinePositions& front, const LinePositions&, meter_t) {
            // TODO
            return false;
        },
        [] (const LinePattern&, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                validPatterns.append({ LinePattern::NONE,        Sign::NEUTRAL, Direction::CENTER });
                validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // JUNCTION_1
        centimeter_t(20),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern& pattern, const LinePositions& front, const LinePositions& rear, meter_t) {
            // TODO
            return false;
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::LEFT   });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::LEFT   });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::RIGHT  });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_2
        centimeter_t(20),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern& pattern, const LinePositions& front, const LinePositions& rear, meter_t) {
            // TODO
            return false;
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::LEFT   });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::LEFT   });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::RIGHT  });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_3
        centimeter_t(20),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern& pattern, const LinePositions& front, const LinePositions& rear, meter_t) {
            // TODO
            return false;
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::LEFT   });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::LEFT   });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::RIGHT  });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // DEAD_END
        centimeter_t(3),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const LinePositions& front, const LinePositions& rear, meter_t) {
            return 0 == front.size() && 1 == rear.size() && isBtw(rear[0], centimeter_t(2), centimeter_t(-2));
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                validPatterns.append({ LinePattern::NONE,        Sign::NEUTRAL, Direction::CENTER });
                validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    }
};

void LinePatternCalculator::update(const ProgramState programState, const LinePositions& front, const LinePositions& rear, meter_t currentDist) {
    this->prevMeas.push_back({ front, rear, currentDist });

    LinePattern& current = this->currentPattern();

    if (this->isPatternChangeCheckActive) {
        for (linePatterns_t::iterator it = possiblePatterns.begin(); it != possiblePatterns.end();) {

            if (this->currentPatternInfo->isValid(this->prevMeas, *it, front, rear, currentDist)) {
                if (abs(currentDist - it->startDist) >= this->currentPatternInfo->validityLength) {
                    this->changePattern(*it);
                    break;
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

        if (!currentPatternInfo->isValid(this->prevMeas, current, front, rear, currentDist)) {
            this->isPatternChangeCheckActive = true;
            this->possiblePatterns = this->currentPatternInfo->validNextPatterns(current, programState);
        }
    }
}

void LinePatternCalculator::changePattern(const LinePattern& newPattern) {
    this->prevPatterns.push_back(newPattern);
    this->isPatternChangeCheckActive = false;
    this->currentPatternInfo = &PATTERN_INFO[static_cast<uint8_t>(newPattern.type)];

    LOG_DEBUG("Pattern changed from %d to %d",
            static_cast<int32_t>(this->prevPatterns.peek_back(1).type),
            static_cast<int32_t>(this->currentPattern().type));
}

meter_t LinePatternCalculator::distanceSinceNumLines(const measurement_buffer_t& prevMeas, uint8_t numLines, meter_t currentDist) {
    meter_t dist = meter_t::zero();

    for (uint32_t i = 0; i < prevMeas.capacity(); ++i) {
        const StampedLines& l = prevMeas.peek_back(i);
        if (l.front.size() != numLines) {
            break;
        }
        dist = currentDist - l.distance;
    }

    return dist;
}

} // namespace micro
