#include <LinePattern.hpp>
#include <micro/utils/log.hpp>
#include <micro/container/map.hpp>
#include <micro/container/vec.hpp>

#include <cfg_track.hpp>

namespace micro {

class LinePatternDescriptor {
public:
    typedef vec<uint8_t, MAX_NUM_LINES + 1> lines;

    LinePatternDescriptor(const std::initializer_list<std::pair<uint8_t, centimeter_t>>& pattern)
        : pattern(pattern) {}

    lines getValidLines(Sign dir, centimeter_t patternDist, centimeter_t eps) const {
        lines validLines;

        centimeter_t d = centimeter_t(0);
        for (uint32_t i = 0; i < this->pattern.size(); ++i) {
            const std::pair<uint8_t, centimeter_t>& entry = this->pattern[Sign::POSITIVE == dir ? i : this->pattern.size() - 1 - i];
            if (patternDist >= d - eps) {
                if (patternDist <= d + entry.second + eps) {
                    if (!validLines.find(entry.first)) {
                        validLines.append(entry.first);
                    }
                }
            } else {
                break;
            }
            d += entry.second;
        }

        return validLines;
    }
private:
    vec<std::pair<uint8_t, centimeter_t>, 20> pattern;
};

const LinePatternCalculator::LinePatternInfo PATTERN_INFO[] = {
    { // NONE
        centimeter_t(10),
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const Lines& lines, meter_t) {
            return 0 == lines.size();
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
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const Lines& lines, meter_t) {
            return 1 == lines.size();
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
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, meter_t currentDist) {

            static const LinePatternDescriptor descriptor = {
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) },
                { 1, centimeter_t(8) },
                { 3, centimeter_t(8) }
            };

            const LinePatternDescriptor::lines validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(3));
            return std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end() && LinePatternCalculator::areClose(lines);
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
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern& pattern, const Lines& lines, meter_t currentDist) {
            static constexpr meter_t PATTERN_LENGTH = centimeter_t(300);
            return currentDist - pattern.startDist < PATTERN_LENGTH + centimeter_t(10) && 3 == lines.size() && LinePatternCalculator::areClose(lines);
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
        centimeter_t(40),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, meter_t currentDist) {

            static const LinePatternDescriptor descriptor = {
                { 2, centimeter_t(16) },
                { 1, centimeter_t(14) },
                { 2, centimeter_t(14) },
                { 1, centimeter_t(12) },
                { 2, centimeter_t(12) },
                { 1, centimeter_t(10) },
                { 2, centimeter_t(10) },
                { 1, centimeter_t(8)  },
                { 2, centimeter_t(8)  }
            };

            bool valid = false;

            const LinePatternDescriptor::lines validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(3));
            if (std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end()) {

                if (1 == lines.size()) {
                    valid = true;
                } else if (2 == lines.size()) {
                    Lines::const_iterator mainLine = LinePatternCalculator::getMainLine(lines, prevMeas);
                    if (Direction::RIGHT == pattern.side) {
                        valid = *mainLine == lines[0];
                    } else if (Direction::LEFT == pattern.side) {
                        valid = *mainLine == lines[1];
                    }
                }
            }

            return valid;
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
        centimeter_t(5),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, meter_t) {
            bool valid = false;
            const LinePatternCalculator::StampedLines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(10));

            if (Sign::POSITIVE == pattern.dir) {
                if (1 == lines.size()) {
                    valid = 1 < pastLines.lines.size() && LinePatternCalculator::areClose(pastLines.lines);
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (1 < lines.size() && LinePatternCalculator::areClose(lines)) {
                    valid = 1 == pastLines.lines.size();
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_2
        centimeter_t(5),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, meter_t) {
            bool valid = false;
            const LinePatternCalculator::StampedLines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(10));

            if (Sign::POSITIVE == pattern.dir) {
                if (2 == lines.size() && LinePatternCalculator::areFar(lines)) {
                    valid = 1 < pastLines.lines.size() && LinePatternCalculator::areClose(pastLines.lines);
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (1 < lines.size() && LinePatternCalculator::areClose(lines)) {
                    if (2 == pastLines.lines.size() && LinePatternCalculator::areFar(pastLines.lines)) {

                        Lines::const_iterator mainLine = LinePatternCalculator::getMainLine(pastLines.lines, prevMeas);
                        if (Direction::RIGHT == pattern.side) {
                            valid = *mainLine == pastLines.lines[0];
                        } else if (Direction::LEFT == pattern.side) {
                            valid = *mainLine == pastLines.lines[1];
                        }
                    }
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_3
        centimeter_t(5),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, meter_t) {
            bool valid = false;
            const LinePatternCalculator::StampedLines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(10));

            if (Sign::POSITIVE == pattern.dir) {
                if (3 == lines.size() && LinePatternCalculator::areFar(lines)) {
                    valid = 1 < pastLines.lines.size() && LinePatternCalculator::areClose(pastLines.lines);
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (1 < lines.size() && LinePatternCalculator::areClose(lines)) {
                    if (3 == pastLines.lines.size() && LinePatternCalculator::areFar(pastLines.lines)) {

                        Lines::const_iterator mainLine = LinePatternCalculator::getMainLine(pastLines.lines, prevMeas);
                        if (Direction::RIGHT == pattern.side) {
                            valid = *mainLine == pastLines.lines[0];
                        } else if (Direction::CENTER == pattern.side) {
                            valid = *mainLine == pastLines.lines[1];
                        } else if (Direction::LEFT == pattern.side) {
                            valid = *mainLine == pastLines.lines[2];
                        }
                    }
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.append({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.append({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // DEAD_END
        centimeter_t(3),
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern&, const Lines& lines, meter_t) {
            bool valid = false;
            if (0 == lines.size()) {
                const LinePatternCalculator::StampedLines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(15));
                valid = 1 == pastLines.lines.size() && isBtw(pastLines.lines[0].pos, centimeter_t(5), centimeter_t(-5));
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramState programState) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramState::ActiveModule::Labyrinth == programState.activeModule()) {
                validPatterns.append({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    }
};

void LinePatternCalculator::update(const ProgramState programState, const Lines& lines, meter_t currentDist) {
    this->prevMeas.push_back({ lines, currentDist });

    LinePattern& current = this->currentPattern();

    if (this->isPatternChangeCheckActive) {
        for (linePatterns_t::iterator it = possiblePatterns.begin(); it != possiblePatterns.end();) {

            if (this->currentPatternInfo->isValid(this->prevMeas, *it, lines, currentDist)) {
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

        if (!currentPatternInfo->isValid(this->prevMeas, current, lines, currentDist)) {
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

LinePatternCalculator::StampedLines LinePatternCalculator::peek_back(const measurement_buffer_t& prevMeas, std::function<bool(const StampedLines&)> condition) {
    StampedLines lines;
    for (uint32_t i = 1; i < prevMeas.capacity(); ++i) {
        const StampedLines& l = prevMeas.peek_back(i);
        if (condition(l)) {
            lines = l;
            break;
        }
    }
    return lines;
}

LinePatternCalculator::StampedLines LinePatternCalculator::peek_back(const measurement_buffer_t& prevMeas, meter_t peekBackDist) {
    const meter_t dist = prevMeas.peek_back(0).distance - peekBackDist;
    return LinePatternCalculator::peek_back(prevMeas, [dist](const StampedLines& l) { return l.distance < dist; });
}

Lines::const_iterator LinePatternCalculator::getMainLine(const Lines& lines, const measurement_buffer_t& prevMeas) {
    const Line lastSingleLine = LinePatternCalculator::peek_back(prevMeas, [](const LinePatternCalculator::StampedLines& l) {
        return 1 == l.lines.size();
    }).lines[0];

    Lines::const_iterator mainLine = LineCalculator::findLine(lines, lastSingleLine.id);
    if (mainLine == lines.end()) {
        mainLine = LineCalculator::findClosestLine(lines, millimeter_t(0));
    }

    return mainLine;
}

bool LinePatternCalculator::areClose(const Lines& lines) {
    constexpr centimeter_t MAX_CLOSE_LINES_DISTANCE = centimeter_t(5.5f);

    bool close = true;
    for (uint32_t i = 1; i < lines.size(); ++i) {
        if (lines[i].pos - lines[i - 1].pos > MAX_CLOSE_LINES_DISTANCE) {
            close = false;
            break;
        }
    }
    return close;
}

bool LinePatternCalculator::areFar(const Lines& lines) {
    constexpr centimeter_t MIN_FAR_LINES_DISTANCE = centimeter_t(7.5f);

    bool close = true;
    for (uint32_t i = 1; i < lines.size(); ++i) {
        if (lines[i].pos - lines[i - 1].pos < MIN_FAR_LINES_DISTANCE) {
            close = false;
            break;
        }
    }
    return close;
}

} // namespace micro
