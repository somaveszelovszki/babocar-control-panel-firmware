#include <LinePattern.hpp>
#include <micro/utils/log.hpp>
#include <micro/container/map.hpp>
#include <micro/container/vec.hpp>
#include <micro/panel/line.h>

#include <cfg_track.hpp>

namespace micro {

constexpr meter_t MAX_CLOSE_LINES_DISTANCE = centimeter_t(5.2f);

bool isInJunctionCenter(const Lines& lines) {
    return 1 < lines.size() && LinePatternCalculator::areClose(lines);
}

class LinePatternDescriptor {
public:
    typedef vec<uint8_t, MAX_NUM_LINES + 1> lines;

    LinePatternDescriptor(const std::initializer_list<std::pair<uint8_t, centimeter_t>>& pattern)
        : pattern(pattern) {}

    lines getValidLines(Sign dir, centimeter_t patternDist, centimeter_t eps) const {
        lines validLines;

        centimeter_t d = centimeter_t(0);
        for (uint32_t i = 0; i < this->pattern.size(); ++i) {
            const std::pair<uint8_t, centimeter_t>& entry = this->pattern[Sign::POSITIVE == dir || Sign::NEUTRAL == dir ? i : this->pattern.size() - 1 - i];
            if (patternDist >= d - eps) {
                if (patternDist <= d + entry.second + eps) {
                    if (std::find(validLines.begin(), validLines.end(), entry.first) == validLines.end()) {
                        validLines.push_back(entry.first);
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
        meter_t::infinity(),
        LinePatternCalculator::LinePatternInfo::NO_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const Lines& lines, uint8_t, meter_t) {
            return 0 == lines.size();
        },
        [] (const LinePattern&, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::Labyrinth == activeTask) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });

            } else if (ProgramTask::RaceTrack == activeTask) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::ACCELERATE,  Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::BRAKE,       Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // SINGLE_LINE
        centimeter_t(7),
        meter_t::infinity(),
        LinePatternCalculator::LinePatternInfo::NO_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const Lines& lines, uint8_t, meter_t) {
            return 1 == lines.size();
        },
        [] (const LinePattern&, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::Labyrinth == activeTask) {
                validPatterns.push_back({ LinePattern::JUNCTION_1,  Sign::NEGATIVE, Direction::CENTER });
                validPatterns.push_back({ LinePattern::JUNCTION_2,  Sign::NEGATIVE, Direction::LEFT   });
                validPatterns.push_back({ LinePattern::JUNCTION_2,  Sign::NEGATIVE, Direction::RIGHT  });
                validPatterns.push_back({ LinePattern::JUNCTION_3,  Sign::NEGATIVE, Direction::LEFT   });
                validPatterns.push_back({ LinePattern::JUNCTION_3,  Sign::NEGATIVE, Direction::CENTER });
                validPatterns.push_back({ LinePattern::JUNCTION_3,  Sign::NEGATIVE, Direction::RIGHT  });
                validPatterns.push_back({ LinePattern::LANE_CHANGE, Sign::POSITIVE, Direction::RIGHT  });
                validPatterns.push_back({ LinePattern::LANE_CHANGE, Sign::NEGATIVE, Direction::LEFT   });
                validPatterns.push_back({ LinePattern::DEAD_END,    Sign::NEUTRAL,  Direction::CENTER });

            } else if (ProgramTask::RaceTrack == activeTask) {
                validPatterns.push_back({ LinePattern::NONE,       Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::ACCELERATE, Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::BRAKE,      Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // ACCELERATE
        centimeter_t(16),
        centimeter_t(85),
        LinePatternCalculator::LinePatternInfo::NO_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, uint8_t, meter_t currentDist) {

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

            LinePatternDescriptor::lines validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(3));
            validLines.push_back(2);
            return std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end();
        },
        [] (const LinePattern&, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::RaceTrack == activeTask) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // BRAKE
        centimeter_t(16),
        centimeter_t(350),
        LinePatternCalculator::LinePatternInfo::NO_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t&, const LinePattern& pattern, const Lines& lines, uint8_t, meter_t currentDist) {
            static constexpr meter_t PATTERN_LENGTH = centimeter_t(300);
            return currentDist - pattern.startDist < PATTERN_LENGTH + centimeter_t(5) && 3 == lines.size();
        },
        [] (const LinePattern&, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::RaceTrack == activeTask) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // LANE_CHANGE
        centimeter_t(30),
        centimeter_t(120),
        LinePatternCalculator::LinePatternInfo::NO_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, uint8_t, meter_t currentDist) {

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

            const LinePatternDescriptor::lines validLines = descriptor.getValidLines(pattern.dir, currentDist - pattern.startDist, centimeter_t(3));
            return std::find(validLines.begin(), validLines.end(), lines.size()) != validLines.end() && LinePatternCalculator::areClose(lines);
        },
        [] (const LinePattern&, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::Labyrinth == activeTask) {
                validPatterns.push_back({ LinePattern::NONE,        Sign::NEUTRAL, Direction::CENTER });
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    },
    { // JUNCTION_1
        centimeter_t(3),
        centimeter_t(100),
        LinePatternCalculator::LinePatternInfo::USES_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, uint8_t, meter_t currentDist) {
            bool valid = false;
            const Lines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(20)).lines;

            if (Sign::POSITIVE == pattern.dir) {
                if (isInJunctionCenter(lines)) {
                    valid = true;
                } else if (1 == lines.size() && isInJunctionCenter(pastLines)) {
                    valid = true;
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (isInJunctionCenter(lines) && 1 == pastLines.size()) {
                    valid = true;
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::Labyrinth == activeTask) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_2
        centimeter_t(3),
        centimeter_t(100),
        LinePatternCalculator::LinePatternInfo::USES_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, uint8_t lastSingleLineId, meter_t) {
            bool valid = false;
            const Lines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(8)).lines;

            static const std::function<bool(const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const Lines&, uint8_t lastSingleLineId)> areValidFarLines = []
                (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, uint8_t lastSingleLineId) {

                bool valid = false;
                if (2 == lines.size() && LinePatternCalculator::areFar(lines)) {
                    Lines::const_iterator mainLine = LinePatternCalculator::getMainLine(lines, lastSingleLineId);
                    if (Direction::RIGHT == pattern.side) {
                        valid = *mainLine == lines[0];
                    } else if (Direction::LEFT == pattern.side) {
                        valid = *mainLine == lines[1];
                    }
                }
                return valid;
            };

            if (Sign::POSITIVE == pattern.dir) {
                if (isInJunctionCenter(lines)) {
                    valid = true;
                } else if (2 == lines.size() && LinePatternCalculator::areFar(lines) && isInJunctionCenter(pastLines)) {
                    valid = true;
                }
//                else {
//                    LOG_WAIT_ERROR("not J2: numLines: %d", (int)lines.size());
//                    for (Line l : lines) {
//                        LOG_WAIT_ERROR("not J2: pos: %fmm", l.pos.get());
//                    }
//                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (2 == lines.size() && areValidFarLines(prevMeas, pattern, lines, lastSingleLineId)) {
                    valid = true;
                } else if (2 == lines.size() && isInJunctionCenter(lines) && areValidFarLines(prevMeas, pattern, pastLines, lastSingleLineId)) {
                    valid = true;
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::Labyrinth == activeTask) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // JUNCTION_3
        centimeter_t(3),
        centimeter_t(100),
        LinePatternCalculator::LinePatternInfo::USES_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, uint8_t lastSingleLineId, meter_t) {
            bool valid = false;
            const Lines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(8)).lines;

            static const std::function<bool(const LinePatternCalculator::measurement_buffer_t&, const LinePattern&, const Lines&, uint8_t lastSingleLineId)> areValidFarLines = []
                (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern& pattern, const Lines& lines, uint8_t lastSingleLineId) {

                bool valid = false;
                if (1 < lines.size() && LinePatternCalculator::areFar(lines)) {
                    Lines::const_iterator mainLine = LinePatternCalculator::getMainLine(lines, lastSingleLineId);
                    if (2 == lines.size()) {
                        if (Direction::RIGHT == pattern.side) {
                            valid = *mainLine == lines[0];
                        } else if (Direction::CENTER == pattern.side) {
                            valid = true;
                        } else if (Direction::LEFT == pattern.side) {
                            valid = *mainLine == lines[1];
                        }
                    } else if (3 == lines.size()) {
                        if (Direction::RIGHT == pattern.side) {
                            valid = *mainLine == lines[0];
                        } else if (Direction::CENTER == pattern.side) {
                            valid = *mainLine == lines[1];
                        } else if (Direction::LEFT == pattern.side) {
                            valid = *mainLine == lines[2];
                        }
                    }
                }
                return valid;
            };

            if (Sign::POSITIVE == pattern.dir) {
                if (isInJunctionCenter(lines)) {
                    valid = true;
                } else if (3 == lines.size() && LinePatternCalculator::areFar(lines) && isInJunctionCenter(pastLines)) {
                    valid = true;
                }
            } else if (Sign::NEGATIVE == pattern.dir) {
                if (areValidFarLines(prevMeas, pattern, lines, lastSingleLineId)) {
                    valid = true;
                } else if (3 == lines.size() && isInJunctionCenter(lines) && areValidFarLines(prevMeas, pattern, pastLines, lastSingleLineId)) {
                    valid = true;
                }
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::Labyrinth == activeTask) {
                if (Sign::NEGATIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::JUNCTION_1, Sign::POSITIVE, Direction::CENTER });
                    validPatterns.push_back({ LinePattern::JUNCTION_2, Sign::POSITIVE, Direction::RIGHT  });
                    validPatterns.push_back({ LinePattern::JUNCTION_3, Sign::POSITIVE, Direction::CENTER });
                } else if (Sign::POSITIVE == pattern.dir) {
                    validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
                }
            }
            return validPatterns;
        }
    },
    { // DEAD_END
        centimeter_t(3),
        meter_t::infinity(),
        LinePatternCalculator::LinePatternInfo::USES_HISTORY,
        [] (const LinePatternCalculator::measurement_buffer_t& prevMeas, const LinePattern&, const Lines& lines, uint8_t, meter_t) {
            bool valid = false;
            const Lines pastLines = LinePatternCalculator::peek_back(prevMeas, centimeter_t(10)).lines;

            if (0 == lines.size()) {
                valid = 1 == pastLines.size() && isBtw(pastLines[0].pos, centimeter_t(8), centimeter_t(-8));
            }
            return valid;
        },
        [] (const LinePattern& pattern, const ProgramTask activeTask) {
            LinePatternCalculator::linePatterns_t validPatterns;
            if (ProgramTask::Labyrinth == activeTask) {
                validPatterns.push_back({ LinePattern::SINGLE_LINE, Sign::NEUTRAL, Direction::CENTER });
            }
            return validPatterns;
        }
    }
};

void LinePatternCalculator::update(const ProgramTask activeTask, const Lines& lines, meter_t currentDist) {

    this->prevMeas.push_back({ lines, currentDist });
    LinePattern& current = this->currentPattern();

    if (LinePattern::SINGLE_LINE == current.type && 1 == lines.size()) {
        this->lastSingleLineId = lines[0].id;
    }

    if (this->isPatternChangeCheckActive) {

        for (linePatterns_t::iterator it = possiblePatterns.begin(); it != possiblePatterns.end();) {
            const LinePatternInfo *patternInfo = &PATTERN_INFO[static_cast<uint8_t>(it->type)];
            if (patternInfo->isValid(this->prevMeas, *it, lines, this->lastSingleLineId, currentDist)) {
                if (1 == possiblePatterns.size() && abs(currentDist - it->startDist) >= patternInfo->minValidityLength) {
                    this->changePattern(*it);
                    break;
                }
                ++it;
            } else {
                it = this->possiblePatterns.erase(it);
            }
        }

        if (this->possiblePatterns.empty()) {
            this->isPatternChangeCheckActive = false;
            //LOG_DEBUG("All possible patterns are invalid, steps back to previous pattern");
        }

    } else {
        const LinePatternInfo *currentPatternInfo = &PATTERN_INFO[static_cast<uint8_t>(this->currentPattern().type)];

        if (currentDist - current.startDist > currentPatternInfo->maxLength) {
            // under normal circumstances, maxLength should never be exceeded
            this->changePattern({ LinePattern::NONE, Sign::NEUTRAL, Direction::CENTER, meter_t(0) });

        } else if (!currentPatternInfo->isValid(this->prevMeas, current, lines, this->lastSingleLineId, currentDist)) {
            this->isPatternChangeCheckActive = true;
            this->possiblePatterns = currentPatternInfo->validNextPatterns(current, activeTask);

            for (LinePattern& pattern : this->possiblePatterns) {
                pattern.startDist = currentDist;
            }
        }
    }
}

void LinePatternCalculator::changePattern(const LinePattern& newPattern) {
    const LinePattern prevPattern = this->currentPattern();
    this->prevPatterns.push_back(newPattern);
    this->isPatternChangeCheckActive = false;

    LOG_DEBUG("Pattern changed from { %s, %s, %s } to { %s, %s, %s }",
        to_string(prevPattern.type), to_string(prevPattern.side), to_string(prevPattern.dir),
        to_string(newPattern.type), to_string(newPattern.side), to_string(newPattern.dir));
}

LinePatternCalculator::StampedLines LinePatternCalculator::peek_back(const measurement_buffer_t& prevMeas, meter_t peekBackDist) {
    const meter_t dist = prevMeas.peek_back(0).distance - peekBackDist;

    uint32_t startIdx = 1;
    uint32_t endIdx = prevMeas.size() - 1;

    while (startIdx < endIdx - 1) {
        const uint8_t i = avg(startIdx, endIdx);
        if (prevMeas.peek_back(i).distance < dist) {
            endIdx = i;
        } else {
            startIdx = i;
        }
    }

    return prevMeas.peek_back(endIdx);
}

Lines::const_iterator LinePatternCalculator::getMainLine(const Lines& lines, const uint8_t lastSingleLineId) {
    Lines::const_iterator mainLine = micro::findLine(lines, lastSingleLineId);
    if (mainLine == lines.end()) {
        mainLine = micro::findClosestLine(lines, millimeter_t(0));
    }
    return mainLine;
}

bool LinePatternCalculator::areClose(const Lines& lines) {

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
    return !LinePatternCalculator::areClose(lines);
}

const char* to_string(const LinePattern::Type& type) {
    const char *str = nullptr;
    switch (type) {
    case LinePattern::NONE:        str = "NONE";        break;
    case LinePattern::SINGLE_LINE: str = "SINGLE_LINE"; break;
    case LinePattern::ACCELERATE:  str = "ACCELERATE";  break;
    case LinePattern::BRAKE:       str = "BRAKE";       break;
    case LinePattern::LANE_CHANGE: str = "LANE_CHANGE"; break;
    case LinePattern::JUNCTION_1:  str = "JUNCTION_1";  break;
    case LinePattern::JUNCTION_2:  str = "JUNCTION_2";  break;
    case LinePattern::JUNCTION_3:  str = "JUNCTION_3";  break;
    case LinePattern::DEAD_END:    str = "DEAD_END";    break;
    default:                       str = "";            break;
    }
    return str;
}

} // namespace micro
