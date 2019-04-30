#include <uns/LedDataAnalyser.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/unit_utils.hpp>
#include <uns/task/common.hpp>
#include <uns/util/numeric.hpp>
#include <uns/config/cfg_car.hpp>

using namespace uns;

const float32_t LedDataAnalyser::INVALID_LINE_POS = -1.0f;

extern ProgramTask PROGRAM_TASK;

namespace {
Point2<distance_t> calcAbsOptoPos(const Point2<distance_t>& carPos, angle_t carOrientation, distance_t optoPos) {
    return Point2<distance_t>(carPos.X - uns::sin(carOrientation) * optoPos, carPos.Y + uns::cos(carOrientation) * optoPos);
}
} // namespace

const uint32_t& LabyrinthLineFilter::update(const uint32_t& measuredValue) {
    static constexpr uint32_t MIN_NUM_SINGLE_LINE = 3;

    this->updateNumCalled();

    if (measuredValue <= 1) {
        if (++this->singleLineCntr >= MIN_NUM_SINGLE_LINE) {
            this->filteredValue = 1;
        }
    } else {
        this->singleLineCntr = 0;
        this->filteredValue = 2;
    }

    return this->filteredValue;
}

LedDataAnalyser::LedDataAnalyser(uint8_t front_numOptos, uint8_t back_numOptos, distance_t _maxPosOptoFront, distance_t _maxPosOptoBack, distance_t _distBtwRows)
    : front(front_numOptos, _maxPosOptoFront)
    , back(back_numOptos, _maxPosOptoBack)
    , distBtwRows(_distBtwRows)
    , sampleIdx(0)
    , numPatternSamples(0) {}

void LedDataAnalyser::recognizeLabyrinthPatterns(const Point2<distance_t>& carPos, angle_t carOrientation, LineData *result) {
    switch (this->labyrinthLineFilter.update(result->coarsePositions.size)) {
    case 1: // single line
        if (this->pattern.type != LinePattern::Type::SINGLE_LINE) { // current pattern is not a single line -> pattern has finished, and a single line pattern has been reached

            //debug::printlog("s");
            this->numPatternSamples = 1;
            this->pattern.type = LinePattern::Type::SINGLE_LINE;
            this->pattern.startPos = carPos;
            this->pattern.endPos = LinePattern::UNKNOWN_POS;

        } else {
            ++this->numPatternSamples;
        }

        this->prevSingleLineAbsPosition = calcAbsOptoPos(carPos, carOrientation, result->coarsePositions[0]);
        updateCenterLineAngleDefault(result);
        break;

    case 2: // 2 lines -> junction
        if (this->pattern.type != LinePattern::Type::JUNCTION) { // current pattern is not a junction -> pattern has finished, and a junction has been reached

            // At the previous measurement there was only 1 line. At the current measurement there is 2.
            // We need to determine which line is the 'new' one (the LEFT or the RIGHT).
            // Checks if the current first (left) or the second (right) coarse position is nearer to the previous (single) line's position.
            this->firstSampleDist = abs(result->coarsePositions[1] - result->coarsePositions[0]);
            this->numPatternSamples = 1;
            this->pattern.type = LinePattern::Type::JUNCTION;
            this->pattern.startPos = this->pattern.endPos = LinePattern::UNKNOWN_POS;

            const Point2<distance_t> absPos0 = calcAbsOptoPos(carPos, carOrientation, result->coarsePositions[0]),
                absPos1 = calcAbsOptoPos(carPos, carOrientation, result->coarsePositions[1]);
            this->pattern.side = absPos0.distance(this->prevSingleLineAbsPosition) < absPos1.distance(this->prevSingleLineAbsPosition) ? RotationDir::RIGHT : RotationDir::LEFT;
            ////debug::printlog("side %s", this->pattern.side == RotationDir::LEFT ? "left" : "right");

            //const distance_t& prevSingleLinesPos = this->prevPositions[this->sampleIdx][0];
            //this->pattern.side = abs(prevSingleLinesPos - result->coarsePositions[0]) < abs(prevSingleLinesPos - result->coarsePositions[1]) ? RotationDir::RIGHT : RotationDir::LEFT;

        } else { // we have been in the junction for at least two measurements, determines junction direction
            static constexpr uint32_t checkSamples = 3; // number of previous samples to check when determining junction direction
            if (++this->numPatternSamples >= checkSamples) {
                if (this->pattern.startPos == LinePattern::UNKNOWN_POS) {

                    // minimum difference between the distance of the first pattern sample lines and the current lines to validate a junction
                    // (at every junction the two lines are getting nearer/farther from each other, depending on the car's direction)
                    static constexpr distance_t minDiff(centimeters(), 5.0f);

                    distance_t cur = abs(result->coarsePositions[1] - result->coarsePositions[0]);

                    if (cur > this->firstSampleDist && cur - this->firstSampleDist >= minDiff) {

                        this->pattern.dir = Sign::POSITIVE;
                        this->pattern.startPos = this->pattern.endPos = carPos;

                        if (this->pattern.side == RotationDir::LEFT) {
                            //debug::printlog("F/L");
                        } else {
                            //debug::printlog("F/R");
                        }

                    } else if (cur < this->firstSampleDist && this->firstSampleDist - cur >= minDiff) {

                        this->pattern.dir = Sign::NEGATIVE;
                        this->pattern.startPos = this->pattern.endPos = carPos;

                        if (this->pattern.side == RotationDir::LEFT) {
                            //debug::printlog("F (L)");
                        } else {
                            //debug::printlog("F (R)");
                        }
                    }
                }
            }
        }


        // makes car follow the current line (prevents center line from changing at every junction)
        result->centerLine.pos = result->coarsePositions[this->pattern.side == RotationDir::LEFT ? 1 : 0];
        if (this->back.centerPos != INVALID_LINE_POS) {
            const float32_t backPos = this->back.coarseLinePositions[this->back.coarseLinePositions.size >= 2 && this->pattern.side == RotationDir::LEFT ? 1 : 0];
            this->updateCenterLineAngle(backPos, result);
        } // else: previous line angle is preserved
        break;
    }
    result->pattern = this->pattern;
}

void LedDataAnalyser::recognizeRaceTrackPatterns(const Point2<distance_t>& pos, angle_t carOrientation, LineData *result) {
    // TODO
}

void LedDataAnalyser::updateCenterLineAngle(float32_t backPos, LineData *result) {
    const distance_t backLinePos = uns::map(backPos, 0.0f, static_cast<float32_t>(this->back.numOptos - 1), this->back.maxOptoPos, -this->back.maxOptoPos); // reverse direction mapping
    if (result->centerLine.pos == backLinePos) {
        result->centerLine.angle = angle_t::from<radians>(0.0f);
    } else {
        result->centerLine.angle = uns::atan(this->distBtwRows / (result->centerLine.pos - backLinePos));
    }

    if (result->centerLine.angle < angle_t::from<radians>(0.0f)) {
        result->centerLine.angle += uns::PI_2;
    } else {
        result->centerLine.angle -= uns::PI_2;
    }
}

void LedDataAnalyser::updateCenterLineAngleDefault(LineData *result) {
    // if front line position is invalid (no line found), then does not update line position and angle, current values will be kept until the line is found again
    if (this->front.centerPos != INVALID_LINE_POS) {
        result->centerLine.pos = uns::map(this->front.centerPos, 0.0f, static_cast<float32_t>(this->front.numOptos - 1), -this->front.maxOptoPos, this->front.maxOptoPos);

        // if back line position is invalid (no line found), then does not update line angle, current value will be kept until the line is found again
        if (this->back.centerPos != INVALID_LINE_POS) {
            this->updateCenterLineAngle(this->back.centerPos, result);
        } // else: previous line angle is preserved
    } // else: previous line position and angle is preserved
}

void LedDataAnalyser::evaluate(const Point2<distance_t>& carPos, angle_t carOrientation, LineData *result) {

    result->coarsePositions.clear();
    for (uint32_t i = 0; i < this->front.coarseLinePositions.size; ++i) {
        result->coarsePositions.append(uns::map(this->front.coarseLinePositions[i], 0.0f, static_cast<float32_t>(this->front.numOptos - 1), -cfg::MAX_POS_OPTO_FRONT, cfg::MAX_POS_OPTO_FRONT));
    }

    switch (PROGRAM_TASK) {
    case ProgramTask::LABYRINTH:
        this->recognizeLabyrinthPatterns(carPos, carOrientation, result);
        break;
    case ProgramTask::SAFETY_CAR_FOLLOW:
    case ProgramTask::OVERTAKE:
    case ProgramTask::RACE_TRACK:
        this->recognizeRaceTrackPatterns(carPos, carOrientation, result);
        break;
    }

    this->sampleIdx = (this->sampleIdx + 1) % NUM_SAMPLES;
    this->prevPositions[sampleIdx] = result->coarsePositions;
}

void LedDataAnalyser::detectLines(const Point2<distance_t>& carPos, angle_t carOrientation, FrontBackByteContainer *measurements, FrontBackBitContainer *indicatorLeds, LineData *result) {

    // Evaluates front and back row data, updates comparator value and sets indicator LED buffer
    this->evaluateRow(&this->front, measurements->front, &indicatorLeds->front);
    this->evaluateRow(&this->back, measurements->back, &indicatorLeds->back);
    this->evaluate(carPos, carOrientation, result); // Prepares the final values, calculates the angle.
}

