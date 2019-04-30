#pragma once

#include <uns/util/types.hpp>
#include <uns/util/debug.hpp>
#include <uns/container/Vec.hpp>
#include <uns/BitArray.hpp>
#include <uns/config/cfg_car.hpp>
#include <uns/config/cfg_track.hpp>
#include <uns/LineData.hpp>
#include <uns/util/numeric.hpp>
#include <uns/Point2.hpp>
#include <uns/sensor/Filter.hpp>

namespace uns {

/* @brief Structure for storing detected lines matched to the optical sensor positions.
 **/
union FrontBackBitContainer {
    BitArray<cfg::NUM_OPTO> all; // Array storing all the calculated lines (every bit of the array indicates if a line has been measured at the given position).
    struct {
        BitArray<cfg::NUM_OPTO_FRONT> front;   // Array storing the front lines.
        BitArray<cfg::NUM_OPTO_BACK> back;      // Array storing the back lines.
    };
};

/* @brief Structure for storing optical sensor data.
 **/
union FrontBackByteContainer {
    uint8_t all[cfg::NUM_OPTO]; // Array storing all the measured optical sensor values.
    struct {
        uint8_t front[cfg::NUM_OPTO_FRONT]; // Array storing the front optical sensor values.
        uint8_t back[cfg::NUM_OPTO_BACK]; // Array storing the back optical sensor values.
    };
};

class LabyrinthLineFilter : public FilterBase<uint32_t> {
public:
    LabyrinthLineFilter() : singleLineCntr(0) {}

    const uint32_t& update(const uint32_t& measuredValue);

private:
    uint32_t singleLineCntr;
};
class LedDataAnalyser {

public:
    static const float32_t INVALID_LINE_POS;

    enum class RowPosition : uint1_t {
        FRONT = 0, BACK = 1
    };

    LedDataAnalyser(uint8_t front_numOptos, uint8_t back_numOptos, distance_t _maxPosOptoFront, distance_t _maxPosOptoBack, distance_t _distBtwRows);

    // Detects + evaluates, prepares the data for the get function.
    void detectLines(const Point2<distance_t>& carPos, angle_t carOrientation, FrontBackByteContainer *measurements, FrontBackBitContainer *indicatorLeds, LineData *result);

private:
    struct LedRowData {
        uint8_t numOptos;       // Number of opto-sensors in the row.
        distance_t maxOptoPos;    // Maximum opto-sensor position.
        uint8_t comparator;     // Comparator level for separating black and white.
        uint8_t avgWhite;       // Average white value of the previous measurement.
        uint8_t avgBlack;       // Average black value of the previous measurement.
        float32_t centerPos;   // Position of the center line.
        Vec<float32_t, cfg::MAX_LINES> coarseLinePositions; // Coarse line positions (not exact positions, but good enough for pattern matching).

        LedRowData(uint8_t _numOptos, distance_t _maxOptoPos)
            : numOptos(_numOptos)
            , maxOptoPos(_maxOptoPos)
            , comparator(128)
            , avgWhite(0)
            , avgBlack(0)
            , centerPos(0.0f) {}
    };

    template<uint32_t numOptos>
    void evaluateRow(LedRowData *pRow, const uint8_t *measurements, BitArray<numOptos> *indicatorLeds);

    // Prepares the center-left-right lines. (only center now)
    void evaluate(const Point2<distance_t>& carPos, angle_t carOrientation, LineData *result);

    void recognizeLabyrinthPatterns(const Point2<distance_t>& carPos, angle_t carOrientation, LineData *result);

    void recognizeRaceTrackPatterns(const Point2<distance_t>& carPos, angle_t carOrientation, LineData *result);

    void updateCenterLineAngle(float32_t backPos, LineData *result);

    void updateCenterLineAngleDefault(LineData *result);

    LedRowData front;                   // Front row data structure.
    LedRowData back;                    // Back row data structure.
    distance_t distBtwRows;               // Distance between rows.

    static constexpr uint32_t NUM_SAMPLES = 300;
    Vec<distance_t, cfg::MAX_LINES> prevPositions[NUM_SAMPLES];
    Point2<distance_t> prevSingleLineAbsPosition;
    uint32_t sampleIdx;
    uint32_t numPatternSamples;
    LinePattern pattern;
    distance_t firstSampleDist;
    LabyrinthLineFilter labyrinthLineFilter;
};

template<uint32_t numOptos>
void LedDataAnalyser::evaluateRow(LedRowData *pRow, const uint8_t *measurements, BitArray<numOptos> *indicatorLeds) {

    static constexpr float32_t comp_w_black = 0.75f; // Weight of the black component when recalculating comparator level.
    static constexpr float32_t comp_w_white = 1.0f - comp_w_black; // Weight of the white component when recalculating comparator level.

    uint16_t sumWhite = 0, // The sum of the measurements under the comparator level.
            sumBlack = 0; // The sum of the measurements above the comparator level.

    uint32_t sumBlackW = 0; // The weighted sum of the measurements above the comparator level.

    uint8_t cntrWhite = 0,  // Counts the values under the comparator level.
            cntrBlack = 0;  // Counts the values above the comparator level.

    pRow->coarseLinePositions.size = 0;

    bool prevBit = false;

    for (uint8_t i = 0; i < pRow->numOptos; ++i) {
        uint8_t meas = measurements[i];
        bool currentBit = meas >= pRow->comparator;

        if (currentBit) {
            sumBlack += meas;
            sumBlackW += meas * i;
            ++cntrBlack;
            indicatorLeds->set(i, true);
            if (!prevBit) {     // riding edge -> new line
                pRow->coarseLinePositions.append(static_cast<float32_t>(i));
            }
        } else {
            sumWhite += meas;
            ++cntrWhite;
            indicatorLeds->set(i, false);
        }

        prevBit = currentBit;
    }

    // calculates detected line position
    pRow->centerPos = cntrBlack > 0 ? (static_cast<float32_t>(sumBlackW) / sumBlack) : INVALID_LINE_POS;

    // updates comparator if there were black and white values as well
    if (cntrWhite > 0 && cntrBlack > 0) {
        pRow->avgWhite = sumWhite / cntrWhite;
        pRow->avgBlack = sumBlack / cntrBlack;
        pRow->comparator = static_cast<uint8_t>(pRow->avgWhite * comp_w_white + pRow->avgBlack * comp_w_black);
    }
}

} // namespace uns
