#include <uns/Line.hpp>
#include <uns/util/unit_utils.hpp>
#include <uns/config/cfg_car.hpp>

namespace uns {

void removeUnmatchedPositions(LinePositions& positions, const Lines& lines, uint32_t targetSize, bool isFront) {
    while (positions.size > targetSize) {

        // Iterates through all the current front line positions, and for each of them,
        // finds the distance to the closest front line position in the previous measurement.
        // For the detected lines that were already calculated in the previous measurement,
        // this distance will be lower than for the positions that are new lines.
        // The second type of positions need to be filtered out, therefore the algorithm
        // finds the position with the largest minimum distance from the previous line positions.

        LinePositions::iterator maxIter = positions.end();
        millimeter_t maxDist;
        for (LinePositions::iterator linePosIter = positions.begin(); linePosIter != positions.end(); ++linePosIter) {

            // finds minimum distance from the previous line positions
            millimeter_t currentMinDist;
            for (Lines::const_iterator lineIter = lines.begin(); lineIter != lines.end(); ++lineIter) {
                const millimeter_t dist = uns::abs((isFront ? lineIter->pos_front : lineIter->pos_rear) - *linePosIter);
                if (dist < currentMinDist) {
                    currentMinDist = dist;
                }
            }

            // stores largest minimum distance
            if (maxIter == positions.end() || currentMinDist > maxDist) {
                maxDist = currentMinDist;
                maxIter = linePosIter;
            }
        }

        positions.remove(maxIter);
    }
}

void calculateLines(LinePositions front, LinePositions rear, Lines& lines, Line& mainLine) {
    static constexpr millimeter_t EPS(2.0f);

    if (!front.size || !rear.size) {
        lines.clear();
        // center line is not changed
    } else {
        removeUnmatchedPositions(front, lines, rear.size, true);
        removeUnmatchedPositions(rear, lines, front.size, false);

        // now front and rear sizes are equal

        lines.clear();

        for (uint32_t i = 0; i < front.size; ++i) {
            Line line;
            line.pos_front = front[i];
            line.pos_rear = rear[i];

            if (uns::eq(line.pos_front, line.pos_rear, EPS)) {
                line.angle = radian_t::ZERO();
            } else {
                line.angle = uns::atan(cfg::DIST_BTW_OPTO_ROWS / (line.pos_front - line.pos_rear));
                if (line.angle < radian_t::ZERO()) {
                    line.angle += uns::PI_2;
                } else {
                    line.angle -= uns::PI_2;
                }
            }
            lines.append(line);
        }

        switch(lines.size) {
        case 1:
            mainLine = lines[0];
            break;
        case 2:
        {
            const Line& line0 = lines[0];
            const Line& line1 = lines[1];
            const millimeter_t diff0 = uns::abs(lines[0].pos_front - mainLine.pos_front) + uns::abs(lines[0].pos_rear - mainLine.pos_rear);
            const millimeter_t diff1 = uns::abs(lines[1].pos_front - mainLine.pos_front) + uns::abs(lines[1].pos_rear - mainLine.pos_rear);

            mainLine = diff0 < diff1 ? line0 : line1;
            break;
        }
        case 3:
            mainLine = lines[1];
            break;
        default:
            // should not get here
            break;
        }
    }
}

} // namespace uns
