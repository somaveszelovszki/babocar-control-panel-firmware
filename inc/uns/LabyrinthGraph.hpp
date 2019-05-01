#pragma once

#include <uns/Graph.hpp>
#include <uns/Point2.hpp>
#include <uns/util/units.hpp>
#include <uns/config/cfg_track.hpp>
#include <uns/CarProps.hpp>

namespace uns {

class Junction;
class Segment;

/* @brief Labyrinth segment connection.
 */
struct Connection : public Edge {
    /* @brief Junction type - needed when deciding which line to follow.
     */
    enum class Type : uint8_t {
        STRAIGHT,   // Follow the straight (uncut) line.
        CURVE       // Follow the curve (new line).
    };

    Connection()
        : junction(nullptr)
        , type(Type::STRAIGHT) {}

    Connection(Segment *_node1, Segment *_node2, Junction *_junction, Type _type);

    void updateNodes();

    Junction *junction;     // The junction.
    Type type;    // The junction type.
};

/* @brief Labyrinth junction (crossroads).
 */
class Junction {
public:
    struct SegmentOrientation {
        Segment *seg;

        // Indicates segment position in junction - used for determining floating segments.
        // Every junction has 3 segments connected to each other in a Y shape.
        // The bottom segment is the CENTER segment, the upper segments are the LEFT and RIGHT segments.
        // Given a connection type and a side, the given segment can be obtained.
        // e.g. if the current connection type is FORWARD and the pattern side is LEFT, then the car is coming from the top left segment of the Y.
        RotationDir pos;

        radian_t value;
    };

    Junction() : idx(-1) {
        this->segOrientations[0].seg = this->segOrientations[1].seg = this->segOrientations[2].seg = nullptr;
    }

    Status setOrientation(Segment *seg, RotationDir segmentPos, radian_t orientation);

    Status getOrientation(Segment *seg, radian_t *result) const;

    Status getOrientation(RotationDir pos, radian_t *result) const;

    Segment* getSegment(Connection::Type type, RotationDir side) const;

    Status updateSegment(Segment *oldSeg, Segment *newSeg);

    Segment* getSegment(RotationDir segmentPos) const;

    int32_t idx;
    Point2<distance_t> pos; // Junction position - relative to car start position.
    SegmentOrientation segOrientations[3];  // Segments orientations at the junction - used for compensating car orientation sensor drift.

private:
    static RotationDir getSegmentPos(Connection::Type type, RotationDir side);
};

/* @brief Labyrinth segment.
 */
class Segment : public Node {
public:
    /* @brief Segment orientation at the end at given junction.
     **/
    struct EndPosOri {
        Junction *junc; // The junction specifying which end of the segment the orientation refers to.
        PosOri value;  // The orientation.
    };

    Segment() : name('_') {
        this->orientations[0].junc = this->orientations[1].junc = nullptr;
    }


    /* @brief Gets fastest route to the other segment.
     * @param dest The destination segment.
     * @param result The result route.
     */
    void getFastestRoute(const Segment& dest, Vec<Segment*, cfg::MAX_NUM_LAB_SEGMENTS>& result);

    bool isFloating() const;

    void reset();

    char name;
    distance_t length;  // The segment length.
    EndPosOri orientations[2];

    Status setEndPosOri(Junction *junc, const PosOri& orientation);

    Status getEndPosOri(Junction *junc, PosOri *result) const;
};

} // namespace uns
