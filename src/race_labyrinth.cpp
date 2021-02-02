#include <cfg_track.hpp>
#include <LabyrinthGraph.hpp>
#include <track.hpp>

using namespace micro;

LabyrinthGraph buildRaceLabyrinthGraph() {
    LabyrinthGraph graph;

    graph.addSegment(Segment('A', centimeter_t(355), false));
    graph.addSegment(Segment('B', centimeter_t(489), false));
    graph.addSegment(Segment('C', centimeter_t(237), false));
    graph.addSegment(Segment('D', centimeter_t(386), false));
    graph.addSegment(Segment('E', centimeter_t(284), false));
    graph.addSegment(Segment('F', centimeter_t(257), false));
    graph.addSegment(Segment('G', centimeter_t(236), true ));
    graph.addSegment(Segment('H', centimeter_t(779), false));
    graph.addSegment(Segment('I', centimeter_t(195), false));
    graph.addSegment(Segment('J', centimeter_t(609), false));
    graph.addSegment(Segment('K', centimeter_t(488), false));
    graph.addSegment(Segment('L', centimeter_t(372), false));
    graph.addSegment(Segment('M', centimeter_t(195), false));
    graph.addSegment(Segment('N', centimeter_t(177), false));
    graph.addSegment(Segment('O', centimeter_t(389), false));
    graph.addSegment(Segment('P', centimeter_t(205), false));
    graph.addSegment(Segment('Q', centimeter_t(197), false));
    graph.addSegment(Segment('R', centimeter_t(154), false));
    graph.addSegment(Segment('S', centimeter_t(80),  false));
    graph.addSegment(Segment('T', centimeter_t(196), false));
    graph.addSegment(Segment('U', centimeter_t(438), false));

    graph.addJunction(Junction(1,  { centimeter_t(-195), centimeter_t(829) }));
    graph.addJunction(Junction(2,  { centimeter_t(  -7), centimeter_t(771) }));
    graph.addJunction(Junction(3,  { centimeter_t( 207), centimeter_t(709) }));
    graph.addJunction(Junction(4,  { centimeter_t(-221), centimeter_t(707) }));
    graph.addJunction(Junction(5,  { centimeter_t(-321), centimeter_t(571) }));
    graph.addJunction(Junction(6,  { centimeter_t(  -3), centimeter_t(579) }));
    graph.addJunction(Junction(7,  { centimeter_t(-221), centimeter_t(476) }));
    graph.addJunction(Junction(8,  { centimeter_t(-143), centimeter_t(471) }));
    graph.addJunction(Junction(9,  { centimeter_t( -29), centimeter_t(357) }));
    graph.addJunction(Junction(10, { centimeter_t(  86), centimeter_t(233) }));
    graph.addJunction(Junction(11, { centimeter_t(-242), centimeter_t(119) }));
    graph.addJunction(Junction(12, { centimeter_t( 207), centimeter_t(114) }));
    graph.addJunction(Junction(13, { centimeter_t(-143), centimeter_t(  0) }));

    graph.connect(graph.findSegment('A'), graph.findJunction(1),   JunctionDecision(3 * PI_2,    Direction::CENTER));
    graph.connect(graph.findSegment('A'), graph.findJunction(5),   JunctionDecision(PI,          Direction::LEFT));

    graph.connect(graph.findSegment('B'), graph.findJunction(1),   JunctionDecision(PI_2,        Direction::LEFT));
    graph.connect(graph.findSegment('B'), graph.findJunction(3),   JunctionDecision(PI,          Direction::CENTER));

    graph.connect(graph.findSegment('C'), graph.findJunction(2),   JunctionDecision(3 * PI_2,    Direction::LEFT));
    graph.connect(graph.findSegment('C'), graph.findJunction(4),   JunctionDecision(PI,          Direction::LEFT));

    graph.connect(graph.findSegment('D'), graph.findJunction(2),   JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('D'), graph.findJunction(6),   JunctionDecision(PI,          Direction::CENTER));

    graph.connect(graph.findSegment('E'), graph.findJunction(4),   JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('E'), graph.findJunction(6),   JunctionDecision(3 * PI_2,    Direction::RIGHT));

    graph.connect(graph.findSegment('F'), graph.findJunction(7),   JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('F'), graph.findJunction(6),   JunctionDecision(3 * PI_2,    Direction::LEFT));

    graph.connect(graph.findSegment('G'), graph.findJunction(3),   JunctionDecision(radian_t(0), Direction::CENTER));

    graph.connect(graph.findSegment('H'), graph.findJunction(3),   JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('H'), graph.findJunction(3),   JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('I'), graph.findJunction(8),   JunctionDecision(PI_2,        Direction::RIGHT));
    graph.connect(graph.findSegment('I'), graph.findJunction(9),   JunctionDecision(PI,          Direction::CENTER));

    graph.connect(graph.findSegment('J'), graph.findJunction(8),   JunctionDecision(PI_2,        Direction::LEFT));
    graph.connect(graph.findSegment('J'), graph.findJunction(12),  JunctionDecision(PI,         Direction::RIGHT));

    graph.connect(graph.findSegment('K'), graph.findJunction(5),   JunctionDecision(radian_t(0), Direction::RIGHT));
    graph.connect(graph.findSegment('K'), graph.findJunction(11),  JunctionDecision(PI,          Direction::LEFT));

    graph.connect(graph.findSegment('L'), graph.findJunction(9),   JunctionDecision(radian_t(0), Direction::RIGHT));
    graph.connect(graph.findSegment('L'), graph.findJunction(11),  JunctionDecision(PI,          Direction::RIGHT));
    
    graph.connect(graph.findSegment('M'), graph.findJunction(9),   JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('M'), graph.findJunction(10),  JunctionDecision(3 * PI_2,    Direction::RIGHT));
    
    graph.connect(graph.findSegment('N'), graph.findJunction(11),  JunctionDecision(radian_t(0), Direction::CENTER));
    graph.connect(graph.findSegment('N'), graph.findJunction(13),  JunctionDecision(3 * PI_2,    Direction::CENTER));

    graph.connect(graph.findSegment('O'), graph.findJunction(11),  JunctionDecision(radian_t(0), Direction::CENTER));
    graph.connect(graph.findSegment('O'), graph.findJunction(10),  JunctionDecision(3 * PI_2,    Direction::LEFT));

    graph.connect(graph.findSegment('W'), graph.findJunction(13),  JunctionDecision(PI_2,         Direction::RIGHT));
    graph.connect(graph.findSegment('W'), graph.findJunction(12),  JunctionDecision(radian_t(0),  Direction::CENTER));

    return graph;
}
