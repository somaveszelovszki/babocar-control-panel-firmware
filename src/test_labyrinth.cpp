#include <cfg_track.hpp>
#include <LabyrinthGraph.hpp>
#include <track.hpp>

using namespace micro;

LabyrinthGraph buildTestLabyrinthGraph() {
    LabyrinthGraph graph;

    graph.addSegment(Segment('A', centimeter_t(844),  false));
    graph.addSegment(Segment('B', centimeter_t(233),  false));
    graph.addSegment(Segment('C', centimeter_t(274),  false));
    graph.addSegment(Segment('D', centimeter_t(428),  false));
    graph.addSegment(Segment('E', centimeter_t(648),  false));
    graph.addSegment(Segment('F', centimeter_t(192),  false));
    graph.addSegment(Segment('G', centimeter_t(210),  false));
    graph.addSegment(Segment('H', centimeter_t(328),  false));
    graph.addSegment(Segment('I', centimeter_t(222),  false));
    graph.addSegment(Segment('J', centimeter_t(676),  false));
    graph.addSegment(Segment('K', centimeter_t(495),  false));
    graph.addSegment(Segment('L', centimeter_t(445),  false));
    graph.addSegment(Segment('M', centimeter_t(446),  false));
    graph.addSegment(Segment('N', centimeter_t(1360), false));
    graph.addSegment(Segment('O', centimeter_t(398),  true ));
    graph.addSegment(Segment('P', centimeter_t(100),  false));
    graph.addSegment(Segment('Q', centimeter_t(198),  false));
    graph.addSegment(Segment('R', centimeter_t(120),  false));
    graph.addSegment(Segment('S', centimeter_t(285),  false));
    graph.addSegment(Segment('T', centimeter_t(245),  false));
    graph.addSegment(Segment('U', centimeter_t(307),  false));
    graph.addSegment(Segment('V', centimeter_t(460),  false));
    graph.addSegment(Segment('W', centimeter_t(539),  false));

    graph.addJunction(Junction(1,  { centimeter_t(-2250), centimeter_t(60)  }));
    graph.addJunction(Junction(2,  { centimeter_t(-2130), centimeter_t(60)  }));
    graph.addJunction(Junction(3,  { centimeter_t(-2040), centimeter_t(150) }));
    graph.addJunction(Junction(4,  { centimeter_t(-1865), centimeter_t(60)  }));
    graph.addJunction(Junction(5,  { centimeter_t(-1740), centimeter_t(60)  }));
    graph.addJunction(Junction(6,  { centimeter_t(-1580), centimeter_t(60)  }));
    graph.addJunction(Junction(7,  { centimeter_t(-1415), centimeter_t(150) }));
    graph.addJunction(Junction(8,  { centimeter_t(-1290), centimeter_t(60)  }));
    graph.addJunction(Junction(9,  { centimeter_t(-1170), centimeter_t(150) }));
    graph.addJunction(Junction(10, { centimeter_t(-1000), centimeter_t(60)  }));
    graph.addJunction(Junction(11, { centimeter_t(-540),  centimeter_t(60)  }));
    graph.addJunction(Junction(12, { centimeter_t(-140),  centimeter_t(0)   }));
    graph.addJunction(Junction(13, { centimeter_t(380),   centimeter_t(0)   }));

    graph.connect(graph.findSegment('A'), graph.findJunction(1),  JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('A'), graph.findJunction(1),  JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('B'), graph.findJunction(1),  JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('P'), graph.findJunction(1),  JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('P'), graph.findJunction(2),  JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('C'), graph.findJunction(2),  JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('D'), graph.findJunction(2),  JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('B'), graph.findJunction(3),  JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('E'), graph.findJunction(3),  JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('Q'), graph.findJunction(3),  JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('C'), graph.findJunction(4),  JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('Q'), graph.findJunction(4),  JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('R'), graph.findJunction(4),  JunctionDecision(radian_t(0), Direction::CENTER));

    graph.connect(graph.findSegment('D'), graph.findJunction(5),  JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('R'), graph.findJunction(5),  JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('F'), graph.findJunction(5),  JunctionDecision(radian_t(0), Direction::CENTER));

    graph.connect(graph.findSegment('F'), graph.findJunction(6),  JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('G'), graph.findJunction(6),  JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('S'), graph.findJunction(6),  JunctionDecision(radian_t(0), Direction::CENTER));
    graph.connect(graph.findSegment('H'), graph.findJunction(6),  JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('G'), graph.findJunction(7),  JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('E'), graph.findJunction(7),  JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('T'), graph.findJunction(7),  JunctionDecision(radian_t(0), Direction::CENTER));

    graph.connect(graph.findSegment('H'), graph.findJunction(8),  JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('S'), graph.findJunction(8),  JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('U'), graph.findJunction(8),  JunctionDecision(radian_t(0), Direction::CENTER));

    graph.connect(graph.findSegment('T'), graph.findJunction(9),  JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('J'), graph.findJunction(9),  JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('I'), graph.findJunction(9),  JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('U'), graph.findJunction(10), JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('I'), graph.findJunction(10), JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('V'), graph.findJunction(10), JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('K'), graph.findJunction(10), JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('K'), graph.findJunction(11), JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('V'), graph.findJunction(11), JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('J'), graph.findJunction(11), JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('N'), graph.findJunction(11), JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('M'), graph.findJunction(11), JunctionDecision(radian_t(0), Direction::CENTER));
    graph.connect(graph.findSegment('L'), graph.findJunction(11), JunctionDecision(radian_t(0), Direction::RIGHT));

    graph.connect(graph.findSegment('L'), graph.findJunction(12), JunctionDecision(PI,          Direction::LEFT));
    graph.connect(graph.findSegment('M'), graph.findJunction(12), JunctionDecision(PI,          Direction::RIGHT));
    graph.connect(graph.findSegment('W'), graph.findJunction(12), JunctionDecision(radian_t(0), Direction::CENTER));

    graph.connect(graph.findSegment('W'), graph.findJunction(13), JunctionDecision(PI,          Direction::CENTER));
    graph.connect(graph.findSegment('N'), graph.findJunction(13), JunctionDecision(radian_t(0), Direction::LEFT));
    graph.connect(graph.findSegment('O'), graph.findJunction(13), JunctionDecision(radian_t(0), Direction::RIGHT));

    return graph;
}
