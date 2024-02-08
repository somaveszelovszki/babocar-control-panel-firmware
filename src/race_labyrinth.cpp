#include <micro/utils/units.hpp>

#include <cfg_track.hpp>

#if TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

#include <LabyrinthGraph.hpp>
#include <track.hpp>

using namespace micro;

void buildRaceLabyrinthGraph(LabyrinthGraph& OUT graph) {
    graph.addJunction(Junction('A', { centimeter_t( 510), centimeter_t(-720) }));
    graph.addJunction(Junction('B', { centimeter_t( 270), centimeter_t(-720) }));
    graph.addJunction(Junction('C', { centimeter_t( 630), centimeter_t(-600) }));
    graph.addJunction(Junction('D', { centimeter_t( 390), centimeter_t(-600) }));
    graph.addJunction(Junction('E', { centimeter_t( 150), centimeter_t(-600) }));
    graph.addJunction(Junction('F', { centimeter_t( 510), centimeter_t(-480) }));
    graph.addJunction(Junction('G', { centimeter_t( 270), centimeter_t(-480) }));
    graph.addJunction(Junction('H', { centimeter_t( 630), centimeter_t(-360) }));
    graph.addJunction(Junction('I', { centimeter_t( 390), centimeter_t(-360) }));
    graph.addJunction(Junction('J', { centimeter_t( 150), centimeter_t(-360) }));
    graph.addJunction(Junction('K', { centimeter_t( 510), centimeter_t(-240) }));
    graph.addJunction(Junction('L', { centimeter_t( 270), centimeter_t(-240) }));
    graph.addJunction(Junction('M', { centimeter_t( 630), centimeter_t(-120) }));
    graph.addJunction(Junction('N', { centimeter_t( 390), centimeter_t(-120) }));
    graph.addJunction(Junction('O', { centimeter_t( 150), centimeter_t(-120) }));
    graph.addJunction(Junction('P', { centimeter_t( 750), centimeter_t(   0) }));
    graph.addJunction(Junction('Q', { centimeter_t( 630), centimeter_t(   0) }));
    graph.addJunction(Junction('R', { centimeter_t( 510), centimeter_t(   0) }));
    graph.addJunction(Junction('S', { centimeter_t( 390), centimeter_t(   0) }));
    graph.addJunction(Junction('T', { centimeter_t( 270), centimeter_t(   0) }));
    graph.addJunction(Junction('U', { centimeter_t(   0), centimeter_t(   0) }));
    graph.addJunction(Junction('V', { centimeter_t( 510), centimeter_t( 120) }));
    graph.addJunction(Junction('W', { centimeter_t( 270), centimeter_t( 120) }));

    graph.connectDeadEnd('P', {PI_2, Direction::CENTER}, centimeter_t(50));
    graph.connectDeadEnd('Q', {PI, Direction::RIGHT}, centimeter_t(750));
    graph.connectDeadEnd('U', {3 * PI_2, Direction::CENTER}, centimeter_t(30));

    graph.connect('A', {3 * PI_2, Direction::LEFT},   'B', {PI_2,        Direction::RIGHT},  centimeter_t(241));
    graph.connect('A', {3 * PI_2, Direction::RIGHT},  'D', {radian_t(0), Direction::LEFT},   centimeter_t(198));
    graph.connect('A', {PI_2,     Direction::CENTER}, 'F', {PI_2,        Direction::RIGHT},  centimeter_t(397));
    graph.connect('B', {PI_2,     Direction::LEFT},   'D', {radian_t(0), Direction::RIGHT},  centimeter_t(198));
    graph.connect('B', {3 * PI_2, Direction::CENTER}, 'E', {radian_t(0), Direction::CENTER}, centimeter_t(198));
    graph.connect('D', {PI,       Direction::LEFT},   'G', {PI_2,        Direction::RIGHT},  centimeter_t(198));
    graph.connect('D', {PI,       Direction::CENTER}, 'I', {radian_t(0), Direction::CENTER}, centimeter_t(241));
    graph.connect('D', {PI,       Direction::RIGHT},  'F', {3 * PI_2,    Direction::LEFT},   centimeter_t(197));
    graph.connect('E', {PI,       Direction::LEFT},   'J', {radian_t(0), Direction::RIGHT},  centimeter_t(241));
    graph.connect('E', {PI,       Direction::RIGHT},  'G', {3 * PI_2,    Direction::LEFT},   centimeter_t(197));
    graph.connect('F', {PI_2,     Direction::LEFT},   'H', {radian_t(0), Direction::CENTER}, centimeter_t(200));
    graph.connect('F', {3 * PI_2, Direction::CENTER}, 'G', {PI_2,        Direction::CENTER}, centimeter_t(241));
    graph.connect('F', {3 * PI_2, Direction::RIGHT},  'I', {radian_t(0), Direction::LEFT},   centimeter_t(198));
    graph.connect('G', {PI_2,     Direction::LEFT},   'I', {radian_t(0), Direction::RIGHT},  centimeter_t(199));
    graph.connect('G', {3 * PI_2, Direction::RIGHT},  'J', {radian_t(0), Direction::LEFT},   centimeter_t(198));
    graph.connect('H', {PI,       Direction::LEFT},   'K', {PI_2,        Direction::RIGHT},  centimeter_t(198));
    graph.connect('H', {PI,       Direction::RIGHT},  'M', {radian_t(0), Direction::LEFT},   centimeter_t(241));
    graph.connect('I', {PI,       Direction::LEFT},   'L', {PI_2,        Direction::RIGHT},  centimeter_t(198));
    graph.connect('I', {PI,       Direction::CENTER}, 'N', {radian_t(0), Direction::CENTER}, centimeter_t(241));
    graph.connect('I', {PI,       Direction::RIGHT},  'K', {3 * PI_2,    Direction::LEFT},   centimeter_t(197));
    graph.connect('J', {PI,       Direction::CENTER}, 'L', {3 * PI_2,    Direction::LEFT},   centimeter_t(197));
    graph.connect('K', {3 * PI_2, Direction::CENTER}, 'L', {PI_2,        Direction::CENTER}, centimeter_t(241));
    graph.connect('K', {3 * PI_2, Direction::RIGHT},  'N', {radian_t(0), Direction::LEFT},   centimeter_t(198));
    graph.connect('K', {PI_2,     Direction::LEFT},   'M', {radian_t(0), Direction::RIGHT},  centimeter_t(200));
    graph.connect('L', {PI_2,     Direction::LEFT},   'N', {radian_t(0), Direction::RIGHT},  centimeter_t(199));
    graph.connect('L', {3 * PI_2, Direction::RIGHT},  'O', {radian_t(0), Direction::CENTER}, centimeter_t(198));
    graph.connect('M', {PI,       Direction::LEFT},   'R', {PI_2,        Direction::RIGHT},  centimeter_t(198));
    graph.connect('M', {PI,       Direction::CENTER}, 'Q', {radian_t(0), Direction::CENTER}, centimeter_t(120));
    graph.connect('M', {PI,       Direction::RIGHT},  'P', {3 * PI_2,    Direction::LEFT},   centimeter_t(197));
    graph.connect('N', {PI,       Direction::LEFT},   'T', {PI_2,        Direction::RIGHT},  centimeter_t(198));
    graph.connect('N', {PI,       Direction::CENTER}, 'S', {radian_t(0), Direction::CENTER}, centimeter_t(120));
    graph.connect('N', {PI,       Direction::RIGHT},  'R', {3 * PI_2,    Direction::LEFT},   centimeter_t(197));
    graph.connect('O', {PI,       Direction::LEFT},   'U', {PI_2,        Direction::RIGHT},  centimeter_t(198));
    graph.connect('O', {PI,       Direction::CENTER}, 'W', {3 * PI_2,    Direction::CENTER}, centimeter_t(318));
    graph.connect('O', {PI,       Direction::RIGHT},  'T', {3 * PI_2,    Direction::LEFT},   centimeter_t(197));
    graph.connect('P', {3 * PI_2, Direction::RIGHT},  'R', {PI_2,        Direction::LEFT},   centimeter_t(241));
    graph.connect('Q', {PI,       Direction::LEFT},   'V', {PI_2,        Direction::CENTER}, centimeter_t(199));
    graph.connect('R', {3 * PI_2, Direction::RIGHT},  'T', {PI_2,        Direction::LEFT},   centimeter_t(241));
    graph.connect('S', {PI,       Direction::LEFT},   'W', {PI_2,        Direction::RIGHT},  centimeter_t(199));
    graph.connect('S', {PI,       Direction::RIGHT},  'V', {3 * PI_2,    Direction::LEFT},   centimeter_t(198));
    graph.connect('T', {3 * PI_2, Direction::RIGHT},  'U', {PI_2,        Direction::LEFT},   centimeter_t(241));
    graph.connect('V', {3 * PI_2, Direction::RIGHT},  'W', {PI_2,        Direction::LEFT},   centimeter_t(241));
}

#endif // TRACK == RACE_TRACK || COMPILE_ALL_TRACKS
