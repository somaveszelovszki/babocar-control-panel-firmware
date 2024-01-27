#include "micro/utils/units.hpp"
#include <cfg_track.hpp>

#if TRACK == TEST_TRACK || COMPILE_ALL_TRACKS

#include <LabyrinthGraph.hpp>
#include <track.hpp>

using namespace micro;

void buildTestLabyrinthGraph(LabyrinthGraph& OUT graph) {
    graph.addJunction(Junction('A', {centimeter_t(0),  centimeter_t(0)}));
    graph.addJunction(Junction('C', {centimeter_t(330),  centimeter_t(-60)}));
    graph.addJunction(Junction('E', {centimeter_t(480),  centimeter_t(-60)}));
    graph.addJunction(Junction('F', {centimeter_t(860),  centimeter_t(90)}));
    graph.addJunction(Junction('G', {centimeter_t(860),  centimeter_t(-60)}));
    graph.addJunction(Junction('H', {centimeter_t(1140), centimeter_t(90)}));
    graph.addJunction(Junction('I', {centimeter_t(1140), centimeter_t(-60)}));
    graph.addJunction(Junction('J', {centimeter_t(1340), centimeter_t(30)}));
    graph.addJunction(Junction('K', {centimeter_t(1540), centimeter_t(90)}));
    graph.addJunction(Junction('L', {centimeter_t(1550), centimeter_t(-60)}));
    graph.addJunction(Junction('M', {centimeter_t(1820), centimeter_t(90)}));
    graph.addJunction(Junction('N', {centimeter_t(1800), centimeter_t(-60)}));
    graph.addJunction(Junction('O', {centimeter_t(2000), centimeter_t(30)}));
    graph.addJunction(Junction('P', {centimeter_t(2230), centimeter_t(90)}));
    graph.addJunction(Junction('Q', {centimeter_t(2240), centimeter_t(-60)}));
    graph.addJunction(Junction('R', {centimeter_t(2530), centimeter_t(90)}));
    graph.addJunction(Junction('S', {centimeter_t(2520), centimeter_t(-30)}));
    graph.addJunction(Junction('T', {centimeter_t(2710), centimeter_t(30)}));
    graph.addJunction(Junction('U', {centimeter_t(2890), centimeter_t(90)}));
    graph.addJunction(Junction('V', {centimeter_t(2920), centimeter_t(-30)}));
    graph.addJunction(Junction('W', {centimeter_t(3060), centimeter_t(130)}));
    graph.addJunction(Junction('X', {centimeter_t(3060), centimeter_t(-90)}));
    graph.addJunction(Junction('Y', {centimeter_t(3310), centimeter_t(90)}));

    graph.connectDeadEnd('A', {PI, Direction::CENTER}, centimeter_t(60));
    graph.connectDeadEnd('Y', {radian_t(0), Direction::CENTER}, centimeter_t(50));
    graph.connectDeadEnd('X', {radian_t(0), Direction::RIGHT}, centimeter_t(407));

    graph.connect('A', {radian_t(0), Direction::CENTER}, 'C', {PI,          Direction::LEFT},   centimeter_t(365));
    graph.connect('C', {radian_t(0), Direction::CENTER}, 'E', {PI,          Direction::CENTER}, centimeter_t(160));
    graph.connect('C', {PI,          Direction::RIGHT},  'F', {PI,          Direction::RIGHT},  centimeter_t(959));
    graph.connect('E', {radian_t(0), Direction::LEFT},   'F', {PI,          Direction::LEFT},   centimeter_t(428));
    graph.connect('E', {radian_t(0), Direction::RIGHT},  'G', {PI,          Direction::CENTER}, centimeter_t(385));
    graph.connect('F', {radian_t(0), Direction::LEFT},   'H', {PI,          Direction::RIGHT},  centimeter_t(284));
    graph.connect('F', {radian_t(0), Direction::RIGHT},  'I', {PI,          Direction::RIGHT},  centimeter_t(335));
    graph.connect('G', {radian_t(0), Direction::LEFT},   'H', {PI,          Direction::LEFT},   centimeter_t(336));
    graph.connect('G', {radian_t(0), Direction::RIGHT},  'I', {PI,          Direction::LEFT},   centimeter_t(284));
    graph.connect('H', {radian_t(0), Direction::LEFT},   'K', {PI,          Direction::RIGHT},  centimeter_t(407));
    graph.connect('H', {radian_t(0), Direction::RIGHT},  'J', {PI,          Direction::CENTER}, centimeter_t(220));
    graph.connect('J', {radian_t(0), Direction::LEFT},   'K', {PI,          Direction::LEFT},   centimeter_t(214));
    graph.connect('J', {radian_t(0), Direction::RIGHT},  'L', {PI,          Direction::RIGHT},  centimeter_t(239));
    graph.connect('I', {radian_t(0), Direction::CENTER}, 'L', {PI,          Direction::LEFT},   centimeter_t(418));
    graph.connect('K', {radian_t(0), Direction::LEFT},   'M', {PI,          Direction::RIGHT},  centimeter_t(288));
    graph.connect('K', {radian_t(0), Direction::RIGHT},  'N', {PI,          Direction::RIGHT},  centimeter_t(319));
    graph.connect('L', {radian_t(0), Direction::LEFT},   'M', {PI,          Direction::LEFT},   centimeter_t(329));
    graph.connect('L', {radian_t(0), Direction::RIGHT},  'N', {PI,          Direction::LEFT},   centimeter_t(258));
    graph.connect('M', {radian_t(0), Direction::LEFT},   'P', {PI,          Direction::RIGHT},  centimeter_t(416));
    graph.connect('M', {radian_t(0), Direction::RIGHT},  'O', {PI,          Direction::RIGHT},  centimeter_t(198));
    graph.connect('N', {radian_t(0), Direction::LEFT},   'O', {PI,          Direction::LEFT},   centimeter_t(228));
    graph.connect('N', {radian_t(0), Direction::RIGHT},  'Q', {PI,          Direction::CENTER}, centimeter_t(447));
    graph.connect('O', {radian_t(0), Direction::CENTER}, 'P', {PI,          Direction::LEFT},   centimeter_t(248));
    graph.connect('P', {radian_t(0), Direction::LEFT},   'R', {PI,          Direction::RIGHT},  centimeter_t(305));
    graph.connect('P', {radian_t(0), Direction::RIGHT},  'S', {PI,          Direction::RIGHT},  centimeter_t(346));
    graph.connect('Q', {radian_t(0), Direction::LEFT},   'R', {PI,          Direction::LEFT},   centimeter_t(346));
    graph.connect('Q', {radian_t(0), Direction::RIGHT},  'S', {PI,          Direction::LEFT},   centimeter_t(284));
    graph.connect('R', {radian_t(0), Direction::LEFT},   'U', {PI,          Direction::RIGHT},  centimeter_t(366));
    graph.connect('R', {radian_t(0), Direction::RIGHT},  'T', {PI,          Direction::RIGHT},  centimeter_t(204));
    graph.connect('S', {radian_t(0), Direction::LEFT},   'T', {PI,          Direction::LEFT},   centimeter_t(223));
    graph.connect('S', {radian_t(0), Direction::RIGHT},  'V', {PI,          Direction::LEFT},   centimeter_t(406));
    graph.connect('T', {radian_t(0), Direction::LEFT},   'U', {PI,          Direction::LEFT},   centimeter_t(192));
    graph.connect('T', {radian_t(0), Direction::RIGHT},  'V', {PI,          Direction::RIGHT},  centimeter_t(233));
    graph.connect('U', {radian_t(0), Direction::CENTER}, 'W', {PI,          Direction::CENTER}, centimeter_t(183));
    graph.connect('V', {radian_t(0), Direction::CENTER}, 'X', {PI,          Direction::CENTER}, centimeter_t(149));
    graph.connect('W', {radian_t(0), Direction::LEFT},   'Y', {PI,          Direction::CENTER}, centimeter_t(264));
    graph.connect('W', {radian_t(0), Direction::RIGHT},  'X', {radian_t(0), Direction::LEFT},   centimeter_t(377));
}

#endif // TRACK == TEST_TRACK || COMPILE_ALL_TRACKS
