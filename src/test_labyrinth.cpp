#include "micro/utils/units.hpp"
#include <cfg_track.hpp>

#if TRACK == TEST_TRACK || COMPILE_ALL_TRACKS

#include <LabyrinthGraph.hpp>
#include <track.hpp>

using namespace micro;

void buildTestLabyrinthGraph(LabyrinthGraph& OUT graph) {
    graph.addJunction(Junction('A', { centimeter_t(3310), centimeter_t(90) }));
    graph.addJunction(Junction('C', { centimeter_t(2980), centimeter_t(150) }));
    graph.addJunction(Junction('E', { centimeter_t(2830), centimeter_t(150) }));
    graph.addJunction(Junction('F', { centimeter_t(2450), centimeter_t(0) }));
    graph.addJunction(Junction('G', { centimeter_t(2450), centimeter_t(150) }));
    graph.addJunction(Junction('H', { centimeter_t(2170), centimeter_t(0) }));
    graph.addJunction(Junction('I', { centimeter_t(2170), centimeter_t(150) }));
    graph.addJunction(Junction('J', { centimeter_t(1970), centimeter_t(60) }));
    graph.addJunction(Junction('K', { centimeter_t(1770), centimeter_t(0) }));
    graph.addJunction(Junction('L', { centimeter_t(1760), centimeter_t(150) }));
    graph.addJunction(Junction('M', { centimeter_t(1490), centimeter_t(0) }));
    graph.addJunction(Junction('N', { centimeter_t(1510), centimeter_t(150) }));
    graph.addJunction(Junction('O', { centimeter_t(1310), centimeter_t(60) }));
    graph.addJunction(Junction('P', { centimeter_t(1080), centimeter_t(0) }));
    graph.addJunction(Junction('Q', { centimeter_t(1070), centimeter_t(150) }));
    graph.addJunction(Junction('R', { centimeter_t(780), centimeter_t(0) }));
    graph.addJunction(Junction('S', { centimeter_t(790), centimeter_t(150) }));
    graph.addJunction(Junction('T', { centimeter_t(600), centimeter_t(60) }));
    graph.addJunction(Junction('U', { centimeter_t(420), centimeter_t(0) }));
    graph.addJunction(Junction('V', { centimeter_t(390), centimeter_t(150) }));
    graph.addJunction(Junction('W', { centimeter_t(250), centimeter_t(-40) }));
    graph.addJunction(Junction('X', { centimeter_t(250), centimeter_t(180) }));
    graph.addJunction(Junction('Y', { centimeter_t(0), centimeter_t(0) }));

    graph.connectDeadEnd('C', {radian_t(0), Direction::LEFT}, centimeter_t(425));
    graph.connectDeadEnd('Y', {PI, Direction::CENTER}, centimeter_t(50));
    graph.connectDeadEnd('X', {PI, Direction::RIGHT}, centimeter_t(407));

    graph.connect('C', {PI,          Direction::CENTER}, 'E', {radian_t(0), Direction::CENTER}, centimeter_t(160));
    graph.connect('C', {radian_t(0), Direction::RIGHT},  'F', {radian_t(0), Direction::RIGHT},  centimeter_t(959));
    graph.connect('E', {PI,          Direction::LEFT},   'F', {radian_t(0), Direction::LEFT},   centimeter_t(428));
    graph.connect('E', {PI,          Direction::RIGHT},  'G', {radian_t(0), Direction::CENTER}, centimeter_t(385));
    graph.connect('F', {PI,          Direction::LEFT},   'H', {radian_t(0), Direction::RIGHT},  centimeter_t(284));
    graph.connect('F', {PI,          Direction::RIGHT},  'I', {radian_t(0), Direction::RIGHT},  centimeter_t(335));
    graph.connect('G', {PI,          Direction::LEFT},   'H', {radian_t(0), Direction::LEFT},   centimeter_t(336));
    graph.connect('G', {PI,          Direction::RIGHT},  'I', {radian_t(0), Direction::LEFT},   centimeter_t(284));
    graph.connect('H', {PI,          Direction::LEFT},   'K', {radian_t(0), Direction::RIGHT},  centimeter_t(407));
    graph.connect('H', {PI,          Direction::RIGHT},  'J', {radian_t(0), Direction::CENTER}, centimeter_t(220));
    graph.connect('J', {PI,          Direction::LEFT},   'K', {radian_t(0), Direction::LEFT},   centimeter_t(214));
    graph.connect('J', {PI,          Direction::RIGHT},  'L', {radian_t(0), Direction::RIGHT},  centimeter_t(239));
    graph.connect('I', {PI,          Direction::CENTER}, 'L', {radian_t(0), Direction::LEFT},   centimeter_t(418));
    graph.connect('K', {PI,          Direction::LEFT},   'M', {radian_t(0), Direction::RIGHT},  centimeter_t(288));
    graph.connect('K', {PI,          Direction::RIGHT},  'N', {radian_t(0), Direction::RIGHT},  centimeter_t(319));
    graph.connect('L', {PI,          Direction::LEFT},   'M', {radian_t(0), Direction::LEFT},   centimeter_t(329));
    graph.connect('L', {PI,          Direction::RIGHT},  'N', {radian_t(0), Direction::LEFT},   centimeter_t(258));
    graph.connect('M', {PI,          Direction::LEFT},   'P', {radian_t(0), Direction::RIGHT},  centimeter_t(416));
    graph.connect('M', {PI,          Direction::RIGHT},  'O', {radian_t(0), Direction::RIGHT},  centimeter_t(198));
    graph.connect('N', {PI,          Direction::LEFT},   'O', {radian_t(0), Direction::LEFT},   centimeter_t(228));
    graph.connect('N', {PI,          Direction::RIGHT},  'Q', {radian_t(0), Direction::CENTER}, centimeter_t(447));
    graph.connect('O', {PI,          Direction::CENTER}, 'P', {radian_t(0), Direction::LEFT},   centimeter_t(248));
    graph.connect('P', {PI,          Direction::LEFT},   'R', {radian_t(0), Direction::RIGHT},  centimeter_t(305));
    graph.connect('P', {PI,          Direction::RIGHT},  'S', {radian_t(0), Direction::RIGHT},  centimeter_t(346));
    graph.connect('Q', {PI,          Direction::LEFT},   'R', {radian_t(0), Direction::LEFT},   centimeter_t(346));
    graph.connect('Q', {PI,          Direction::RIGHT},  'S', {radian_t(0), Direction::LEFT},   centimeter_t(284));
    graph.connect('R', {PI,          Direction::LEFT},   'U', {radian_t(0), Direction::RIGHT},  centimeter_t(366));
    graph.connect('R', {PI,          Direction::RIGHT},  'T', {radian_t(0), Direction::RIGHT},  centimeter_t(204));
    graph.connect('S', {PI,          Direction::LEFT},   'T', {radian_t(0), Direction::LEFT},   centimeter_t(223));
    graph.connect('S', {PI,          Direction::RIGHT},  'V', {radian_t(0), Direction::LEFT},   centimeter_t(406));
    graph.connect('T', {PI,          Direction::LEFT},   'U', {radian_t(0), Direction::LEFT},   centimeter_t(192));
    graph.connect('T', {PI,          Direction::RIGHT},  'V', {radian_t(0), Direction::RIGHT},  centimeter_t(233));
    graph.connect('U', {PI,          Direction::CENTER}, 'W', {radian_t(0), Direction::CENTER}, centimeter_t(183));
    graph.connect('V', {PI,          Direction::CENTER}, 'X', {radian_t(0), Direction::CENTER}, centimeter_t(149));
    graph.connect('W', {PI,          Direction::LEFT},   'Y', {radian_t(0), Direction::CENTER}, centimeter_t(264));
    graph.connect('W', {PI,          Direction::RIGHT},  'X', {PI,          Direction::LEFT},   centimeter_t(377));
}

#endif // TRACK == TEST_TRACK || COMPILE_ALL_TRACKS
