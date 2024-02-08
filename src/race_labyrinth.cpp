#include <micro/utils/units.hpp>

#include <cfg_track.hpp>

#if TRACK == RACE_TRACK || COMPILE_ALL_TRACKS

#include <LabyrinthGraph.hpp>
#include <track.hpp>

using namespace micro;

void buildRaceLabyrinthGraph(LabyrinthGraph& OUT graph) {
    graph.addJunction(Junction('A', { centimeter_t( 480), centimeter_t(-720) }));
    graph.addJunction(Junction('B', { centimeter_t( 240), centimeter_t(-720) }));
    graph.addJunction(Junction('C', { centimeter_t( 600), centimeter_t(-600) }));
    graph.addJunction(Junction('D', { centimeter_t( 360), centimeter_t(-600) }));
    graph.addJunction(Junction('E', { centimeter_t( 120), centimeter_t(-600) }));
    graph.addJunction(Junction('F', { centimeter_t( 480), centimeter_t(-480) }));
    graph.addJunction(Junction('G', { centimeter_t( 240), centimeter_t(-480) }));
    graph.addJunction(Junction('H', { centimeter_t( 600), centimeter_t(-360) }));
    graph.addJunction(Junction('I', { centimeter_t( 360), centimeter_t(-360) }));
    graph.addJunction(Junction('J', { centimeter_t( 120), centimeter_t(-360) }));
    graph.addJunction(Junction('K', { centimeter_t( 480), centimeter_t(-240) }));
    graph.addJunction(Junction('L', { centimeter_t( 240), centimeter_t(-240) }));
    graph.addJunction(Junction('M', { centimeter_t( 600), centimeter_t(-120) }));
    graph.addJunction(Junction('N', { centimeter_t( 360), centimeter_t(-120) }));
    graph.addJunction(Junction('O', { centimeter_t( 120), centimeter_t(-120) }));
    graph.addJunction(Junction('P', { centimeter_t( 720), centimeter_t(   0) }));
    graph.addJunction(Junction('Q', { centimeter_t( 600), centimeter_t(   0) }));
    graph.addJunction(Junction('R', { centimeter_t( 480), centimeter_t(   0) }));
    graph.addJunction(Junction('S', { centimeter_t( 360), centimeter_t(   0) }));
    graph.addJunction(Junction('T', { centimeter_t( 240), centimeter_t(   0) }));
    graph.addJunction(Junction('U', { centimeter_t(   0), centimeter_t(   0) }));
    graph.addJunction(Junction('V', { centimeter_t( 480), centimeter_t( 120) }));
    graph.addJunction(Junction('W', { centimeter_t( 240), centimeter_t( 120) }));

  // TODO: dead ends

  // TODO: sections
}

#endif // TRACK == RACE_TRACK || COMPILE_ALL_TRACKS
