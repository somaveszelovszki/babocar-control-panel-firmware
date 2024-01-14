#include <micro/test/utils.hpp>

#include <LabyrinthGraph.hpp>
#include <LabyrinthRoute.hpp>
#include <track.hpp>

#include "route.hpp"

using namespace micro;

TEST(race_labyrinth, valid) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();
    ASSERT_TRUE(graph.valid());
}

TEST(race_labyrinth, U_T) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('T');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, false, {
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::LEFT) }
    });
}

TEST(race_labyrinth, U_J) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('J');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, false, {
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::RIGHT) }
    });
}

TEST(race_labyrinth, U_N) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('N');
    const Junction& prevJunc   = *graph.findJunction(12);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(13), JunctionDecision(PI, Direction::CENTER) },
    });
}

TEST(race_labyrinth, U_O) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('O');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    // TODO: ???
    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(13), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(13), JunctionDecision(radian_t(0), Direction::LEFT)   }
    });
}

TEST(race_labyrinth, U_M) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('M');
    const Junction& prevJunc   = *graph.findJunction(13);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::LEFT)  },
        { graph.findJunction(10), JunctionDecision(PI,   Direction::RIGHT) },
    });
}

TEST(race_labyrinth, U_M_backward) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('M');
    const Junction& prevJunc   = *graph.findJunction(12);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(12), JunctionDecision(PI_2, Direction::LEFT)  },
        { graph.findJunction(10), JunctionDecision(PI,   Direction::RIGHT) },
    });
}

TEST(race_labyrinth, U_K) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('U');
    const Segment& dest        = *graph.findSegment('K');
    const Junction& prevJunc   = *graph.findJunction(12);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(13), JunctionDecision(PI,   Direction::CENTER) },
        { graph.findJunction(11), JunctionDecision(PI_2, Direction::LEFT)   },
    });
}


TEST(race_labyrinth, L_J) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('L');
    const Segment& dest        = *graph.findSegment('J');
    const Junction& prevJunc   = *graph.findJunction(11);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(9), JunctionDecision(PI_2,        Direction::CENTER) },
        { graph.findJunction(8), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(8), JunctionDecision(radian_t(0), Direction::LEFT)   },
    });
}

TEST(race_labyrinth, K_E) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('K');
    const Segment& dest        = *graph.findSegment('E');
    const Junction& prevJunc   = *graph.findJunction(11);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(5), JunctionDecision(PI_2,        Direction::RIGHT) },
        { graph.findJunction(4), JunctionDecision(radian_t(0), Direction::RIGHT) }
    });
}

TEST(race_labyrinth, K_G_deadend) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('K');
    const Segment& dest        = *graph.findSegment('G');
    const Junction& prevJunc   = *graph.findJunction(11);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(5), JunctionDecision(PI_2,        Direction::LEFT)   },
        { graph.findJunction(1), JunctionDecision(radian_t(0), Direction::LEFT)   },
        { graph.findJunction(3), JunctionDecision(3 * PI_2,    Direction::CENTER) }
    });
}

TEST(race_labyrinth, G_D_deadend) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('G');
    const Segment& dest        = *graph.findSegment('D');
    const Junction& prevJunc   = *graph.findJunction(3);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(3), JunctionDecision(PI_2,        Direction::CENTER) },
        { graph.findJunction(1), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(1), JunctionDecision(radian_t(0), Direction::RIGHT)  },
        { graph.findJunction(2), JunctionDecision(radian_t(0), Direction::CENTER) }
    });
}

TEST(race_labyrinth, H_F) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('H');
    const Segment& dest        = *graph.findSegment('F');
    const Junction& prevJunc   = *graph.findJunction(3);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(3), JunctionDecision(PI_2,        Direction::CENTER) },
        { graph.findJunction(1), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(5), JunctionDecision(3 * PI_2,    Direction::LEFT)   },
        { graph.findJunction(7), JunctionDecision(radian_t(0), Direction::LEFT)   }
    });
}

TEST(race_labyrinth, M_G_long) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('M');
    const Segment& dest        = *graph.findSegment('G');
    const Junction& prevJunc   = *graph.findJunction(10);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(9), JunctionDecision(PI_2,        Direction::CENTER) },
        { graph.findJunction(8), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(7), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(5), JunctionDecision(PI_2,        Direction::LEFT)   },
        { graph.findJunction(1), JunctionDecision(radian_t(0), Direction::LEFT)   },
        { graph.findJunction(3), JunctionDecision(3 * PI_2,    Direction::CENTER) },
    });
}

TEST(race_labyrinth, N_D) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('N');
    const Segment& dest        = *graph.findSegment('D');
    const Junction& prevJunc   = *graph.findJunction(11);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(11), JunctionDecision(PI_2,       Direction::LEFT)   },
        { graph.findJunction(5), JunctionDecision(PI_2,        Direction::RIGHT)  },
        { graph.findJunction(4), JunctionDecision(radian_t(0), Direction::LEFT)   },
        { graph.findJunction(2), JunctionDecision(radian_t(0), Direction::CENTER) },
    });
}

TEST(race_labyrinth, E_H) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('E');
    const Segment& dest        = *graph.findSegment('H');
    const Junction& prevJunc   = *graph.findJunction(4);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(4), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(5), JunctionDecision(3 * PI_2,    Direction::LEFT)   },
        { graph.findJunction(5), JunctionDecision(PI_2,        Direction::LEFT)   },
        { graph.findJunction(1), JunctionDecision(radian_t(0), Direction::LEFT)   },
        { graph.findJunction(3), JunctionDecision(3 * PI_2,    Direction::LEFT)   }
    });
}

TEST(race_labyrinth, I_J) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('I');
    const Segment& dest        = *graph.findSegment('J');
    const Junction& prevJunc   = *graph.findJunction(8);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(8), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(8), JunctionDecision(radian_t(0), Direction::LEFT)   },
    });
}

TEST(race_labyrinth, I_R) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('I');
    const Segment& dest        = *graph.findSegment('R');
    const Junction& prevJunc   = *graph.findJunction(8);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(8), JunctionDecision(PI, Direction::CENTER) },
        { graph.findJunction(7), JunctionDecision(PI, Direction::CENTER) },
    });
}

TEST(race_labyrinth, I_F) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('I');
    const Segment& dest        = *graph.findSegment('F');
    const Junction& prevJunc   = *graph.findJunction(9);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(8), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(7), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(7), JunctionDecision(radian_t(0), Direction::LEFT)   },
    });
}

TEST(race_labyrinth, H_E) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('H');
    const Segment& dest        = *graph.findSegment('E');
    const Junction& prevJunc   = *graph.findJunction(3);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(3), JunctionDecision(PI_2,        Direction::CENTER) },
        { graph.findJunction(1), JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(5), JunctionDecision(3 * PI_2,    Direction::LEFT)   },
        { graph.findJunction(5), JunctionDecision(PI_2,        Direction::RIGHT)  },
        { graph.findJunction(4), JunctionDecision(radian_t(0), Direction::RIGHT)  } 
    });
}

TEST(race_labyrinth, D_O) {
    LabyrinthGraph graph = buildRaceLabyrinthGraph();

    const Segment& src         = *graph.findSegment('D');
    const Segment& dest        = *graph.findSegment('O');
    const Junction& prevJunc   = *graph.findJunction(6);
    const Connection& prevConn = **std::find_if(src.edges.begin(), src.edges.end(), [&prevJunc](const Connection *c) {
        return c->junction == &prevJunc;
    });

    checkRoute(prevConn, src, dest, {}, true, {
        { graph.findJunction(6),  JunctionDecision(PI,          Direction::LEFT)   },
        { graph.findJunction(7),  JunctionDecision(PI,          Direction::CENTER) },
        { graph.findJunction(7),  JunctionDecision(radian_t(0), Direction::RIGHT)  },
        { graph.findJunction(8),  JunctionDecision(radian_t(0), Direction::RIGHT)  },
        { graph.findJunction(9), JunctionDecision(3 * PI_2,     Direction::LEFT)   },
        { graph.findJunction(10), JunctionDecision(radian_t(0), Direction::CENTER) },
        { graph.findJunction(10), JunctionDecision(PI,          Direction::LEFT)   },
    });
}