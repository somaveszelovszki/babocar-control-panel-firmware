#pragma once

#include <micro/container/vec.hpp>

namespace micro {

struct Node;

/* @brief Graph edge.
 */
struct Edge {
    Node *node1;    // The first node.
    Node *node2;    // The second node.
};

/* @brief Graph node.
 */
struct Node {
    static constexpr uint8_t MAX_NUM_EDGES = 4; // Maximum number of edges for one node.

    vec<Edge*, MAX_NUM_EDGES> edges; // The edges.
};

} // namespace micro
