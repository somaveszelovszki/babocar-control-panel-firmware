#pragma once

#include <etl/vector.h>

/* @brief Graph edge.
 */
template <typename NodeType>
struct Edge {
    Edge(NodeType& node1, NodeType& node2)
        : node1(&node1)
        , node2(&node2) {}

    NodeType *node1;    // The first node.
    NodeType *node2;    // The second node.
};

/* @brief Graph node.
 */
template <typename EdgeType, uint32_t N>
struct Node {
    etl::vector<EdgeType*, N> edges; // The edges.
};
