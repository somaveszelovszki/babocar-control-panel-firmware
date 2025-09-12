#pragma once

#include <micro/container/vector.hpp>

/* @brief Graph edge.
 */
template <typename NodeType> struct Edge {
    NodeType* node1{nullptr}; // The first node.
    NodeType* node2{nullptr}; // The second node.

    Edge() = default;

    Edge(NodeType& node1, NodeType& node2) : node1(&node1), node2(&node2) {}
};

/* @brief Graph node.
 */
template <typename EdgeType, uint32_t N> struct Node {
    micro::vector<EdgeType*, N> edges; // The edges.
};
