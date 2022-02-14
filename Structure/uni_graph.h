/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef TEX_UNIGRAPH_HEADER
#define TEX_UNIGRAPH_HEADER

#include <algorithm>
#include <cassert>
#include <vector>
#include "ChunkManager.h"

/**
 * Implementation of a unidirectional graph with dynamic amount of nodes using
 * adjacency lists.
 */
class UniGraph {
 private:
  std::vector<std::vector<std::size_t>> adj_lists;
  std::vector<std::size_t> labels;
  std::size_t edges;

 public:
  std::unordered_map<chisel::ChunkID, std::size_t, chisel::ChunkHasher> chunks;

  /**
   * Creates a unidirectional graph without edges.
   * @param nodes number of nodes.
   */
  UniGraph(std::size_t nodes);

  /**
   * Adds a new node
   * returns the indice of new node
   */
  bool add_node(chisel::ChunkID id);

  /**
   * Check if chunk's neighbour exists
   * if exists, add edge
   */
  void add_edge_by_node(chisel::ChunkID id);

  void add_edge_by_node(chisel::ChunkID id, bool flag[6]);

  /**
   * Adds an edge between the nodes with indices n1 and n2.
   * If the edge exists nothing happens.
   * @warning asserts that the indices are valid.
   */
  void add_edge(std::size_t n1, std::size_t n2);

  /**
   * Removes the edge between the nodes with indices n1 and n2.
   * If the edge does not exist nothing happens.
   * @warning asserts that the indices are valid.
   */
  void remove_edge(std::size_t n1, std::size_t n2);

  /**
   * Returns true if an edge between the nodes with indices n1 and n2 exists.
   * @warning asserts that the indices are valid.
   */
  bool has_edge(std::size_t n1, std::size_t n2) const;

  /** Returns the number of edges. */
  std::size_t num_edges() const;

  /** Returns the number of nodes. */
  std::size_t num_nodes() const;

  /**
   * Sets the label of node with index n to label.
   * @warning asserts that the index is valid.
   */
  void set_label(std::size_t n, std::size_t label);

  /**
   * Returns the label of node with index n.
   * @warning asserts that the index is valid.
   */
  std::size_t get_label(std::size_t n) const;

  /**
   * Select subgraph of given index.
   */
  void get_subgraphs(std::vector<std::size_t> index,
                     std::vector<std::vector<std::size_t>>& subgraphs);

  /**
   * Fills given vector with all subgraphs of the given label.
   * A subgraph is a vector containing all indices of connected nodes with the
   * same label.
   */
  // void get_subgraphs(std::size_t label, std::vector<std::vector<std::size_t>
  // > * subgraphs);

  std::vector<std::size_t> const& get_adj_nodes(std::size_t node) const;

  void clear();

  void remove_node(chisel::ChunkID id);
};

inline void UniGraph::add_edge(std::size_t n1, std::size_t n2) {
  assert(n1 < num_nodes() && n2 < num_nodes());
  if (!has_edge(n1, n2) && n1 != n2) {
    adj_lists[n1].emplace_back(n2);
    adj_lists[n2].emplace_back(n1);
    ++edges;
  }
}

inline void delete_element(std::vector<std::size_t>* vec, std::size_t element) {
  vec->erase(std::remove(vec->begin(), vec->end(), element), vec->end());
}

inline void UniGraph::remove_edge(std::size_t n1, std::size_t n2) {
  assert(n1 < num_nodes() && n2 < num_nodes());
  if (has_edge(n1, n2)) {
    delete_element(&adj_lists[n1], n2);
    delete_element(&adj_lists[n2], n1);
    --edges;
  }
}

inline bool UniGraph::has_edge(std::size_t n1, std::size_t n2) const {
  assert(n1 < num_nodes() && n2 < num_nodes());
  std::vector<std::size_t> const& adj_list = adj_lists[n1];
  return std::find(adj_list.begin(), adj_list.end(), n2) != adj_list.end();
}

inline std::size_t UniGraph::num_edges() const { return edges; }

inline std::size_t UniGraph::num_nodes() const { return adj_lists.size(); }

inline void UniGraph::set_label(std::size_t n, std::size_t label) {
  assert(n < num_nodes());
  labels[n] = label;
}

inline std::size_t UniGraph::get_label(std::size_t n) const {
  assert(n < num_nodes());
  return labels[n];
}

inline std::vector<std::size_t> const& UniGraph::get_adj_nodes(
    std::size_t node) const {
  assert(node < num_nodes());
  return adj_lists[node];
}

inline void UniGraph::clear() {
  adj_lists.clear();
  labels.clear();
  chunks.clear();
  edges = 0;
}

#endif /* TEX_UNIGRAPH_HEADER */
