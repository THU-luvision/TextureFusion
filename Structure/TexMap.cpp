// 3-Clause BSD License
// Copyright 2018 L. Han and S. Gu

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "TexMap.h"

struct cmp_quality {
  bool operator()(const NodeCost& n1, const NodeCost& n2) {
    return n1.second < n2.second;
  }
} mycmp;

TexMap::TexMap() : chunkGraph(0), dataCost() {
  // configure solver controller
  ctr.use_multilevel = true;
  ctr.use_spanning_tree = true;
  ctr.use_acyclic = true;
  ctr.spanning_tree_multilevel_after_n_iterations = 5;
  ctr.force_acyclic = true;
  ctr.min_acyclic_iterations = 5;
  ctr.relax_acyclic_maximal = true;
  ctr.tree_algorithm = mapmap::LOCK_FREE_TREE_SAMPLER;
  ctr.sample_deterministic = true;
  ctr.initial_seed = 548923723;
}

TexMap::~TexMap() {}

void TexMap::update_chunkgraph(chisel::ChunkIDList& chunksToUpdate,
                               chisel::ChunkManager& chunkManager) {
  chisel::MeshMap& allMeshes = chunkManager.GetAllMutableMeshes();
  for (int i = 0; i < chunksToUpdate.size(); i++) {
    chunkGraph.add_node(chunksToUpdate[i]);
  }
  for (int i = 0; i < chunksToUpdate.size(); i++) {
    if (allMeshes.find(chunksToUpdate[i]) == allMeshes.end()) continue;
    chisel::MeshPtr mesh = allMeshes.find(chunksToUpdate[i])->second;
    chunkGraph.add_edge_by_node(chunksToUpdate[i], mesh->adj);
  }
}

void TexMap::update_datacost(chisel::ChunkIDList& chunksToUpdate,
                             chisel::ChunkManager& chunkManager,
                             std::vector<int>& lookup, int frameindex,
                             std::vector<int>& framesToUpdate) {
  for (int i = 0; i < chunksToUpdate.size(); i++) {
    float quality = 0.0f;
    chisel::ChunkPtr chunk = chunkManager.GetChunk(chunksToUpdate[i]);
    std::size_t chunkindex = chunkGraph.chunks.find(chunksToUpdate[i])->second;
    statistic.resize(chunkindex + 1, 1.0f);

    auto its = chunk->observations.find(frameindex);
    if (its != chunk->observations.end()) quality = its->second;
    if (quality > statistic[chunkindex]) statistic[chunkindex] = quality;

    if (quality > 0.0f)
      dataCost.add_value(chunkindex, lookup[frameindex], quality);
    if (dataCost.cols() <= chunkindex) dataCost.resize(chunkindex + 1);

    quality = 0.0f;
    for (int j = 0; j < framesToUpdate.size(); j++) {
      auto it = chunk->observations.find(framesToUpdate[j]);
      if (it == chunk->observations.end()) {
        if (lookup[framesToUpdate[j]] < 0)
          std::cout << "warning, index wrong!" << std::endl;
        auto fob =
            dataCost.col(chunkindex)
                .find(static_cast<std::size_t>(lookup[framesToUpdate[j]]));
        if (fob != dataCost.col(chunkindex).end())
          dataCost.remove_observation(
              chunkindex, static_cast<std::size_t>(lookup[framesToUpdate[j]]));
      } else {
        quality = it->second;
        if (quality > statistic[chunkindex]) statistic[chunkindex] = quality;
        if (lookup[framesToUpdate[j]] < 0)
          std::cout << "warning, index wrong!" << std::endl;
        if (quality > 0.0f)
          dataCost.set_value(
              chunkindex, static_cast<std::size_t>(lookup[framesToUpdate[j]]),
              quality);
      }
    }
  }
}

void TexMap::check_graph(chisel::ChunkManager& chunkManager) {
  const chisel::MeshMap& allMeshes = chunkManager.GetAllMeshes();
  int cnt = 0;
  for (auto chunkit : chunkGraph.chunks) {
    bool exist = (allMeshes.find(chunkit.first) != allMeshes.end());
    if (!exist) {
      chunkGraph.remove_node(chunkit.first);
      dataCost.remove_node(chunkit.second);
      cnt++;
    }
  }
}

void TexMap::view_selection(
    const std::vector<MultiViewGeometry::KeyFrameDatabase>& kflist) {
  std::size_t num_nodes = chunkGraph.num_nodes();
  mapmap::Graph<cost_t> mgraph(num_nodes);
  for (std::size_t i = 0; i < num_nodes; ++i) {
    if (dataCost.col(i).empty()) continue;
    std::vector<std::size_t> adj_nodes = chunkGraph.get_adj_nodes(i);
    for (std::size_t j = 0; j < adj_nodes.size(); ++j) {
      std::size_t adj_node = adj_nodes[j];
      if (dataCost.col(adj_node).empty()) continue;

      // Uni directional
      if (i < adj_node) {
        mgraph.add_edge(i, adj_node, adjacent_cost);
      }
    }
  }
  mgraph.update_components();

  mapmap::LabelSet<cost_t, simd_w> label_set(num_nodes, false);
  for (std::size_t i = 0; i < num_nodes; i++) {
    std::vector<NodeCost> data_costs_for_node(dataCost.col(i).begin(),
                                              dataCost.col(i).end());
    std::vector<mapmap::_iv_st<cost_t, simd_w>> labels;

    if (data_costs_for_node.empty()) {
      labels.emplace_back(0);
    } else {
      labels.resize(data_costs_for_node.size());
      for (std::size_t j = 0; j < data_costs_for_node.size(); j++) {
        labels[j] =
            static_cast<std::uint16_t>(data_costs_for_node[j].first + 1);
      }
    }
    label_set.set_label_set_for_node(i, labels);
  }

  std::vector<unary_t> unaries;
  unaries.reserve(num_nodes);
  pairwise_t pairwise(pairwise_cost);
  for (std::size_t i = 0; i < num_nodes; ++i) {
    std::vector<NodeCost> data_costs_for_node(dataCost.col(i).begin(),
                                              dataCost.col(i).end());
    std::vector<mapmap::_s_t<cost_t, simd_w>> costs;

    if (data_costs_for_node.empty()) {
      costs.emplace_back(1.0f);
    } else {
      float column_max = std::max_element(data_costs_for_node.begin(),
                                          data_costs_for_node.end(), mycmp)
                             ->second;
      costs.resize(data_costs_for_node.size());
      for (std::size_t j = 0; j < data_costs_for_node.size(); j++) {
        float cost = data_costs_for_node[j].second;
        costs[j] = 1.0f - cost / column_max;
      }
    }

    unaries.emplace_back(i, &label_set);
    unaries.back().set_costs(costs);
  }

  mapmap::mapMAP<cost_t, simd_w> solver;
  static mapmap::StopWhenReturnsDiminish<cost_t, simd_w> terminate(10, 0.01);
  solver.set_termination_criterion(&terminate);

  auto display = [](const mapmap::luint_t time_ms,
                    const mapmap::_iv_st<cost_t, simd_w> objective) {
    // std::cout << "\t\t" << time_ms / 1000 << "\t" << objective << std::endl;
  };
  solver.set_logging_callback(display);

  solver.set_graph(&mgraph);
  solver.set_label_set(&label_set);
  for (std::size_t i = 0; i < num_nodes; ++i) {
    solver.set_unary(i, &unaries[i]);
  }
  solver.set_pairwise(&pairwise);

  std::vector<mapmap::_iv_st<cost_t, simd_w>> solution;
  if (labelstorage.empty()) {
    try {
      solver.optimize(solution, ctr);
    } catch (std::runtime_error& e) {
      std::cout << "out of run time!" << std::endl;
      throw 'o';
    }
  } else {
    for (int i = 0; i < labelstorage.size(); i++) {
      int offset = label_set.offset_for_label(i, labelstorage[i]);
      if (offset >= 0)
        solution.emplace_back(offset);
      else
        solution.emplace_back(0);
    }
    for (int i = labelstorage.size() - 1; i < num_nodes; i++) {
      solution.emplace_back(0);
    }
    // std::cout << std::endl << "solution initialized" << std::endl;
    try {
      solver.optimize(solution, ctr, true);
    } catch (std::runtime_error& e) {
      throw std::runtime_error("optimization failure");
    }
    labelstorage.clear();
  }

  // Label 0 is undefined.
  std::size_t num_labels = dataCost.rows();
  std::size_t undefined = 0;
  // Extract resulting labeling from solver.
  for (std::size_t i = 0; i < num_nodes; ++i) {
    int label =
        static_cast<std::size_t>(label_set.label_from_offset(i, solution[i]));
    if (label < 0 || num_labels < static_cast<std::size_t>(label)) {
      // std::cout << i << " label:" << label << std::endl;
      throw std::runtime_error("Incorrect labeling");
    }
    if (label == 0) {
      undefined += 1;
      if (chunkGraph.get_label(i) == 0) {
        chunkGraph.set_label(i, kflist[kflist.size() - 2].keyFrameIndex);
      }
    } else
      chunkGraph.set_label(i, kflist[label - 1].keyFrameIndex);
    labelstorage.emplace_back(label);
  }
  /*if(chunkGraph.chunks.find(ChunkID(-10,8,14))!=chunkGraph.chunks.end())
  {
      int tmp = chunkGraph.chunks.find(ChunkID(-10,8,14))->second;
      std::cout << "assignment of chunk: " << label_set.label_from_offset(tmp,
  solution[tmp])-1 << "/" << chunkGraph.get_label(tmp) << std::endl;
  }*/

  // solver.reset();
}

void TexMap::view_selection(
    chisel::ChunkIDList& chunksToUpdate,
    const std::vector<MultiViewGeometry::KeyFrameDatabase>& kflist) {
  std::cout << "CHECK 0" << std::endl;
  std::vector<std::size_t> concerns;
  for (std::size_t i = 0; i < chunksToUpdate.size(); ++i) {
    if (chunkGraph.chunks.find(chunksToUpdate[i]) != chunkGraph.chunks.end()) {
      std::size_t k = chunkGraph.chunks.find(chunksToUpdate[i])->second;
      concerns.emplace_back(k);
    }
  }
  if (concerns.empty()) return;

  std::size_t num_nodes = concerns.size();  // chunkGraph.num_nodes();
  mapmap::Graph<cost_t> mgraph(num_nodes);
  for (std::size_t i = 0; i < num_nodes; ++i) {
    std::size_t k = concerns[i];
    if (dataCost.col(k).empty()) continue;
    std::vector<std::size_t> adj_nodes = chunkGraph.get_adj_nodes(k);
    for (std::size_t j = 0; j < adj_nodes.size(); ++j) {
      std::size_t adj_node = adj_nodes[j];
      std::vector<std::size_t>::iterator it =
          std::find(concerns.begin(), concerns.end(), adj_node);
      if (it == concerns.end()) continue;
      if (dataCost.col(adj_node).empty()) continue;
      adj_node = std::distance(concerns.begin(), it);

      // Uni directional
      if (i < adj_node) {
        mgraph.add_edge(i, adj_node, adjacent_cost);
      }
    }
  }
  mgraph.update_components();

  mapmap::LabelSet<cost_t, simd_w> label_set(num_nodes, false);
  for (std::size_t i = 0; i < num_nodes; i++) {
    std::size_t k = concerns[i];
    std::vector<NodeCost> data_costs_for_node(dataCost.col(k).begin(),
                                              dataCost.col(k).end());
    std::vector<mapmap::_iv_st<cost_t, simd_w>> labels;

    if (data_costs_for_node.empty()) {
      labels.emplace_back(0);
    } else {
      labels.resize(data_costs_for_node.size());
      for (std::size_t j = 0; j < data_costs_for_node.size(); j++) {
        labels[j] =
            static_cast<std::uint16_t>(data_costs_for_node[j].first + 1);
      }
    }
    label_set.set_label_set_for_node(i, labels);
  }

  std::vector<unary_t> unaries;
  unaries.reserve(num_nodes);
  pairwise_t pairwise(pairwise_cost);
  for (std::size_t i = 0; i < num_nodes; ++i) {
    std::size_t k = concerns[i];
    std::vector<NodeCost> data_costs_for_node(dataCost.col(k).begin(),
                                              dataCost.col(k).end());
    std::vector<mapmap::_s_t<cost_t, simd_w>> costs;

    if (data_costs_for_node.empty()) {
      costs.emplace_back(1.0f);
    } else {
      float column_max = std::max_element(data_costs_for_node.begin(),
                                          data_costs_for_node.end(), mycmp)
                             ->second;
      costs.resize(data_costs_for_node.size());
      for (std::size_t j = 0; j < data_costs_for_node.size(); j++) {
        float cost = data_costs_for_node[j].second;
        costs[j] = 1.0f - cost / column_max;
      }
    }

    unaries.emplace_back(i, &label_set);
    unaries.back().set_costs(costs);
  }

  mapmap::mapMAP<cost_t, simd_w> solver;
  static mapmap::StopWhenReturnsDiminish<cost_t, simd_w> terminate(10, 0.01);
  solver.set_termination_criterion(&terminate);

  auto display = [](const mapmap::luint_t time_ms,
                    const mapmap::_iv_st<cost_t, simd_w> objective) {
    // std::cout << "\t\t" << time_ms / 1000 << "\t" << objective << std::endl;
  };
  solver.set_logging_callback(display);

  // pass variables to solver
  solver.set_graph(&mgraph);
  solver.set_label_set(&label_set);
  for (std::size_t i = 0; i < num_nodes; ++i) {
    solver.set_unary(i, &unaries[i]);
  }
  solver.set_pairwise(&pairwise);

  std::vector<mapmap::_iv_st<cost_t, simd_w>> solution;
  if (true)  // labels.empty())
  {
    try {
      solver.optimize(solution, ctr);
    } catch (std::runtime_error& e) {
      std::cout << "out of run time!" << std::endl;
      throw 'o';
    }
  } else {
    /*for (int i = 0; i < labels.size(); i++)
    {
        int offset = label_set.offset_for_label(i, labels[i]);
        if (offset >= 0) solution.emplace_back(offset);
        else solution.emplace_back(0);
    }
    for (int i = labels.size() ; i < num_nodes ; i++)
    {
        //solution.emplace_back(0);
        int offset = label_set.offset_for_label(i, kflist.size()-2);
        if (offset > 0) solution.emplace_back(offset);
        else solution.emplace_back(0);
    }*/

    try {
      solver.optimize(solution, ctr);
    } catch (std::runtime_error& e) {
      throw std::runtime_error("out of run time");
    }
  }

  // Label 0 is undefined.
  std::size_t num_labels = dataCost.rows();
  std::size_t undefined = 0;
  // Extract resulting labeling from solver.
  for (std::size_t i = 0; i < num_nodes; ++i) {
    std::size_t k = concerns[i];
    int label =
        static_cast<std::size_t>(label_set.label_from_offset(i, solution[i]));
    if (label < 0 || num_labels < static_cast<std::size_t>(label)) {
      // std::cout << i << " label:" << label << std::endl;
      throw std::runtime_error("Incorrect labeling");
    }
    if (label == 0) {
      undefined += 1;
      if (chunkGraph.get_label(k) == 0) {
        chunkGraph.set_label(k, kflist[kflist.size() - 2].keyFrameIndex);
      }
    } else
      chunkGraph.set_label(k, kflist[label - 1].keyFrameIndex);
  }
}

void TexMap::clear() {
  chunkGraph.clear();
  dataCost.clear();
  max_quality = 0;
}
