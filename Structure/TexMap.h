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
#ifndef TEXMAP_H
#define TEXMAP_H

#include <exception>
#include <unordered_map>
#include "ChunkManager.h"
#include "GCSLAM/MultiViewGeometry.h"
#include "Stopwatch.h"
#include "full.h"
#include "sparse_matrix.h"
#include "uni_graph.h"

#include <cmath>
#include <cstdio>

typedef unsigned int uint_t;
typedef float cost_t;
constexpr uint_t simd_w = mapmap::sys_max_simd_width<cost_t>();
typedef mapmap::UnaryTable<cost_t, simd_w> unary_t;
typedef mapmap::PairwisePotts<cost_t, simd_w> pairwise_t;
typedef SparseMat DataCosts;

class TexMap {
 public:
  //	cv::Mat texture_buffer;
  //	int texture_buffer_width;
  //  int texture_buffer_height;

  float adjacent_cost = 0.5f;  // edge cost
  float pairwise_cost = 1.0f;  // pairwise cost

  UniGraph chunkGraph;
  DataCosts dataCost;  // related to observation quality and visibility
  float max_quality;   // used to normalize datacost: max of all
                       // observationqualities
  std::vector<float> statistic;

  // for MRFsolver
  std::vector<int> labelstorage;
  mapmap::mapMAP_control ctr;

  TexMap();
  ~TexMap();

  void update_chunkgraph(chisel::ChunkIDList &chunksToUpdate,
                         chisel::ChunkManager &chunkManager);
  void update_datacost(chisel::ChunkIDList &chunksToUpdate,
                       chisel::ChunkManager &chunkManager,
                       std::vector<int> &lookup, int frameindex,
                       std::vector<int> &framesToUpdate);

  void view_selection(
      chisel::ChunkIDList &chunksToUpdate,
      const std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist);
  void view_selection(
      const std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist);
  void check_graph(chisel::ChunkManager &chunkManager);

  void clear();
};

typedef TexMap *TexPtr;

#endif  // TEXMAP_H
