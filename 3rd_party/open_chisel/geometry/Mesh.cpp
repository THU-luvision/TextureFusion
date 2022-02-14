// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "geometry/Mesh.h"
#include <cmath>
#include <iostream>


namespace chisel {

Mesh::Mesh() {
  simplified = false;
  outlier_checked = false;
  memset(adj, false, 6 * sizeof(bool));
  // memset(offset,-1, GRID_RESERVED*sizeof(short));
  origin = Vec3(0, 0, 0);
}

Mesh::~Mesh() {}

void Mesh::SimplifyByClustering(float resolution, Vec3 chunkOri) {
  if (simplified) return;

  origin = chunkOri;
  grid_resolution = resolution;

  for (int i = 0; i < vertices.size(); i++) int k = GetIndice(vertices[i]);

  simplified = true;
}

int Mesh::GetIndice(Vec3 vert) {
  Point3 pos;
  for (int j = 0; j < 3; j++) {
    pos[j] = floor((vert(j) - origin(j)) / grid_resolution);
  }
  if (pos[0] >= GRID_EACH_DIM) {
    adj[1] = true;
  }
  if (pos[1] >= GRID_EACH_DIM) {
    adj[3] = true;
  }
  if (pos[2] >= GRID_EACH_DIM) {
    adj[5] = true;
  }
  if (pos[0] <= 0) {
    adj[0] = true;
  }
  if (pos[1] <= 0) {
    adj[2] = true;
  }
  if (pos[2] <= 0) {
    adj[4] = true;
  }
  /*if ( pos[0] >= GRID_EACH_DIM && pos[1] >= GRID_EACH_DIM && pos[2] <
  GRID_EACH_DIM) return -3; if ( pos[0] >= GRID_EACH_DIM && pos[1] <
  GRID_EACH_DIM && pos[2] >= GRID_EACH_DIM) return -5; if ( pos[0] <
  GRID_EACH_DIM && pos[1] >= GRID_EACH_DIM && pos[2] >= GRID_EACH_DIM) return
  -6; if ( pos[0] >= GRID_EACH_DIM && pos[1] >= GRID_EACH_DIM && pos[2] >=
  GRID_EACH_DIM) return -7;*/

  int k = pos[0] + pos[1] * GRID_EACH_DIM +
          pos[2] * (GRID_EACH_DIM * GRID_EACH_DIM);
  return k;
}

}  // namespace chisel
