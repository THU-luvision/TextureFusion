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
#ifndef PATCH_H
#define PATCH_H

#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include "ChunkManager.h"
#include "GCSLAM/frame.h"
#include "camera/PinholeCamera.h"
#include "geometry/Geometry.h"


#include <Eigen/Core>

namespace chisel {

void rgb2lab(Vec3List& rgbs, Vec3List& labs);
void lab2rgb(Vec3List& labs, Vec3List& rgbs);
Vec3 computeMean(Vec3List& colors);
Vec3 computeVar(Vec3List& colors);
Mat3x3 computeCov(Vec3List& colors);
void computeMeanAndCov(Vec3List& colors, Vec3& mean, Mat3x3& cov);

typedef cv::Rect Box;

class Patch {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Patch(MeshPtr meshit);

  int frameid;
  MeshPtr mesh;
  std::size_t texloc;
  cv::Mat image;

  bool has_image;
  bool has_adjusted;
  bool has_updated;
  bool wrong_mapping;

  Box boundingbox;
  Vec2List texcoord;
  Vec3List texcolor;
  Vec2 ratio;

  Vec3List paras;
  Vec3List labs;
  Vec3List labt;
  std::vector<size_t> caution;
  void Process();
  void preProcess();
  void postProcess();
  void postProcess(Vec3List& parameter);

  int CalculateTexColors();
  int CalculateTexCoords(Frame& pos, PinholeCamera& camera);
  void SetImage(cv::Mat& view);
  void clear();
  bool complete();

  inline void SetMesh(MeshPtr meshit) { mesh = meshit; }
  inline void SetFrameid(int frameit) { frameid = frameit; }
  inline ChunkID GetChunkid() { return mesh->chunkID; }
  inline float GetWidth() { return boundingbox.width; }
  inline float GetHeight() { return boundingbox.height; }
  inline int GetCoordsNum() { return texcoord.size(); }
  Vec3 bilinear(cv::Mat& view, Vec2 loc);
  float bilinear_depth(cv::Mat& view, Vec2 loc);
};
typedef std::shared_ptr<Patch> PatchPtr;
typedef std::shared_ptr<const Patch> PatchConstPtr;

}  // namespace chisel
#endif