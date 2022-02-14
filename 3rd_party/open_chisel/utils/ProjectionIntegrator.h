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

#ifndef PROJECTIONINTEGRATOR_H_
#define PROJECTIONINTEGRATOR_H_

#include "camera/ColorImage.h"
#include "camera/DepthImage.h"
#include "camera/PinholeCamera.h"
#include "geometry/AABB.h"
#include "geometry/Chunk.h"
#include "geometry/Frustum.h"
#include "geometry/Geometry.h"

#include "truncation/Truncator.h"
#include "weighting/Weighter.h"

#include <immintrin.h>
#include <smmintrin.h>
#include <xmmintrin.h>

namespace chisel {

class ProjectionIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ProjectionIntegrator();
  ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w,
                       float carvingDist, bool enableCarving,
                       const Vec3List& centroids);

  ~ProjectionIntegrator();

  bool voxelUpdateSIMD(const float* depth_pointer, unsigned char* colorImage,
                       const PinholeCamera& depthCamera,
                       const Transform& depthCameraPose, int integrateFlag,
                       Chunk* chunk, float* observationQualityPointer,
                       float& chunkObservationQuality) const;
  inline bool IntegrateColor(float* depthImage,
                             const PinholeCamera& depthCamera,
                             const Transform& depthCameraPose,
                             unsigned char* colorImage, Chunk* chunk,
                             int integrate_flag, float& chunkObservationQuality,
                             float* observationQualityPointer) const {
    bool updated = false;
    chunkObservationQuality = 0;
    updated = voxelUpdateSIMD(
        depthImage, colorImage, depthCamera, depthCameraPose, integrate_flag,
        chunk, observationQualityPointer, chunkObservationQuality);
    return updated;
  }

  inline const TruncatorPtr& GetTruncator() const { return truncator; }
  inline void SetTruncator(const TruncatorPtr& value) { truncator = value; }
  inline const WeighterPtr& GetWeighter() const { return weighter; }
  inline void SetWeighter(const WeighterPtr& value) { weighter = value; }

  inline float GetCarvingDist() const { return carvingDist; }
  inline bool IsCarvingEnabled() const { return enableVoxelCarving; }
  inline void SetCarvingDist(float dist) { carvingDist = dist; }
  inline void SetCarvingEnabled(bool enabled) { enableVoxelCarving = enabled; }

  inline void SetCentroids(const Vec3List& c) { centroids = c; }

  __m256* centroids_simd0;
  __m256* centroids_simd1;
  __m256* centroids_simd2;
  Vec3List diff_centroids;
  Vec3List centroids;

 protected:
  TruncatorPtr truncator;
  WeighterPtr weighter;
  float carvingDist;
  bool enableVoxelCarving;
};

}  // namespace chisel

#endif  // PROJECTIONINTEGRATOR_H_
