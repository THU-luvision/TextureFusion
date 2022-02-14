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
#ifndef BASICAPI_H
#define BASICAPI_H

#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include "GCSLAM/MultiViewGeometry.h"
#include "Tools/OpenNI2/LiveLogReader.h"
#include "Tools/OpenNI2/LogReader.h"

namespace BasicAPI {
struct vec8 {
  __m256 xmm;

  vec8(__m256 v) : xmm(v) {}

  vec8(float v) { xmm = _mm256_set1_ps(v); }

  vec8(float a, float b, float c, float d, float e, float f, float g, float h) {
    xmm = _mm256_set_ps(h, g, f, e, d, c, b, a);
  }

  vec8 floor() { return _mm256_floor_ps(xmm); }

  vec8(const float *v) { xmm = _mm256_load_ps(v); }

  vec8 operator&(const vec8 &v) const {
    return vec8(_mm256_and_ps(xmm, v.xmm));
  }

  vec8 operator>(const vec8 &v) const {
    return vec8(_mm256_cmp_ps(xmm, v.xmm, _CMP_GT_OS));
  }
  vec8 operator<(const vec8 &v) const {
    return vec8(_mm256_cmp_ps(v.xmm, xmm, _CMP_GT_OS));
  }

  vec8 operator*(const vec8 &v) const {
    return vec8(_mm256_mul_ps(xmm, v.xmm));
  }

  vec8 operator+(const vec8 &v) const {
    return vec8(_mm256_add_ps(xmm, v.xmm));
  }

  vec8 operator-(const vec8 &v) const {
    return vec8(_mm256_sub_ps(xmm, v.xmm));
  }

  vec8 operator/(const vec8 &v) const {
    return vec8(_mm256_div_ps(xmm, v.xmm));
  }

  void operator*=(const vec8 &v) { xmm = _mm256_mul_ps(xmm, v.xmm); }

  void operator+=(const vec8 &v) { xmm = _mm256_add_ps(xmm, v.xmm); }

  void operator-=(const vec8 &v) { xmm = _mm256_sub_ps(xmm, v.xmm); }

  void operator/=(const vec8 &v) { xmm = _mm256_div_ps(xmm, v.xmm); }

  void operator>>(float *v) { _mm256_store_ps(v, xmm); }
};

// must be initialized
void loadGlobalParameters(MultiViewGeometry::GlobalParameters &g_para,
                          std::string para_file);

// save trajectory
void saveTrajectoryFrameList(std::vector<Frame> &F, std::string fileName);
void saveTrajectoryKeyFrameList(std::vector<Frame> &F, std::string fileName);

void findCubeCorner(Frame &frame_ref,
                    MultiViewGeometry::CameraPara &cameraModel);
// dump PLY model
void savePLYFiles(
    std::string fileName,
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3d> > p,
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3d> >
        color);
void savePLYFrame(std::string fileName, const Frame &f,
                  const MultiViewGeometry::CameraPara &para);
void saveRefinedFrame(std::string fileName, const Frame &frame_new,
                      const MultiViewGeometry::CameraPara &para);

int detectAndExtractFeatures(Frame &t, int feature_num,
                             MultiViewGeometry::CameraPara para);

void refineDepthUseNormal(float *normal, float *depth, float fx, float fy,
                          float cx, float cy, float width, float height);
// eliminate outliers
void refineNewframes(Frame &frame_ref, Frame &frame_new,
                     MultiViewGeometry::CameraPara &cameraModel);

void refineNewframesSIMD(Frame &frame_ref, Frame &frame_new,
                         MultiViewGeometry::CameraPara &cameraModel);
void refineKeyframesSIMD(Frame &frame_ref, Frame &frame_new,
                         MultiViewGeometry::CameraPara &cameraModel);
void refineKeyframes(Frame &frame_ref, Frame &frame_new,
                     MultiViewGeometry::CameraPara &cameraModel);
void refineDepthUseNormalSIMD(float *normal, float *depth, float fx, float fy,
                              float cx, float cy, float width, float height);
void extractNormalMapSIMD(const cv::Mat &depthMap, cv::Mat &normalMap, float fx,
                          float fy, float cx, float cy);
void checkColorQuality(const cv::Mat &normalMap, cv::Mat &validColorFlag,
                       float fx, float fy, float cx, float cy);
void estimateColorQuality(const cv::Mat &depthMap, const cv::Mat &normalMap,
                          cv::Mat &qualityMap, const cv::Mat &rgb, float fx,
                          float fy, float cx, float cy);
// calculate reprojection error between two frames: (p1+x1) - (p2+x2)

void framePreprocess(Frame &t, MultiViewGeometry::CameraPara &camera);

int LoadRawData(int index, Frame &t, const std::vector<std::string> &rgb_files,
                const std::vector<std::string> &depth_files,
                const std::vector<double> &time_stamp,
                MultiViewGeometry::CameraPara &camera);
void spilt_word(std::string ori, std::vector<std::string> &res);

bool DirectoryExists(const char *pzPath);
void makeDir(const std::string &directory);
void printHelpFunctions();
/* sensorType: 0 for offline data
 *             1 for xtion, 2 for realsense
 *
 */
void parseInput(int argc, char **argv, int &showCaseMode,
                float &ipnutVoxelResolution, std::string &basepath,
                MultiViewGeometry::GlobalParameters &para, int &sensorType);

void initOfflineData(std::string work_folder,
                     std::vector<std::string> &rgb_files,
                     std::vector<std::string> &depth_files,
                     std::vector<double> &time_stamp,
                     Eigen::MatrixXd &ground_truth,
                     MultiViewGeometry::CameraPara &camera);
void initOpenNICamera(LogReader *logReader,
                      MultiViewGeometry::CameraPara &camera);
int initRS2Camera(rs2::pipeline &pipe, MultiViewGeometry::CameraPara &camera);

void blurriness(Frame &t);
}  // namespace BasicAPI

#endif  // BASICAPI_HPP
