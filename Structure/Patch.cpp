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
#include "Patch.h"
#include <iostream>
namespace chisel {
Patch::Patch(MeshPtr meshit) {
  frameid = -1;
  mesh = meshit;
  has_image = false;
  has_updated = false;
  has_adjusted = false;
  wrong_mapping = false;
  ratio = Vec2(1, 1);
  texloc = 0;
}

int Patch::CalculateTexCoords(Frame& pos, PinholeCamera& camera) {
  int flag = 0;
  texcoord.resize(mesh->vertices.size());
  texcolor.resize(mesh->vertices.size(), Vec3(0, 0, 0));
  caution.clear();

  float minX = camera.GetWidth();
  float maxX = 0.0;
  float minY = camera.GetHeight();
  float maxY = 0.0;

  Mat4x4 T_g_l = pos.pose_sophus[0].inverse().matrix().cast<float>();

  int depth_compare = 0;
  int color_compare = 0;
  for (int i = 0; i < mesh->vertices.size(); i++) {
    if (i < 0) continue;
    Vec3 vert = mesh->vertices[i];
    Vec4 v_g = Vec4(vert(0), vert(1), vert(2), 1);
    Vec4 v_l = T_g_l * v_g;
    float dist = v_l(2);  // Vec3(v_l(0),v_l(1),v_l(2)).norm();
    float x = v_l(0) / v_l(2);
    float y = v_l(1) / v_l(2);
    float cameraX = x * camera.GetFx() + camera.GetCx() + 0.5;
    float cameraY = y * camera.GetFy() + camera.GetCy() + 0.5;

    if (cameraX < 0 || cameraX >= camera.GetWidth() || cameraY < 0 ||
        cameraY >= camera.GetHeight()) {
      flag = -1;
      caution.emplace_back(i);
    }

    if (cameraX < 0) cameraX = 0;
    if (cameraX >= camera.GetWidth()) cameraX = camera.GetWidth();
    if (cameraY < 0) cameraY = 0;
    if (cameraY >= camera.GetHeight()) cameraY = camera.GetHeight();
    texcoord[i] = Vec2(cameraX, cameraY);

    minX = minX < cameraX ? minX : cameraX;
    maxX = maxX > cameraX ? maxX : cameraX;
    minY = minY < cameraY ? minY : cameraY;
    maxY = maxY > cameraY ? maxY : cameraY;

    // int loc = (round(cameraY)* pos.rgb.cols + round(cameraX))*3;
    // texcolor[i]=
    // Vec3(pos.rgb.data[loc],pos.rgb.data[loc+1],pos.rgb.data[loc+2])/255.0;
    texcolor[i] = bilinear(pos.rgb, Vec2(cameraX, cameraY)) / 255.0;
    float depth = bilinear_depth(pos.refined_depth, Vec2(cameraX, cameraY));
    if (Vec3(texcolor[i] - mesh->colors[i]).norm() > 0.6) color_compare++;
    if (fabs(dist - depth) > 0.7) depth_compare++;
  }

  if (depth_compare > 0.3 * mesh->vertices.size() ||
      color_compare > 0.3 * mesh->vertices.size()) {
    wrong_mapping = true;
  } else
    wrong_mapping = false;

  if (maxX >= minX && maxY >= minY) {
    boundingbox = Box(minX - 2, minY - 2, maxX - minX + 5, maxY - minY + 5) &
                  Box(0, 0, camera.GetWidth() - 1, camera.GetHeight() - 1);
    for (int i = 0; i < texcoord.size(); i++) {
      texcoord[i] -= Vec2(boundingbox.x, boundingbox.y);
    }
  } else
    boundingbox = Box(0, 0, 0, 0);

  return flag;
}

Vec3 Patch::bilinear(cv::Mat& view, Vec2 loc) {
  int x = floor(loc(0));
  int y = floor(loc(1));
  if (x < view.cols - 1 && y < view.rows - 1) {
    Vec3 c1 = Vec3(view.at<cv::Vec3b>(y, x)[0], view.at<cv::Vec3b>(y, x)[1],
                   view.at<cv::Vec3b>(y, x)[2]);
    Vec3 c2 =
        Vec3(view.at<cv::Vec3b>(y, x + 1)[0], view.at<cv::Vec3b>(y, x + 1)[1],
             view.at<cv::Vec3b>(y, x + 1)[2]);
    Vec3 c3 =
        Vec3(view.at<cv::Vec3b>(y + 1, x)[0], view.at<cv::Vec3b>(y + 1, x)[1],
             view.at<cv::Vec3b>(y + 1, x)[2]);
    Vec3 c4 = Vec3(view.at<cv::Vec3b>(y + 1, x + 1)[0],
                   view.at<cv::Vec3b>(y + 1, x + 1)[1],
                   view.at<cv::Vec3b>(y + 1, x + 1)[2]);
    return c1 * (x + 1 - loc(0)) * (y + 1 - loc(1)) +
           c2 * (loc(0) - x) * (y + 1 - loc(1)) +
           c3 * (x + 1 - loc(0)) * (loc(1) - y) +
           c2 * (loc(0) - x) * (loc(1) - y);
  } else if (x < view.cols - 1 && y == view.rows - 1) {
    Vec3 c1 = Vec3(view.at<cv::Vec3b>(y, x)[0], view.at<cv::Vec3b>(y, x)[1],
                   view.at<cv::Vec3b>(y, x)[2]);
    Vec3 c2 =
        Vec3(view.at<cv::Vec3b>(y, x + 1)[0], view.at<cv::Vec3b>(y, x + 1)[1],
             view.at<cv::Vec3b>(y, x + 1)[2]);
    return c1 * (x + 1 - loc(0)) + c2 * (loc(0) - x);
  } else if (x == view.cols - 1 && y < view.rows - 1) {
    Vec3 c1 = Vec3(view.at<cv::Vec3b>(y, x)[0], view.at<cv::Vec3b>(y, x)[1],
                   view.at<cv::Vec3b>(y, x)[2]);
    Vec3 c2 =
        Vec3(view.at<cv::Vec3b>(y + 1, x)[0], view.at<cv::Vec3b>(y + 1, x)[1],
             view.at<cv::Vec3b>(y + 1, x)[2]);
    return c1 * (y + 1 - loc(1)) + c2 * (loc(1) - y);
  } else
    return Vec3(view.at<cv::Vec3b>(y, x)[0], view.at<cv::Vec3b>(y, x)[1],
                view.at<cv::Vec3b>(y, x)[2]);
}

float Patch::bilinear_depth(cv::Mat& view, Vec2 loc) {
  int x = floor(loc(0));
  int y = floor(loc(1));
  if (x < view.cols - 1 && y < view.rows - 1) {
    float c1 = view.at<float>(y, x);
    float c2 = view.at<float>(y, x + 1);
    float c3 = view.at<float>(y + 1, x);
    float c4 = view.at<float>(y + 1, x + 1);
    return c1 * (x + 1 - loc(0)) * (y + 1 - loc(1)) +
           c2 * (loc(0) - x) * (y + 1 - loc(1)) +
           c3 * (x + 1 - loc(0)) * (loc(1) - y) +
           c2 * (loc(0) - x) * (loc(1) - y);
  } else if (x < view.cols - 1 && y == view.rows - 1) {
    float c1 = view.at<float>(y, x);
    float c2 = view.at<float>(y, x + 1);
    return c1 * (x + 1 - loc(0)) + c2 * (loc(0) - x);
  } else if (x == view.cols - 1 && y < view.rows - 1) {
    float c1 = view.at<float>(y, x);
    float c2 = view.at<float>(y + 1, x);
    return c1 * (y + 1 - loc(1)) + c2 * (loc(1) - y);
  } else
    return view.at<float>(y, x);
}

void Patch::SetImage(cv::Mat& view) {
  image = view(boundingbox);  //浅拷贝
  has_image = true;
}

void Patch::clear() {
  frameid = -1;
  image.release();

  has_image = false;
  has_adjusted = false;

  texcoord.clear();
  texcolor.clear();
  labs.clear();
  paras.clear();
  ratio = Vec2(1, 1);
}

bool Patch::complete() {
  if (mesh == nullptr || mesh->vertices.empty() || !mesh->simplified)
    return false;
  if (!has_image || texcoord.empty() || frameid < 0) return false;
  return true;
}

void Patch::Process() {
  if (texcolor.empty() || texcolor.size() != mesh->colors.size()) return;

  Vec3List& lab_src = texcolor;
  Vec3List& lab_tar = mesh->colors;
  // rgb2lab(texcolor, lab_src);
  // rgb2lab(mesh->colors, lab_tar);
  Vec3 mean_src = computeMean(lab_src);
  Vec3 mean_tar = computeMean(lab_tar);
  Mat3x3 cov_src = computeCov(lab_src);
  Mat3x3 cov_tar = computeCov(lab_tar);

  float factor = cov_tar.trace() / cov_src.trace();
  factor = exp(-pow(log(factor), 2));

  Eigen::SelfAdjointEigenSolver<Mat3x3> es_src(cov_src);
  Mat3x3 diag_src = es_src.eigenvalues().asDiagonal();
  diag_src = diag_src.array().sqrt().matrix();
  Mat3x3 oth_src = es_src.eigenvectors();

  Mat3x3 media = diag_src * oth_src.transpose() * cov_tar * oth_src * diag_src;
  Eigen::SelfAdjointEigenSolver<Mat3x3> es_media(media);
  Mat3x3 diag_media = es_media.eigenvalues().asDiagonal();
  diag_media = diag_media.array().sqrt().matrix();
  Mat3x3 oth_media = es_media.eigenvectors();

  diag_src(0, 0) = 1 / (diag_src(0, 0) + 1e-3);
  diag_src(1, 1) = 1 / (diag_src(1, 1) + 1e-3);
  diag_src(2, 2) = 1 / (diag_src(2, 2) + 1e-3);

  Mat3x3 T = oth_src * diag_src * oth_media * diag_media *
             oth_media.transpose() * diag_src * oth_src.transpose();

  labs.clear();
  for (int i = 0; i < lab_src.size(); i++) {
    Vec3 tar = T * (lab_src[i] - mean_src) * factor + mean_tar;
    labs.emplace_back(tar);
  }
  // lab2rgb(labs, labs);
  has_adjusted = true;
}

void rgb2lab(Vec3List& rgbs, Vec3List& labs) {
  labs.resize(rgbs.size());

  for (int i = 0; i < rgbs.size(); i++) {
    Vec3 srgb;
    srgb(0) = rgbs[i](0) > 0.04045 ? pow((rgbs[i](0) + 0.055) / 1.055, 2.4)
                                   : rgbs[i](0) / 12.92;
    srgb(1) = rgbs[i](1) > 0.04045 ? pow((rgbs[i](1) + 0.055) / 1.055, 2.4)
                                   : rgbs[i](1) / 12.92;
    srgb(2) = rgbs[i](2) > 0.04045 ? pow((rgbs[i](2) + 0.055) / 1.055, 2.4)
                                   : rgbs[i](2) / 12.92;

    Vec3 xyz;
    xyz(0) = (0.412453f * srgb(0) + 0.357580f * srgb(1) +
              0.180423f * srgb(2));  // 0.95047f;
    xyz(1) = (0.212671f * srgb(0) + 0.715160f * srgb(1) + 0.072169f * srgb(2));
    xyz(2) = (0.019334f * srgb(0) + 0.119193f * srgb(1) +
              0.950227f * srgb(2));  // 1.08883f;

    float FX = xyz(0) > 0.008856f ? pow(xyz(0), 1.0f / 3.0f)
                                  : (7.787f * xyz(0) + 0.137931f);
    float FY = xyz(1) > 0.008856f ? pow(xyz(1), 1.0f / 3.0f)
                                  : (7.787f * xyz(1) + 0.137931f);
    float FZ = xyz(2) > 0.008856f ? pow(xyz(2), 1.0f / 3.0f)
                                  : (7.787f * xyz(2) + 0.137931f);

    labs[i](0) = xyz(1) > 0.008856f ? (116.0f * FY - 16.0f) : (903.3f * xyz(1));
    labs[i](1) = 500.f * (FX - FY);
    labs[i](2) = 200.f * (FY - FZ);
  }
}

void lab2rgb(Vec3List& labs, Vec3List& rgbs) {
  /*Mat3x3 XYZ2RGB;
  XYZ2RGB << 3.240479, -1.537150, -0.498535,
                          -0.969256, 1.875992, 0.041556,
                          0.055648, -0.204042, 1.057311;*/
  for (int i = 0; i < labs.size(); i++) {
    Vec3 lab = labs[i];
    float FY = (lab(0) + 16.0f) / 116.0f;
    float FX = lab(1) / 500.0f + FY;
    float FZ = FY - lab(2) / 200.0f;

    Vec3 xyz;
    xyz(0) = FX > 0.206893f ? pow(FX, 3) : (FX - 0.137931f) / 7.787f;
    xyz(1) = FY > 0.206893f ? pow(FY, 3) : (FY - 0.137931f) / 7.787f;
    xyz(2) = FZ > 0.206893f ? pow(FZ, 3) : (FZ - 0.137931f) / 7.787f;

    // xyz(0)*= 0.95047f;
    // xyz(1)*= 1.0f;
    // xyz(2)*= 1.08883f;

    rgbs[i](0) = 3.240479f * xyz(0) - 1.537150f * xyz(1) - 0.498535f * xyz(2);
    rgbs[i](1) = -0.969256f * xyz(0) + 1.875992f * xyz(1) + 0.041556f * xyz(2);
    rgbs[i](2) = 0.055648f * xyz(0) - 0.204043f * xyz(1) + 1.057311f * xyz(2);

    rgbs[i](0) = rgbs[i](0) > 0.0031308
                     ? 1.055 * (pow(rgbs[i](0), 1 / 2.4)) - 0.055
                     : rgbs[i](0) * 12.92;
    rgbs[i](1) = rgbs[i](1) > 0.0031308
                     ? 1.055 * (pow(rgbs[i](1), 1 / 2.4)) - 0.055
                     : rgbs[i](1) * 12.92;
    rgbs[i](2) = rgbs[i](2) > 0.0031308
                     ? 1.055 * (pow(rgbs[i](2), 1 / 2.4)) - 0.055
                     : rgbs[i](2) * 12.92;

    rgbs[i](0) = rgbs[i](0) < 0 ? 0 : (rgbs[i](0) > 1 ? 1 : rgbs[i](0));
    rgbs[i](1) = rgbs[i](1) < 0 ? 0 : (rgbs[i](1) > 1 ? 1 : rgbs[i](1));
    rgbs[i](2) = rgbs[i](2) < 0 ? 0 : (rgbs[i](2) > 1 ? 1 : rgbs[i](2));
  }
}

Vec3 computeMean(Vec3List& colors) {
  if (colors.empty()) return Vec3(0, 0, 0);
  Eigen::Matrix3Xf src =
      Eigen::Map<Eigen::MatrixXf>(colors[0].data(), 3, colors.size());
  Vec3 mean = src.rowwise().mean();
  return mean;
}

Vec3 computeVar(Vec3List& colors) {
  if (colors.empty()) return Vec3(0, 0, 0);
  Eigen::Matrix3Xf src =
      Eigen::Map<Eigen::MatrixXf>(colors[0].data(), 3, colors.size());
  Vec3 mean = src.rowwise().mean();
  src.colwise() -= mean;

  Mat3x3 cov = src * src.transpose() / (src.cols() - 1);
  Vec3 var = cov.diagonal();
  return var;
}

Mat3x3 computeCov(Vec3List& colors) {
  Eigen::Matrix3Xf src =
      Eigen::Map<Eigen::MatrixXf>(colors[0].data(), 3, colors.size());
  Vec3 mean = src.rowwise().mean();
  src.colwise() -= mean;

  Mat3x3 cov = src * src.transpose() / (src.cols() - 1);
  return cov;
}

void computeMeanAndCov(Vec3List& colors, Vec3& mean, Mat3x3& cov) {
  Eigen::Matrix3Xf src =
      Eigen::Map<Eigen::MatrixXf>(colors[0].data(), 3, colors.size());
  mean = src.rowwise().mean();
  src.colwise() -= mean;
  cov = src * src.transpose() / (src.cols() - 1);
}
}  // namespace chisel
