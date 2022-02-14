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

#include "Chisel.h"

#include "io/PLY.h"

#include "geometry/Raycast.h"

#include <cmath>
#include <fstream>
#include <iostream>

namespace chisel {

Chisel::Chisel() : atlas(0.01f, &chunkManager) {
  // TODO Auto-generated constructor stub
}

Chisel::Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution,
               bool useColor)
    : chunkManager(chunkSize, voxelResolution, useColor),
      atlas(voxelResolution, &chunkManager) {}

Chisel::~Chisel() {
  // TODO Auto-generated destructor stub
}

void Chisel::Reset() {
  chunkManager.Reset();
  meshesToUpdate.clear();
}

void Chisel::bufferIntegratorSIMDCentroids(ProjectionIntegrator& integrator,
                                           const Transform& depthExtrinsic) {
  // set updated centroids
  Vec3 halfVoxel =
      Vec3(chunkManager.GetResolution(), chunkManager.GetResolution(),
           chunkManager.GetResolution()) *
      0.5f;
  Vec3List diff_centroids;
  diff_centroids.resize(static_cast<size_t>(chunkManager.GetChunkSize()(0) *
                                            chunkManager.GetChunkSize()(1) *
                                            chunkManager.GetChunkSize()(2)));
  int i = 0;
  for (int z = 0; z < chunkManager.GetChunkSize()(2); z++) {
    for (int y = 0; y < chunkManager.GetChunkSize()(1); y++) {
      for (int x = 0; x < chunkManager.GetChunkSize()(0); x++) {
        diff_centroids[i] = depthExtrinsic.linear().transpose() *
                                Vec3(x, y, z) * chunkManager.GetResolution() +
                            halfVoxel;
        i++;
      }
    }
  }

  integrator.diff_centroids = diff_centroids;

  int NumVoxels = chunkManager.GetChunkSize()(2) *
                  chunkManager.GetChunkSize()(0) *
                  chunkManager.GetChunkSize()(0);

  float data0[8], data1[8], data2[8];

  for (int z = 0; z < chunkManager.GetChunkSize()(2); z++) {
    for (int y = 0; y < chunkManager.GetChunkSize()(1); y++) {
      for (int x = 0; x < chunkManager.GetChunkSize()(0); x += 8) {
        int pos = z * chunkManager.GetChunkSize()(0) *
                      chunkManager.GetChunkSize()(1) +
                  y * chunkManager.GetChunkSize()(0) + x;
        // 8 * float3 vectors are converted to 3 * float8 vectors
        //(f00 f01 f02 f10 f11 f12) to (f00 f10 f20 f30 ...)
        for (int a = 0; a < 8; a++) {
          int local_pos = a;
          data0[local_pos] = integrator.diff_centroids[pos + local_pos](0);
          data1[local_pos] = integrator.diff_centroids[pos + local_pos](1);
          data2[local_pos] = integrator.diff_centroids[pos + local_pos](2);
        }

#if 1
        int centroids_pos_simd = z * chunkManager.GetChunkSize()(0) *
                                     chunkManager.GetChunkSize()(1) +
                                 y * chunkManager.GetChunkSize()(0) + x;
        centroids_pos_simd /= 8;
        integrator.centroids_simd0[centroids_pos_simd] = _mm256_loadu_ps(data0);
        integrator.centroids_simd1[centroids_pos_simd] = _mm256_loadu_ps(data1);
        integrator.centroids_simd2[centroids_pos_simd] = _mm256_loadu_ps(data2);
#endif
      }
    }
  }
}

void Chisel::CompressMeshes(ChunkSet& chunksToUpdate) {
  float gridResolution = chunkManager.GetResolution() * (8 / GRID_EACH_DIM);
  MeshMap& allMeshes = chunkManager.GetAllMutableMeshes();

  for (const std::pair<ChunkID, bool>& it : chunksToUpdate) {
    if (!it.second) continue;
    if (allMeshes.find(it.first) == allMeshes.end()) continue;
    MeshPtr meshit = allMeshes.find(it.first)->second;

    const ChunkPtr& chunkPtr = chunkManager.GetChunk(it.first);
    Vec3 chunkOri = chunkPtr->GetOrigin();
    if (meshit->vertices.size() != meshit->colors.size() ||
        meshit->vertices.size() != meshit->normals.size())
      std::cout << "mesh vertex error!" << std::endl;
    meshit->SimplifyByClustering(gridResolution, chunkOri);
  }

  for (const std::pair<ChunkID, bool>& it : chunksToUpdate) {
    if (!it.second) continue;
    if (allMeshes.find(it.first) == allMeshes.end()) continue;
    MeshPtr meshit = allMeshes.find(it.first)->second;

    for (int k = 0; k < 6; k++) {
      if (allMeshes.find(it.first + neighbourhood[k]) == allMeshes.end())
        continue;
      MeshPtr meshadj = allMeshes.find(it.first + neighbourhood[k])->second;
      if (!meshadj->simplified) continue;
      int m = ((k % 2 == 0) ? k + 1 : k - 1);
      if (meshit->adj[k] == true && meshadj->adj[m] == false)
        meshadj->adj[m] = true;
      if (meshit->adj[k] == false && meshadj->adj[m] == true)
        meshit->adj[k] = true;
    }
  }
  chunksToUpdate.clear();
}

int Chisel::GeneratePatches(ChunkIDList& chunksToUpdate, UniGraph& labelset,
                            std::vector<Frame>& frame_list,
                            PinholeCamera& cameraModel) {
  float gridResolution = chunkManager.GetResolution() * (8 / GRID_EACH_DIM);
  std::size_t loc_start = MAX_PATCH_HEIGHT * MAX_PATCH_WIDTH;
  std::size_t loc_end = 0;

  for (int i = 0; i < chunksToUpdate.size(); i++) {
    if (!chunkManager.HasMesh(chunksToUpdate[i])) continue;
    MeshPtr meshit = chunkManager.GetMutableMesh(chunksToUpdate[i]);
    int frameid =
        labelset.get_label(labelset.chunks.find(chunksToUpdate[i])->second);

    int frame_past = -1;
    if (atlas.HasPatch(chunksToUpdate[i]))
      frame_past = atlas.GetPatch(chunksToUpdate[i])->frameid;
    PatchPtr piece;
    // XXX sometimes memory fault here
    try {
      piece = atlas.AddPatch(meshit);
      int success = piece->CalculateTexCoords(frame_list[frameid], cameraModel);
    } catch (std::exception& e) {
      std::cout << "memory fault: " << e.what();
      return -1;
    }

    // if (success < 0 && frame_past > 0) std::cout << "consider prevoius
    // label!" << std::endl;
    piece->SetFrameid(frameid);
    piece->SetImage(frame_list[frameid].rgb);

    if (piece->texloc < loc_start) loc_start = piece->texloc;
    if (piece->texloc > loc_end) loc_end = piece->texloc;
  }

  atlas.hot_start = (loc_start / MAX_PATCH_WIDTH) * MAX_PATCH_WIDTH;
  atlas.hot_end =
      (loc_end / MAX_PATCH_WIDTH + atlas.PATCH_HEIGHT) * MAX_PATCH_WIDTH;

  return 0;
}

void Chisel::UpdateAtlas(ChunkIDList& chunksToUpdate) {
  for (int i = 0; i < chunksToUpdate.size(); i++) {
    if (atlas.HasPatch(chunksToUpdate[i]))
      atlas.UpdateBuffer(chunksToUpdate[i]);
  }
}

void Chisel::CompensateColor() {
  std::vector<std::vector<PatchPtr>> clusters;
  for (auto it : chunkManager.GetAllMeshes()) {
    PatchPtr patch = it.second->m_patch;
    if (patch == nullptr || patch->has_adjusted) continue;
    int k = 0;
    for (k = 0; k < clusters.size(); k++) {
      if (patch->frameid != clusters[k][0]->frameid) continue;
      clusters[k].emplace_back(patch);
      break;
    }
    if (k == clusters.size()) {
      clusters.resize(k + 1);
      clusters[k].emplace_back(patch);
    }
  }

  for (int i = 0; i < clusters.size(); i++) {
    Vec3List color_src;
    Vec3List color_tar;

    for (int j = 0; j < clusters[i].size(); j++) {
      Vec3List lab_src;
      Vec3List lab_tar;
      PatchPtr patch = clusters[i][j];
      // rgb2lab(patch->texcolor, lab_src);
      // rgb2lab(patch->mesh->colors, lab_tar);

      lab_src = patch->texcolor;
      lab_tar = patch->mesh->colors;
      patch->labs = lab_src;
      patch->labt = lab_tar;

      if (patch->wrong_mapping) {
        patch->labs.clear();
        continue;
      }

      color_src.insert(color_src.end(), lab_src.begin(), lab_src.end());
      color_tar.insert(color_tar.end(), lab_tar.begin(), lab_tar.end());
    }

    if (color_src.empty() || color_tar.empty()) continue;

    Vec3 mean_src, mean_tar;
    Mat3x3 cov_src, cov_tar;
    computeMeanAndCov(color_src, mean_src, cov_src);
    computeMeanAndCov(color_tar, mean_tar, cov_tar);

    Vec3List().swap(color_src);
    Vec3List().swap(color_tar);

    Eigen::SelfAdjointEigenSolver<Mat3x3> es_src(cov_src);
    Mat3x3 diag_src = es_src.eigenvalues().cwiseSqrt().asDiagonal();
    Mat3x3 oth_src = es_src.eigenvectors();

    Mat3x3 media;
    media.noalias() =
        diag_src * oth_src.transpose() * cov_tar * oth_src * diag_src;

    Eigen::SelfAdjointEigenSolver<Mat3x3> es_media(media);
    Mat3x3 diag_media = es_media.eigenvalues().cwiseSqrt().asDiagonal();
    Mat3x3 oth_media = es_media.eigenvectors();

    diag_src(0, 0) = 1 / (diag_src(0, 0) + 1e-2);
    diag_src(1, 1) = 1 / (diag_src(1, 1) + 1e-2);
    diag_src(2, 2) = 1 / (diag_src(2, 2) + 1e-2);

    Mat3x3 T;
    T.noalias() = oth_src * diag_src * oth_media * diag_media *
                  oth_media.transpose() * diag_src * oth_src.transpose();

    for (int j = 0; j < clusters[i].size(); j++) {
      PatchPtr patch = clusters[i][j];
      Vec3List& lab_src = patch->labs;
      Vec3List& lab_tar = patch->labt;

      for (int k = 0; k < lab_src.size(); k++) {
        patch->labs[k] = T * (lab_src[k] - mean_src) + mean_tar;
      }

      if (patch->wrong_mapping) {
        patch->labs.clear();
      }
      // lab2rgb(patch -> labs, patch -> labs);
      patch->has_adjusted = true;
    }
  }
}

void Chisel::DrawMeshes(float* vertices, unsigned int* indices,
                        unsigned int& tsdf_indice_num,
                        unsigned int& tsdf_vertice_num) {
  unsigned int vert_num = 0;
  unsigned int index_num = 0;
  // bool forbid_skip = false;

  for (auto it : chunkManager.GetAllMeshes()) {
    MeshPtr mesh = it.second;
    PatchPtr patch = mesh->m_patch;
    if (patch == nullptr || !patch->complete()) continue;

    for (std::size_t j = 0; j < mesh->indices.size(); j++) {
      unsigned int* cur_index = &indices[index_num];
      cur_index[0] = mesh->indices[j] + vert_num;
      index_num++;
      // TODO check indice;
    }

    for (std::size_t j = 0; j < mesh->vertices.size(); j++) {
      float* cur_vert = &vertices[12 * vert_num];

      Vec3 vert = mesh->vertices[j];
      Vec3 color = mesh->colors[j];
      Vec3 normal = mesh->normals[j];

      Vec2 texcoord = patch->texcoord[j];
      Vec2 ratio = patch->ratio;
      if (ratio(0) < 1) texcoord(0) *= ratio(0);
      if (ratio(1) < 1) texcoord(1) *= ratio(1);
      texcoord += atlas.GetTexLoc(mesh->chunkID);

      cur_vert[0] = vert(0);
      cur_vert[1] = vert(1);
      cur_vert[2] = vert(2);
      cur_vert[3] = 50;

      int rgb_value = int(color(0) * 255);
      rgb_value = (rgb_value << 8) + int(color(1) * 255);
      rgb_value = (rgb_value << 8) + int(color(2) * 255);
      cur_vert[4] = rgb_value;

      Vec3 color_ad = Vec3(0, 0, 0);
      if (patch->has_adjusted && !patch->labs.empty()) {
        color_ad = patch->labs[j] - patch->texcolor[j];
        int ad_color = int(color_ad(0) * 255) + 255;
        ad_color = (ad_color << 9) + int(color_ad(1) * 255) + 255;
        ad_color = (ad_color << 9) + int(color_ad(2) * 255) + 255;
        cur_vert[5] = ad_color;
      } else
        cur_vert[5] = 0;

      cur_vert[6] = texcoord(0) / MAX_PATCH_WIDTH;
      cur_vert[7] = texcoord(1) / MAX_PATCH_HEIGHT;
      cur_vert[8] = normal(0);
      cur_vert[9] = normal(1);
      cur_vert[10] = normal(2);
      cur_vert[11] = 0.0f;
      if (patch->wrong_mapping) cur_vert[11] = 1.0f;

      vert_num++;
    }
    patch->has_updated = true;
  }
  tsdf_indice_num = index_num;
  tsdf_vertice_num = vert_num;
  // std::cout << tsdf_vertice_num << "/" << tsdf_indice_num << std::endl;
}

bool Chisel::SaveAllMeshesToPLY(const std::string& filename) {
  printf("Saving all meshes to PLY file...\n");

  chisel::MeshPtr fullMesh(new chisel::Mesh());

  size_t v = 0;
  for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes()) {
    for (int i = 0; i < it.second->indices.size(); i++) {
      size_t index = it.second->indices[i];
      fullMesh->indices.emplace_back(i + v);
      fullMesh->vertices.emplace_back(it.second->vertices[index]);
      fullMesh->colors.emplace_back(it.second->colors[index]);
      fullMesh->normals.emplace_back(it.second->normals[index]);
    }
    v += it.second->indices.size();
  }

  printf("Full mesh has %lu verts\n", v);
  bool success = SaveMeshPLYASCII(filename, fullMesh);
  if (!success) printf("Saving failed!\n");

  return success;
}

bool Chisel::SaveTSDFFiles(const std::string& fileName) {
  printf("Saving tsdf to PLY file...\n");
  chisel::MeshPtr fullMesh(new chisel::Mesh());
  Vec3List V;
  Point3List C;
  Vec3List N;
  size_t v = 0;

  int X = chunkManager.GetChunkSize()(0);
  int Y = chunkManager.GetChunkSize()(1);
  int Z = chunkManager.GetChunkSize()(2);
  const ChunkMap& chunks = chunkManager.GetChunks();

  Vec3 halfVoxel =
      Vec3(chunkManager.GetResolution(), chunkManager.GetResolution(),
           chunkManager.GetResolution()) *
      0.5f;
  for (const std::pair<ChunkID, ChunkPtr>& chunk : chunks) {
    ChunkPtr cPtr = chunk.second;
    const DistVoxel& voxels = cPtr->voxels;
    Vec3 ori = cPtr->GetOrigin();
    for (int z = 0; z < chunkManager.GetChunkSize()(2); z++) {
      for (int y = 0; y < chunkManager.GetChunkSize()(1); y++) {
        for (int x = 0; x < chunkManager.GetChunkSize()(0); x++) {
          Vec3 pos =
              ori + Vec3(x, y, z) * chunkManager.GetResolution() + halfVoxel;
          int voxelIndex = x + y * X + z * X * Y;

          Point3 color = Point3(cPtr->colors.GetBlue(voxelIndex) * 255,
                                cPtr->colors.GetGreen(voxelIndex) * 255,
                                cPtr->colors.GetRed(voxelIndex) * 255);

          Vec3 normal =
              Vec3((voxels.sdf[voxelIndex] + 0.2) * 30,
                   voxels.weight[voxelIndex], voxels.sdf[voxelIndex] * 100);

          V.emplace_back(pos);
          C.emplace_back(color);
          N.emplace_back(normal);
        }
      }
    }
  }

  std::ofstream output_file(fileName.c_str(), std::ios::out | std::ios::trunc);
  int pointNum = fmin(V.size(), C.size());
  output_file << "ply" << std::endl;
  output_file
      << "format ascii 1.0           { ascii/binary, format version number }"
      << std::endl;
  output_file << "comment made by Greg Turk  { comments keyword specified, "
                 "like all lines }"
              << std::endl;
  output_file << "comment this file is a cube" << std::endl;
  output_file << "element vertex " << pointNum
              << "           { define \"vertex\" element, 8 of them in file }"
              << std::endl;
  output_file << "property float x" << std::endl;
  output_file << "property float y" << std::endl;
  output_file << "property float z" << std::endl;
  output_file << "property float nx" << std::endl;
  output_file << "property float ny" << std::endl;
  output_file << "property float nz" << std::endl;
  output_file << "property float intensity" << std::endl;
  output_file << "property uchar red" << std::endl;
  output_file << "property uchar green" << std::endl;
  output_file << "property uchar blue" << std::endl;

  output_file << "end_header" << std::endl;
  for (int i = 0; i < V.size(); i++) {
    output_file << V[i](0) << " " << V[i](1) << " " << V[i](2) << " " << N[i](0)
                << " " << N[i](1) << " " << N[i](2) << " " << 1 << " "
                << C[i](0) << " " << C[i](1) << " " << C[i](2) << " "
                << std::endl;
  }
  output_file.close();
  printf("Full tsdf has %lu verts\n", V.size());
}

}  // namespace chisel
