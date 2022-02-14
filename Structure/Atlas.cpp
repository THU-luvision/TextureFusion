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

#include "Atlas.h"
#include <iostream>
#include <list>

namespace chisel {
Atlas::Atlas(float resolution, ChunkManager* chunkManager) {
  SetResolution(resolution);
  texture_buffer.create(MAX_PATCH_HEIGHT, MAX_PATCH_WIDTH, CV_8UC3);
  memset(texture_buffer.data, 0,
         texture_buffer.datalimit - texture_buffer.data);
  loc_next = 0;
  hot_start = 0;
  hot_end = 0;
  manager = chunkManager;
}

PatchPtr Atlas::AddPatch(MeshPtr mesh) {
  if (mesh->m_patch == nullptr) {
    // if (patchit.boundingbox.width <= PATCH_WIDTH &&
    // patchit.boundingbox.height <= PATCH_HEIGHT)
    mesh->m_patch =
        std::allocate_shared<Patch>(Eigen::aligned_allocator<Patch>(), mesh);
    mesh->m_patch->texloc = loc_next;
    std::size_t x = loc_next % MAX_PATCH_WIDTH;
    std::size_t y = loc_next / MAX_PATCH_WIDTH;
    if (x >= MAX_PATCH_WIDTH || y >= MAX_PATCH_HEIGHT)
      throw std::overflow_error("No enough space for texture storage.");
    if (x + PATCH_WIDTH >= MAX_PATCH_WIDTH) {
      x = 0;
      y += PATCH_HEIGHT;
    } else
      x += PATCH_WIDTH;
    loc_next = x + y * MAX_PATCH_WIDTH;
  } else {
    mesh->m_patch->clear();
  }
  return mesh->m_patch;
}

Vec2 Atlas::GetTexLoc(ChunkID id) {
  std::size_t k = GetPatch(id)->texloc;
  return Vec2(k % MAX_PATCH_WIDTH, k / MAX_PATCH_WIDTH);
}

void Atlas::UpdateBuffer(ChunkID id) {
  if (!HasPatch(id)) return;
  PatchPtr patch = GetPatch(id);
  if (patch == nullptr || !patch->complete()) return;
  Vec2 origin = GetTexLoc(id);

  if (patch->image.cols > PATCH_WIDTH)
    patch->ratio(0) = float(PATCH_WIDTH) / patch->image.cols;
  if (patch->image.rows > PATCH_HEIGHT)
    patch->ratio(1) = float(PATCH_HEIGHT) / patch->image.rows;

  if (patch->ratio(0) < 1 || patch->ratio(1) < 1) {
    cv::Mat texroi =
        texture_buffer(Box(origin(0), origin(1), PATCH_WIDTH, PATCH_HEIGHT));
    cv::resize(patch->image, texroi, texroi.size());
  } else {
    cv::Mat texroi = texture_buffer(
        Box(origin(0), origin(1), patch->image.cols, patch->image.rows));
    patch->image.copyTo(texroi);
  }
}

void Atlas::SaveTexturedModel(std::string const& basepath) {
  // TODO: save result of adjusted color
  char fileName[2560];
  memset(fileName, 0, 2560);
  sprintf(fileName, "%s/texture_material.png", basepath.c_str());
  cv::Mat texture_bgr;
  cv::cvtColor(texture_buffer, texture_bgr, CV_RGB2BGR);
  cv::imwrite(fileName, texture_bgr);

  Vec3List vertices;
  Vec2List texcoords;
  Vec3List normals;
  VertIndexList indices;
  VertIndexList txindices;

  std::size_t vts = 0;
  for (auto it : manager->GetAllMeshes()) {
    MeshPtr mesh = it.second;
    PatchPtr patch = mesh->m_patch;
    if (patch == nullptr || !patch->complete()) continue;

    for (std::size_t j = 0; j < mesh->indices.size(); j++) {
      std::size_t k = mesh->indices[j] + vts;
      indices.emplace_back(k);
      txindices.emplace_back(k);
    }

    for (std::size_t j = 0; j < mesh->vertices.size(); j++) {
      vertices.emplace_back(mesh->vertices[j]);
      normals.emplace_back(mesh->normals[j]);

      Vec2 tex = GetTexLoc(mesh->chunkID);
      tex(0) += patch->texcoord[j](0) * patch->ratio(0);
      tex(1) += patch->texcoord[j](1) * patch->ratio(1);
      tex(0) /= MAX_PATCH_WIDTH;
      tex(1) /= MAX_PATCH_HEIGHT;
      texcoords.emplace_back(tex);
      vts++;
    }
  }

  std::ofstream mout((basepath + "/texture_model.obj").c_str());
  mout << "mtllib "
       << "texture_model.mtl" << '\n';
  mout << std::fixed << std::setprecision(6);
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    mout << "v " << vertices[i](0) << " " << vertices[i](1) << " "
         << vertices[i](2) << '\n';
  }

  for (std::size_t i = 0; i < texcoords.size(); ++i) {
    mout << "vt " << texcoords[i](0) << " " << 1.0f - texcoords[i](1) << '\n';
  }

  for (std::size_t i = 0; i < normals.size(); ++i) {
    mout << "vn " << normals[i](0) << " " << normals[i](1) << " "
         << normals[i](2) << '\n';
  }

  mout << "s off" << '\n';
  mout << "usemtl "
       << "demo_texture" << '\n';
  for (std::size_t i = 0; i < indices.size() / 3; ++i) {
    mout << "g "
         << "face" << i << '\n';
    mout << "f";
    for (std::size_t k = 0; k < 3; ++k) {
      mout << " " << indices[i * 3 + k] + 1 << "/" << txindices[i * 3 + k] + 1
           << "/" << indices[i * 3 + k] + 1;
    }
    mout << '\n';
  }
  mout.close();

  std::ofstream out((basepath + "/texture_model.mtl").c_str());
  out << "newmtl "
      << "demo_texture" << '\n'
      << "Ka 1.000000 1.000000 1.000000" << '\n'
      << "Kd 1.000000 1.000000 1.000000" << '\n'
      << "Ks 0.000000 0.000000 0.000000" << '\n'
      << "Tr 0.000000" << '\n'  // *Tr*ansparancy vs. *d*issolve: Tr = 1.0 - d
      << "illum 1" << '\n'
      << "Ns 1.000000" << '\n'
      << "map_Kd "
      << "texture_material.png" << std::endl;
  out.close();
}

/*void Atlas::DivideByLabel(std::vector<std::vector<std::size_t>>& graph)
{
        graph.clear();
        std::vector<bool> lookup(size(), false);
        for (std::size_t i = 0; i < size(); i++)
        {
                if (lookup[i]) continue;
                if (patchlist[i].has_adjusted||patchlist[i].texcolor.empty())
{lookup[i] = true; continue;} graph.emplace_back(std::vector<std::size_t>());
                
                std::list<std::size_t> sequence;
                sequence.emplace_back(i);
                while(!sequence.empty())
                {
                        MeshPtr mesh = patchlist[sequence.front()].mesh;
                        graph.back().emplace_back(sequence.front());
                        for (int j = 0;j < 6; j++)
                        {
                                if (!mesh -> adj[j]) continue;
                                chisel::ChunkID id = mesh->chunkID +
chisel::neighbourhood[j]; if (checker.find(id)==checker.end()) continue;
                                std::size_t k = checker.find(id)->second;
                                if (lookup[k]) continue;
                                if
(patchlist[k].has_adjusted||patchlist[k].texcolor.empty()) {lookup[k] = true;
continue;}
                                
                                MeshPtr meshadj = patchlist[k].mesh;
                                if (patchlist[sequence.front()].frameid !=
patchlist[k].frameid) continue; sequence.emplace_back(k); lookup[k] = true;
                        }
                        sequence.pop_front();
                }
        }
}
*/
}  // namespace chisel
