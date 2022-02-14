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
#ifndef ATLAS_H
#define ATLAS_H

#define MAX_PATCH_WIDTH (96 * 72 * 2)
#define MAX_PATCH_HEIGHT (72 * 96 * 2)
#define MAX_INDICE_NUM (1024 * 1024 * 20)

#include <exception>
#include <fstream>
#include <string>
#include <vector>
#include "ChunkManager.h"
#include "Patch.h"
#include "geometry/Geometry.h"


namespace chisel {
class Atlas {
 public:
  cv::Mat texture_buffer;

  std::size_t loc_next;
  std::size_t PATCH_WIDTH;
  std::size_t PATCH_HEIGHT;

  // area of update
  std::size_t hot_start;
  std::size_t hot_end;

  Atlas(float resolution, ChunkManager* chunkManager);

  // inline std::size_t size(){return manager..size();}
  inline bool HasPatch(ChunkID id) { return manager->HasMesh(id); }
  inline PatchPtr GetPatch(ChunkID id) {
    return manager->GetMutableMesh(id)->m_patch;
  }
  inline void SetResolution(float resolution) {
    PATCH_WIDTH = floor(4800 * resolution);
    PATCH_HEIGHT = floor(3600 * resolution);
  }

  PatchPtr AddPatch(MeshPtr mesh);
  Vec2 GetTexLoc(ChunkID id);
  void DivideByLabel(std::vector<std::vector<std::size_t>>& graph);
  void UpdateBuffer(ChunkID id);
  void SaveTexturedModel(std::string const& basepath);

 private:
  ChunkManager* manager;
};
}  // namespace chisel

#endif