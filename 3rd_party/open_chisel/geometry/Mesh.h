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
// The above copyright noticec and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MESH_H_
#define MESH_H_

#include <memory>
#include <vector>
#include "geometry/Geometry.h"

#define GRID_EACH_DIM 8
#define GRID_RESERVED (8*8*8)

namespace chisel
{
    typedef std::size_t VertIndex;
    typedef std::vector<VertIndex> VertIndexList;
    class Patch;
	
    class Mesh
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Mesh();
            virtual ~Mesh();

            inline bool HasVertices() const { return !vertices.empty(); }
            inline bool HasNormals() const { return !normals.empty(); }
            inline bool HasColors() const { return !colors.empty(); }
            inline bool HasIndices() const { return !indices.empty(); }
			
            inline void Clear()
            {
                int vertexNum = indices.size();
                vertices.clear();
                normals.clear();
                colors.clear();
                indices.clear();

                vertices.reserve(vertexNum + 3);
                normals.reserve(vertexNum + 3);
                colors.reserve(vertexNum + 3);
				indices.reserve(vertexNum + 3);

				memset(adj, false, 6*sizeof(bool));
				simplified = false;
            }

			int GetIndice(Vec3 vert);
			void SimplifyByClustering(float resolution, Vec3 chunkOri = Vec3(0, 0, 0));

            Vec3List vertices;
            VertIndexList indices;
            Vec3List normals;
            Vec3List colors;

            Vec3 averageNormal;
            Point3 chunkID;
			bool simplified;
			bool adj[6];
			
            bool outlier_checked;
            
            Vec3 origin;
            float grid_resolution;
            
            std::shared_ptr<Patch> m_patch;
    };
    typedef std::shared_ptr<Mesh> MeshPtr;
    typedef std::shared_ptr<const Mesh> MeshConstPtr;

} // namespace chisel 

#endif // MESH_H_ 
