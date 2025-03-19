//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef CHUNKYTRIMESH_H
#define CHUNKYTRIMESH_H

struct rcChunkyTriMeshNode
{
	float bmin[2];
	float bmax[2];
	int i;
	int n;
};

struct rcChunkyTriMesh
{
	inline rcChunkyTriMesh() : nodes(0), nnodes(0), tris(0), ntris(0), maxTrisPerChunk(0) {}
	inline ~rcChunkyTriMesh() { delete [] nodes; delete [] tris; }

	rcChunkyTriMeshNode* nodes; // 节点数组
	int nnodes; // 节点的数量
	int* tris; // 三角形的索引数组
	int ntris; // 三角形的数量
	int maxTrisPerChunk; // 单个分块中最大的三角形数量

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcChunkyTriMesh(const rcChunkyTriMesh&);
	rcChunkyTriMesh& operator=(const rcChunkyTriMesh&);
};

/// Creates partitioned triangle mesh (AABB tree),
/// where each node contains at max trisPerChunk triangles.
/// 生成 AABB 树，每个节点包含最多 trisPerChunk 个三角形，它是一棵 2D 的树，提供平面上的快速查找
bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
						   int trisPerChunk, rcChunkyTriMesh* cm);

/// Returns the chunk indices which overlap the input rectable.
/// 返回与输入矩形重叠的分块索引
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, float bmin[2], float bmax[2], int* ids, const int maxIds);

/// Returns the chunk indices which overlap the input segment.
/// 返回与输入线段重叠的分块索引
int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm, float p[2], float q[2], int* ids, const int maxIds);


#endif // CHUNKYTRIMESH_H
