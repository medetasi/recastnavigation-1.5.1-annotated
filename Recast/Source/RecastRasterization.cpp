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

#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const float* aMin, const float* aMax, const float* bMin, const float* bMax)
{
	return
		aMin[0] <= bMax[0] && aMax[0] >= bMin[0] &&
		aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&
		aMin[2] <= bMax[2] && aMax[2] >= bMin[2];
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 从 freelist 中获取一个 span，如果 freelist 为空，则创建一个新的 span 池，并更新 freelist
/// 
/// @param[in]	hf		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocSpan(rcHeightfield& hf)
{
	// If necessary, allocate new page and update the freelist.
	if (hf.freelist == NULL || hf.freelist->next == NULL)
	{
		// Create new page.
		// Allocate memory for the new pool.
		// 创建一个新的 span 池，它内部有 RC_SPANS_PER_POOL(2048) 个 span
		rcSpanPool* spanPool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (spanPool == NULL)
		{
			return NULL;
		}

		// Add the pool into the list of pools. 加在了链表的头部
		spanPool->next = hf.pools;
		hf.pools = spanPool;
		
		// Add new spans to the free list.
		// 将创建的 span 添加到 freelist 中
		rcSpan* freeList = hf.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		// 从后向前遍历 spanPool->items, 这样的好处是不用每次修改处理表头，直接把最后一项的 next 改掉就行了
		do
		{
			--it;
			it->next = freeList; // 将 it 加入到 freeList 的最前面
			freeList = it;
		}
		while (it != head);
		hf.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = hf.freelist;
	hf.freelist = hf.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	hf		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& hf, rcSpan* span)
{
	if (span == NULL)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = hf.freelist;
	hf.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	hf					Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaID				The new span's area type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge area type IDs
static bool addSpan(rcHeightfield& hf,
                    const int x, const int z,
                    const unsigned short min, const unsigned short max,
                    const unsigned char areaID, const int flagMergeThreshold)
{
	// Create the new span.
	rcSpan* newSpan = allocSpan(hf);
	if (newSpan == NULL)
	{
		return false;
	}
	newSpan->smin = min;
	newSpan->smax = max;
	newSpan->area = areaID;
	newSpan->next = NULL;
	
	const int columnIndex = x + z * hf.width; // 计算在 spans 数组中的索引，是放在一个数组中的
	rcSpan* previousSpan = NULL; // 尝试寻找的链表前置节点
	rcSpan* currentSpan = hf.spans[columnIndex]; // 获取链表中的第一个元素, span 本身是一个链表，保存了本格子上的所有 span
	
	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != NULL)
	{
		if (currentSpan->smin > newSpan->smax)
		{
			// Current span is completely after the new span, break.
			// 如果当前 span 的最小值大于 newSpan 的最大值，说明当前 span 在 newSpan 的后面，直接跳出循环
			break;
		}
		
		if (currentSpan->smax < newSpan->smin)
		{
			// Current span is completely before the new span.  Keep going.
			// 如果当前 span 的最大值小于 newSpan 的最小值，说明当前 span 在 newSpan 的前面，继续遍历下一个 span
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			// 如果当前 span 的最小值小于 newSpan 的最小值，说明当前 span 与 newSpan 有重叠，需要合并
			if (currentSpan->smin < newSpan->smin)
			{
				newSpan->smin = currentSpan->smin;
			}
			// 如果当前 span 的最大值大于 newSpan 的最大值，说明当前 span 与 newSpan 有重叠，需要合并
			if (currentSpan->smax > newSpan->smax)
			{
				newSpan->smax = currentSpan->smax;
			}
			
			// Merge flags.
			// 根据阈值合并 areaID
			if (rcAbs((int)newSpan->smax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				newSpan->area = rcMax(newSpan->area, currentSpan->area);
			}
			
			// Remove the current span since it's now merged with newSpan.
			// Keep going because there might be other overlapping spans that also need to be merged.
			// 移除 currentSpan，因为现在它已经被合并到 newSpan 中，继续遍历下一个 span
			rcSpan* next = currentSpan->next;
			freeSpan(hf, currentSpan);
			if (previousSpan)
			{
				previousSpan->next = next;
			}
			else
			{
				hf.spans[columnIndex] = next;
			}
			currentSpan = next;
		}
	}
	
	// Insert new span after prev
	// 如果有 previousSpan, 则需要将 newSpan 插入到 previousSpan 的后面
	if (previousSpan != NULL)
	{
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// This span should go before the others in the list
		// 如果 previousSpan 为 NULL, 则需要将 newSpan 插入到链表的头部
		newSpan->next = hf.spans[columnIndex];
		hf.spans[columnIndex] = newSpan;
	}

	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const unsigned short spanMin, const unsigned short spanMax,
               const unsigned char areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

enum rcAxis
{
	RC_AXIS_X = 0,
	RC_AXIS_Y = 1,
	RC_AXIS_Z = 2
};

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
/// 
/// @param[in]	inVerts			The input polygon vertices
/// @param[in]	inVertsCount	The number of input polygon vertices
/// @param[out]	outVerts1		Resulting polygon 1's vertices
/// @param[out]	outVerts1Count	The number of resulting polygon 1 vertices
/// @param[out]	outVerts2		Resulting polygon 2's vertices
/// @param[out]	outVerts2Count	The number of resulting polygon 2 vertices
/// @param[in]	axisOffset		THe offset along the specified axis
/// @param[in]	axis			The separating axis
static void dividePoly(const float* inVerts, int inVertsCount,
                       float* outVerts1, int* outVerts1Count,
                       float* outVerts2, int* outVerts2Count,
                       float axisOffset, rcAxis axis)
{
	rcAssert(inVertsCount <= 12); // 三角形被切割产生的多边形，不会超过 12 个顶点
	
	// How far positive or negative away from the separating axis is each vertex.
	// 计算每个顶点相对于分离轴的距离，正值表示在分离轴的正方向，负值表示在分离轴的反方向
	float inVertAxisDelta[12];
	for (int inVert = 0; inVert < inVertsCount; ++inVert)
	{
		inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis]; // axis 的值为 0, 1, 2，分别代表 x, y, z 轴
	}

	int poly1Vert = 0; // 保存多边形 1 的顶点数
	int poly2Vert = 0; // 保存多边形 2 的顶点数
	
	// 使用两个指针遍历多边形的边，inVertA 和 inVertB 分别指向边的起点和终点
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		// If the two vertices are on the same side of the separating axis
		// 判断两个点是否在分离轴的同一侧
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

		if (!sameSide)
		{
			// 如果不在同一侧，说明该边与分离轴相交
			// 使用线性插值计算边与分离轴的交点
			float s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
			outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			rcVcopy(&outVerts2[poly2Vert * 3], &outVerts1[poly1Vert * 3]); // 同时将交点复制到 outVerts2 中
			poly1Vert++;
			poly2Vert++;
			
			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			// 如果 inVertA 在分离轴的正方向，则将 inVertA 添加到多边形 1 中，否则添加到多边形 2 中
			if (inVertAxisDelta[inVertA] > 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
			}
			else if (inVertAxisDelta[inVertA] < 0)
			{
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				poly2Vert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			// 如果 inVertA 在分离轴的正方向或在分离轴上，则将 inVertA 添加到多边形 1 中
			if (inVertAxisDelta[inVertA] >= 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0)
				{
					// 如果 inVertA 不在分离轴上，则跳过该点
					continue;
				}
			}
			rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]); // 如果 inVertA 在分离轴反方向或者在分离轴上，则将 inVertA 添加到多边形 2 中
			poly2Vert++;
		}
	}

	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// 将 v0, v1, v2 三个顶点组成的三角形，投影到高度场中，并生成对应的 span
///
/// @param[in] 	v0					Triangle vertex 0
/// @param[in] 	v1					Triangle vertex 1
/// @param[in] 	v2					Triangle vertex 2
/// @param[in] 	areaID				The area ID to assign to the rasterized spans
/// @param[in] 	hf					Heightfield to rasterize into
/// @param[in] 	hfBBMin				The min extents of the heightfield bounding box
/// @param[in] 	hfBBMax				The max extents of the heightfield bounding box
/// @param[in] 	cellSize			The x and z axis size of a voxel in the heightfield
/// @param[in] 	inverseCellSize		1 / cellSize
/// @param[in] 	inverseCellHeight	1 / cellHeight
/// @param[in] 	flagMergeThreshold	The threshold in which area flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& hf,
                         const float* hfBBMin, const float* hfBBMax,
                         const float cellSize, const float inverseCellSize, const float inverseCellHeight,
                         const int flagMergeThreshold)
{
	// Calculate the bounding box of the triangle.
	// 计算三角形的包围盒
	float triBBMin[3];
	rcVcopy(triBBMin, v0);
	rcVmin(triBBMin, v1);
	rcVmin(triBBMin, v2);

	float triBBMax[3];
	rcVcopy(triBBMax, v0);
	rcVmax(triBBMax, v1);
	rcVmax(triBBMax, v2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	// 如果三角形的包围盒不与高度场的包围盒重叠，说明三角形不在高度场的范围内，直接返回
	if (!overlapBounds(triBBMin, triBBMax, hfBBMin, hfBBMax))
	{
		return true;
	}

	const int w = hf.width; // x 轴长度
	const int h = hf.height; // z 轴长度
	const float by = hfBBMax[1] - hfBBMin[1]; // y 轴长度

	// Calculate the footprint of the triangle on the grid's z-axis 
	// 计算三角形在 z 轴上的投影
	int z0 = (int)((triBBMin[2] - hfBBMin[2]) * inverseCellSize); // 三角形的最小值在 z 轴上的位置，单位是格子，结果被截断为整数
	int z1 = (int)((triBBMax[2] - hfBBMin[2]) * inverseCellSize); // 三角形的最大值在 z 轴上的位置，单位是格子，结果被截断为整数

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	// Clamp 函数将第一个参数限制在第二个和第三个参数之间
	z0 = rcClamp(z0, -1, h - 1);
	z1 = rcClamp(z1, 0, h - 1);

	// Clip the triangle into all grid cells it touches.
	// 一个重要的点是，一个三角形被平面切割后，最多产生 7 个顶点
	// 此处一共分配了 84 个 float 空间，前面的 7 * 3 个代表的是  21 个顶点
	// 21 * 4 是因为一共使用了四个数组来保存不同含义的顶点
	float buf[7 * 3 * 4];
	float* in = buf;           // 存储当前处理的多边形
	float* inRow = buf + 7*3;  // 存储当前行的切割结果
	float* p1 = inRow + 7*3;   // 临时存储区1
	float* p2 = p1 + 7*3;      // 临时存储区2

	// 将三角形的三个顶点复制到 in 数组中
	rcVcopy(&in[0], v0);
	rcVcopy(&in[1 * 3], v1);
	rcVcopy(&in[2 * 3], v2);
	int nvRow;
	int nvIn = 3; // 三角形的顶点数

	// 按 z 轴方向执行行扫描
	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cellZ = hfBBMin[2] + (float)z * cellSize; // 计算当前遍历到的 z 轴位置
		dividePoly(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ + cellSize, RC_AXIS_Z); // 将三角形切割成两部分，并保存到 inRow 和 p1 中
		rcSwap(in, p1); // 交换 in 和 p1 的指针，以便下一次循环使用
		
		if (nvRow < 3)
		{
			// 如果 nvRow 小于 3，说明三角形在当前行上没有投影，直接跳过，否则只要有一个点在当前行上，就会至少被切割成一个三角形，也就是 nvRow 至少为 3
			continue;
		}
		if (z < 0)
		{
			continue;
		}
		
		// find X-axis bounds of the row
		// 计算三角形在当前行上的 x 轴范围，inRow 中保存的是三角形在当前行上的投影
		float minX = inRow[0];
		float maxX = inRow[0];
		for (int vert = 1; vert < nvRow; ++vert)
		{
			if (minX > inRow[vert * 3])
			{
				minX = inRow[vert * 3];
			}
			if (maxX < inRow[vert * 3])
			{
				maxX = inRow[vert * 3];
			}
		}
		int x0 = (int)((minX - hfBBMin[0]) * inverseCellSize); // 计算当前列的 x 轴最小值
		int x1 = (int)((maxX - hfBBMin[0]) * inverseCellSize); // 计算当前列的 x 轴最大值
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}
		x0 = rcClamp(x0, -1, w - 1); // 将 x0 限制在 -1 和 w - 1 之间
		x1 = rcClamp(x1, 0, w - 1); // 将 x1 限制在 0 和 w - 1 之间

		int nv;
		int nv2 = nvRow;

		// 按 x 轴方向执行列扫描
		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = hfBBMin[0] + (float)x * cellSize; // 计算当前列的 x 轴位置
			dividePoly(inRow, nv2, p1, &nv, p2, &nv2, cx + cellSize, RC_AXIS_X); // 将多边形切割成两部分，并保存到 p1 和 p2 中
			rcSwap(inRow, p2); // 交换 inRow 和 p2 的指针，以便下一次循环使用
			
			if (nv < 3)
			{
				continue;
			}
			if (x < 0)
			{
				continue;
			}
			
			// Calculate min and max of the span.
			// 计算在 z 行 x 列的格子内的多边形的 y 轴最小值和最大值
			float spanMin = p1[1];
			float spanMax = p1[1];
			for (int vert = 1; vert < nv; ++vert)
			{
				spanMin = rcMin(spanMin, p1[vert * 3 + 1]);
				spanMax = rcMax(spanMax, p1[vert * 3 + 1]);
			}
			spanMin -= hfBBMin[1];
			spanMax -= hfBBMin[1];
			
			// Skip the span if it's completely outside the heightfield bounding box
			if (spanMax < 0.0f)
			{
				// 如果 spanMax 小于 0，说明多边形在当前行上没有投影，直接跳过
				continue;
			}
			if (spanMin > by)
			{
				// 如果 spanMin 大于 by，说明多边形在当前行上没有投影，直接跳过
				continue;
			}
			
			// Clamp the span to the heightfield bounding box.
			// 起到修正的作用
			if (spanMin < 0.0f)
			{
				// 如果 spanMin 小于 0，则将 spanMin 设置为 0
				spanMin = 0;
			}
			if (spanMax > by)
			{
				// 如果 spanMax 大于 by，则将 spanMax 设置为 by
				spanMax = by;
			}

			// Snap the span to the heightfield height grid.
			// 计算 spanMin 和 spanMax 在高度场中的格子索引
			unsigned short spanMinCellIndex = (unsigned short)rcClamp((int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short spanMaxCellIndex = (unsigned short)rcClamp((int)ceilf(spanMax * inverseCellHeight), (int)spanMinCellIndex + 1, RC_SPAN_MAX_HEIGHT);

			// 将 x z 对应的竖列，从 spanMinCellIndex 到 spanMaxCellIndex 的格子设置为 areaID, 应该只有 RC_WALKABLE_AREA 这一类 areaID
			if (!addSpan(hf, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold))
			{
				return false;
			}
		}
	}

	return true;
}

bool rcRasterizeTriangle(rcContext* context,
                         const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	if (!rasterizeTri(v0, v1, v2, areaID, heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const int* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES); // 用来统计用时，在析构时停止计时
	
	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const unsigned short* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[(triIndex * 3 + 0) * 3];
		const float* v1 = &verts[(triIndex * 3 + 1) * 3];
		const float* v2 = &verts[(triIndex * 3 + 2) * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}
