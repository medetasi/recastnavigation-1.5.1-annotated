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

#include <float.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

namespace
{
struct LevelStackEntry
{
	LevelStackEntry(int x_, int y_, int index_) : x(x_), y(y_), index(index_) {}
	int x;
	int y;
	int index;
};
}  // namespace

static void calculateDistanceField(rcCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
{
	const int w = chf.width;
	const int h = chf.height;
	
	// Init distance and points.
	for (int i = 0; i < chf.spanCount; ++i)
		src[i] = 0xffff;
	
	// Mark boundary cells.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];
				
				int nc = 0;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						if (area == chf.areas[ai])
							nc++;
					}
				}
				if (nc != 4)
					src[i] = 0; // 如果 span 的轴向邻居不足 4 个，那么它一定是 border span
			}
		}
	}
	
			
	// Pass 1
	// 从左上角开始，向右下角遍历，从边缘开始向内计算距离，边缘的 span 都是 border span
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED) // 如果左邻居存在
				{
					// (-1,0) 左邻居
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2; // 轴向相邻的距离 + 2
					
					// (-1,-1) 左上邻居
					if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 3);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3; // 对角相邻的距离 + 3
					}
				}
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED) // 如果上邻居存在
				{
					// (0,-1) 上邻居
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2; // 轴向相邻的距离 + 2
					
					// (1,-1) 右上邻居
					if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 2);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3; // 对角相邻的距离 + 3
					}
				}
			}
		}
	}
	
	// Pass 2
	for (int y = h-1; y >= 0; --y)
	{
		for (int x = w-1; x >= 0; --x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 2);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (1,1)
					if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 1);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 1);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (-1,1)
					if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 0);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}	
	
	// 计算最大距离
	maxDist = 0;
	for (int i = 0; i < chf.spanCount; ++i)
		maxDist = rcMax(src[i], maxDist);
	
}

// 盒式模糊，用来处理距离场，使其更加平滑
// 核心思路是用一个点和周围所有邻居的平均距离值来代替当前点的距离值
static unsigned short* boxBlur(rcCompactHeightfield& chf, int thr,
							   unsigned short* src, unsigned short* dst)
{
	const int w = chf.width;
	const int h = chf.height;
	
	thr *= 2;
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned short cd = src[i]; // 当前点的距离值

				// 1. 如果距离小于阈值，直接保持原值
				if (cd <= thr)
				{
					dst[i] = cd;
					continue;
				}

				// 2. 开始计算模糊值
				int d = (int)cd; // 累加器，初始值为当前点的距离

				// 3. 遍历四个主要方向（上、右、下、左）
				for (int dir = 0; dir < 4; ++dir)
				{
					// 4. 检查该方向是否有相邻点
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						// 5. 计算相邻点的坐标和索引
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);

						// 6. 加入直接相邻点的值
						d += (int)src[ai];
						
						const rcCompactSpan& as = chf.spans[ai];

						// 7. 计算对角线方向
						const int dir2 = (dir+1) & 0x3; // 下一个方向（顺时针旋转）

						// 8. 检查对角线方向是否有点
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							// 9. 计算对角线点的坐标和索引
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);

							// 10. 加入对角线点的值
							d += (int)src[ai2];
						}
						else
						{
							// 11. 如果对角线方向没有点，使用中心点的值代替
							d += cd;
						}
					}
					else
					{
						// 12. 如果主方向没有点，使用中心点的值代替（权重为2）
						d += cd*2;
					}
				}

				// 13. 计算最终的模糊值：(d+5)/9 是为了四舍五入
				dst[i] = (unsigned short)((d+5)/9);
			}
		}
	}
	return dst;
}


static bool floodRegion(int x, int y, int i,
						unsigned short level, unsigned short r,
						rcCompactHeightfield& chf,
						unsigned short* srcReg, unsigned short* srcDist,
						rcTempVector<LevelStackEntry>& stack)
{
	const int w = chf.width;
	
	const unsigned char area = chf.areas[i];
	
	// Flood fill mark region.
	stack.clear();
	stack.push_back(LevelStackEntry(x, y, i));
	srcReg[i] = r;
	srcDist[i] = 0;
	
	unsigned short lev = level >= 2 ? level-2 : 0;
	int count = 0;
	
	while (stack.size() > 0)
	{
		LevelStackEntry& back = stack.back();
		int cx = back.x;
		int cy = back.y;
		int ci = back.index;
		stack.pop_back();
		
		const rcCompactSpan& cs = chf.spans[ci];
		
		// Check if any of the neighbours already have a valid region set.
		unsigned short ar = 0;
		for (int dir = 0; dir < 4; ++dir)
		{
			// 8 connected
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;
				unsigned short nr = srcReg[ai];
				if (nr & RC_BORDER_REG) // Do not take borders into account.
					continue;
				if (nr != 0 && nr != r)
				{
					ar = nr;
					break;
				}
				
				const rcCompactSpan& as = chf.spans[ai];
				
				const int dir2 = (dir+1) & 0x3;
				if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
				{
					const int ax2 = ax + rcGetDirOffsetX(dir2);
					const int ay2 = ay + rcGetDirOffsetY(dir2);
					const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
					if (chf.areas[ai2] != area)
						continue;
					unsigned short nr2 = srcReg[ai2];
					if (nr2 != 0 && nr2 != r)
					{
						ar = nr2;
						break;
					}
				}				
			}
		}
		if (ar != 0)
		{
			srcReg[ci] = 0;
			continue;
		}
		
		count++;
		
		// Expand neighbours.
		for (int dir = 0; dir < 4; ++dir)
		{
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;
				if (chf.dist[ai] >= lev && srcReg[ai] == 0)
				{
					srcReg[ai] = r;
					srcDist[ai] = 0;
					stack.push_back(LevelStackEntry(ax, ay, ai));
				}
			}
		}
	}
	
	return count > 0;
}

// Struct to keep track of entries in the region table that have been changed.
struct DirtyEntry
{
	DirtyEntry(int index_, unsigned short region_, unsigned short distance2_)
		: index(index_), region(region_), distance2(distance2_) {}
	int index;
	unsigned short region;
	unsigned short distance2;
};

static void expandRegions(int maxIter, // 最大迭代次数
					      unsigned short level, // 当前层级
					      rcCompactHeightfield& chf, // 压缩高度场
					      unsigned short* srcReg, // 区域 id 数组
						  unsigned short* srcDist, // 距离场数组
					      rcTempVector<LevelStackEntry>& stack, // 堆栈
					      bool fillStack) // 是否填充堆栈
{
	const int w = chf.width; // 压缩高度场的宽度
	const int h = chf.height; // 压缩高度场的高度

	if (fillStack)
	{
		// Find cells revealed by the raised level.
		// 遍历整个压缩高度场，将所有距离场值大于等于当前层级且未分配区域的单元格添加到堆栈中
		stack.clear();
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x+y*w];
				for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
				{
					if (chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA)
					{
						stack.push_back(LevelStackEntry(x, y, i));
					}
				}
			}
		}
	}
	else // use cells in the input stack 如果 fillStack 为 false，则使用输入的堆栈
	{
		// mark all cells which already have a region 标记已经拥有区域的单元格
		for (int j=0; j<stack.size(); j++)
		{
			int i = stack[j].index;
			if (srcReg[i] != 0) // 如果单元格已经拥有区域，则标记为已使用
				stack[j].index = -1;
		}
	}

	rcTempVector<DirtyEntry> dirtyEntries; // 用来临时存储找到的区域信息
	int iter = 0;
	while (stack.size() > 0) // 如果堆栈不为空就一直循环，直到内部跳出
	{
		int failed = 0;
		dirtyEntries.clear();
		
		// 处理堆栈中的每个单元格
		for (int j = 0; j < stack.size(); j++)
		{
			int x = stack[j].x;
			int y = stack[j].y;
			int i = stack[j].index;
			if (i < 0)
			{
				failed++; // 如果单元格已经被标记过，累加 failed 计数器
				continue; // 如果单元格已经被标记为已使用，则跳过
			}
			
			unsigned short r = srcReg[i]; // 获取目标单元格的区域 ID, 此时 r 为 0
			unsigned short d2 = 0xffff; // 设置距离标志为最大值，后续会更新为最小距离
			const unsigned char area = chf.areas[i];
			const rcCompactSpan& s = chf.spans[i];

			// 检查四个方向的邻居
			for (int dir = 0; dir < 4; ++dir)
			{
				if (rcGetCon(s, dir) == RC_NOT_CONNECTED) continue; // 如果该方向不相连，那么直接跳过
				const int ax = x + rcGetDirOffsetX(dir); // 计算邻居的 x
				const int ay = y + rcGetDirOffsetY(dir); // 计算邻居的 y
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir); // 计算邻居的索引

				// 如果邻居的区域与当前区域不同，则跳过
				if (chf.areas[ai] != area) continue;

				// 如果邻居已经拥有区域，且不是边界区域，则更新区域和距离
				if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0)
				{
					// 如果邻居的距离小于当前距离，则更新区域和距离的标志
					if ((int)srcDist[ai]+2 < (int)d2)
					{
						r = srcReg[ai];
						d2 = srcDist[ai]+2;
					}
				}
			}
			if (r) // 如果 r 不为 0，说明找到了邻居区域，将当前单元格标记为已使用
			{
				stack[j].index = -1; // mark as used
				dirtyEntries.push_back(DirtyEntry(i, r, d2)); // 将要修改的单元格信息添加到 dirtyEntries 中
			}
			else
			{
				failed++; // 如果没有找到邻居区域，则累加 failed 计数器
			}
		}
		
		// Copy entries that differ between src and dst to keep them in sync.
		// 将 dirtyEntries 中的信息落地到 srcReg 和 srcDist 中
		for (int i = 0; i < dirtyEntries.size(); i++) {
			int idx = dirtyEntries[i].index;
			srcReg[idx] = dirtyEntries[i].region;
			srcDist[idx] = dirtyEntries[i].distance2;
		}
		
		// 如果所有单元格都已经被标记过或者找不到邻居区域，则跳出循环
		if (failed == stack.size())
			break;
		
		// 判断最大迭代次数
		if (level > 0)
		{
			++iter;
			if (iter >= maxIter)
				break;
		}
	}
}


// 将单元格按层级排序
// 将所有未分配区域的单元格按照它们的距离场值分配到不同的堆栈中。这是实现分水岭算法（watershed algorithm）的关键步骤，用于区域划分。
// 这个函数实际的作用是从 startLevel 开始，将 8 个 level 内的所有未分配的各自存入参数 stacks 中
static void sortCellsByLevel(unsigned short startLevel, // 起始层级
							  rcCompactHeightfield& chf, // 压缩高度场
							  const unsigned short* srcReg, // 源区域数组
							  unsigned int nbStacks, // 堆栈数量，固定是 8 个
							  rcTempVector<LevelStackEntry>* stacks, // 堆栈数组
							  unsigned short loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift 每个堆栈的层级数（以 2 为底的对数），实现中为 1
{
	const int w = chf.width;
	const int h = chf.height;
	startLevel = startLevel >> loglevelsPerStack; // 将 startLevel 右移 loglevelsPerStack 位，可能是为了防止后续计算出的 sId 超出堆栈数量

	// 将 stacks 中的所有数据清空
	for (unsigned int j=0; j<nbStacks; ++j)
		stacks[j].clear();

	// put all cells in the level range into the appropriate stacks
	// 遍历所有单元格，将位于指定层级范围内的单元格放入适当的堆栈中
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w]; // 获取当前单元格
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i) // 遍历当前单元格的所有 open span
			{
				// 跳过空区域或已分配区域的单元格
				if (chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0)
					continue;

				int level = chf.dist[i] >> loglevelsPerStack; // 将距离场值右移 loglevelsPerStack 位，得到层级，这个层级可能比较大，两位数是有可能的
				int sId = startLevel - level; // 计算堆栈索引，索引越小，说明层级越大，也就是距离越远

				// 如果 sId 超出堆栈数量，则跳过，会交给后续的循环处理，后续 startLevel 会减小，sId 会变小
				if (sId >= (int)nbStacks)
					continue;

				// 如果 sId 小于 0，则将 sId 设为 0，是一种容错的方法，这意味着这个单元格其实没有被正确处理，但是后续 startLevel 会继续减小，sId 只会变得更小，不在再有机会正确处理它了
				// 但是区别不会很大，因为起始的 startLevel 就是大于等于 maxDistance 的最小偶数，只是 startLevel 先减了 2, 所以 sId 才有可能小于 0
				if (sId < 0)
					sId = 0;

				// 将当前单元格添加到相应的堆栈中
				stacks[sId].push_back(LevelStackEntry(x, y, i));
			}
		}
	}
}

// 将 srcStack 中的单元格添加到 dstStack 中，如果单元格未分配区域或已分配区域，则跳过
static void appendStacks(const rcTempVector<LevelStackEntry>& srcStack,
						 rcTempVector<LevelStackEntry>& dstStack,
						 const unsigned short* srcReg)
{
	for (int j=0; j<srcStack.size(); j++)
	{
		int i = srcStack[j].index;
		if ((i < 0) || (srcReg[i] != 0))
			continue;
		dstStack.push_back(srcStack[j]);
	}
}

struct rcRegion
{
	inline rcRegion(unsigned short i) :
		spanCount(0),
		id(i),
		areaType(0),
		remap(false),
		visited(false),
		overlap(false),
		connectsToBorder(false),
		ymin(0xffff),
		ymax(0)
	{}
	
	int spanCount;					// Number of spans belonging to this region
	unsigned short id;				// ID of the region
	unsigned char areaType;			// Are type.
	bool remap;
	bool visited;
	bool overlap;
	bool connectsToBorder;
	unsigned short ymin, ymax;
	rcIntArray connections;
	rcIntArray floors;
};

static void removeAdjacentNeighbours(rcRegion& reg)
{
	// Remove adjacent duplicates.
	for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1; )
	{
		int ni = (i+1) % reg.connections.size();
		if (reg.connections[i] == reg.connections[ni])
		{
			// Remove duplicate
			for (int j = i; j < reg.connections.size()-1; ++j)
				reg.connections[j] = reg.connections[j+1];
			reg.connections.pop();
		}
		else
			++i;
	}
}

static void replaceNeighbour(rcRegion& reg, unsigned short oldId, unsigned short newId)
{
	bool neiChanged = false;
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == oldId)
		{
			reg.connections[i] = newId;
			neiChanged = true;
		}
	}
	for (int i = 0; i < reg.floors.size(); ++i)
	{
		if (reg.floors[i] == oldId)
			reg.floors[i] = newId;
	}
	if (neiChanged)
		removeAdjacentNeighbours(reg);
}

static bool canMergeWithRegion(const rcRegion& rega, const rcRegion& regb)
{
	if (rega.areaType != regb.areaType)
		return false;
	int n = 0;
	for (int i = 0; i < rega.connections.size(); ++i)
	{
		if (rega.connections[i] == regb.id)
			n++;
	}
	if (n > 1)
		return false;
	for (int i = 0; i < rega.floors.size(); ++i)
	{
		if (rega.floors[i] == regb.id)
			return false;
	}
	return true;
}

static void addUniqueFloorRegion(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.floors.size(); ++i)
		if (reg.floors[i] == n)
			return;
	reg.floors.push(n);
}

static bool mergeRegions(rcRegion& rega, rcRegion& regb)
{
	unsigned short aid = rega.id;
	unsigned short bid = regb.id;
	
	// Duplicate current neighbourhood.
	rcIntArray acon;
	acon.resize(rega.connections.size());
	for (int i = 0; i < rega.connections.size(); ++i)
		acon[i] = rega.connections[i];
	rcIntArray& bcon = regb.connections;
	
	// Find insertion point on A.
	int insa = -1;
	for (int i = 0; i < acon.size(); ++i)
	{
		if (acon[i] == bid)
		{
			insa = i;
			break;
		}
	}
	if (insa == -1)
		return false;
	
	// Find insertion point on B.
	int insb = -1;
	for (int i = 0; i < bcon.size(); ++i)
	{
		if (bcon[i] == aid)
		{
			insb = i;
			break;
		}
	}
	if (insb == -1)
		return false;
	
	// Merge neighbours.
	rega.connections.clear();
	for (int i = 0, ni = acon.size(); i < ni-1; ++i)
		rega.connections.push(acon[(insa+1+i) % ni]);
		
	for (int i = 0, ni = bcon.size(); i < ni-1; ++i)
		rega.connections.push(bcon[(insb+1+i) % ni]);
	
	removeAdjacentNeighbours(rega);
	
	for (int j = 0; j < regb.floors.size(); ++j)
		addUniqueFloorRegion(rega, regb.floors[j]);
	rega.spanCount += regb.spanCount;
	regb.spanCount = 0;
	regb.connections.resize(0);

	return true;
}

static bool isRegionConnectedToBorder(const rcRegion& reg)
{
	// Region is connected to border if
	// one of the neighbours is null id.
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == 0)
			return true;
	}
	return false;
}

static bool isSolidEdge(rcCompactHeightfield& chf, const unsigned short* srcReg,
						int x, int y, int i, int dir)
{
	const rcCompactSpan& s = chf.spans[i];
	unsigned short r = 0;
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		r = srcReg[ai];
	}
	if (r == srcReg[i])
		return false;
	return true;
}

static void walkContour(int x, int y, int i, int dir,
						rcCompactHeightfield& chf,
						const unsigned short* srcReg,
						rcIntArray& cont)
{
	int startDir = dir;
	int starti = i;

	const rcCompactSpan& ss = chf.spans[i];
	unsigned short curReg = 0;
	if (rcGetCon(ss, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(ss, dir);
		curReg = srcReg[ai];
	}
	cont.push(curReg);
			
	int iter = 0;
	while (++iter < 40000)
	{
		const rcCompactSpan& s = chf.spans[i];
		
		if (isSolidEdge(chf, srcReg, x, y, i, dir))
		{
			// Choose the edge corner
			unsigned short r = 0;
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
				r = srcReg[ai];
			}
			if (r != curReg)
			{
				curReg = r;
				cont.push(curReg);
			}
			
			dir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = chf.cells[nx+ny*chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (starti == i && startDir == dir)
		{
			break;
		}
	}

	// Remove adjacent duplicates.
	if (cont.size() > 1)
	{
		for (int j = 0; j < cont.size(); )
		{
			int nj = (j+1) % cont.size();
			if (cont[j] == cont[nj])
			{
				for (int k = j; k < cont.size()-1; ++k)
					cont[k] = cont[k+1];
				cont.pop();
			}
			else
				++j;
		}
	}
}


static bool mergeAndFilterRegions(rcContext* ctx, int minRegionArea, int mergeRegionSize,
								  unsigned short& maxRegionId,
								  rcCompactHeightfield& chf,
								  unsigned short* srcReg, rcIntArray& overlaps)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}

	// Construct regions
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find edge of a region and find connections around the contour.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned short r = srcReg[i];
				if (r == 0 || r >= nreg)
					continue;
				
				rcRegion& reg = regions[r];
				reg.spanCount++;
				
				// Update floors.
				for (int j = (int)c.index; j < ni; ++j)
				{
					if (i == j) continue;
					unsigned short floorId = srcReg[j];
					if (floorId == 0 || floorId >= nreg)
						continue;
					if (floorId == r)
						reg.overlap = true;
					addUniqueFloorRegion(reg, floorId);
				}
				
				// Have found contour
				if (reg.connections.size() > 0)
					continue;
				
				reg.areaType = chf.areas[i];
				
				// Check if this cell is next to a border.
				int ndir = -1;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (isSolidEdge(chf, srcReg, x, y, i, dir))
					{
						ndir = dir;
						break;
					}
				}
				
				if (ndir != -1)
				{
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContour(x, y, i, ndir, chf, srcReg, reg.connections);
				}
			}
		}
	}

	// Remove too small regions.
	rcIntArray stack(32);
	rcIntArray trace(32);
	for (int i = 0; i < nreg; ++i)
	{
		rcRegion& reg = regions[i];
		if (reg.id == 0 || (reg.id & RC_BORDER_REG))
			continue;                       
		if (reg.spanCount == 0)
			continue;
		if (reg.visited)
			continue;
		
		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		bool connectsToBorder = false;
		int spanCount = 0;
		stack.clear();
		trace.clear();

		reg.visited = true;
		stack.push(i);
		
		while (stack.size())
		{
			// Pop
			int ri = stack.pop();
			
			rcRegion& creg = regions[ri];

			spanCount += creg.spanCount;
			trace.push(ri);

			for (int j = 0; j < creg.connections.size(); ++j)
			{
				if (creg.connections[j] & RC_BORDER_REG)
				{
					connectsToBorder = true;
					continue;
				}
				rcRegion& neireg = regions[creg.connections[j]];
				if (neireg.visited)
					continue;
				if (neireg.id == 0 || (neireg.id & RC_BORDER_REG))
					continue;
				// Visit
				stack.push(neireg.id);
				neireg.visited = true;
			}
		}
		
		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		if (spanCount < minRegionArea && !connectsToBorder)
		{
			// Kill all visited regions.
			for (int j = 0; j < trace.size(); ++j)
			{
				regions[trace[j]].spanCount = 0;
				regions[trace[j]].id = 0;
			}
		}
	}
	
	// Merge too small regions to neighbour regions.
	int mergeCount = 0 ;
	do
	{
		mergeCount = 0;
		for (int i = 0; i < nreg; ++i)
		{
			rcRegion& reg = regions[i];
			if (reg.id == 0 || (reg.id & RC_BORDER_REG))
				continue;
			if (reg.overlap)
				continue;
			if (reg.spanCount == 0)
				continue;
			
			// Check to see if the region should be merged.
			if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg))
				continue;
			
			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			int smallest = 0xfffffff;
			unsigned short mergeId = reg.id;
			for (int j = 0; j < reg.connections.size(); ++j)
			{
				if (reg.connections[j] & RC_BORDER_REG) continue;
				rcRegion& mreg = regions[reg.connections[j]];
				if (mreg.id == 0 || (mreg.id & RC_BORDER_REG) || mreg.overlap) continue;
				if (mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg))
				{
					smallest = mreg.spanCount;
					mergeId = mreg.id;
				}
			}
			// Found new id.
			if (mergeId != reg.id)
			{
				unsigned short oldId = reg.id;
				rcRegion& target = regions[mergeId];
				
				// Merge neighbours.
				if (mergeRegions(target, reg))
				{
					// Fixup regions pointing to current region.
					for (int j = 0; j < nreg; ++j)
					{
						if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REG)) continue;
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if (regions[j].id == oldId)
							regions[j].id = mergeId;
						// Replace the current region with the new one if the
						// current regions is neighbour.
						replaceNeighbour(regions[j], oldId, mergeId);
					}
					mergeCount++;
				}
			}
		}
	}
	while (mergeCount > 0);
	
	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;       // Skip nil regions.
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// Remap regions.
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}

	// Return regions that we found to be overlapping.
	for (int i = 0; i < nreg; ++i)
		if (regions[i].overlap)
			overlaps.push(regions[i].id);

	return true;
}


static void addUniqueConnection(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.connections.size(); ++i)
		if (reg.connections[i] == n)
			return;
	reg.connections.push(n);
}

static bool mergeAndFilterLayerRegions(rcContext* ctx, int minRegionArea,
									   unsigned short& maxRegionId,
									   rcCompactHeightfield& chf,
									   unsigned short* srcReg)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	
	// Construct regions
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterLayerRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find region neighbours and overlapping regions.
	rcIntArray lregs(32);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];

			lregs.clear();
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned short ri = srcReg[i];
				if (ri == 0 || ri >= nreg) continue;
				rcRegion& reg = regions[ri];
				
				reg.spanCount++;
				
				reg.ymin = rcMin(reg.ymin, s.y);
				reg.ymax = rcMax(reg.ymax, s.y);
				
				// Collect all region layers.
				lregs.push(ri);
				
				// Update neighbours
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const unsigned short rai = srcReg[ai];
						if (rai > 0 && rai < nreg && rai != ri)
							addUniqueConnection(reg, rai);
						if (rai & RC_BORDER_REG)
							reg.connectsToBorder = true;
					}
				}
				
			}
			
			// Update overlapping regions.
			for (int i = 0; i < lregs.size()-1; ++i)
			{
				for (int j = i+1; j < lregs.size(); ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcRegion& ri = regions[lregs[i]];
						rcRegion& rj = regions[lregs[j]];
						addUniqueFloorRegion(ri, lregs[j]);
						addUniqueFloorRegion(rj, lregs[i]);
					}
				}
			}
			
		}
	}

	// Create 2D layers from regions.
	unsigned short layerId = 1;

	for (int i = 0; i < nreg; ++i)
		regions[i].id = 0;

	// Merge montone regions to create non-overlapping areas.
	rcIntArray stack(32);
	for (int i = 1; i < nreg; ++i)
	{
		rcRegion& root = regions[i];
		// Skip already visited.
		if (root.id != 0)
			continue;
		
		// Start search.
		root.id = layerId;

		stack.clear();
		stack.push(i);
		
		while (stack.size() > 0)
		{
			// Pop front
			rcRegion& reg = regions[stack[0]];
			for (int j = 0; j < stack.size()-1; ++j)
				stack[j] = stack[j+1];
			stack.resize(stack.size()-1);
			
			const int ncons = (int)reg.connections.size();
			for (int j = 0; j < ncons; ++j)
			{
				const int nei = reg.connections[j];
				rcRegion& regn = regions[nei];
				// Skip already visited.
				if (regn.id != 0)
					continue;
				// Skip if the neighbour is overlapping root region.
				bool overlap = false;
				for (int k = 0; k < root.floors.size(); k++)
				{
					if (root.floors[k] == nei)
					{
						overlap = true;
						break;
					}
				}
				if (overlap)
					continue;
					
				// Deepen
				stack.push(nei);
					
				// Mark layer id
				regn.id = layerId;
				// Merge current layers to root.
				for (int k = 0; k < regn.floors.size(); ++k)
					addUniqueFloorRegion(root, regn.floors[k]);
				root.ymin = rcMin(root.ymin, regn.ymin);
				root.ymax = rcMax(root.ymax, regn.ymax);
				root.spanCount += regn.spanCount;
				regn.spanCount = 0;
				root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
			}
		}
		
		layerId++;
	}
	
	// Remove small regions
	for (int i = 0; i < nreg; ++i)
	{
		if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder)
		{
			unsigned short reg = regions[i].id;
			for (int j = 0; j < nreg; ++j)
				if (regions[j].id == reg)
					regions[j].id = 0;
		}
	}
	
	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;				// Skip nil regions.
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// Remap regions.
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}
	
	return true;
}



/// @par
/// 
/// This is usually the second to the last step in creating a fully built
/// compact heightfield.  This step is required before regions are built
/// using #rcBuildRegions or #rcBuildRegionsMonotone.
/// 
/// After this step, the distance data is available via the rcCompactHeightfield::maxDistance
/// and rcCompactHeightfield::dist fields.
///
/// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_DISTANCEFIELD);
	
	if (chf.dist)
	{
		rcFree(chf.dist);
		chf.dist = 0;
	}
	
	unsigned short* src = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!src)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	unsigned short* dst = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!dst)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'dst' (%d).", chf.spanCount);
		rcFree(src);
		return false;
	}
	
	unsigned short maxDist = 0;

	{
		rcScopedTimer timerDist(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST);

		calculateDistanceField(chf, src, maxDist);
		chf.maxDistance = maxDist;
	}

	{
		rcScopedTimer timerBlur(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

		// Blur 盒式模糊，将距离场进行平滑，忽略掉距离场中较小的突变，阈值是固定的 1
		if (boxBlur(chf, 1, src, dst) != src)
			rcSwap(src, dst);

		// Store distance.
		chf.dist = src;
	}
	
	rcFree(dst);
	
	return true;
}

static void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regId,
							rcCompactHeightfield& chf, unsigned short* srcReg)
{
	const int w = chf.width;	
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.areas[i] != RC_NULL_AREA)
					srcReg[i] = regId;
			}
		}
	}
}


static const unsigned short RC_NULL_NEI = 0xffff;

struct rcSweepSpan
{
	unsigned short rid;	// row id
	unsigned short id;	// region id
	unsigned short ns;	// number samples
	unsigned short nei;	// neighbour id
};

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps 
/// reduce unecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
							const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcIntArray prev(256);

	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}


	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcIntArray overlaps;
		chf.maxRegions = id;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// Monotone partitioning does not generate overlapping regions.
	}
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors. 
/// @p mergeRegionArea helps reduce unecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
					const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedDelete<unsigned short> buf((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount*2, RC_ALLOC_TEMP)); // buf 长度为 chf.spanCount*2
	if (!buf)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegions: Out of memory 'tmp' (%d).", chf.spanCount*4);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	// 创建了 8 个栈，用来进行分层处理，最终会分为 8 层
	const int LOG_NB_STACKS = 3;
	const int NB_STACKS = 1 << LOG_NB_STACKS;
	rcTempVector<LevelStackEntry> lvlStacks[NB_STACKS];
	for (int i=0; i<NB_STACKS; ++i)
		lvlStacks[i].reserve(256); // 每层最多 256 个节点

	rcTempVector<LevelStackEntry> stack;
	stack.reserve(256);
	
	unsigned short* srcReg = buf; // 区域 id 数组，长度为 chf.spanCount
	unsigned short* srcDist = buf+chf.spanCount; // 距离场数组，长度为 chf.spanCount, 由于 buf 长度为 chf.spanCount*2，所以使用 buf+chf.spanCount 表示距离场数组
	
	memset(srcReg, 0, sizeof(unsigned short)*chf.spanCount);
	memset(srcDist, 0, sizeof(unsigned short)*chf.spanCount);
	
	unsigned short regionId = 1; // 区域的 id, 递增的
	unsigned short level = (chf.maxDistance+1) & ~1; // 将起始的 level 设为不小于 maxDistance 的最小偶数，maxDistance 不会很小，示例地图一般为两位数，比如 50 左右

	// TODO: Figure better formula, expandIters defines how much the 
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
//	const int expandIters = 4 + walkableRadius * 2;
	const int expandIters = 8; // 扩展迭代次数，默认为 8, 因为 lvlStacks 长度为 8

	// 如果边框尺寸大于 0, 则处理四个边界区域，可以实现边界区域不参与区域划分，即导航不会接近边界，该值默认为 0
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		
		// Paint regions 将四个边界区域标记为 RC_BORDER_REG
		paintRectRegion(0, bw, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(w-bw, w, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, 0, bh, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, h-bh, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
	}

	chf.borderSize = borderSize;
	
	int sId = -1;
	while (level > 0) // 从最高层级向下遍历
	{
		level = level >= 2 ? level-2 : 0; // 每次降低两个 level
		sId = (sId+1) & (NB_STACKS-1); // 更新栈索引，每次循环递增，该索引每隔 8 个会回到 0, 相当于有一个轮次的概念

//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if (sId == 0) // 如果是一轮的开始，那么先把所有数据分层，结果存在 lvlStatcks 中，一共会被执行 总层数 / 2 / 8 次
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1);
		else 
			appendStacks(lvlStacks[sId-1], lvlStacks[sId], srcReg); // copy left overs from last level 上一层没处理完的需要复制到这一层中

//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			rcScopedTimer timerExpand(ctx, RC_TIMER_BUILD_REGIONS_EXPAND);

			// Expand current regions until no empty connected cells found.
			// 会先处理的是 0 层，层级越小，level 越大，距离越远，是从 distance 大到小的顺序来处理的，距离越远说明越靠近中间，所以会先处理中间的区域
			// 该函数会填充 srcReg 和 srcDist 数组
			expandRegions(expandIters, level, chf, srcReg, srcDist, lvlStacks[sId], false);
		}
		
		{
			rcScopedTimer timerFloor(ctx, RC_TIMER_BUILD_REGIONS_FLOOD);

			// Mark new regions with IDs.
			for (int j = 0; j<lvlStacks[sId].size(); j++)
			{
				LevelStackEntry current = lvlStacks[sId][j];
				int x = current.x;
				int y = current.y;
				int i = current.index;
				if (i >= 0 && srcReg[i] == 0)
				{
					if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack)) // 填充这个区域
					{
						if (regionId == 0xFFFF)
						{
							ctx->log(RC_LOG_ERROR, "rcBuildRegions: Region ID overflow");
							return false;
						}
						
						regionId++; // 递增区域ID
					}
				}
			}
		}
	}
	
	// Expand current regions until no empty connected cells found.
	expandRegions(expandIters*8, 0, chf, srcReg, srcDist, stack, true);
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out smalle regions.
		rcIntArray overlaps;
		chf.maxRegions = regionId;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// If overlapping regions were found during merging, split those regions.
		if (overlaps.size() > 0)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildRegions: %d overlapping regions.", overlaps.size());
		}
	}
		
	// Write the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}


bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf,
						 const int borderSize, const int minRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);
	
	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcIntArray prev(256);
	
	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}
				
				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}
				
				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}
	
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge monotone regions to layers and remove small regions.
		chf.maxRegions = id;
		if (!mergeAndFilterLayerRegions(ctx, minRegionArea, chf.maxRegions, chf, srcReg))
			return false;
	}
	
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}
