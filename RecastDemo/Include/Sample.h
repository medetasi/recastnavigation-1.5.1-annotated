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

#ifndef RECASTSAMPLE_H
#define RECASTSAMPLE_H

#include "Recast.h"
#include "SampleInterfaces.h"


/// Tool types.
enum SampleToolType
{
	TOOL_NONE = 0,
	TOOL_TILE_EDIT,
	TOOL_TILE_HIGHLIGHT,
	TOOL_TEMP_OBSTACLE,
	TOOL_NAVMESH_TESTER,
	TOOL_NAVMESH_PRUNE,
	TOOL_OFFMESH_CONNECTION,
	TOOL_CONVEX_VOLUME,
	TOOL_CROWD,
	MAX_TOOLS
};

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL		= 0xffff	// All abilities.
};

class SampleDebugDraw : public DebugDrawGL
{
public:
	virtual unsigned int areaToCol(unsigned int area);
};

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED, // 分水岭分区算法
	SAMPLE_PARTITION_MONOTONE, // 单调分区算法
	SAMPLE_PARTITION_LAYERS // 层级分区算法
};

struct SampleTool
{
	virtual ~SampleTool();
	virtual int type() = 0;
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleMenu() = 0;
	virtual void handleClick(const float* s, const float* p, bool shift) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleToggle() = 0;
	virtual void handleStep() = 0;
	virtual void handleUpdate(const float dt) = 0;
};

struct SampleToolState {
	virtual ~SampleToolState();
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleUpdate(const float dt) = 0;
};

// 所有类型 sample 的基类
class Sample
{
protected:
	class InputGeom* m_geom; // 输入的 mesh
	class dtNavMesh* m_navMesh; // 生成的导航网格
	class dtNavMeshQuery* m_navQuery; // 导航网格查询
	class dtCrowd* m_crowd; // 人群，似乎是一个群体寻路的功能

	unsigned char m_navMeshDrawFlags; // 导航网格的绘制标志

	float m_cellSize; // 每个 cell 的大小
	float m_cellHeight; // 每个 cell 的高度
	float m_agentHeight; // 代理的高度
	float m_agentRadius; // 代理的半径
	float m_agentMaxClimb; // 代理的最大爬升高度
	float m_agentMaxSlope; // 代理的最大坡度
	float m_regionMinSize; // 区域的最小大小
	float m_regionMergeSize; // 区域合并的大小
	float m_edgeMaxLen; // 边的最大长度
	float m_edgeMaxError; // 边的最大误差
	float m_vertsPerPoly; // 每个多边形的顶点数
	float m_detailSampleDist; // 细节采样距离
	float m_detailSampleMaxError; // 细节采样最大误差
	int m_partitionType; // 分区类型

	bool m_filterLowHangingObstacles; // 过滤低悬挂障碍物
	bool m_filterLedgeSpans; // 过滤边缘跨度
	bool m_filterWalkableLowHeightSpans; // 过滤可行走的低高度跨度
	
	SampleTool* m_tool; // 当前选中的工具
	SampleToolState* m_toolStates[MAX_TOOLS]; // 工具状态
	
	BuildContext* m_ctx; // 构建上下文

	SampleDebugDraw m_dd; // 调试绘制
	
	/*
	从文件中加载导航网格，solo 和 tile 模式都用这个函数
	@param path 文件路径
	@return 加载的导航网格
	*/
	dtNavMesh* loadAll(const char* path);

	/*
	保存导航网格到文件
	@param path 文件路径
	@param mesh 导航网格
	*/
	void saveAll(const char* path, const dtNavMesh* mesh);

public:
	Sample();
	virtual ~Sample();
	
	void setContext(BuildContext* ctx) { m_ctx = ctx; }
	
	void setTool(SampleTool* tool);
	SampleToolState* getToolState(int type) { return m_toolStates[type]; }
	void setToolState(int type, SampleToolState* s) { m_toolStates[type] = s; }

	SampleDebugDraw& getDebugDraw() { return m_dd; }

	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	virtual void handleUpdate(const float dt);
	virtual void collectSettings(struct BuildSettings& settings);

	virtual class InputGeom* getInputGeom() { return m_geom; }
	virtual class dtNavMesh* getNavMesh() { return m_navMesh; }
	virtual class dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	virtual class dtCrowd* getCrowd() { return m_crowd; }
	virtual float getAgentRadius() { return m_agentRadius; }
	virtual float getAgentHeight() { return m_agentHeight; }
	virtual float getAgentClimb() { return m_agentMaxClimb; }
	
	unsigned char getNavMeshDrawFlags() const { return m_navMeshDrawFlags; }
	void setNavMeshDrawFlags(unsigned char flags) { m_navMeshDrawFlags = flags; }

	void updateToolStates(const float dt);
	void initToolStates(Sample* sample);
	void resetToolStates();
	void renderToolStates();
	void renderOverlayToolStates(double* proj, double* model, int* view);

	void resetCommonSettings();
	void handleCommonSettings();

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample(const Sample&);
	Sample& operator=(const Sample&);
};


#endif // RECASTSAMPLE_H
