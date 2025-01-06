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

#ifndef MESHLOADER_OBJ
#define MESHLOADER_OBJ

#include <string>

// 一个 obj 文件的加载器
class rcMeshLoaderObj
{
public:
	rcMeshLoaderObj();
	~rcMeshLoaderObj();

	/*
	 * 从文件中加载数据，读取了顶点和三角形数据，然后计算出了法线数据
	 * @param[in] fileName 文件名
	 * @return bool 是否加载成功
	*/
	bool load(const std::string& fileName);

	const float* getVerts() const { return m_verts; }
	const float* getNormals() const { return m_normals; }
	const int* getTris() const { return m_tris; }
	int getVertCount() const { return m_vertCount; }
	int getTriCount() const { return m_triCount; }
	const std::string& getFileName() const { return m_filename; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcMeshLoaderObj(const rcMeshLoaderObj&);
	rcMeshLoaderObj& operator=(const rcMeshLoaderObj&);

	/*
	 * 增加一个顶点
	 * @param[in] x 顶点的 x 坐标
	 * @param[in] y 顶点的 y 坐标
	 * @param[in] z 顶点的 z 坐标
	 * @param[out] cap 顶点的容量
	 * @return void
	*/
	void addVertex(float x, float y, float z, int& cap);

	/*
	 * 增加一个三角形
	 * @param[in] a 三角形的第一个顶点的索引
	 * @param[in] b 三角形的第二个顶点的索引
	 * @param[in] c 三角形的第三个顶点的索引
	 * @param[out] cap 三角形的容量
	 * @return void
	*/
	void addTriangle(int a, int b, int c, int& cap);
	
	std::string m_filename;
	float m_scale; // 缩放，只支持一倍的缩放
	float* m_verts; // 顶点的坐标集合
	int* m_tris; // 三角形的顶点索引集合
	float* m_normals; // 法线
	int m_vertCount; // 顶点数量
	int m_triCount; // 三角形数量
};

#endif // MESHLOADER_OBJ
