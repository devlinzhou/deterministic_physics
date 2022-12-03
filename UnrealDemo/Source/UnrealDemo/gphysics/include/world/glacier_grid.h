/* 
 * Copyright (C) 2022 zhou xuan, Email: zhouxuan6676@gmail.com
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at * 
 * http://www.apache.org/licenses/LICENSE-2.0 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License. 
 */
#pragma once

#include "glacier_vector.h"
#include "glacier_aabb.h"
#include <vector>

struct GGridPosition
{
	//static const LGridPosition3D zero;
	int32_t x;
	int32_t y;
	int32_t z;

	GGridPosition(): x(0), y(0), z(0){}

	GGridPosition(int32_t _x, int32_t _y, int32_t _z) :
		x(_x), y(_y), z(_z)
	{
	}

	inline GGridPosition operator + (const GGridPosition& b) const
	{
		return GGridPosition(x + b.x, y + b.y, z + b.z);
	}

	inline GGridPosition& operator += (const GGridPosition& b)
	{
		x += b.x;
		y += b.y;
		z += b.z;
		return *this;
	}

	inline bool operator == (const GGridPosition& b) const
	{
		return x == b.x && y == b.y && z == b.z;
	}

	inline bool operator != (const GGridPosition& b) const
	{
		return x != b.x || y != b.y || z != b.z;
	}

	inline bool operator < (const GGridPosition& b) const
	{
		if (z != b.z)
		{
			return z < b.z;
		}
		else
		{
			if (y != b.y)
			{
				return y < b.y;
			}
			else
			{
				return x < b.x;
			}
		}
	}
};

class GCObject;

//class GOCtree

class GGridCell
{
public:
	GGridPosition               m_pos;
	std::vector<GCObject*>      m_Objects;
	GAABB                       m_AABB;
	GAABB                       m_CeilAABB;

public:
	GGridCell(const GGridPosition& TPos, f32 CeilWide, f32 CeilHeight) : m_pos(TPos)
	{
		GVector3 VMin = GVector3(
			f32(TPos.x) * CeilWide,
			f32(TPos.y) * CeilWide,
			f32(TPos.z) * CeilHeight);

		GVector3 VMax = GVector3(
			f32(TPos.x + 1) * CeilWide,
			f32(TPos.y + 1) * CeilWide,
			f32(TPos.z + 1) * CeilHeight);

		m_CeilAABB = GAABB(VMin, VMax);
	}

	~GGridCell() {}

public:
	inline bool IsAdjacency(const GGridCell& other)
	{
		return
			(abs(m_pos.x - other.m_pos.x) <= 1) &&
			(abs(m_pos.y - other.m_pos.y) <= 1) &&
			(abs(m_pos.z - other.m_pos.z) <= 1);
	}

	inline GVector3 GetCenter() const
	{
		return m_AABB.GetCenter();
	}

	inline GVector3 GetHalfSize() const
	{
		return m_AABB.GetHalfSize();
	}

	void UpdateAABB();

	bool AddCollisionObject(GCObject* pObject);

	bool RemoveObject(GCObject* pObject);

	void DebugDraw(class IGlacierDraw* pDraw, uint32_t mask) const;

private:

};