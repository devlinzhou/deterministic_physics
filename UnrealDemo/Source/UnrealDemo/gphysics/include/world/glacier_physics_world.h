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
#include "glacier_transform_qt.h"
#include "glacier_collision_object.h"
#include "glacier_debug_draw.h"
#include <vector>
#include <map>

struct GGridPosition
{
    //static const LGridPosition3D zero;
    int32_t x;
    int32_t y;
    int32_t z;

    GGridPosition(int32_t _x, int32_t _y, int32_t _z) :
        x(_x), y(_y), z(_z)
    {
    
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
        if( z != b.z )
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

class GGridCell
{
public:
    GGridPosition                   m_pos = GGridPosition(0,0,0);
    std::vector<GCollisionObject*>  m_AllCollisionObject;
    GVector3                        m_min;// = GVector3(10000, 10000, 10000);
    GVector3                        m_max;// = GVector3(-10000, -10000, -10000);

public:
    GGridCell(int32_t _x, int32_t _y, int32_t _z) : m_pos( GGridPosition(_x,_y,_z))
    {}

    ~GGridCell() {}

public:
    inline bool IsAdjacency(const GGridCell& other)
    {
        return 
            ( abs(m_pos.x - other.m_pos.x) <= 1 ) &&
            ( abs(m_pos.y - other.m_pos.y) <= 1 ) &&
            ( abs(m_pos.z - other.m_pos.z) <= 1 );
    }


    inline GVector3 GetCenter() const
    {
        (m_min + m_max) * f32::Half();
    }

    inline GVector3 GetHalfSize()
    {
        (m_max - m_min ) * f32::Half();
    }



    void DebugDraw(IGlacierDraw* pDraw );



private:
  

    
};


class GPhysicsWorld
{
public:

    void DebugDraw(IGlacierDraw* pDraw );

private:

    //GGrid   m_Grids;
    std::map<GGridPosition, GGridCell*> m_Grids;
};