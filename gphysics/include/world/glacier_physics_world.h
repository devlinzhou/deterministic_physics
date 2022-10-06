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

#include "glacier_transform_qt.h"


#include <vector>
#include <map>

struct GGridPosition
{
    //static const LGridPosition3D zero;
    int32_t x;
    int32_t y;
    int32_t z;

    GGridPosition( )
    {

    }

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
class IGlacierDraw;
class GCollisionObject;
class GGridCell
{
public:
    GGridPosition                   m_pos;
    std::vector<GCollisionObject*>  m_Objects;
    GVector3                        m_min = GVector3( GMath::Zero(), GMath::Zero(), GMath::Zero());
    GVector3                        m_max = GVector3( GMath::Zero(), GMath::Zero(), GMath::Zero());

public:
    GGridCell( const GGridPosition& TPos ) : m_pos(TPos)
    {}

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
        return (m_min + m_max) * GMath::Half();
    }

    inline GVector3 GetHalfSize() const
    {
        return (m_max - m_min) * GMath::Half();
    }

    bool AddCollisionObject(GCollisionObject* pObject);


    void DebugDraw(IGlacierDraw* pDraw ) const;


private:
  
};


class GPhysicsWorld
{
public:

    void Init( int32_t nCellWide = 20, int32_t nCellHeight = 20 )
    {
        m_nCellWide     = f32(nCellWide);
        m_nCellHeight   = f32(nCellHeight);
    }

    void AddCollisionObject( GCollisionObject* pObject );


    void PreTick(  );

    void SimulateTick( f32 DetltaTime );

    void PostTick();

    void DebugDraw(IGlacierDraw* pDraw ) const;

private:

    f32 m_nCellWide;
    f32 m_nCellHeight;

    //GGrid   m_Grids;
    std::map<GGridPosition, GGridCell*> m_Grids;


};