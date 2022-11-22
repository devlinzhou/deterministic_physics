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
#include "glacier_collision_object.h"
#include "glacier_contact.h"
#include "glacier_collision_algorithm.h"
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

    inline GGridPosition operator + (const GGridPosition& b) const
    {
        return GGridPosition(x+b.x, y+b.y, z+b.z);
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
class GCObject;
class GGridCell
{
public:
    GGridPosition               m_pos;
    std::vector<GCObject*>      m_Objects;
    GAABB                       m_AABB;

public:
    GGridCell( const GGridPosition& TPos, f32 CeilWide, f32 CeilHeight ) : m_pos(TPos)
    {
        GVector3 VMin = GVector3( 
            f32(TPos.x) * CeilWide,
            f32(TPos.y) * CeilWide,
            f32(TPos.z) * CeilHeight );

        GVector3 VMax = GVector3(
            f32(TPos.x + 1) * CeilWide,
            f32(TPos.y + 1) * CeilWide,
            f32(TPos.z + 1) * CeilHeight);
       
        m_AABB = GAABB(VMin, VMax );
 
    }

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
        return m_AABB.GetCenter();
    }

    inline GVector3 GetHalfSize() const
    {
        return m_AABB.GetHalfSize();
    }

    bool AddCollisionObject(GCObject* pObject);

    bool RemoveObject(GCObject* pObject);


    void DebugDraw(IGlacierDraw* pDraw, uint32_t mask ) const;


private:
  
};

class GRigidBody;
class GBroadPhasePair
{
public:

    GBroadPhasePair() = default;
    GBroadPhasePair(const GBroadPhasePair&) = default;
    GBroadPhasePair(GCObject* p1, GCObject* p2)
    {
        if (p1->GetId() < p2->GetId())
        {
            pObjectA = p1;
            pObjectB = p2;

            PairId = ((uint64_t)p1->GetId() << 32 ) | (uint64_t)p2->GetId();
        }
        else
        {
            pObjectA = p2;
            pObjectB = p1;

            PairId = ((uint64_t)p2->GetId() << 32 ) | (uint64_t)p1->GetId();
        }

        Solved = false;
    }

    void CalculateContactStaticDepth( )
    {
        if(pObjectA->m_ContactStaticDepth > (pObjectB->m_ContactStaticDepth + 1 ) )
        {
            pObjectA->m_ContactStaticDepth = (pObjectB->m_ContactStaticDepth + 1 );
        }

        if (pObjectB->m_ContactStaticDepth > (pObjectA->m_ContactStaticDepth + 1))
        {
            pObjectB->m_ContactStaticDepth = (pObjectA->m_ContactStaticDepth + 1);
        }

        m_ContactStaticDepth = pObjectA->m_ContactStaticDepth < pObjectB->m_ContactStaticDepth ?
            pObjectA->m_ContactStaticDepth : pObjectB->m_ContactStaticDepth;
    }


    GVector3 GetWorldRelativeB(const GVector3& VPos ) const;

    f32 GetContactPairEnergy( ) const;

    f32 GetContactPairMomentum_world() const;

    void SeparatePair( GRigidBody* pRA, GRigidBody* pRB, bool bSwap );

    uint64_t            PairId;
    uint32_t            m_ContactStaticDepth;
    GCObject*           pObjectA;
    GCObject*           pObjectB;

    GCollisionContact   PairContact;

    bool                Solved;

};

enum
{
    GPDraw_Shape        = 1 << 0,
    GPDraw_LocalBox     = 1 << 1,
    GPDraw_WorldBox     = 1 << 2,
    GPDraw_CeilBox      = 1 << 3,
    GPDraw_Contact      = 1 << 4,
    GPDraw_Momentum     = 1 << 5,
};

class GPhysicsWorld
{
public:

    void Init( int32_t nCellWide = 20, int32_t nCellHeight = 20 )
    {
        m_nCellWide             = f32(nCellWide);
        m_nCellHeight           = f32(nCellHeight);
        m_CObjectId             = 0;
        m_Friction              = GMath::Makef32(0,5,10);
        m_FrictionVelocity      = GMath::Makef32(0,1,100);
        m_CollisionEnergyLost   = GMath::Makef32(0,9,10);
        m_fCollisionExtern      = GMath::Makef32(0,3,10);
        m_CollisionManager.Init();
    }

    void UnInit(){}

    inline GGridPosition GetGridPos( const GVector3& VPos )
    {
        return GGridPosition (
            GMath::FloorToInt(VPos.x / m_nCellWide),
            GMath::FloorToInt(VPos.y / m_nCellWide),
            GMath::FloorToInt(VPos.z / m_nCellHeight));
    }

    bool AddCollisionObject( GCObject* pObject );
    bool DeleteCollisionObject( GCObject* pObject );

    const GCObject* FindCollisionObject( uint32_t id ) const; 


    bool AddStaticLargeObj(GCObject* pObject );
    bool DeleteStaticLargeObj( GCObject* pObject );

    bool UpdateCollisionObject( GCObject* pObject );

    void PreTick(  );

    void Tick( f32 DetltaTime );

    void PostTick();

    static void DebugDrawObject( IGlacierDraw* pDraw, const GCObject* pObj, uint32_t mask );

    void DebugDraw(IGlacierDraw* pDraw, uint32_t mask ) const;

public:

    static inline uint64_t GetPairId( uint32_t IdA, uint32_t IdB )
    {
        if (IdA < IdB)
        {
            return ((uint64_t)IdA << 32) | (uint64_t)IdB;
        }
        else
        {
            return ((uint64_t)IdB << 32) | (uint64_t)IdA;
        }
    }


public:

    void Simulate( f32 DetltaTime );

    void CollisionBroadPhase( );

    void CollisionNarrowPhase( );

    void SolveContactConstraint( GBroadPhasePair& pPair );

    void SolveContactConstraint( );

    void UpdateSceneGrid( );

    void ClearContactPair();

    void AddContactPair(GCObject* p1, GCObject* p2);


    uint32_t GetNewCObjectId()
    {
        return m_CObjectId++;
    }

public:

    f32   m_Friction;
    f32   m_FrictionVelocity;
    f32   m_CollisionEnergyLost;

private:

    uint32_t                                m_CObjectId;
    f32                                     m_nCellWide;
    f32                                     m_nCellHeight;
    f32                                     m_fCollisionExtern;

    //GGrid   m_Grids;
    std::map<GGridPosition, GGridCell*>     m_Grids;
    std::map<uint32_t, GCObject*>           m_ObjectMap;
    std::vector<GCObject*>                  m_Objects;
    std::vector<GCObject*>                  m_StaticLargeObj;

    std::vector<GBroadPhasePair>            m_BroadPhasePairs;
    std::map<uint64_t, GBroadPhasePair>     m_BroadPhaseMap;

    GCollisionManerger                      m_CollisionManager;
    GContactManerger                        m_ContactManager;
};