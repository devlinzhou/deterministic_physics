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
#include "glacier_grid.h"
#include <vector>
#include <map>



enum GPDraw
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
        m_Friction              = GMath::Makef32(0,1,10);
        m_FrictionClampVelocity = GMath::Makef32(0,1,100);
        m_CollisionEnergyLost   = GMath::Makef32(0,10,100);
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

    f32 GetTotalEnergy();
    GVector3 GetTotalLinearMomentum();
    GVector3 GetTotalAngularMomentum();


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
    f32   m_FrictionClampVelocity;
    f32   m_CollisionEnergyLost;

    bool solvesort = false;

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
};