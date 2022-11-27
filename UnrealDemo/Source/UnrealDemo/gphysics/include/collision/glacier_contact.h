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
#include <vector>
#include <map>



class GManifoldPoint
{
public:

    GVector3    m_PosWorld;
    GVector3    m_Normal;
    f32         m_depth;
    uint32_t    m_FaceIndex;


    bool    m_bCurrentSeparate;
};



class GCollisionContact
{
public:

    GCollisionContact() = default;
    GCollisionContact(const GCollisionContact&) = default;


    void AddContactPoint( 
        const GVector3& PosWorld,
        const GVector3& Normal,
        f32 depth,
        uint32_t faceid = -1);

     void Clear()
     {
         PairId = 0;
         m_nPointCount = 0;
         PointOnSurface = -1;
     }

    void ClearPoint()
    {
        m_nPointCount = 0;
    }

    int32_t GetPointCount() const
    {
        return  m_nPointCount < MaxPoint ? m_nPointCount : MaxPoint;
    }


    uint64_t        PairId;
    uint32_t        PointOnSurface;

    static constexpr int32_t MaxPoint = 16;

    int32_t         m_nPointCount;
    GManifoldPoint  m_Point[MaxPoint];
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

            PairId = ((uint64_t)p1->GetId() << 32) | (uint64_t)p2->GetId();
        }
        else
        {
            pObjectA = p2;
            pObjectB = p1;

            PairId = ((uint64_t)p2->GetId() << 32) | (uint64_t)p1->GetId();
        }

        Solved = false;
    }

    void CalculateContactStaticDepth()
    {
        if (pObjectA->m_ContactStaticDepth > (pObjectB->m_ContactStaticDepth + 1))
        {
            pObjectA->m_ContactStaticDepth = (pObjectB->m_ContactStaticDepth + 1);
        }

        if (pObjectB->m_ContactStaticDepth > (pObjectA->m_ContactStaticDepth + 1))
        {
            pObjectB->m_ContactStaticDepth = (pObjectA->m_ContactStaticDepth + 1);
        }

        m_ContactStaticDepth = pObjectA->m_ContactStaticDepth < pObjectB->m_ContactStaticDepth ?
            pObjectA->m_ContactStaticDepth : pObjectB->m_ContactStaticDepth;
    }


    GVector3 GetWorldRelativeB(const GVector3& VPos) const;

    f32 GetContactPairEnergy_World() const;
    f32 GetContactPairEnergy_Relative(const GVector3& CenterVelocity) const;

    f32 GetContactPairMomentum_world() const;

    void SeparatePair(GRigidBody* pRA, GRigidBody* pRB, bool bSwap);

    uint64_t            PairId;
    uint32_t            m_ContactStaticDepth;
    GCObject* pObjectA;
    GCObject* pObjectB;

    GCollisionContact   PairContact;

    bool                Solved;

};