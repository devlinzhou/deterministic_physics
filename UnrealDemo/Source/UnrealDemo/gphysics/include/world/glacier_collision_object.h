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
#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"
#include <set>
#include <vector>

enum ECollisionObjectType
{
    CO_Base = 0,
    CO_Rigid_Body,
    CO_Soft_Body,

};

class GGridCell;
class GCObject // Glacier Physics Collision Object
{
public:

    GCObject(  uint32_t uId, EShape TShape ): 
        m_Shape(TShape), 
        m_Id(uId), 
        m_LoaclAABB(GVector3::Zero()), 
        m_WorldAABB(GVector3::Zero())
    {
        m_CollisionType     = CO_Base;
        m_Transform         = GTransform_QT::Identity();
        m_Transform_Last    = GTransform_QT::Identity();
        m_UserId            = -1;
        m_pGridCell         = nullptr;
        m_bNeedUpdate       = true;
    }

    virtual ~GCObject()
    {
    
    }

    virtual ECollisionObjectType GetCOType() const
    {
        return m_CollisionType;
    }

    inline uint32_t GetId()const {return m_Id;}

    virtual bool IsDynamic() const
    {
        return false;
    }


    void NeedUpdate()
    {
        m_bNeedUpdate = true;
    }

    void UpdateAABB()
    {
        m_WorldAABB = m_Transform.TransformAABB( m_LoaclAABB );
    }

    const GAABB& GetAABB() const 
    {
        return m_WorldAABB;
    }

    void UpdateLocalBox()
    {
        m_LoaclAABB = m_Shape.GetLocalBox();
    }

    const GAABB& GetLocalAABB() const
    { 
        return m_LoaclAABB;
    }

    void ClearContactPair( )
    {
        m_ContactObjs.clear();
        m_ContactArray.clear();
    }

    void AddContactPair( uint64_t uPairId, int32_t id )
    {
        uint32_t IdA = (uint32_t)(uPairId >> 32 );
        uint32_t IdB = (uint32_t)(uPairId & 0xFFFFFFFF);

        if( IdA != m_Id ) 
        {
            m_ContactObjs.insert( IdA );
        }

        if (IdB != m_Id)
        {
            m_ContactObjs.insert(IdB);
        }

        m_ContactArray.push_back(id);
    }


public:
    GCollisionShape         m_Shape;  
    uint32_t                m_Id;
    ECollisionObjectType    m_CollisionType;
    
    GTransform_QT           m_Transform;
    GTransform_QT           m_Transform_Last;

    GAABB                   m_LoaclAABB;
    GAABB                   m_WorldAABB;

    uint32_t                m_UserId;

    GGridCell*              m_pGridCell;
    bool                    m_bNeedUpdate;


    std::set<uint32_t>      m_ContactObjs;
    std::vector<int32_t>    m_ContactArray;

};