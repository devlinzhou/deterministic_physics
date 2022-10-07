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
#include "glacier_box.h"
#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"

enum ECollisionObjectType
{
    Static= 0,
    Dynamic,

};

class GCollisionObject
{
public:

    ECollisionObjectType GetCollisionObjectType() const {return m_CollisionType;}

    uint32_t GetId()const {return m_Id;}

    void UpdateAABB()
    {
        if( m_Shape.ShapType == EShape::EShape_Sphere)
        {
            m_AABB = GAABB( 
                m_Transform.m_Translation - GVector3( m_Shape.GetRaiuds() ),
                m_Transform.m_Translation + GVector3( m_Shape.GetRaiuds() ) );
        }
    }

    const GAABB& GetAABB() const { return m_AABB; }

    


public:
    GCollisionShape         m_Shape;  
    ECollisionObjectType    m_CollisionType;
    GTransform_QT           m_Transform;
    GTransform_QT           m_Transform_Last;
    GAABB                   m_AABB;

    uint32_t                m_UserId;
    uint32_t                m_Id;
};