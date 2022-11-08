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

#include "glacier_collision_object.h"
#include "glacier_matrix.h"

class GRigidBody : public GCollisionObject
{
public:

    GRigidBody(uint32_t id, EShape TShape) : GCollisionObject(id, TShape)
    {
        m_CollisionType = CollisionObject_Rigid_Body;
        m_Gravity = GVector3(GMath::Zero(), GMath::Zero(), -GMath::Makef32(9,8,10) );
        m_VelocityMax = GMath::Makef32(100,0,1);
        m_bDynamic = false;
    }

    void Tick_PreTransform( const f32 DetalTime );

    void AddImpulse_World( const GVector3& VPos, const GVector3& VImpulse );
    void AddImpulse_Local( const GVector3& VPos, const GVector3& VImpulse );


    GVector3 GetMassCenterPos() const
    {
        return m_Transform.m_Pos;
    }

    GMatrix3 getGlobalInertiaTensorInverse() const
    {
        return m_MoumentInertia * GMatrix3( m_Transform.m_Rot);
    }

public:

    bool            m_bDynamic;
    GTransform_QT   m_Transform_Pre;


    f32             m_Mass;
    f32             m_InvMass;

    GVector3        m_Velocity;
    f32             m_VelocityMax;

    GMatrix3        m_MoumentInertia;           // local
    GMatrix3        m_InvMoumentInertia;

    GVector3        m_AngleVelocity;
    f32             m_AngleVelocityMax;




    GVector3        m_Gravity;
};