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


#include "glacier_rigid_body.h"

void GRigidBody::Tick_PreTransform(const f32 DetalTime)
{
    GVector3 VNew = m_Velocity + m_Gravity * DetalTime;

    if (VNew.SizeSquare() > (m_VelocityMax * m_VelocityMax))
    {
        VNew = VNew.GetNormalize() * m_VelocityMax;
    }

    m_Transform.m_Pos += (m_Velocity + VNew) * GMath::Half() * DetalTime;

    m_Velocity = VNew;

    m_bNeedUpdate = true;
}

void GRigidBody::AddImpulse_World( const GVector3& VPos, const GVector3& VImpulse)
{


}

void GRigidBody::AddImpulse_Local( const GVector3& VPos, const GVector3& VImpulse)
{


}