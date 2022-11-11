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
#include "glacier_physics_utils.h"

void GRigidBody::Tick_PreTransform(const f32 DetalTime)
{
    GVector3 VNew = m_LinearVelocity + m_Gravity * DetalTime;

    if (VNew.SizeSquare() > (m_LinearVelocityMax * m_LinearVelocityMax))
    {
        VNew = VNew.GetNormalize() * m_LinearVelocityMax;
    }

    m_Transform.m_Pos += (m_LinearVelocity + VNew) * GMath::Half() * DetalTime;

    m_LinearVelocity = VNew;


    f32	fAngle = m_AngularVelocity.Size(); 

    fAngle = GMath::Min(fAngle, m_AngularVelocityMax );

    f32 fDeltaAngle = fAngle * DetalTime;

    GQuaternion DeltaRot = GQuaternion( m_AngularVelocity.GetNormalize(), fDeltaAngle );


    m_Transform.m_Rot = m_Transform.m_Rot * DeltaRot;

    m_Transform.m_Rot.Normalize();


    m_bNeedUpdate = true;
}

void GRigidBody::AddImpulse_World( const GVector3& VPos, const GVector3& VImpulse)
{
    m_LinearVelocity += VImpulse * m_InvMass;

    GMatrix3 Inv_Ineria = getGlobalInertiaTensorInverse();

    GVector3 Torque = GVector3::CrossProduct(VPos - GetMassCenterPos(), VImpulse );

    m_AngularVelocity += Inv_Ineria.TransformVector( Torque); 
}

void GRigidBody::AddImpulse_Local( const GVector3& VPos, const GVector3& VImpulse)
{
    

}

void GRigidBody::CalculateInertiaTensor()
{
    m_Mass              = GMath::Makef32(10,0,1);// m_density * GPhyscsUtils::CalculateVolume( m_Shape);
    m_MoumentInertia    = m_Mass * GPhyscsUtils::CalculateInertiaTensor( m_Shape );
    m_InvMoumentInertia = m_MoumentInertia.GetInverse();

    m_InvMass           = GMath::One() / m_Mass;
}