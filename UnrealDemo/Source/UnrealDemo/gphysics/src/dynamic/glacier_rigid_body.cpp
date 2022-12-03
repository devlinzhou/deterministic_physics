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
    GVector3 VNewLinearVelocity = m_LinearVelocity + m_Gravity * DetalTime;

    GVector3::ClampVector(VNewLinearVelocity, m_LinearVelocityMax);
    GVector3::ClampVector(m_AngularVelocity, m_AngularVelocityMax);

    m_Transform.m_Pos += (m_LinearVelocity + VNewLinearVelocity) * GMath::Half() * DetalTime;

    m_LinearVelocity = VNewLinearVelocity;

    f32	fAngle = m_AngularVelocity.Size(); 

    f32 fDeltaAngle = fAngle * DetalTime;

    GQuaternion DeltaRot = GQuaternion( m_AngularVelocity.GetNormalize(), fDeltaAngle );

    m_Transform.m_Rot = m_Transform.m_Rot * DeltaRot;

    m_Transform.m_Rot.Normalize();

	GMatrix3 MRot = GMatrix3(m_Transform.m_Rot);

	m_InvInertiaTensorWorld = MRot.Transpose() * m_InvInertiaTensor * MRot;

    m_bNeedUpdate = true;
}

void GRigidBody::CalculateInertiaTensor()
{
    m_Mass             = m_density * GPhyscsUtils::CalculateVolume( m_Shape);
    m_InertiaTensor    = m_Mass * GPhyscsUtils::CalculateInertiaTensor( m_Shape );
    m_InvInertiaTensor = m_InertiaTensor.GetInverse();

    m_InvMass          = GMath::One() / m_Mass;
}