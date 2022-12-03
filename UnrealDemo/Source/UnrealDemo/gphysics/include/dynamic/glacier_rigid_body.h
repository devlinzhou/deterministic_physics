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
#include "glacier_physics_utils.h"

class GRigidBody : public GCObject
{
public:

    GRigidBody(uint32_t id, EShape TShape) : GCObject(id, TShape)
    {
        m_CollisionType         = CO_Rigid_Body;
        m_Gravity               = GVector3(GMath::Zero(), GMath::Zero(), -GMath::Makef32(9,8,10) );
        m_LinearVelocityMax     = GMath::Makef32(100,0,1);
		m_LinearDamping			= GMath::Makef32(0, 1, 10);
        m_AngularVelocityMax    = GMath::Makef32(100,0,1);
		m_AngularDamping		= GMath::Makef32(0, 1, 10);
        m_bDynamic              = false;
        m_Mass                  = GMath::One();
        m_InvMass               = GMath::One();
        m_LinearVelocity        = GVector3::Zero();
        m_AngularVelocity       = GVector3::Zero();
        m_density               = GMath::Makef32(100,0,1); // tenth of water's density
        m_ContactStaticDepth    = 0;
    }

    virtual bool IsDynamic() const override
    {
        return m_bDynamic;
    }

    void Tick_PreTransform( const f32 DetalTime );

	void AddDamping(const f32 DetalTime)
	{
		m_LinearVelocity	*= GMath::Pow(GMath::One() - m_LinearDamping, DetalTime);
		m_AngularVelocity	*= GMath::Pow(GMath::One() - m_AngularDamping, DetalTime);
	}

    void AddImpulse_World(const GVector3& VPos, const GVector3& VImpulse)
    {
        m_LinearVelocity += VImpulse * m_InvMass;

        GVector3 VPosLocal = VPos - GetMassCenterPos();

        GVector3 Torque = GVector3::CrossProduct(VPosLocal, VImpulse);

        GVector3 VDeltaAngular = m_InvInertiaTensorWorld.TransformVector(Torque);

        m_AngularVelocity += VDeltaAngular;
    }

    GVector3 GetWorldPosVelocity(const GVector3& VPos) const
    {
        GVector3 V1 = GVector3::CrossProduct(m_AngularVelocity, (VPos - m_Transform.m_Pos));

        return V1 + m_LinearVelocity;
    }

    f32 GetEquivalentMass( const GVector3& VWorldPos, const GVector3& VImpulse )
    {
        if( m_bDynamic )
        {
            GVector3 VLocalPos = VWorldPos - GetMassCenterPos();

            GVector3 Torque = GVector3::CrossProduct(VLocalPos, VImpulse);

            GVector3 VDeltaAngular = m_InvInertiaTensorWorld.TransformVector(Torque);

            GVector3 V1 = GVector3::CrossProduct(VDeltaAngular, VLocalPos);

            V1 = GPhyscsUtils::Project_Parallel(VImpulse, V1);

            GVector3 V2 = VImpulse * m_InvMass;

            GVector3 VDeltaVelocity = V1 + V2;

            f32 invEquivalentMass = VDeltaVelocity.SizeSquare() / VImpulse.SizeSquare();

            return GMath::InvSqrt(invEquivalentMass);
        }
        else
        {
            return GMath::Zero();
        }
    }

    f32 GetEnergy()
    {
        if( m_bDynamic) 
        {
            f32 fLinear =  m_Mass * m_LinearVelocity.SizeSquare();

            GVector3 VLocalAngular = m_Transform.m_Rot.UnRotateVector(m_AngularVelocity);

            f32 fAngular = (m_InertiaTensor.TransformVector(VLocalAngular).Size()) * VLocalAngular.Size();

            return (fLinear + fAngular ) * GMath::Half();
        }
        else
        {
            return GMath::Zero();
        }
    }

    f32 GetMomentum()
    {
        if( m_bDynamic )
        {
            f32 fLinear = m_Mass * m_LinearVelocity.Size();

            GVector3 VLocalAngular = m_Transform.m_Rot.UnRotateVector(m_AngularVelocity);

            f32 fAngular = m_InertiaTensor.TransformVector(VLocalAngular).Size();

            return fLinear + fAngular;
        }
        else
        {
            return GMath::Zero();
        }
    }


    GVector3 GetLinearMomentum()
    {
        if (m_bDynamic)
        {
            return m_Mass * m_LinearVelocity;
        }
        else
        {
            return GVector3::Zero();
        }
    }

    GVector3 GetAngularMomentum()
    {
        if (m_bDynamic)
        {
            GVector3 VLocalAngular = m_Transform.m_Rot.UnRotateVector(m_AngularVelocity);

            GVector3 VlocalMomentum = m_InertiaTensor.TransformVector(VLocalAngular);

            return m_Transform.m_Rot.RotateVector(VlocalMomentum);
        }
        else
        {
            return GVector3::Zero();
        }
    }

    inline GVector3 GetMassCenterPos() const
    {
        return m_Transform.m_Pos;
    }

    void CalculateInertiaTensor();

public:

    bool            m_bDynamic;
    GVector3        m_Gravity;
    f32             m_density;

    GTransform_QT   m_Transform_Solve;

    f32             m_Mass;
    f32             m_InvMass;

    GMatrix3        m_InertiaTensor;           // local
    GMatrix3        m_InvInertiaTensor;
	GMatrix3		m_InvInertiaTensorWorld;

    GVector3        m_LinearVelocity;
    f32             m_LinearDamping;
    f32             m_LinearVelocityMax;

    GVector3        m_AngularVelocity;
    f32             m_AngularDamping;
    f32             m_AngularVelocityMax;
};