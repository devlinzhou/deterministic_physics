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

#include "glacier_contact.h"
#include "glacier_rigid_body.h"



void GCollisionContact::AddContactPoint(const GVector3& PosWorld, const GVector3& Normal, f32 depth, uint32_t faceid)
{
    if(0<= m_nPointCount && m_nPointCount < (MaxPoint-1) )
    {
        GManifoldPoint& TPoint = m_Point[m_nPointCount++];

        TPoint.m_PosWorld   = PosWorld;
        TPoint.m_Normal     = Normal;
        TPoint.m_depth      = depth;
        TPoint.m_FaceIndex  = faceid;
    }
}



GVector3 GBroadPhasePair::GetWorldRelativeB(const GVector3& VPos) const
{
    GVector3 V1 = GVector3::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body && ((GRigidBody*)pObjectA)->m_bDynamic)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        V1 = pRA->GetWorldPosVelocity(VPos);
    }

    GVector3 V2 = GVector3::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body && ((GRigidBody*)pObjectB)->m_bDynamic)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        V2 = pRB->GetWorldPosVelocity(VPos);
    }

    return V1 - V2;
}


f32 GBroadPhasePair::GetContactPairEnergy_World() const
{
    f32 fEnergyA = GMath::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        fEnergyA = pRA->GetEnergy();
    }

    f32 fEnergyB = GMath::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        fEnergyB = pRB->GetEnergy();
    }

    return fEnergyA + fEnergyB;
}

f32 GBroadPhasePair::GetContactPairEnergy_Relative( const GVector3& CenterVelocity ) const
{

    f32 fEnergyA = GMath::Zero();
    f32 fEnergyB = GMath::Zero();
   /* if (pObjectA->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;

        f32 fLinear = GMath::Half() * pRA->m_Mass * pRA->m_LinearVelocity.SizeSquare();

        GVector3 VLocalAngular = pRA->m_Transform.m_Rot.UnRotateVector(pRA->m_AngularVelocity);

        f32 fAngular = GMath::Half() * (pRA->m_InertiaTensor.TransformVector(VLocalAngular).Size()) * VLocalAngular.Size();

        fEnergyA = fLinear + fAngular;
    }

    f32 fEnergyB = GMath::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        fEnergyB = pRB->GetEnergy();
    }*/

    return fEnergyA + fEnergyB;

}


f32 GBroadPhasePair::GetContactPairMomentum_world() const
{
    f32 fMomentumA = GMath::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        fMomentumA = pRA->GetMomentum();
    }

    f32 fMomentumB = GMath::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        fMomentumB = pRB->GetMomentum();
    }

    return fMomentumA + fMomentumB;
}

void GBroadPhasePair::SeparatePair( GRigidBody* pRA, GRigidBody* pRB, bool bSwap )
{
    int32_t nPointCount = PairContact.GetPointCount();
    if(nPointCount <= 0)
        return;

    GVector3 VSep = GVector3::Zero();

    if( nPointCount == 1 )
    {
        GManifoldPoint TPoint = PairContact.m_Point[0];

        GVector3 VNormal = bSwap ? -TPoint.m_Normal : TPoint.m_Normal;

        VSep = VNormal * TPoint.m_depth;
    }
    else
    {
        GVector3 VNormal = GVector3::Zero();

        for( int32_t i = 0; i < nPointCount; ++i )
        {
            GManifoldPoint TPoint = PairContact.m_Point[i];

            VNormal += bSwap ? -TPoint.m_Normal : TPoint.m_Normal;
        }
    
        VNormal.Normalize();

        f32 fDis = GMath::Zero();
        for (int32_t i = 0; i < nPointCount; ++i)
        {
            GManifoldPoint TPoint = PairContact.m_Point[i];

            GVector3 VTNormal = bSwap ? -TPoint.m_Normal : TPoint.m_Normal;

            f32 fCos = GVector3::DotProduct( VNormal, VTNormal );

            f32 fT2 = GMath::Max(fCos, GMath::Makef32(0,1,10));

            fT2 = TPoint.m_depth / fT2;

            if( fT2 < fDis ) 
            {
                fDis = fT2;
            }
        }

        VSep = VNormal * fDis;
    }

    if (pRA != nullptr && pRB != nullptr)
    {
        f32 fAlphaA = pRB->m_Mass / (pRA->m_Mass + pRB->m_Mass);
        f32 fAlphaB = GMath::One() - fAlphaA;

        pRA->m_Transform_Solve.m_Pos = pRA->m_Transform.m_Pos - VSep * fAlphaA;

        pRB->m_Transform_Solve.m_Pos = pRB->m_Transform.m_Pos + VSep * fAlphaB;

        pRA->m_Transform.m_Pos -= VSep * fAlphaA;
        pRB->m_Transform.m_Pos += VSep * fAlphaB;
    }
    else if (pRA != nullptr)
    {
        pRA->m_Transform_Solve.m_Pos = pRA->m_Transform.m_Pos - VSep;
        pRA->m_Transform.m_Pos -= VSep;
    }
    else if (pRB != nullptr)
    {
        pRB->m_Transform_Solve.m_Pos = pRB->m_Transform.m_Pos + VSep;
        pRB->m_Transform.m_Pos += VSep;
    }
}
