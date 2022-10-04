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


#include "glacier_collision_gjk.h"
#include "glacier_distance.h"
#include "glacier_debug_draw.h"

static inline GVector3 s_GetSupportPos(
    const GVector3&         DirectionA,
    const GShapeConvexBase& ShapA,
    const GShapeConvexBase& ShapB, 
    const GTransform_QT&    LocalBToLocalA,
    const GTransform_QT&    LocalAToLocalB,
    const GTransform_QT&    TransformShapA,
    const GTransform_QT&    TransformShapB,
    IGLacierDraw*           pDebugDraw,
    uint32_t                uColor)
{
    GVector3 DirectionB = LocalAToLocalB.TransformNormal( -DirectionA );
    GVector3 PosA       = ShapA.GetSupportLocalPos( DirectionA );
    GVector3 PosB       = ShapB.GetSupportLocalPos( DirectionB );

    if( pDebugDraw != nullptr )
    {
        pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), TransformShapA.TransformPosition(PosA)), f32(0.02f), uColor, 12);
        pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), TransformShapB.TransformPosition(PosB)), f32(0.02f), uColor, 12);
        pDebugDraw->DrawLine(TransformShapA.m_Translation, TransformShapA.m_Translation + TransformShapA.TransformNormal( DirectionA) * f32(0.4f), uColor);
        pDebugDraw->DrawLine(TransformShapB.m_Translation, TransformShapB.m_Translation + TransformShapB.TransformNormal( DirectionB) * f32(0.4f), uColor);
    }

    return PosA - LocalBToLocalA.TransformPosition(PosB);
}

bool GCollision_GJK::GJKTest(
    const GShapeConvexBase&     ShapA,
    const GTransform_QT&        TransformA,
    const GShapeConvexBase&     ShapB, 
    const GTransform_QT&        TransformB,
    IGLacierDraw*               pDebugDraw)
{
    // use shapA local system

    GTransform_QT WorlToLocal_A     = TransformA.GetInverse();
    GTransform_QT LocalBToLocalA    = TransformB * WorlToLocal_A;
    GTransform_QT LocalAToLocalB    = LocalBToLocalA.GetInverse();

    GVector3 DirectionA     = LocalBToLocalA.TransformPosition(GVector3::Zero()).GetNormalize();
    GVector3 SupportPos0    = s_GetSupportPos(DirectionA, ShapA, ShapB, LocalBToLocalA, LocalAToLocalB, TransformA, TransformB, pDebugDraw, 0x00FF0000);
        
    if( GVector3::DotProduct( DirectionA, SupportPos0 ) < f32::Zero() )
    {
        return false;
    }


    DirectionA = (-SupportPos0).GetNormalize();
    GVector3 SupportPos1 = s_GetSupportPos(DirectionA, ShapA, ShapB, LocalBToLocalA, LocalAToLocalB, TransformA, TransformB, pDebugDraw, 0x0000FFFF);

    if (GVector3::DotProduct(DirectionA, SupportPos1) < f32::Zero())
    {
        return false;
    }

    GVector3 closeSeg =  GDistance::ClosestPtPointSegment( GVector3::Zero(),SupportPos0, SupportPos1);
    DirectionA = (-closeSeg).GetNormalize();

    GVector3 SupportPos2 = s_GetSupportPos(DirectionA, ShapA, ShapB, LocalBToLocalA, LocalAToLocalB, TransformA, TransformB, pDebugDraw, 0x000000FF);

    if (GVector3::DotProduct(DirectionA, SupportPos2) < f32::Zero())
    {
        return false;
    }

    GVector3 closeTri = GDistance::ClosestPointTriangle( GVector3::Zero(),SupportPos0, SupportPos1, SupportPos2 );
    DirectionA = (-closeTri).GetNormalize();
    GVector3 SupportPos3 = s_GetSupportPos(DirectionA, ShapA, ShapB, LocalBToLocalA, LocalAToLocalB, TransformA, TransformB, pDebugDraw, 0x00000000);

    if (GVector3::DotProduct(DirectionA, SupportPos3) < f32::Zero())
    {
        return false;
    }

    GVector3 closeTetra = GDistance::ClosestPtPointTetrahedron( GVector3::Zero(),SupportPos0, SupportPos1, SupportPos2, SupportPos3 );
    DirectionA = (-closeTetra).GetNormalize();

   // GVector3 SupportPos4 = s_GetSupportPos(DirectionA, ShapA, ShapB, LocalBToLocalA, LocalAToLocalB, TransformShapA, TransformShapB, pDebugDraw, 0x00FFFFFF);

    if(pDebugDraw!= nullptr)
    {

        GVector3 V0 = (SupportPos0);
        GVector3 V1 = (SupportPos1);
        GVector3 V2 = (SupportPos2);
        GVector3 V3 = (SupportPos3);

        //GVector3 V4 = TransformShapA.TransformPosition(SupportPos4);

        pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), GVector3::Zero()), f32(0.02f), 0x00FFFFFF, 12);
        pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), V0), f32(0.02f), 0x00FF0000, 12);
        pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), V1), f32(0.02f), 0x0000FF00, 12);
        pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), V2), f32(0.02f), 0x000000FF, 12);
        pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), V3), f32(0.02f), 0x00000000, 12);

         pDebugDraw->DrawLine(V0, V1, 0xFFFFFFFF);
         pDebugDraw->DrawLine(V1, V2, 0xFFFFFFFF);
         pDebugDraw->DrawLine(V2, V0, 0xFFFFFFFF);

         pDebugDraw->DrawLine(V0, V3, 0xFFFFFFFF);
         pDebugDraw->DrawLine(V1, V3, 0xFFFFFFFF);
         pDebugDraw->DrawLine(V2, V3, 0xFFFFFFFF);

    }



    return true;
}


