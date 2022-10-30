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
#include "glacier_collision_shape.h"
#include "glacier_debug_draw.h"

class IGlacierDraw;


class GSimplex
{
public:


    GVector3 m_Points[4];
};


class GCollision_GJK
{
public:

    static const f32 s_DefaultEpsilon;// = GMath::Makef32(0,1,1000);

    static bool GJKTest( 
        const GShapeConvexBase& ShapA,
        const GTransform_QT&    TransformA,
        const GShapeConvexBase& ShapB,
        const GTransform_QT&    TransformB,
        IGlacierDraw*           pDebugDraw = nullptr );


    template <typename ShapTypeA, typename ShapTypeB>
    static inline GVector3 s_GetSupportPos(
        const GVector3& DirectionA,
        const ShapTypeA& ShapA,
        const ShapTypeB& ShapB,
        const GTransform_QT& LocalBToLocalA,
        const GTransform_QT& LocalAToLocalB,
        const GTransform_QT& TransformShapA,
        const GTransform_QT& TransformShapB,
        IGlacierDraw* pDebugDraw,
        uint32_t                uColor)
    {
        GVector3 DirectionB = LocalAToLocalB.TransformNormal(-DirectionA);
        GVector3 PosA = ShapA.GetSupportLocalPos(DirectionA);
        GVector3 PosB = ShapB.GetSupportLocalPos(DirectionB);

        if (pDebugDraw != nullptr)
        {

            f32 ff = GMath::FromFloat(0.02f);
            f32 f2 = GMath::FromFloat(0.04f);

            pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), TransformShapA.TransformPosition(PosA)), ff, uColor, 12);
            pDebugDraw->DrawSphere(GTransform_QT(GQuaternion::Identity(), TransformShapB.TransformPosition(PosB)), ff, uColor, 12);
            pDebugDraw->DrawLine(TransformShapA.m_Translation, TransformShapA.m_Translation + TransformShapA.TransformNormal(DirectionA) * f2, uColor);
            pDebugDraw->DrawLine(TransformShapB.m_Translation, TransformShapB.m_Translation + TransformShapB.TransformNormal(DirectionB) * f2, uColor);
        }

        return PosA - LocalBToLocalA.TransformPosition(PosB);
    }

   /* template <typename ShapTypeA, typename ShapTypeB>
    static inline f32 GJKDistance(
        const ShapTypeA& ShapA, const GTransform_QT& TransformA,
        const ShapTypeB& ShapB, const GTransform_QT& TransformB,
        f32 Epsilon = s_DefaultEpsilon,
        IGlacierDraw* pDebugDraw = nullptr )
    {
        GTransform_QT WorlToLocal_A = TransformA.GetInverse();
        GTransform_QT LocalBToLocalA = TransformB * WorlToLocal_A;
        GTransform_QT LocalAToLocalB = LocalBToLocalA.GetInverse();


        GVector3 DirectionA = LocalBToLocalA.TransformPosition(GVector3::Zero()).GetNormalize();
        GVector3 SupportPos0 = s_GetSupportPos(DirectionA, ShapA, ShapB, LocalBToLocalA, LocalAToLocalB, TransformA, TransformB, pDebugDraw, 0x00FF0000);

        f32 fDistance = SupportPos0.Size();
        if (f < GMath::Epsilon())
        {
            return fDistance;
        }

        while ( fDistance > Epsilon )
        {
            f32 fDotTest = GVector3::DotProduct( SupportPos0 );
            if (fDotTest > GMath::Zero())
            {
                return fDotTest; // not accurate
            }







            f32 fDistance = SupportPos0.Size();
            if (f < GMath::Epsilon())
            {
                return fDistance;
            }
        }



        return fDistance;
    
    }*/

};


