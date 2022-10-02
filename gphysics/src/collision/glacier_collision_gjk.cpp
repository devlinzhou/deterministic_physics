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


bool GCollision_GJK::GJKTest(
    const GShapeConvexBase& ShapA, const GTransform_QT& TransformShapA,
    const GShapeConvexBase& ShapB, const GTransform_QT& TransformShapB)
{
    // use shapA local system

    GTransform_QT WorlToLocal_A     = TransformShapA.GetInverse();
    GTransform_QT LocalBToLocalA    = TransformShapB * WorlToLocal_A;
    GTransform_QT LocalAToLocalB    = LocalBToLocalA.GetInverse();


    GVector3 DirectionA = -TransformShapB.m_Translation.GetNormalize();
    GVector3 DirectionB = LocalAToLocalB.TransformNormal( -DirectionA);


    GVector3 SupportPos0 = ShapA.GetSupportLocalPos( DirectionA ) + LocalBToLocalA.TransformPosition( ShapA.GetSupportLocalPos( DirectionB ) );

    DirectionA = (-SupportPos0).GetNormalize();
    DirectionB = LocalAToLocalB.TransformNormal( -DirectionA);

    GVector3 SupportPos1 = ShapA.GetSupportLocalPos( DirectionA ) + LocalBToLocalA.TransformPosition( ShapA.GetSupportLocalPos( DirectionB ) );

    GVector3 closeSeg =  GDistance::ClosestPtPointSegment( GVector3::Zero(),SupportPos0, SupportPos1);

    DirectionA = (-closeSeg).GetNormalize();
    DirectionB = LocalAToLocalB.TransformNormal(-DirectionA);

    GVector3 SupportPos2 = ShapA.GetSupportLocalPos( DirectionA ) + LocalBToLocalA.TransformPosition( ShapA.GetSupportLocalPos( DirectionB ) );

    GVector3 closeTri = GDistance::ClosestPointTriangle( GVector3::Zero(),SupportPos0, SupportPos1, SupportPos2 );
    DirectionA = (-closeTri).GetNormalize();
    DirectionB = LocalAToLocalB.TransformNormal(-DirectionA);

    GVector3 SupportPos3 = ShapA.GetSupportLocalPos( DirectionA ) + LocalBToLocalA.TransformPosition( ShapA.GetSupportLocalPos( DirectionB ) );

    GVector3 closeTetra = GDistance::ClosestPtPointTetrahedron( GVector3::Zero(),SupportPos0, SupportPos1, SupportPos2, SupportPos3 );
    DirectionA = (-closeTetra).GetNormalize();
    DirectionB = LocalAToLocalB.TransformNormal(-DirectionA);

    GVector3 SupportPos4 = ShapA.GetSupportLocalPos( DirectionA ) + LocalBToLocalA.TransformPosition( ShapA.GetSupportLocalPos( DirectionB ) );



    return false;
}

