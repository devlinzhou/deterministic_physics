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
#include "glacier_transform_qt.h"

class IGLacierDraw
{
public:

    virtual void DrawLine( const GVector3& V0, const GVector3& V1, uint32_t uColor ) = 0;

    virtual void DrawBox(const GTransform_QT& TTrans, const GVector3& LocalCenter, const GVector3& HalfSize, uint32_t TColor);

    virtual void DrawsSphere(const GTransform_QT& TTrans, f32 Radius, uint32_t TColor, int32_t nSeg);

    virtual void DrawPlane(const GTransform_QT& TTrans, const GVector3& PlaneNormal, f32 PlaneDis, f32 Size, uint32_t TColor);
//     {
//         FTransform Transform;
//         Transform.SetTranslation(Plane.GetNormal() * Plane.W);
//         Transform.SetRotation(FQuat::FindBetweenNormals(FVector(0, 0, 1), Plane.GetNormal()));
// 
//         auto P0 = Transform.TransformPosition(FVector(-1, -1, 0) * Size);
//         auto P1 = Transform.TransformPosition(FVector(1, -1, 0) * Size);
//         auto P2 = Transform.TransformPosition(FVector(1, 1, 0) * Size);
//         auto P3 = Transform.TransformPosition(FVector(-1, 1, 0) * Size);
// 
//         DrawLine(TTrans.TransformPosition(P0), TTrans.TransformPosition(P1), TColor);
//         DrawLine(TTrans.TransformPosition(P1), TTrans.TransformPosition(P2), TColor);
//         DrawLine(TTrans.TransformPosition(P2), TTrans.TransformPosition(P3), TColor);
//         DrawLine(TTrans.TransformPosition(P3), TTrans.TransformPosition(P0), TColor);
//         DrawLine(TTrans.TransformPosition(P0), TTrans.TransformPosition(P2), TColor);
//         DrawLine(TTrans.TransformPosition(P1), TTrans.TransformPosition(P3), TColor);
//         DrawLine(TTrans.TransformPosition(Transform.GetLocation()), TTrans.TransformPosition(Transform.GetLocation() + Plane.GetNormal() * Size * 0.5f), TColor);
//     }



};
