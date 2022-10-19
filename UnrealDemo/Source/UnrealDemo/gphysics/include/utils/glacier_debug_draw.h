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

#include "glacier_color.h"
#include "glacier_math.h"
class GVector3;
class GTransform_QT;
class GPlane;
class IGlacierDraw
{
public:

    virtual void DrawLine(const GVector3& V0, const GVector3& V1, GColor uColor) = 0;
    virtual void DrawBox(const GTransform_QT& TTrans, const GVector3& LocalCenter, const GVector3& HalfSize, GColor TColor);
    virtual void DrawSphere(const GTransform_QT& TTrans, f32 Radius, GColor TColor, int32_t nSeg);
    virtual void DrawCapsule(const GTransform_QT& TTrans, f32 Radius, f32 HalfHeight, GColor TColor, int nSeg = 18);
    virtual void DrawCylinder(const GTransform_QT& TTrans, f32 Radius, f32 HalfHeight, GColor TColor, int nSeg = 18);
    virtual void DrawPlane(const GTransform_QT& TTrans, const GPlane& TPlane, f32 Size, GColor TColor);
    virtual void DrawArrow(const GVector3& V0, const GVector3& VDirection, f32 Size, GColor TColor);




};
