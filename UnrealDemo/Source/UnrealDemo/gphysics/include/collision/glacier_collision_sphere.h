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

class GCollision_Sphere
{
public:

    static inline bool Sphere_Sphere(const GVector3& c0, const f32 r0, const GVector3& c1, const f32 r1)
    {
        GVector3 d = c0 - c1;
        f32 len2 = d.SizeSquare();
        return len2 < (r1+r0) * (r1+r0);
    }

    static inline bool Sphere_Point(const GVector3& c0, const f32 r0, const GVector3& Point) { return Sphere_Sphere(c0, r0, Point, GMath::Zero()); }
    static inline bool Sphere_Sphere( const GVector3& center0, const f32 radius0, const GVector3& center1, const f32 radius1, GVector3& finalCenter, GVector3* pOutNormal, bool bInner = false)
    {
        f32 r = radius1 + (bInner ? -radius0 : radius0);
        f32 r2 = r * r;

        GVector3 d = center0 - center1;
        f32 len2 = d.SizeSquare();

        if (bInner ^ (len2 >= r2)) //|| (bInner && len2 <= r2)
        {
            return false;
        }
        else
        {
            if (len2 < GMath::Epsilon())
            {
                finalCenter = center1 + GVector3(r, GMath::Zero(), GMath::Zero());
                if (pOutNormal != nullptr)
                {
                    *pOutNormal = GVector3(GMath::One(), GMath::Zero(), GMath::Zero());
                }
                return true;
            }
            else
            {
                f32 invlen = GMath::InvSqrt(len2);

                GVector3 TNormal = d * invlen;

                if (pOutNormal != nullptr)
                {
                    *pOutNormal = bInner ? -TNormal : TNormal;
                }

                finalCenter = center1 + TNormal * r;
                return true;
            }
        }
    }

    static inline bool LineSphere(const GVector3& lPoint1, const GVector3& lPoint2, const GVector3& dCenter, f32 dRadius)
    {
        const GVector3& TDir = lPoint2 - lPoint1;

        f32 uLength = TDir.SizeSquare();

        if (uLength == GMath::Zero())
        {
            if ((dCenter - lPoint1).SizeSquare()> dRadius * dRadius)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            f32 T = GVector3::DotProduct(dCenter - lPoint1, TDir) / uLength;

            GVector3 VNearest = lPoint1 + GMath::Clamp(T, GMath::Zero(), GMath::One()) * TDir;

            if ((dCenter - VNearest).SizeSquare() > dRadius * dRadius)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    static inline bool Sphere_Box(const GVector3& c, f32 r, const GVector3& p, const GVector3& h)
    {
        GVector3 v = (p - c).Abs();
        GVector3 u = GVector3::Max(v - h, GVector3::Zero());

        return GVector3::DotProduct(u, u) <= r * r;
    }

    static bool Sphere_Box(const GVector3& center, const f32 radius, const GVector3& boxCenter, const GVector3& boxExtents, GVector3& finalCenter, GVector3* pOutNormal);
    static bool Sphere_Capsule( const GVector3& sphereCenter, const f32 sphereRadius, const GVector3& capsuleP0, const GVector3& capsuleP1, const f32 Radius0, const f32 Radius1, GVector3& result, GVector3* pOutNormal);



};


