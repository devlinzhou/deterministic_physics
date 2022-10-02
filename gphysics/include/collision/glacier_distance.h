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

class GDistance
{
public:

    static inline GVector3 closestPtPointSphere(const GVector3& p, const GVector3& a, f32 radius)
    {
        return (p - a).GetNormalize() * radius + a;
    }

    static inline GVector3 closestPtPointPlane(const GVector3&  p, const GVector3&  PlaneNormal, const GVector3&  planePos)
    {
        return p - PlaneNormal * GVector3::DotProduct(p - planePos, PlaneNormal);
    }

    static inline GVector3 closestPtPointLine(const GVector3& p, const GVector3& Pos, const GVector3& Vdir)
    {
        return Pos + Vdir * GVector3::DotProduct(Vdir, p - Pos);
    }

    static inline void ClosestPtPointLine(const GVector3& point, const GVector3& sP0, const GVector3& sP1, f32& t, GVector3& d)
    {
        GVector3 seg = sP1 - sP0;

        t = GVector3::DotProduct(point - sP0, seg);

        f32 denom = GVector3::DotProduct(seg, seg);

        t = t / denom;
        d = sP0 + t * seg;
    }

    static inline GVector3 ClosestPtPointSegment(const GVector3& point, const GVector3& sP0, const GVector3& sP1 )
    {
        GVector3 seg = sP1 - sP0;

        f32 t = GVector3::DotProduct(point - sP0, seg);
        if (t <= GMath::Zero())
        {
            return sP0;
        }
        else
        {
            f32 denom = GVector3::DotProduct(seg, seg);
            if (t >= denom)
            {
                return sP1;
            }
            else
            {
                t = t / denom;
                return sP0 + t * seg;
            }
        }
    }

    static inline void ClosestPtPointSegment(const GVector3& point, const GVector3& sP0, const GVector3& sP1, f32& t, GVector3& d)
    {
        GVector3 seg = sP1 - sP0;

        t = GVector3::DotProduct(point - sP0, seg);
        if (t <= GMath::Zero())
        {
            t = GMath::Zero();
            d = sP0;
        }
        else
        {
            f32 denom = GVector3::DotProduct(seg, seg);
            if (t >= denom)
            {
                t = GMath::One();
                d = sP1;
            }
            else
            {
                t = t / denom;
                d = sP0 + t * seg;
            }
        }
    }

    static inline GVector3 closestPtPointAABB(const GVector3& p, const GVector3& VHalfSize)
    {
        return GVector3(
            GMath::Clamp(p.x, -VHalfSize.x, VHalfSize.x),
            GMath::Clamp(p.y, -VHalfSize.y, VHalfSize.y),
            GMath::Clamp(p.z, -VHalfSize.z, VHalfSize.z));
    }

    static GVector3 ClosestPointTriangle(const GVector3& point, const GVector3& a, const GVector3& b, const GVector3& c);

    static inline int pointOutsideOfPlane(const GVector3& p, const GVector3& a, const GVector3& b, const GVector3& c, const GVector3& d)
    {
        GVector3 normal = GVector3::CrossProduct(b - a, c - a);

        f32 signp = GVector3::DotProduct(p - a, normal); // [AP AB AC]
        f32 signd = GVector3::DotProduct(d - a, normal); // [AD AB AC]

        if (signd * signd < GMath::Epsilon())
        {
            return -1;
        }
        // Points on opposite sides if expression signs are opposite
        return signp * signd < GMath::Zero() ?  1 : 0;
    }


    static inline GVector3 ClosestPtPointTetrahedron(const GVector3& p, const GVector3& a, const GVector3& b, const GVector3& c, const GVector3& d)
    {
         int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
         int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
         int pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
         int pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);

         if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
         {
             return p;
         }

         if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
         {
             return p;
         }

         // If point outside face abc then compute closest point on abc
         if (pointOutsideABC != 0)
         {
             return ClosestPointTriangle(p, a, b, c);
         }


         // Repeat test for face acd
         if (pointOutsideACD != 0)
         {
             return ClosestPointTriangle(p, a, c, d);

         }
         // Repeat test for face adb


         if (pointOutsideADB != 0)
         {
             return ClosestPointTriangle(p, a, d, b);
         }
         // Repeat test for face bdc


         if (pointOutsideBDC != 0)
         {
             return ClosestPointTriangle(p, b, d, c);
         }

         return p;
     }

};



