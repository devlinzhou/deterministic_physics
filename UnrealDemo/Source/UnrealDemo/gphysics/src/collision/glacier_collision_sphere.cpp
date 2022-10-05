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

#include "glacier_collision_sphere.h"


bool GCollision_Sphere::Sphere_Box(const GVector3& center, const f32 radius, const GVector3& boxCenter, const GVector3& boxExtents, GVector3& finalCenter, GVector3* pOutNormal)
{ 
    GVector3 DeltaV = center - boxCenter;

    GVector3 NearestExt = GVector3(
        DeltaV.x < GMath::Zero() ? -boxExtents.x : boxExtents.x,
        DeltaV.y < GMath::Zero() ? -boxExtents.y : boxExtents.y,
        DeltaV.z < GMath::Zero() ? -boxExtents.z : boxExtents.z);


    GVector3 DeltaU = GVector3(
        (-boxExtents.x < DeltaV.x&& DeltaV.x < boxExtents.x) ? GMath::Zero() : DeltaV.x - NearestExt.x,
        (-boxExtents.y < DeltaV.y&& DeltaV.y < boxExtents.y) ? GMath::Zero() : DeltaV.y - NearestExt.y,
        (-boxExtents.z < DeltaV.z&& DeltaV.z < boxExtents.z) ? GMath::Zero() : DeltaV.z - NearestExt.z);

    f32 lenthSqr = DeltaU.SizeSquare();

    if (lenthSqr <= radius * radius)
    {
        if (lenthSqr > GMath::Epsilon())
        {
            GVector3 TNormal = DeltaU * GMath::InvSqrt(lenthSqr);
            if (pOutNormal != nullptr)
            {
                *pOutNormal = TNormal;
            }

            //finalCenter = center + DeltaU * (FMath::InvSqrt(lenthSqr) * radius - 1.f);

            finalCenter = center + TNormal * radius - DeltaU;
        }
        else
        {
            GVector3 ABSV = (DeltaV - NearestExt).Abs();

            if (ABSV.x < ABSV.y && ABSV.x < ABSV.z)
            {
                finalCenter = GVector3(boxCenter.x + (DeltaV.x < GMath::Zero() ? -boxExtents.x - radius : boxExtents.x + radius), center.y, center.z);
                if (pOutNormal != nullptr)
                {
                    *pOutNormal = GVector3(DeltaV.x < GMath::Zero() ? -GMath::One() : GMath::One(), GMath::Zero(), GMath::Zero());
                }
            }
            else if (ABSV.y < ABSV.z)
            {
                finalCenter = GVector3(center.x, boxCenter.y + (DeltaV.y < GMath::Zero() ? -boxExtents.y - radius : boxExtents.y + radius), center.z);
                if (pOutNormal != nullptr)
                {
                    *pOutNormal = GVector3(GMath::Zero(), DeltaV.y < GMath::Zero() ? -GMath::One() : GMath::One(), GMath::Zero());
                }
            }
            else
            {
                finalCenter = GVector3(center.x, center.y, boxCenter.z + (DeltaV.z < GMath::Zero() ? -boxExtents.z - radius : boxExtents.z + radius));
                if (pOutNormal != nullptr)
                {
                    *pOutNormal = GVector3(GMath::Zero(), GMath::Zero(), DeltaV.z < GMath::Zero() ? -GMath::One() : GMath::One());
                }
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool GCollision_Sphere::Sphere_Capsule(
    const GVector3& sphereCenter,
    const f32       sphereRadius,
    const GVector3& capsuleP0,
    const GVector3& capsuleP1,
    const f32       Radius0,
    const f32       Radius1,
    GVector3& result,
    GVector3* pOutNormal)
{
    f32 deltaRadius = Radius1 - Radius0;
    f32 distance_sqr = GVector3::DistanceSquare(capsuleP0, capsuleP1);

    if (distance_sqr < GMath::Epsilon())
    {
        return Sphere_Sphere(sphereCenter, sphereRadius, capsuleP1, GMath::Max(Radius0, Radius1), result, pOutNormal);
    }
    else if ((Radius0 + distance_sqr) <= Radius1)
    {
        return Sphere_Sphere(sphereCenter, sphereRadius, capsuleP1, Radius1, result, pOutNormal);
    }
    else if ((Radius1 + distance_sqr) <= Radius0)
    {
        return Sphere_Sphere(sphereCenter, sphereRadius, capsuleP0, Radius0, result, pOutNormal);
    }
    else
    {
        GVector3 seg = capsuleP1 - capsuleP0;
        f32   t = GVector3::DotProduct(sphereCenter - capsuleP0, seg) / distance_sqr;

        f32   inv_dis = GMath::InvSqrt(distance_sqr);
        f32   fSin = (Radius1 - Radius0) * inv_dis;
        f32   fCos = GMath::Sqrt(GMath::One() - fSin * fSin);
        f32   fTan = fSin / fCos;

        f32 factor = fTan * inv_dis;

        GVector3 VOrigNearest = capsuleP0 + (capsuleP1 - capsuleP0) * t;
        f32   fOriDis = GVector3::Distance(VOrigNearest, sphereCenter);

        f32 Line_t = t + fOriDis * factor;

        f32 Current_t = GMath::Clamp(Line_t, GMath::Zero(), GMath::One());

        GVector3 VNearest = capsuleP0 + (capsuleP1 - capsuleP0) * Current_t;
        f32   LerpRadius = Radius0 + (Radius1 - Radius0) * Current_t;

        f32 fDistance_sqr = GVector3::DistanceSquare(VNearest, sphereCenter);

        if (fDistance_sqr < (LerpRadius + sphereRadius) * (LerpRadius + sphereRadius))
        {
            if (fDistance_sqr < GMath::Epsilon())
            {
                result = VNearest + GVector3(GMath::One(), GMath::Zero(), GMath::Zero()) * (LerpRadius + sphereRadius);
                if (pOutNormal)
                    *pOutNormal = GVector3(GMath::One(), GMath::Zero(), GMath::Zero());
            }
            else
            {
                GVector3 VNor = (sphereCenter - VNearest) * GMath::InvSqrt(fDistance_sqr);
                result = VNearest + VNor * (LerpRadius + sphereRadius);
                if (pOutNormal)
                    *pOutNormal = VNor;
            }
            return true;
        }
    }
    return false;
}



