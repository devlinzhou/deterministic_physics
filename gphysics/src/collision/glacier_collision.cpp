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


#include "glacier_collision.h"


bool GCollision::Sphere_Box(const GVector3& center, const f32 radius, const GVector3& boxCenter, const GVector3& boxExtents, GVector3& finalCenter, GVector3* pOutNormal)
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



