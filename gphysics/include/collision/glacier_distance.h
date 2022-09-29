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

    static inline void ClosestPtPointLine(const GVector3& point, const GVector3& sP0, const GVector3& sP1, f32& t, GVector3& d)
    {
        GVector3 seg = sP1 - sP0;

        t = GVector3::DotProduct(point - sP0, seg);

        f32 denom = GVector3::DotProduct(seg, seg);

        t = t / denom;
        d = sP0 + t * seg;
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

     static GVector3 ClosestPointTriangle(const GVector3& point, const GVector3& a, const GVector3& b, const GVector3& c);


};


