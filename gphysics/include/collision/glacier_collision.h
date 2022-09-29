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

class GCollision
{
public:

    static inline bool Sphere_Sphere(const GVector3& c0, const f32 r0, const GVector3& c1, const f32 r1)
    {
        GVector3 d = c0 - c1;
        f32 len2 = d.SizeSquare();
        return len2 < (r1+r0) * (r1+r0);
    }

    static inline bool Sphere_Point(const GVector3& c0, const f32 r0, const GVector3& Point )
    {
        return Sphere_Sphere( c0, r0, Point, GMath::Zero());
    }



     void Test();
};


