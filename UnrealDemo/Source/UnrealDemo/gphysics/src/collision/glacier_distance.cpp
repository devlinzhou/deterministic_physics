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

#include "glacier_distance.h"

GVector3 GDistance::ClosestPointTriangle(const GVector3& p, const GVector3& a, const GVector3& b, const GVector3& c)
{
    GVector3 ab = b - a;
    GVector3 ac = c - a;
    GVector3 ap = p - a;
    f32 d1 = GVector3::DotProduct(ab, ap);
    f32 d2 = GVector3::DotProduct(ac, ap);
    if (d1 <= GMath::Zero() && d2 <= GMath::Zero())
        return a; // barycentric coordinates (1,0,0)

    // Check if P in vertex region outside B
    GVector3 bp = p - b;
    f32 d3 = GVector3::DotProduct(ab, bp);
    f32 d4 = GVector3::DotProduct(ac, bp);
    if (d3 >= GMath::Zero() && d4 <= d3)
        return b;// barycentric ccordinates (0,1,0)

    //Check if P in edge region of AB, if so return projecrion of P onto AB
    f32 vc = d1 * d4 - d3 * d2;
    if (vc <= GMath::Zero() && d1 >= GMath::Zero() && d3 <= GMath::Zero()) {
        f32 v = d1 / (d1 - d3);
        return a + v * ab; // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    GVector3 cp = p - c;
    f32 d5 = GVector3::DotProduct(ab, cp);
    f32 d6 = GVector3::DotProduct(ac, cp);
    if (d6 >= GMath::Zero() && d5 <= d6)
        return c; // barycentric coordinates (0,0,1)

    // Check if P in edge region of AC, if so return projection of P onto AC
    f32 vb = d5 * d2 - d1 * d6;
    if (vb <= GMath::Zero() && d2 >= GMath::Zero() && d6 <= GMath::Zero()) {
        f32 w = d2 / (d2 - d6);
        return a + w * ac; // barycentric coordinates (1-w,0,w)
    }
    // Check if P in edge region of BC, if so return projection of P onto BC
    f32 va = d3 * d6 - d5 * d4;
    if (va <= GMath::Zero() && (d4 - d3) >= GMath::Zero() && (d5 - d6) >= GMath::Zero()) {
        f32 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b); // barycentric coordinates (0,1-w,w)
    }
    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    f32 denom = GMath::One() / (va + vb + vc);
    f32 v = vb * denom;
    f32 w = vc * denom;
    return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = 1.0f-v-w
}




