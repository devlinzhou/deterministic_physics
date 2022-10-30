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

#include "glacier_collision_shape.h"


GAABB GCollisionShape::GetLocalBox() const
{
    switch (ShapType)
    {
    case EShape_ConvexBase:
    {

    }
        break;
    case EShape_Sphere:
    {
        return GAABB( GVector3(-GetRaiuds()), GVector3(GetRaiuds()) );
    }
        break;
    case EShape_Box:
    {
        return GAABB( -GetHalfExtern(), GetHalfExtern() );
    }
        break;
    case EShape_Capsule:
        break;
    case EShape_Cylinder:
        break;
    case EShape_ConvexHull:
        break;
    case EShape_ConcaveBase:
        break;
    case EShape_Plane:
        break;
    case EShape_HightField:
        break;
    case EShape_TriangleMesh:
        break;
    case EShape_Max:
        break;
    default:
        break;
    }

    return GAABB(GVector3::Zero());
}
