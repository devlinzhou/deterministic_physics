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
#include "glacier_physics_utils.h"
#include "glacier_plane.h"
#include "glacier_collision_shape.h"
#include "glacier_debug_draw.h"

void GPhyscsUtils::DrawShape(const GTransform_QT& Trans, const GCollisionShape& pShape, IGlacierDraw* pDebugDraw )
{
    switch (pShape.ShapType )
    {
    case  EShape::EShape_ConvexBase:
    {
        
    }
    break;
    case  EShape::EShape_Sphere:
    {
        pDebugDraw->DrawSphere(Trans, pShape.GetRaiuds(), GColor::White(), 18 );
    }
    break;
    case  EShape::EShape_Box:
    {
        pDebugDraw->DrawBox(Trans, GVector3::Zero(), pShape.GetHalfExtern(), GColor::White());
    }
    break;
    case  EShape::EShape_Capsule:
    {
        pDebugDraw->DrawCapsule(Trans, pShape.GetRaiuds(), pShape.GetHalfHeight(), GColor::White());
    }
    break;
    case  EShape::EShape_Cylinder:
    {

    }
    break;
    case  EShape::EShape_ConvexHull:
    {

    }
    break;

    case  EShape::EShape_ConcaveBase:
    {

    }
    break;
    case  EShape::EShape_Plane:
    {
         pDebugDraw->DrawPlane(Trans, GPlane( pShape.GetPlaneNormal(), GMath::Zero() ), f32(50), GColor::White());
    }
    break;
    case  EShape::EShape_HightField:
    {

    }
    break;
    case  EShape::EShape_TriangleMesh:
    {

    }
    break;

    default:
        break;
    }

}

