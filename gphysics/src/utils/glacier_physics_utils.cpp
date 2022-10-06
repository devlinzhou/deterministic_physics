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
#include "glacier_collision_shape.h"
#include "glacier_debug_draw.h"

void GPhyscsUtils::DrawShape(const GTransform_QT& Trans, const GShapeBase* pShape, IGlacierDraw* pDebugDraw )
{
    switch (pShape->ShapType )
    {
    case  ConvexBase:
    {
        
    }
    break;
    case  Sphere:
    {
        GShapeSphere* pSphere = (GShapeSphere*)pShape;
        pDebugDraw->DrawSphere(Trans, pSphere->Radius, GColor::White(), 18 );
    }
    break;
    case  Box:
    {
        GShapeBox* pBox = (GShapeBox*)pShape;
        pDebugDraw->DrawBox(Trans, GVector3::Zero(), pBox->HalfExtern, GColor::White());
    }
    break;
    case  Capsule:
    {
        GShapeCapsule* pCapsule = (GShapeCapsule*)pShape;
        pDebugDraw->DrawCapsule(Trans, pCapsule->Raius, pCapsule->HalfHeight, GColor::White());
    }
    break;
    case  Cylinder:
    {

    }
    break;
    case  ConvexHull:
    {

    }
    break;

    case  ConcaveBase:
    {

    }
    break;
    case  HightField:
    {

    }
    break;
    case  TriangleMesh:
    {

    }
    break;

    default:
        break;
    }

}

