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
#include "glacier_contact.h"

void GPhyscsUtils::DrawShape(const GTransform_QT& Trans, const GCollisionShape& pShape, IGlacierDraw* pDebugDraw, GColor TColor )
{
    switch (pShape.ShapType )
    {
    case  EShape::EShape_ConvexBase:
    {
        
    }
    break;
    case  EShape::EShape_Sphere:
    {
        pDebugDraw->DrawSphere(Trans, pShape.GetRaiuds(), TColor, 10 );
    }
    break;
    case  EShape::EShape_Box:
    {
        pDebugDraw->DrawBox(Trans, GVector3::Zero(), pShape.GetHalfExtern(), TColor);
    }
    break;
    case  EShape::EShape_Capsule:
    {
        pDebugDraw->DrawCapsule(Trans, pShape.GetRaiuds(), pShape.GetHalfHeight(), TColor);
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
         pDebugDraw->DrawPlane(Trans, GPlane( pShape.GetPlaneNormal(), GMath::Zero() ), f32(50), TColor);
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

void GPhyscsUtils::DrawContact( const GCollisionContact& TContact, IGlacierDraw* pDebugDraw, GColor TColor )
{
    for (int i = 0; i < TContact.GetPointCount(); ++i)
    {
        const GManifoldPoint& TMn = TContact.m_Point[i];

        GVector3 VPos = TMn.m_PosWorld;
        GVector3 VNor = TMn.m_NormalOnB;

        GVector3 Vdes = VPos + VNor * TMn.m_depth;

        pDebugDraw->DrawSphere(VPos, GMath::Makef32(0,10,1000), TColor, 4);
        pDebugDraw->DrawSphere(Vdes, GMath::Makef32(0, 5, 1000), TColor, 4);
        pDebugDraw->DrawLine( VPos, Vdes, TColor);
    }
}

GMatrix3 GPhyscsUtils::CalculateInertiaTensor( const GCollisionShape& pShape )
{

    switch (pShape.ShapType)
    {
    case  EShape::EShape_ConvexBase:
    {
         return GMatrix3::Identity();
    }
    break;
    case  EShape::EShape_Sphere:
    {
        f32 t = pShape.GetRaiuds() * pShape.GetRaiuds() * GMath::Makef32(0,4,10);
        return GMatrix3(
            t, GMath::Zero(), GMath::Zero(),
            GMath::Zero(), t, GMath::Zero(),
            GMath::Zero(), GMath::Zero(), t );
    }
    break;
    case  EShape::EShape_Box:
    {
        GVector3 t = ( pShape.GetHalfExtern() * pShape.GetHalfExtern() ) * GMath::Makef32(0,1,3);

        return GMatrix3(
            t.y + t.z, GMath::Zero(), GMath::Zero(),
            GMath::Zero(), t.x + t.z, GMath::Zero(),
            GMath::Zero(), GMath::Zero(), t.x + t.y);
    }
    break;
    case  EShape::EShape_Capsule:
    {
       
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
        return GMatrix3::Identity();
    }
    break;
    case  EShape::EShape_Plane:
    {
        return GMatrix3::Identity();
    }
    break;
    case  EShape::EShape_HightField:
    {
        return GMatrix3::Identity();
    }
    break;
    case  EShape::EShape_TriangleMesh:
    {
        return GMatrix3::Identity();
    }
    break;

    default:
        break;
    }


    return GMatrix3::Identity();

}