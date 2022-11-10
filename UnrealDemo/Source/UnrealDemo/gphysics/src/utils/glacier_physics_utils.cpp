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
#include "glacier_convexhull.h"
#include "glacier_physics_world.h"


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
        pDebugDraw->DrawCylinder(Trans, pShape.GetRaiuds(), pShape.GetHalfHeight(), TColor);
    }
    break;
    case  EShape::EShape_ConvexHull:
    {
        if( pShape.pComplexShape != nullptr )
        {
            const GConvexHull* pConvex = (GConvexHull*)pShape.pComplexShape;

            pConvex->Draw( pDebugDraw, Trans, TColor );
        }
    }
    break;

    case  EShape::EShape_ConcaveBase:
    {

    }
    break;
    case  EShape::EShape_Plane:
    {
         pDebugDraw->DrawPlane(Trans, f32(50), TColor);
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

void GPhyscsUtils::DrawSphere(const GTransform_QT& Trans, const GShapeSphere& pShape, IGlacierDraw* pDebugDraw, GColor TColor)
{
    pDebugDraw->DrawSphere(Trans, pShape.Radius, TColor, 10 );
}


void GPhyscsUtils::DrawBox(const GTransform_QT& Trans, const GShapeBox& pShape, IGlacierDraw* pDebugDraw, GColor TColor)
{
    pDebugDraw->DrawBox(Trans, GVector3::Zero(), pShape.HalfExtern, TColor );
}

void GPhyscsUtils::DrawCapsule(const GTransform_QT& Trans, const GShapeCapsule& pShape, IGlacierDraw* pDebugDraw, GColor TColor)
{
    pDebugDraw->DrawCapsule(Trans, pShape.Raius,  pShape.HalfHeight, TColor );
}

void GPhyscsUtils::DrawCoordinateSystem(IGlacierDraw* pDebugDraw, const GTransform_QT& Trans, f32 fSize)
{
    GMatrix3 M33(Trans.m_Rot);

    pDebugDraw->DrawLine(Trans.m_Pos, Trans.m_Pos + M33.GetRow(0), GColor::Red() );
    pDebugDraw->DrawLine(Trans.m_Pos, Trans.m_Pos + M33.GetRow(1), GColor::Green());
    pDebugDraw->DrawLine(Trans.m_Pos, Trans.m_Pos + M33.GetRow(2), GColor::Blue());
}


void GPhyscsUtils::DrawContact( const GCollisionContact& TContact, IGlacierDraw* pDebugDraw, GColor TColor, const GPhysicsWorld* pWorld )
{
    GVector3 vPosOnObj;
    const GCObject* pObj = nullptr;
    if( pWorld != nullptr )
    {
         pObj = pWorld->FindCollisionObject( TContact.PointOnSurface );
    }


    for (int i = 0; i < TContact.GetPointCount(); ++i)
    {
        const GManifoldPoint& TMn = TContact.m_Point[i];

        GVector3 VPos = TMn.m_PosWorld;
        GVector3 VNor = TMn.m_Normal;

        GVector3 Vdes = VPos - VNor * TMn.m_depth;

        pDebugDraw->DrawSphere(VPos, GMath::Makef32(0,10, 1000), TColor, 4);
        pDebugDraw->DrawSphere(Vdes, GMath::Makef32(0, 7, 1000), GColor::Gray(), 4);
        pDebugDraw->DrawLine( VPos, Vdes, TColor);

        if (pObj != nullptr)
        {
            pDebugDraw->DrawLine( VPos, pObj->m_Transform.m_Pos, GColor::Gray());
        }
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
        f32 Radius = pShape.GetRaiuds();
        f32 RSqure = Radius * Radius;

        f32 I =  
            GMath::Makef32(0,1,4) * RSqure +
            GMath::Makef32(0,1,3) * pShape.GetHalfHeight() * pShape.GetHalfHeight();

        return GMatrix3(
            I, GMath::Zero(), GMath::Zero(),
            GMath::Zero(), I, GMath::Zero(),
            GMath::Zero(), GMath::Zero(), GMath::Half() * RSqure);
    }
    break;
    case  EShape::EShape_Cylinder:
    {
        f32 HalfH = pShape.GetHalfHeight();
        f32 Radius = pShape.GetRaiuds();
        f32 RSqure = Radius * Radius;

        f32 I = GMath::Makef32(0, 1, 4) * RSqure + GMath::Makef32(0, 1, 3) * HalfH * HalfH;

        return GMatrix3(
            I, GMath::Zero(), GMath::Zero(),
            GMath::Zero(), I, GMath::Zero(),
            GMath::Zero(), GMath::Zero(), GMath::Half() * RSqure);
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

f32 GPhyscsUtils::CalculateVolume(const GCollisionShape& pShape)
{
    switch (pShape.ShapType)
    {
    case  EShape::EShape_ConvexBase:
    {
        return GMath::Zero();
    }
    break;
    case  EShape::EShape_Sphere:
    {
        f32 t = pShape.GetRaiuds() * pShape.GetRaiuds() * GMath::Makef32(0, 4, 10);
    }
    break;
    case  EShape::EShape_Box:
    {
        f32 t = pShape.GetHalfExtern().x * pShape.GetHalfExtern().y * pShape.GetHalfExtern().z * GMath::Makef32(8,0,1) ;

        return t;
    }
    break;
    case  EShape::EShape_Capsule:
    {
        f32 t = pShape.GetRaiuds() * pShape.GetRaiuds() * pShape.GetHalfHeight() * GMath::Pi_Two();
        t += pShape.GetRaiuds() * pShape.GetRaiuds() * pShape.GetRaiuds() * GMath::Makef32(1,1,3);
        return t;
    }
    break;
    case  EShape::EShape_Cylinder:
    {
        f32 t = pShape.GetRaiuds() * pShape.GetRaiuds() * pShape.GetHalfHeight() * GMath::Pi_Two() ;

        return t;
    }
    break;
    case  EShape::EShape_ConvexHull:
    {

    }
    break;
    case  EShape::EShape_ConcaveBase:
    {
         return GMath::Zero();
    }
    break;
    case  EShape::EShape_Plane:
    {
        return GMath::Zero();
    }
    break;
    case  EShape::EShape_HightField:
    {
        return GMath::Zero();
    }
    break;
    case  EShape::EShape_TriangleMesh:
    {
        return GMath::Zero();
    }
    break;

    default:
        break;
    }


    return GMath::Zero();
}