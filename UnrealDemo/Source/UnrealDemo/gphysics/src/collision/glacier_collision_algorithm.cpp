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

#include "glacier_collision_algorithm.h"
#include "glacier_collision_sphere.h"
#include "glacier_collision_gjk.h"
#include "glacier_collision_box.h"


bool GCollisionAlgorithm::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Sphere::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact )
{
    GVector3 d = ObjA->m_Transform.m_Pos - ObjB->m_Transform.m_Pos;
    f32 len2 = d.SizeSquare();
    f32 TRadius = ObjA->m_Shape.GetRaiuds() + ObjB->m_Shape.GetRaiuds();

    if( len2 < (TRadius * TRadius) )
    {
        if (pContact != nullptr)
        {
            f32 InvLen = GMath::InvSqrt(len2);

            GVector3 VNormalOnB = len2 < GMath::Epsilon() ? GVector3::UnitX() : d * InvLen;
            GVector3 VPosOnB = ObjA->m_Transform.m_Pos - VNormalOnB * ObjA->m_Shape.GetRaiuds();

            pContact->AddContactPoint( VPosOnB, VNormalOnB, len2 * InvLen - TRadius );

            pContact->PointOnSurface = ObjA->GetId();
        }

        return true;
    }

    return false;
}

bool GCG_Sphere_Box::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact )
{
    const GTransform_QT& TransS = m_bSwap ? ObjB->m_Transform : ObjA->m_Transform;
    const GTransform_QT& TransB = m_bSwap ? ObjA->m_Transform : ObjB->m_Transform;

    GShapeSphere TSphere(m_bSwap ? ObjB->m_Shape.GetRaiuds()     : ObjA->m_Shape.GetRaiuds());
    GShapeBox    TBox(   m_bSwap ? ObjA->m_Shape.GetHalfExtern() : ObjB->m_Shape.GetHalfExtern());

    int32_t nCount = GCollision_Sphere::Sphere_Box_Contact(TSphere, TransS, TBox, TransB, pContact );

    if (pContact)
    {
        pContact->PointOnSurface = m_bSwap ? ObjB->GetId() : ObjA->GetId();
    }

    return nCount > 0;
}

bool GCG_Sphere_Capusle::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Cylinder::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Plane::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact )
{
    const GTransform_QT& TransS = m_bSwap ? ObjB->m_Transform : ObjA->m_Transform;
    const GTransform_QT& TransP = m_bSwap ? ObjA->m_Transform : ObjB->m_Transform;

    GShapeSphere TSphere( m_bSwap ? ObjB->m_Shape.GetRaiuds() : ObjA->m_Shape.GetRaiuds() );
    GShapePlane  TPlane;

    int32_t nCount = GCollision_Sphere::Sphere_Plane_Contact(TSphere, TransS, TPlane, TransP, pContact);
    if (pContact)
    {
        pContact->PointOnSurface =  m_bSwap ? ObjB->GetId() : ObjA->GetId();;
    }

    return nCount > 0;
}

bool GCG_Box_Box::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact )
{
    GShapeBox BoxA(ObjA->m_Shape.GetHalfExtern());
    GShapeBox BoxB(ObjB->m_Shape.GetHalfExtern());

    if (GCollision_Box::Box_Box_Contact( BoxA, ObjA->m_Transform, BoxB ,ObjB->m_Transform, pContact) != 0)
    {
        return true;
    }

    if (pContact)
    {
        pContact->PointOnSurface = ObjA->GetId();
    }

    return false;
}

bool GCG_Plane_Box::ProcessCollision(const GCObject* ObjA, const GCObject* ObjB, GCollisionContact* pContact)
{
    const GTransform_QT& TransP = m_bSwap ? ObjB->m_Transform : ObjA->m_Transform;
    const GTransform_QT& TransB = m_bSwap ? ObjA->m_Transform : ObjB->m_Transform;

    GShapePlane  TPlane;
    GShapeBox    TBox(m_bSwap ? ObjA->m_Shape.GetHalfExtern() : ObjB->m_Shape.GetHalfExtern());

    int32_t nCount = GCollision_Box::Plane_Box_Contact(TPlane, TransP, TBox, TransB, pContact);
    if (pContact)
    {
        pContact->PointOnSurface = m_bSwap ? ObjA->GetId() : ObjB->GetId();
    }

    return nCount > 0;
}


GCollisionManerger::GCollisionManerger()
{
    for (int32_t i = 0; i < EShape_Max; ++i)
    {
        for (int32_t j = 0; j < EShape_Max; ++j)
        {
            m_Algorithm[i][j] = nullptr;
        }
    }
}

GCollisionManerger::~GCollisionManerger()
{
    for (int32_t i = 0; i < EShape_Max; ++i)
    {
        for (int32_t j = 0; j < EShape_Max; ++j)
        {
            if( m_Algorithm[i][j] != nullptr )
            {
                delete m_Algorithm[i][j];
                m_Algorithm[i][j] = nullptr;
            }
        }
    }
}


void GCollisionManerger::Init( )
{

    m_Algorithm[EShape_Sphere][EShape_Sphere]   = new GCG_Sphere_Sphere();
    m_Algorithm[EShape_Box][EShape_Box]         = new GCG_Box_Box();

    m_Algorithm[EShape_Sphere][EShape_Box]      = new GCG_Sphere_Box();
    m_Algorithm[EShape_Box][EShape_Sphere]      = new GCG_Sphere_Box(true);


    m_Algorithm[EShape_Sphere][EShape_Plane]    = new GCG_Sphere_Plane();
    m_Algorithm[EShape_Plane][EShape_Sphere]    = new GCG_Sphere_Plane(true);


    m_Algorithm[EShape_Plane][EShape_Box]       = new GCG_Plane_Box();
    m_Algorithm[EShape_Box][EShape_Plane]       = new GCG_Plane_Box(true);


}
