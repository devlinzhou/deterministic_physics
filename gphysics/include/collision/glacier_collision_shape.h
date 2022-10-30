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
#include "glacier_aabb.h"

enum EShape
{
    EShape_ConvexBase   = 0,
    EShape_Sphere       = 1,
    EShape_Box          = 2,
    EShape_Capsule      = 3,
    EShape_Cylinder     = 4,
    EShape_ConvexHull   = 5,
    EShape_ConcaveBase  = 6,
    EShape_Plane        = 7,
    EShape_HightField   = 8,
    EShape_TriangleMesh = 9,
    EShape_Max          = 10
};

class GShapeBase;
class GCollisionShape
{
public:
    GCollisionShape( EShape TShape ) : 
        ShapType(TShape), HalfSize(GVector3::Identity()), pComplexShape(nullptr)
    {
    
    }

public:
    EShape      ShapType; // simple shape
    GVector3    HalfSize;
    GShapeBase* pComplexShape;  

    f32 GetRaiuds() const { return HalfSize.x;} 
    void SetRadius( const f32 value ){ HalfSize.x = value;}

    f32 GetHalfHeight() const { return HalfSize.y; }
    void SetHalfHeight(const f32 value) { HalfSize.y = value; }

    GVector3 GetHalfExtern() const { return HalfSize; }
    void SetHalfExtern(const GVector3& value) { HalfSize = value; }

    GVector3 GetPlaneNormal() const { return HalfSize; }
    void SetPlaneNormal(const GVector3& value) { HalfSize = value; }

    GAABB GetLocalBox() const;

};

class GShapeBase
{
public:
    EShape  ShapType;

    GShapeBase() : ShapType(EShape::EShape_Max)
    {
    
    }
};

class GShapeConvexBase : public GShapeBase
{
public:

    GShapeConvexBase() 
    {
        ShapType = EShape::EShape_ConvexBase;
    }

    // Dir is normalize
    virtual GVector3 GetSupportLocalPos( const GVector3& Dir ) const
    {
        return Dir;
    }

};

class GShapeSphere : public GShapeConvexBase
{
public:
    GShapeSphere( f32 tRaduis ) : Radius(tRaduis)
    {
        ShapType = EShape::EShape_Sphere;
    }

    virtual GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return Dir * Radius;
    }

    f32 Radius;
};
class GShapeBox : public GShapeConvexBase
{
public:
    GShapeBox(GVector3 tvalue) : HalfExtern(tvalue)
    {
        ShapType = EShape::EShape_Box;
    }

    virtual GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return GVector3(
            Dir.x > 0 ? HalfExtern.x : -HalfExtern.x,
            Dir.y > 0 ? HalfExtern.y : -HalfExtern.y,
            Dir.z > 0 ? HalfExtern.z : -HalfExtern.z);
    }

    GVector3 HalfExtern;
};


class GShapeCapsule : public GShapeConvexBase
{
public:

    GShapeCapsule( f32 THalfHeight, f32 tRaduis) : HalfHeight(THalfHeight), Raius(tRaduis)
    {
        ShapType = EShape::EShape_Capsule;
    }

    virtual GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        GVector3 supVec = GVector3::Zero();

        f32  maxDot(f32(-10000));

        f32 newDot;
        {
            GVector3 pos = GVector3( GMath::Zero(), GMath::Zero(), HalfHeight );

            GVector3 vtx = pos + Dir * Raius ;
            newDot = GVector3::DotProduct( Dir, vtx );
            if (newDot > maxDot)
            {
                maxDot = newDot;
                supVec = vtx;
            }
        }
        {
            GVector3 pos = GVector3( GMath::Zero(), GMath::Zero(),-HalfHeight );
            GVector3 vtx = pos + Dir * Raius;
            newDot = GVector3::DotProduct( Dir, vtx );
            if (newDot > maxDot)
            {
                maxDot = newDot;
                supVec = vtx;
            }
        }

        return supVec;
    }

    f32 HalfHeight;
    f32 Raius;
};

class GShapeCylinder : public GShapeConvexBase
{
public:

    GShapeCylinder(f32 THalfHeight, f32 tRaduis) : HalfHeight(THalfHeight), Raius(tRaduis)
    {
        ShapType = EShape::EShape_Capsule;
    }

    GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return Dir * Raius;
    }

    f32 HalfHeight;
    f32 Raius;
};


class GShapeConcaveBase : public GShapeBase
{
public:

    GShapeConcaveBase()
    {
        ShapType = EShape::EShape_ConcaveBase;
    }


};

class GShapeHightField : public GShapeConcaveBase
{
public:

    GShapeHightField( )
    {
        ShapType = EShape::EShape_HightField;
    }


};

class GShapeTriangleMesh : public GShapeConcaveBase
{
public:

    GShapeTriangleMesh()
    {
        ShapType = EShape::EShape_TriangleMesh;
    }


};