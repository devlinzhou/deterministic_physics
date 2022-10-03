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

enum EShape
{
    Null = 0,
    Sphere,
    Box,
    Capsule,
    Cylinder,
    ConvexHull,
};

class GShapeConvexBase
{
public:
    EShape  ShapType;
public:
    GShapeConvexBase( ) : ShapType(EShape::Null)
    {
    
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
    GShapeSphere( f32 tRaduis ) : Raius(tRaduis)
    {
        ShapType = EShape::Sphere;
    }

    virtual GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return Dir * Raius;
    }

    f32 Raius;
};
class GShapeBox : public GShapeConvexBase
{
public:
    GShapeBox(GVector3 tvalue) : HalfExtern(tvalue)
    {
        ShapType = EShape::Box;
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
        ShapType = EShape::Capsule;
    }

    GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return Dir * Raius;
    }

    f32 HalfHeight;
    f32 Raius;
};

class GShapeCylinder : public GShapeConvexBase
{
public:

    GShapeCylinder(f32 THalfHeight, f32 tRaduis) : HalfHeight(THalfHeight), Raius(tRaduis)
    {
        ShapType = EShape::Capsule;
    }

    GVector3 GetSupportLocalPos(const GVector3& Dir) const
    {
        return Dir * Raius;
    }

    f32 HalfHeight;
    f32 Raius;
};