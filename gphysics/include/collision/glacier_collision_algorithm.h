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

#include "glacier_transform_qt.h"
#include "glacier_collision_object.h"
#include "glacier_contact.h"
#include <vector>
#include <map>


class GCollisionAlgorithm
{
public:

    GCollisionAlgorithm( bool value = false) : m_bSwap( value ), CallTimes(0)
    {
    
    }

    virtual bool ProcessCollision( const GCollisionObject* ObjA, const GCollisionObject* ObjB, GCollisionContact* pContact = nullptr );

    bool    m_bSwap;
    int32_t CallTimes;
};

class GCG_Sphere_Sphere : public GCollisionAlgorithm
{
public:
    virtual bool ProcessCollision(const GCollisionObject* ObjA, const GCollisionObject* ObjB, GCollisionContact* pContact = nullptr  ) override;
};

class GCG_Sphere_Box : public GCollisionAlgorithm
{
public:
    virtual bool ProcessCollision(const GCollisionObject* ObjA, const GCollisionObject* ObjB, GCollisionContact* pContact = nullptr) override;
};

class GCG_Sphere_Capusle : public GCollisionAlgorithm
{
public:
    virtual bool ProcessCollision(const GCollisionObject* ObjA, const GCollisionObject* ObjB, GCollisionContact* pContact = nullptr) override;
};

class GCG_Sphere_Cylinder : public GCollisionAlgorithm
{
public:
    virtual bool ProcessCollision(const GCollisionObject* ObjA, const GCollisionObject* ObjB, GCollisionContact* pContact = nullptr) override;
};

class GCG_Sphere_Plane : public GCollisionAlgorithm
{
public:
    virtual bool ProcessCollision(const GCollisionObject* ObjA, const GCollisionObject* ObjB, GCollisionContact* pContact = nullptr) override;
};

class GCG_Box_Box : public GCollisionAlgorithm
{
public:
    virtual bool ProcessCollision(const GCollisionObject* ObjA, const GCollisionObject* ObjB, GCollisionContact* pContact = nullptr) override;
};



class GCollisionManerger
{
public:

    GCollisionManerger();
    ~GCollisionManerger();


    void Init( );
    
    inline GCollisionAlgorithm* GetAlgrithm( int32_t nShapeA, int32_t nShapeB)
    {
        return m_Algorithm[nShapeA][nShapeB];
    }

private:
    GCollisionAlgorithm* m_Algorithm[EShape_Max][EShape_Max];

};