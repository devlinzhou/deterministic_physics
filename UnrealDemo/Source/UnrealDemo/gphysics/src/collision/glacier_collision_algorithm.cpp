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


bool GCollisionAlgorithm::ProcessCollision(const GCollisionObject* ShapA, const GCollisionObject* ShapB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Sphere::ProcessCollision(const GCollisionObject* ShapA, const GCollisionObject* ShapB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Box::ProcessCollision(const GCollisionObject* ShapA, const GCollisionObject* ShapB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Capusle::ProcessCollision(const GCollisionObject* ShapA, const GCollisionObject* ShapB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Cylinder::ProcessCollision(const GCollisionObject* ShapA, const GCollisionObject* ShapB, GCollisionContact* pContact )
{
    return false;
}

bool GCG_Sphere_Plane::ProcessCollision(const GCollisionObject* ShapA, const GCollisionObject* ShapB, GCollisionContact* pContact )
{
    return false;
}


GCollisionManerger::GCollisionManerger()
{
    for (int32_t i = 0; i < EShape_Max; ++i)
    {
        for (int32_t j = 0; j < EShape_Max; ++j)
        {
            m_Glorithm[i][j] = nullptr;
        }
    }
}

GCollisionManerger::~GCollisionManerger()
{
    for (int32_t i = 0; i < EShape_Max; ++i)
    {
        for (int32_t j = 0; j < EShape_Max; ++j)
        {
            if( m_Glorithm[i][j] != nullptr )
            {
                delete m_Glorithm[i][j];
                m_Glorithm[i][j] = nullptr;
            }
        }
    }
}


void GCollisionManerger::Init( )
{

    m_Glorithm[EShape_Sphere][EShape_Sphere]= new GCG_Sphere_Sphere();



}
