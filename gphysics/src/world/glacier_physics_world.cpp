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
#include "glacier_physics_world.h"
#include "glacier_vector.h"
#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"
#include "glacier_physics_utils.h"
#include "glacier_debug_draw.h"
#include "glacier_collision_object.h"

void GGridCell::DebugDraw(IGlacierDraw* pDraw) const
{
    pDraw->DrawBox( GTransform_QT::Identity(), GetCenter(), GetHalfSize(),0x00FFFF00 );

    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        const GCollisionObject* pObject = m_Objects[i];
        GPhyscsUtils::DrawShape( pObject->m_Transform, pObject->m_pShape, pDraw);
    }

}

bool GGridCell::AddCollisionObject(GCollisionObject* pObject)
{
    for( int32_t i = 0; i < (int32_t)m_Objects.size(); ++i )
    {
        if( pObject == m_Objects[i])
            return false;
    }

    m_Objects.push_back(pObject);

    return true;
}


void GPhysicsWorld::AddCollisionObject(GCollisionObject* pObject)
{
    GVector3 VPos = pObject->m_Transform.m_Translation;

    GGridPosition CellPos;

    CellPos.x = GMath::FloorToInt(VPos.x / m_nCellWide );
    CellPos.y = GMath::FloorToInt(VPos.y / m_nCellWide);
    CellPos.z = GMath::FloorToInt(VPos.z / m_nCellHeight);

    GGridCell* pCell = nullptr;

    std::map<GGridPosition, GGridCell*>::iterator iter = m_Grids.find(CellPos); 

    if( iter != m_Grids.end() )
    {
        pCell = iter->second;
    }
    else
    {
        pCell = new GGridCell( CellPos);
        m_Grids[CellPos] = pCell;
    }

    if( pCell != nullptr)
    {
        pCell->AddCollisionObject(pObject);
    }


}

void GPhysicsWorld::PreTick()
{

}

void GPhysicsWorld::SimulateTick(f32 DetltaTime)
{

}

void GPhysicsWorld::PostTick()
{


}


void GPhysicsWorld::DebugDraw(IGlacierDraw* pDraw ) const
{
   // std::map<GGridPosition, GGridCell*> m_Grids;

    for( std::map<GGridPosition, GGridCell*>::const_iterator iter = m_Grids.begin(); iter != m_Grids.end(); ++iter )
    {
        iter->second->DebugDraw(pDraw);
    }

}